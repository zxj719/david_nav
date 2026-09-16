"""Bounded, restartable outward projection of local navrun timeline records.

The existing monitor supplies publication; this module owns no ROS resources,
threads or timers. The original JSONL is never rewritten or consumed destructively.
"""

import fcntl
import json
import math
import os
from pathlib import Path
import re
import stat
import tempfile
from typing import Callable, Mapping
import uuid

from nav_bringup.recorder.events import SEVERITIES
from nav_bringup.recorder.nav_events import _finite_tree, _pairs, TRANSPORT_KEY

_MAX_LINE = 65536
_BYTE_BUDGET = 1048576
_TOKEN = re.compile(r'[A-Za-z0-9_.:/-]{1,128}\Z')
_TEXT_KEYS = frozenset(('name', 'phase', 'stage', 'mode', 'robot', 'site', 'exit_reason'))
_NUMBER_KEYS = frozenset(('pid', 'exit_code', 'return_code', 'signal', 'bytes', 'removed_bytes'))
_BOOL_KEYS = frozenset(('found', 'exited_early', 'success'))


class RunEventStream:
    """Own one run's outward cursor and lease; pump/close are caller-loop confined."""

    def __init__(
        self, path: Path, publish: Callable[[Mapping[str, object]], None],
        warn: Callable[[str], None],
    ) -> None:
        """Load/create private cursor state without publishing anything.

        Args:
            path: Existing run directory's append-only events.jsonl path.
            publish: Caller-owned generated-message construction and publication sink.
            warn: Failure-category diagnostics; never receives raw records or paths.

        Raises:
            OSError: State/lease I/O failed or another publisher owns the run.
            ValueError: Existing state is invalid and must not be overwritten.
        """
        self._path = Path(path)
        self._state_path = self._path.with_name('nav_event_cursor.json')
        self._publish, self._warn = publish, warn
        self._closed = False
        self._offset = 0
        self._discarding = False
        self._dirty = False
        self._file = None
        self._identity = uuid.uuid4().hex
        descriptor = os.open(self._path.with_name('.nav_event_publisher.lock'),
                             os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
        self._lease = os.fdopen(descriptor, 'wb')
        try:
            if not stat.S_ISREG(os.fstat(descriptor).st_mode):
                raise ValueError('event publisher lease must be a regular file')
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._load()
        except BaseException:
            self._lease.close()
            raise

    @property
    def identity_prefix(self) -> str:
        """str: Persistent run identity used to suppress this publisher's own echoes."""
        return 'navrun:' + self._identity + ':'

    @property
    def offset(self) -> int:
        """int: Bytes processed locally, not an acknowledgement from any DDS subscriber."""
        return self._offset

    def pump(self, *, max_records: int = 128) -> int:
        """Attempt a bounded batch; keep a failed publication available for later retry.

        Partial trailing lines wait for their newline. Invalid/oversized complete
        records are skipped with a category warning; the original file is intact.
        Replacement/truncation seals the stream instead of reusing its identities.
        A checkpoint is not a DDS-delivery acknowledgement or an exactly-once claim.

        Args:
            max_records: Positive maximum number of read fragments in this call.

        Returns:
            Number of publication calls that returned successfully.

        Raises:
            ValueError: The caller's batch bound is not a positive integer.
        """
        if type(max_records) is not int or max_records <= 0:
            raise ValueError('event batch bound must be a positive integer')
        if self._closed:
            return 0
        published = 0
        previous = self._offset, self._discarding
        try:
            if self._dirty:
                self._save()
            descriptor = os.open(self._path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            with os.fdopen(descriptor, 'rb') as handle:
                fcntl.flock(handle.fileno(), fcntl.LOCK_SH | fcntl.LOCK_NB)
                info = os.fstat(handle.fileno())
                identity = [info.st_dev, info.st_ino]
                if (not stat.S_ISREG(info.st_mode) or info.st_size < self._offset
                        or (self._file is not None and self._file != identity)):
                    self.close()
                    raise ValueError('event timeline is not the original append-only file')
                if self._file is None:
                    self._file = identity
                    self._save()  # Bind the file before the first publication, including crashes.
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
                handle.seek(self._offset)
                budget = min(_BYTE_BUDGET, info.st_size - self._offset)
                for _ in range(max_records):
                    if budget <= 0 or self._closed:
                        break
                    start = handle.tell()
                    line = handle.readline(min(_MAX_LINE + 1, budget))
                    if not line:
                        break
                    budget -= len(line)
                    if self._discarding:
                        self._offset = handle.tell()
                        self._discarding = not line.endswith(b'\n')
                        continue
                    if len(line) > _MAX_LINE:
                        self._offset = handle.tell()
                        self._discarding = not line.endswith(b'\n')
                        self._warning('oversized record')
                        continue
                    if not line.endswith(b'\n'):
                        break
                    try:
                        message = self._message(line, start)
                    except Exception as error:
                        self._warning(type(error).__name__)
                        self._offset = handle.tell()
                        continue
                    if message is not None:
                        try:
                            self._publish(message)
                        except Exception as error:
                            self._warning(type(error).__name__)
                            break
                        published += 1
                    self._offset = handle.tell()
        except (FileNotFoundError, BlockingIOError):
            pass  # The parent has not created the file yet, or is appending this instant.
        except Exception as error:
            self._warning(type(error).__name__)
        if not self._closed and (previous != (self._offset, self._discarding) or self._dirty):
            try:
                self._save()
            except Exception as error:
                self._warning(type(error).__name__)
        return published

    def close(self) -> None:
        """Seal publication and release only this run's lease; preserve files and ROS owners."""
        if not self._closed:
            self._closed = True
            self._lease.close()

    def _load(self):
        try:
            descriptor = os.open(self._state_path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        except FileNotFoundError:
            self._save()
            return
        with os.fdopen(descriptor, 'rb') as handle:
            if not stat.S_ISREG(os.fstat(descriptor).st_mode):
                raise ValueError('event cursor must be a regular file')
            encoded = handle.read(4097)
        if len(encoded) > 4096:
            raise ValueError('event cursor is too large')
        value = json.loads(encoded, object_pairs_hook=_pairs)
        if (not isinstance(value, dict) or set(value) != {
                'version', 'identity', 'file', 'offset', 'discarding'}
                or type(value['version']) is not int or value['version'] != 1
                or not isinstance(value['identity'], str)
                or not re.fullmatch('[0-9a-f]{32}', value['identity'])
                or type(value['offset']) is not int or value['offset'] < 0
                or type(value['discarding']) is not bool):
            raise ValueError('invalid event cursor')
        file = value['file']
        if file is not None and (not isinstance(file, list) or len(file) != 2
                                 or any(type(item) is not int or item < 0 for item in file)):
            raise ValueError('invalid event file identity')
        if file is None and (value['offset'] != 0 or value['discarding']):
            raise ValueError('cursor position has no file identity')
        self._identity, self._file = value['identity'], file
        self._offset, self._discarding = value['offset'], value['discarding']

    def _save(self):
        self._dirty = True
        value = dict(version=1, identity=self._identity, file=self._file,
                     offset=self._offset, discarding=self._discarding)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile('w', encoding='utf-8',
                                             dir=self._path.parent, prefix='.nav-event-',
                                             delete=False) as handle:
                temporary_path = Path(handle.name)
                json.dump(value, handle)
            os.replace(temporary_path, self._state_path)
            self._dirty = False
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    def _warning(self, category):
        try:
            self._warn(f'navrun event publication: {category}')
        except Exception:
            pass

    def _message(self, line, offset):
        value = json.loads(line, object_pairs_hook=_pairs)
        if not isinstance(value, dict):
            raise ValueError('event must be an object')
        data = value.get('data')
        if not isinstance(data, dict):
            raise ValueError('event data must be an object')
        if value.get('source') != 'navrun' or TRANSPORT_KEY in data:
            return None
        stamp, name, severity = value['stamp'], value['type'], value['severity']
        if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                or not isinstance(name, str) or not _TOKEN.fullmatch(name)
                or severity not in SEVERITIES):
            raise ValueError('invalid local event envelope')
        _finite_tree(data)
        safe = {}
        for key, item in data.items():
            if key in _TEXT_KEYS and isinstance(item, str) and _TOKEN.fullmatch(item):
                safe[key] = item
            elif key in _NUMBER_KEYS and type(item) is int and abs(item) < 2**63:
                safe[key] = item
            elif key in _BOOL_KEYS and type(item) is bool:
                safe[key] = item
        safe.update(record_offset=offset, fields_omitted=len(data) - len(safe))
        return dict(event_id=self.identity_prefix + str(offset), stamp=float(stamp),
                    source='navrun', event_type=name, severity=severity,
                    data_json=json.dumps(safe, ensure_ascii=False, allow_nan=False))
