"""Validate an inherited Linux flock capability without acquiring or unlocking it.

The supervisor retains its RunLock. Its one keyboard child receives the same open
file description explicitly, not a PID, process name or environment exemption.
Linux fdinfo associates flock records with that description; opening the same
path independently does not inherit the record. No flock operation is issued here.
"""

from __future__ import annotations

import os
from pathlib import Path
import stat
from typing import Optional

from nav_bringup.navrun.locking import DEFAULT_LOCK_PATH


class DelegatedLock:
    """A close-only duplicate of an already exclusively held navigation lease.

    Attributes:
        path: 预期导航锁文件的绝对路径。
        source: 显式继承的原始描述符；本对象仅管理其副本。
    """

    def __init__(self, descriptor: int, path: Path = DEFAULT_LOCK_PATH) -> None:
        """Store the capability selector without opening or changing any resource.

        Args:
            descriptor: Explicit inherited descriptor, never stdin/stdout/stderr.
            path: Expected navigation lock path; tests use a private file.

        Raises:
            ValueError: Descriptor or path is malformed.
        """
        if type(descriptor) is not int or descriptor < 3 or not Path(path).is_absolute():
            raise ValueError('delegation requires an explicit descriptor and absolute lock path')
        self.source = descriptor
        self.path = Path(path)
        self._descriptor: Optional[int] = None

    def __enter__(self) -> DelegatedLock:
        """Duplicate and validate exclusive ownership without changing the shared flock.

        Returns:
            The validated duplicate, which is close-on-exec.

        Raises:
            RuntimeError: Already entered.
            ValueError: Not the expected, exclusively held open file description.
            OSError: Descriptor, path or Linux ownership evidence cannot be read.
        """
        if self._descriptor is not None:
            raise RuntimeError('delegated lock is already open')
        self._descriptor = os.dup(self.source)
        try:
            os.set_inheritable(self._descriptor, False)
            self.verify()
        except BaseException:
            self.close()
            raise
        return self

    def fileno(self) -> int:
        """Return this object's duplicate, not the supervisor's descriptor.

        Returns:
            The owned duplicate; callers must never unlock it.

        Raises:
            RuntimeError: The context has not been entered or is closed.
        """
        if self._descriptor is None:
            raise RuntimeError('delegated lock is not open')
        return self._descriptor

    def verify(self) -> None:
        """Recheck path identity and exclusive flock evidence without a locking syscall.

        Raises:
            RuntimeError: The context is closed.
            ValueError: Ownership changed or evidence is not exclusive.
            OSError: Required Linux ownership evidence is unavailable.
        """
        descriptor = self.fileno()
        info, current = os.fstat(descriptor), self.path.lstat()
        if (not stat.S_ISREG(info.st_mode) or not stat.S_ISREG(current.st_mode)
                or info.st_uid != os.geteuid() or info.st_nlink != 1 or info.st_mode & 0o022
                or (info.st_dev, info.st_ino) != (current.st_dev, current.st_ino)):
            raise ValueError('delegated lock is not the expected owned file')
        evidence = Path('/proc/self/fdinfo', str(descriptor)).read_text(encoding='ascii')
        locks = [line.split() for line in evidence.splitlines() if line.startswith('lock:')]
        if not any(len(fields) == 9 and fields[2:5] == ['FLOCK', 'ADVISORY', 'WRITE']
                   and fields[-2:] == ['0', 'EOF'] for fields in locks):
            raise ValueError('open file description does not hold an exclusively held flock')

    def close(self) -> None:
        """Close only our duplicate; never issue LOCK_UN on the shared description."""
        if self._descriptor is not None:
            descriptor, self._descriptor = self._descriptor, None
            os.close(descriptor)

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """Drop only this duplicate even when keyboard startup fails.

        Args:
            exc_type: Context exception class.
            exc: Context exception instance.
            traceback: Context exception traceback.
        """
        self.close()
