"""把本地 navrun 时间线记录有界、可续跑地向外投影.

发布动作由既有 monitor 提供；本模块不持有任何 ROS 资源、线程或定时器，
也不重写或破坏性消费原始 JSONL。游标与租约是本模块自己的文件。
"已推进"只表示本地读到了哪里，不表示任何订阅者收到过。
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
    """持有一次运行的对外游标与租约；pump/close 只在调用方自己的循环里使用.

    本类不是线程安全的：同一实例的 pump 与 close 必须由同一个循环串行调用。
    互斥依靠 flock 租约，只能挡住同机上遵守该租约的其它发布者，
    挡不住无视租约直接改写时间线的写入方。
    """

    def __init__(
        self, path: Path, publish: Callable[[Mapping[str, object]], None],
        warn: Callable[[str], None],
    ) -> None:
        """加载或创建私有游标状态，构造期间不发布任何事件.

        先取得租约再读游标。坏游标一律报错，不会被当作"从零开始"，
        以免把已经发过的记录整批重发一遍。

        Args:
            path: 已存在运行目录中只追加的 events.jsonl 路径。
            publish: 调用方自有的消息构造与发布接收器。
            warn: 故障分类诊断；不会收到原始记录内容或路径。

        Raises:
            OSError: 状态/租约的 I/O 失败，或该运行已被另一个发布者占用。
            ValueError: 既有状态无效，不允许被覆盖。
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
        """str: 持久化的发布者身份前缀，续跑后保持不变，用于识别并跳过本发布者自己的回声."""
        return 'navrun:' + self._identity + ':'

    @property
    def closed(self) -> bool:
        """bool: 发布是否已封存；检测到时间线被替换或截断也会封存."""
        return self._closed

    @property
    def offset(self) -> int:
        """int: 本地已处理的字节数，不是任何 DDS 订阅者的投递确认."""
        return self._offset

    def pump(self, *, max_records: int = 128) -> int:
        """尝试推进一个有界批次；发布失败的那条保留到下次重试.

        行尾没有换行的半条记录留到下次；完整但非法或超长的记录会被跳过并给出
        分类告警，原文件不动。文件被替换或截断时整条流封存，不复用旧的事件身份。
        已封存或本次无进展时返回 0，不抛异常。检查点只说明本地读到哪里，
        既不是 DDS 投递确认，也不构成"恰好一次"。

        Args:
            max_records: 本次调用最多读取的记录条数，必须是正整数。

        Returns:
            本次成功返回的发布调用次数；不代表订阅者已经收到。

        Raises:
            ValueError: 调用方给出的批量上限不是正整数。
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
        """封存发布并只释放本运行的租约，不删除任何文件，也不动 ROS 资源的持有者.

        可重复调用：已封存时再次调用不做任何事，也不报错；封存后 pump 恒返回 0。
        """
        if not self._closed:
            self._closed = True
            self._lease.close()

    def _load(self):
        """读取并验证本运行的有界事件游标，不存在时持久化初始状态.

        游标必须为普通文件且不跟随链接；精确字段、UUID、非负偏移及文件身份
        均验证后才回填内存，不能把坏游标当从零开始继续发送。

        Raises:
            ValueError: 游标格式、尺寸、身份或偏移状态无效。
            OSError: 游标打开/读取或初始保存失败。
        """
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
        """用同目录临时文件原子替换游标，成功替换后才清除 dirty 标记.

        不提供 fsync 持久化保证；finally 只尝试移除本次创建的临时路径。

        Raises:
            OSError: 创建、替换或临时文件收尾失败；错误可能发生在替换已经完成之后。
        """
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
        """尽力报告事件发布诊断，日志接收器的普通异常不会再次外抛.

        不重发事件、不更改原失败类别，也不把日志写出视作事件发布成功。

        Args:
            category: 当前事件发布问题的分类文本。
        """
        try:
            self._warn(f'navrun event publication: {category}')
        except Exception:
            pass

    def _message(self, line, offset):
        """把允许转发的本地事件转换为脱敏传输映射，拒绝已带传输标记的回声.

        Args:
            line: 已取得的一条 JSON 事件原文。
            offset: 该记录在源文件中的字节位置，用于稳定事件身份和追踪。

        Returns:
            白名单字段、遗漏数量和偏移组成的传输消息；不是 navrun 来源或已有
            传输标记时为 None，不发送消息、不修改原记录。

        Raises:
            ValueError: JSON、重复键、信封、数据结构或数值不合法。
            KeyError: 本地事件缺少必需信封字段。
        """
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
