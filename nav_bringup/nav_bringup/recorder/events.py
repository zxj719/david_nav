"""以 JSON 行保存运行事件时间线（PLAN §6.4、§13.2）.

每行包含 stamp、source、type、severity 和 data，覆盖任务、恢复、进程及检查事件。
写入使用线程锁与协作文件锁，flush 但不 fsync，不保证断电持久性或失败事务恢复。
本地编解码沿用 Python JSON 的宽松规则，不替代严格的对外协议验证。
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
import fcntl
import json
from pathlib import Path
import threading
import time
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple, Union

SEVERITIES = ('info', 'warn', 'error')


@dataclass(frozen=True)
class Event:
    """表示一条本地运行事件，冻结字段但不深复制数据映射.

    Attributes:
        stamp: 从纪元起的墙钟秒数，构造时不检查有限性。
        source: 产生事件的组件名称。
        event_type: 事件类型，在 JSON 中使用 type 键。
        severity: info、warn 或 error，构造时检查此枚举。
        data: 待 JSON 序列化的详情，可含共享的嵌套对象。
    """

    stamp: float
    source: str
    event_type: str
    severity: str = 'info'
    data: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """只校验严重性枚举，其它字段由生产者或传输适配器验证.

        Raises:
            ValueError: severity 不在允许枚举内。
        """
        if self.severity not in SEVERITIES:
            raise ValueError(f'invalid severity {self.severity!r}')

    def to_json(self, *, precise_stamp: bool = False) -> str:
        """序列化为不含末尾换行的 JSON 文本，保留非 ASCII 字符.

        只复制 data 的外层映射；不强制有限数值，沿用 Python JSON 的 NaN/Infinity 扩展。

        Args:
            precise_stamp: True 保留生产者浮点时间，否则按本地规则保留六位小数。

        Returns:
            JSON 文本，不代表已写入文件或通过严格线协议校验。

        Raises:
            TypeError: 时间或详情数据不支持既定转换/JSON 编码。
            ValueError: 循环等数据结构不能编码。
        """
        payload: Dict[str, Any] = {
            'stamp': self.stamp if precise_stamp else round(self.stamp, 6),
            'source': self.source,
            'type': self.event_type,
            'severity': self.severity,
            'data': dict(self.data),
        }
        return json.dumps(payload, ensure_ascii=False)

    @classmethod
    def from_json(cls, line: str) -> 'Event':
        """按本地宽松规则解析一行事件，并转换时间、文本和详情字段.

        时间转 float、来源与类型转 str，空值详情按空映射处理；不保证严格 schema、
        非空标识或有限时间。非法严重性仍由构造器拒绝。

        Args:
            line: 一条待解析的 JSON 文本。

        Returns:
            解析并转换后的事件对象。

        Raises:
            ValueError: JSON 无效、顶层非对象、必需字段缺失或常规字段转换失败。
        """
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f'invalid JSON: {error}') from error
        if not isinstance(payload, dict):
            raise ValueError('an event must be a JSON object')
        try:
            return cls(
                stamp=float(payload['stamp']),
                source=str(payload['source']),
                event_type=str(payload['type']),
                severity=str(payload.get('severity', 'info')),
                data=dict(payload.get('data') or {}),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'invalid event: {error}') from error


class EventLog:
    """表示可在线程间共享、以协作文件锁追加事件的写入器.

    线程锁只保护同一实例，独立实例/进程依靠 flock 协作；不阻止不遵守锁的写者。
    不验证记录路径所有权，不提供崩溃回滚或断电持久性保证。
    """

    def __init__(self, path: Union[str, Path], clock: Callable[[], float] = time.time) -> None:
        """绑定事件路径、时钟与实例锁，不在构造时创建或打开文件.

        Args:
            path: 事件文件路径，写入时父目录必须存在。
            clock: 墙钟时间端口，测试可替换。
        """
        self._path = Path(path)
        self._clock = clock
        self._lock = threading.Lock()

    @property
    def path(self) -> Path:
        """Path: 追加事件的文件路径，不代表文件已存在."""
        return self._path

    def append(
        self,
        source: str,
        event_type: str,
        severity: str = 'info',
        data: Optional[Mapping[str, Any]] = None,
    ) -> Event:
        """创建本地事件并同步追加，成功返回该事件对象.

        在持有写入锁之前读取时钟并浅复制 data，写入顺序不保证按时间戳排序。
        嵌套数据仍共享；写入前的并发修改由调用方避免。

        Args:
            source: 产生事件的组件。
            event_type: 事件类型。
            severity: info、warn 或 error。
            data: 可 JSON 编码的详情，仅复制最外层字典。

        Returns:
            已完成本次追加的事件，不表示已 fsync 或被消费者接收。

        Raises:
            ValueError: 严重性无效，或数据无法编码为 JSON/UTF-8。
            TypeError: 数据不可 JSON 序列化。
            OSError: 文件打开、锁定、写入或关闭失败。
        """
        if severity not in SEVERITIES:
            raise ValueError(f'invalid severity {severity!r}')
        event = Event(self._clock(), source, event_type, severity, dict(data or {}))
        self.record(event)
        return event

    def record(self, event: Event, *, precise_stamp: bool = False) -> None:
        """追加现有事件，不重置生产者时间；精度取决于 precise_stamp.

        先完成序列化与 UTF-8 预检，再在实例锁与 flock 内写入并 flush。
        协作锁避免正常写者交错，但磁盘满等异常可能留下部分行，不做事务回滚。
        路径打开跟随普通文件语义，外层须保证记录目录可信。

        Args:
            event: 已构造的事件，传输适配器应先验证外部输入。
            precise_stamp: 是否保留外部事件的完整浮点时间精度。

        Raises:
            OSError: 文件打开、锁定、写入或关闭失败。
            ValueError: 数据不能编码为 JSON/UTF-8 行。
            TypeError: 数据不可 JSON 序列化。
        """
        line = event.to_json(precise_stamp=precise_stamp) + '\n'
        line.encode('utf-8')  # Reject unpaired surrogates before creating/opening the file.
        with self._lock:
            with self._path.open('a', encoding='utf-8') as handle:
                fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
                handle.write(line)
                handle.flush()


def read_events(path: Union[str, Path]) -> Tuple[List[Event], int]:
    """按文件顺序读取本地事件，跳过并计数普通解析错误.

    忽略空白行，不加共享读锁或字节预算；末尾未写完整的非空行会计为跳过。
    路径按普通文件语义处理，不是受限、原子或严格协议读取器。

    Args:
        path: 事件文件路径，缺失或不是文件时返回空结果。

    Returns:
        有效事件列表与无法解析的非空行数；保持文件顺序而非按时间排序。

    Raises:
        OSError: 文件打开或读取失败。
        UnicodeError: 文件不是有效 UTF-8。
    """
    file_path = Path(path)
    if not file_path.is_file():
        return [], 0
    events: List[Event] = []
    skipped = 0
    with file_path.open('r', encoding='utf-8') as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                events.append(Event.from_json(line))
            except ValueError:
                skipped += 1
    return events, skipped
