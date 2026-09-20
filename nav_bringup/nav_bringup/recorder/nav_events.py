"""严格的共享事件接收；不导入 ROS，也不具备任何运动权限.

传输身份放在保留字段 data.__nav_event 里，这样原有按五字段解析 JSONL 的消费者
不受影响。本模块只落盘，从不转发或重新发布任何事件。
"""

from collections import OrderedDict
import json
import math
import threading
from typing import Callable

from nav_bringup.recorder.events import Event, EventLog, SEVERITIES

TRANSPORT_KEY = '__nav_event'
MAX_DATA_BYTES = 65536
MAX_DEPTH = 32


class NavEventRecorder:
    """在既有 monitor 节点上校验、去重并追加观察到的事件.

    去重窗口是有界的内存 LRU：重启、淘汰与部分写盘都可能让同一事件再次落盘，
    因此不要把它当作"恰好一次"。
    """

    def __init__(
        self, log: EventLog, warn: Callable[[str], None], *, capacity: int = 4096,
        ignored_prefixes: tuple[str, ...] = (),
    ) -> None:
        """绑定某次运行的写入器与一个有界的内存去重窗口.

        Args:
            log: 已存在的运行事件文件，不是新起的记录器进程。
            warn: 诊断输出端，只收到失败类别，绝不收到远端报文内容。
            capacity: 最多记住多少条事件身份；重复出现会刷新其新近度。
            ignored_prefixes: 自身持久生产者的命名空间，跳过以免把自己的回声写回去。

        Raises:
            ValueError: 容量或生产者命名空间非法。
        """
        if type(capacity) is not int or capacity <= 0:
            raise ValueError('event identity capacity must be a positive integer')
        if (not isinstance(ignored_prefixes, tuple)
                or any(not isinstance(prefix, str) or not prefix for prefix in ignored_prefixes)):
            raise ValueError('ignored event namespaces must be nonempty strings')
        self._log = log
        self._warn = warn
        self._capacity = capacity
        self._ignored = ignored_prefixes
        self._seen: OrderedDict[str, None] = OrderedDict()
        self._lock = threading.Lock()
        self._closed = False

    def receive(self, message: object) -> bool:
        """记录一条已解码的 ROS 信封，且绝不让输入把 monitor 搞崩.

        只有写成功的事件才进 LRU：某次回调失败时，后来重复到达的同一事件仍有机会补上。
        这不是"恰好一次"持久化——重启、缓存淘汰与部分写盘都是可以观察到的边界。

        Args:
            message: 形如 NavEvent 的对象，含生成接口的六个字段。

        Returns:
            追加成功返回 True；重复、已封存或输入非法返回 False。
        """
        with self._lock:
            if self._closed:
                return False
            try:
                identity, event = _decode(message)
                if identity.startswith(self._ignored):
                    return False
                if identity in self._seen:
                    self._seen.move_to_end(identity)
                    return False
                self._log.record(event, precise_stamp=True)
                self._seen[identity] = None
                if len(self._seen) > self._capacity:
                    self._seen.popitem(last=False)
                return True
            except Exception as error:
                try:
                    self._warn(f'cannot record navigation event ({type(error).__name__})')
                except Exception:
                    pass  # A failed diagnostic sink is not navigation authority either.
                return False

    def close(self) -> None:
        """等在途记录写完后封存迟到回调，不删除任何数据.

        封存之后 receive 一律返回 False；已经写进文件的事件不受影响。
        """
        with self._lock:
            self._closed = True
            self._seen.clear()


def _nonempty(value, limit):
    """验证非空事件字符串及 UTF-8 字节上限，保留原文不 strip.

    长度限制按编码字节而非字符数量，纯空白文本不被接受。

    Args:
        value: 待写入事件的文本。
        limit: 调用方选定的最大 UTF-8 字节数。

    Returns:
        通过检查的原字符串。

    Raises:
        ValueError: 类型、空白内容或编码后长度不符合要求。
        UnicodeError: 输入不能编码为有效 UTF-8。
    """
    if not isinstance(value, str) or not value.strip() or len(value.encode('utf-8')) > limit:
        raise ValueError('invalid event string')
    return value


def _pairs(pairs):
    """从 JSON 键值对构建事件对象，拒绝重复字段覆盖已有证据.

    用作 object_pairs_hook，同一层重复键即报错，不采用最后一个值。

    Args:
        pairs: JSON 解码器按出现顺序提供的键值对。

    Returns:
        无重复键且保留输入键顺序的对象字典。

    Raises:
        ValueError: 同一事件对象含重复键。
    """
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate event JSON key')
        result[key] = value
    return result


def _finite_tree(value, depth=0):
    """递归检查 JSON 数据的最大嵌套深度及浮点有限性.

    Args:
        value: 已解码的数据子树。
        depth: 当前嵌套深度，从零开始递增。

    Raises:
        ValueError: 深度超过 MAX_DEPTH 或遇到 NaN/无穷浮点值。
    """
    if depth > MAX_DEPTH:
        raise ValueError('event JSON nesting limit exceeded')
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError('nonfinite event JSON number')
    if isinstance(value, dict):
        for child in value.values():
            _finite_tree(child, depth + 1)
    elif isinstance(value, list):
        for child in value:
            _finite_tree(child, depth + 1)


def _decode(message):
    """验证 NavEvent 信封和数据对象，附传输身份后构造本地记录事件.

    Args:
        message: 含事件标识、秒制时间戳、来源、等级和 JSON 数据的消息。

    Returns:
        原事件 ID 与 Event 二元组；仅转换值，不发布或落盘。

    Raises:
        ValueError: 标识、等级、时间、JSON 尺寸/结构/重复键或保留字段无效。
        UnicodeError: JSON 字符串无法编码为 UTF-8。
        RecursionError: 解码嵌套超出 Python 解析器限制。
    """
    identity = _nonempty(message.event_id, 256)
    source = _nonempty(message.source, 128)
    name = _nonempty(message.event_type, 128)
    stamp = message.stamp
    if type(stamp) not in (int, float) or not math.isfinite(stamp):
        raise ValueError('invalid event timestamp')
    if message.severity not in SEVERITIES:
        raise ValueError('invalid event severity')
    encoded = message.data_json
    if not isinstance(encoded, str) or len(encoded.encode('utf-8')) > MAX_DATA_BYTES:
        raise ValueError('invalid event JSON size or type')
    data = json.loads(encoded, object_pairs_hook=_pairs)
    if not isinstance(data, dict) or TRANSPORT_KEY in data:
        raise ValueError('invalid event object or reserved key')
    _finite_tree(data)
    data[TRANSPORT_KEY] = {'event_id': identity}
    return identity, Event(float(stamp), source, name, message.severity, data)
