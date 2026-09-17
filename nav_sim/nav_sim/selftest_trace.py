"""Bounded selftest observations and child-exit evidence, independent of ROS."""

import json
import math
from pathlib import Path
import signal
from typing import Mapping, Optional, Sequence

from nav_sim.selftest_evidence import ExpectedCase
from nav_sim.selftest_scene import Scene


def _pairs(items):
    """按 JSON 键值对构造对象，拒绝重复键而不静默覆盖证据字段.

    供 json.loads 的 object_pairs_hook 使用，每层 JSON 对象分别检查。

    Args:
        items: 解码器按输入顺序提供的字符串键与已解码值序列。

    Returns:
        保持输入键顺序且无重复键的字典。

    Raises:
        ValueError: 同一对象中出现重复键。
    """
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError('duplicate JSON key')
        result[key] = value
    return result


def _tree(value, depth=0):
    """检查已解码 JSON 树的深度与浮点有限性，不修改输入.

    根节点深度为零；字典值和列表元素各增加一级，节点深度不得超过 32。
    只处理解码后的 JSON 结构，不作为任意 Python 对象的通用类型校验器。

    Args:
        value: 已由 JSON 解码器生成的当前节点。
        depth: 当前节点相对根的深度，外部调用通常使用默认零。

    Raises:
        ValueError: 节点深度超过 32，或任一浮点值为 NaN 或无穷大。
    """
    if depth > 32 or isinstance(value, float) and not math.isfinite(value):
        raise ValueError('invalid JSON depth or number')
    if isinstance(value, dict):
        for child in value.values():
            _tree(child, depth + 1)
    elif isinstance(value, list):
        for child in value:
            _tree(child, depth + 1)


def _decode(raw):
    """严格解码不超过 64 KiB 的 JSON 对象，拒绝重复键和结构/数值超限.

    Args:
        raw: 待记录的字符串帧，以 UTF-8 编码后的字节数计上限。

    Returns:
        已检查非有限浮点数和嵌套深度的对象字典，不执行或落盘。

    Raises:
        ValueError: 类型、大小、JSON 语法、重复键、深度或数值不合法；
            解析递归/溢出异常也转换为此类型。
        UnicodeError: 字符串不能编码为有效 UTF-8。
    """
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > 65536:
        raise ValueError('invalid frame type or size limit')
    try:
        value = json.loads(raw, object_pairs_hook=_pairs)
        if not isinstance(value, dict):
            raise ValueError('expected JSON object')
        _tree(value)
        return value
    except (RecursionError, OverflowError) as error:
        raise ValueError('JSON structural limit exceeded') from error


class EvidenceTrace:
    """Exclusive JSONL output and bounded in-memory observations; overflow is fatal.

    Attributes:
        nav: 按接收顺序保留的已校验 NavEvent 列表。
        ws: 按接收顺序保留的已校验 WS 消息列表，与 nav 不作总序合并。
    """

    def __init__(self, root: Path, max_bytes: int = 8 * 1024 * 1024,
                 max_records: int = 50000) -> None:
        """Open a new trace without overwriting any experiment.

        Args:
            root: Existing private experiment directory.
            max_bytes: Total normalized evidence bytes allowed in memory.
            max_records: Total observations allowed across both transports.

        Raises:
            ValueError: A limit is not a positive integer.
            OSError: Exclusive trace creation fails.
        """
        if any(type(value) is not int or value <= 0 for value in (max_bytes, max_records)):
            raise ValueError('trace limits must be positive integers')
        self.nav, self.ws = [], []
        self._size, self._count = 0, 0
        self._max_bytes, self._max_records = max_bytes, max_records
        self._output = (root / 'observations.jsonl').open('x', encoding='utf-8')

    def receive(self, channel: str, raw: str) -> Mapping[str, object]:
        """Validate and persist one frame before retaining it as evidence.

        Args:
            channel: nav or ws; never mixes transport ordering.
            raw: UTF-8 JSON object text, at most 64 KiB.

        Returns:
            Validated object retained in the selected channel.

        Raises:
            ValueError: Input is invalid, closed, or exceeds a bound.
            OSError: Appending/flushing evidence fails.
        """
        if channel not in ('nav', 'ws'):
            raise ValueError('unknown observation channel')
        value = _decode(raw)
        if channel == 'nav' and not isinstance(value.get('data'), dict):
            raise ValueError('NavEvent data must be a JSON object')
        size = len(json.dumps(value, allow_nan=False).encode('utf-8'))
        if self._size + size > self._max_bytes or self._count >= self._max_records:
            raise ValueError('observation limit exceeded')
        self._output.write(json.dumps({'channel': channel, 'record': value},
                                      allow_nan=False) + '\n')
        self._output.flush()
        getattr(self, channel).append(value)
        self._size += size
        self._count += 1
        return value

    def close(self) -> None:
        """Close the trace without discarding or overwriting evidence."""
        self._output.close()

    def __enter__(self):
        """Return the exclusive evidence writer."""
        return self

    def __exit__(self, *_args):
        """Close the evidence writer on every exit path."""
        self.close()


def reached(case: ExpectedCase, pose: Optional[Sequence[float]]) -> bool:
    """Check a separately observed navigation-centre pose against the planned goal.

    Args:
        case: Immutable expected goal and tolerances.
        pose: Fresh measured x/y/yaw, or None when observation is unavailable.

    Returns:
        Whether all measurements are finite and inside both tolerances.
    """
    if (pose is None or len(pose) != 3
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in pose)):
        return False
    return (math.hypot(pose[0] - case.goal[0], pose[1] - case.goal[1]) <= case.xy_ok
            and abs(math.remainder(pose[2] - case.goal[2], 2 * math.pi)) <= case.yaw_ok)


def audit_processes(scene: Scene, final: bool = False) -> bool:
    """Validate all private graph starts, or complete normal per-child shutdown.

    Args:
        scene: Verified private graph and its exclusive audit path.
        final: Require exactly one normal exit per planned process.

    Returns:
        Whether every process has started; missing startup records remain pending.

    Raises:
        ValueError: Malformed, unexpected, duplicate, early or abnormal child records.
        OSError: The audit cannot be read, other than not yet being created.
    """
    path = scene.root / 'process_events.jsonl'
    try:
        with path.open('rb') as stream:
            raw = stream.read(1024 * 1024 + 1)
    except FileNotFoundError:
        if final:
            raise ValueError('missing process audit')
        return False
    if len(raw) > 1024 * 1024:
        raise ValueError('process audit limit exceeded')
    if final and raw and not raw.endswith(b'\n'):
        raise ValueError('incomplete final audit record')
    rows = [_decode(line.decode('utf-8')) for line in raw.split(b'\n')[:-1]]
    expected = {process.name: process for process in scene.graph.processes}
    starts, exits = {}, {}
    for row in rows:
        matches = [name for name, process in expected.items() if (
            row.get('node') == '/' + scene.namespace + '/' + name if process.ros_node else
            row.get('node') is None and (
                row.get('name') == name or str(row.get('name')).startswith(name + '-')))]
        pid = row.get('pid')
        if len(matches) != 1 or type(pid) is not int or pid <= 1:
            raise ValueError('unexpected process identity')
        name = matches[0]
        if row.get('event') == 'start':
            if name in starts or pid in starts.values():
                raise ValueError('duplicate process start')
            starts[name] = pid
        elif row.get('event') == 'exit':
            code = row.get('returncode')
            if (not final or name in exits or starts.get(name) != pid
                    or type(code) is not int or code not in (0, 130, -signal.SIGINT)):
                raise ValueError('early, duplicate or abnormal process exit')
            exits[name] = pid
        else:
            raise ValueError('unknown process event')
    complete = set(starts) == set(expected)
    if final and (not complete or exits != starts):
        raise ValueError('incomplete child shutdown audit')
    return complete
