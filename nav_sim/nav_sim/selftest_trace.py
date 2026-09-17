"""提供不依赖 ROS 的有界自测观察记录及逐子进程退出证据核验."""

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
    """排他写入 JSONL 并限制规范化证据量，超限抛异常而非丢弃后判成功.

    Attributes:
        nav: 按接收顺序保留的已通过结构检查的 NavEvent 列表。
        ws: 按接收顺序保留的已通过结构检查的 WS 对象列表，与 nav 不作总序合并。
    """

    def __init__(self, root: Path, max_bytes: int = 8 * 1024 * 1024,
                 max_records: int = 50000) -> None:
        """排他创建观察文件，不覆盖已有实验记录.

        Args:
            root: 已存在且由调用方持有的私有实验目录。
            max_bytes: 两通道记录规范化 JSON 的累计 UTF-8 字节上限，
                不包括 JSONL 外壳，也不等于 Python 对象实际内存上限。
            max_records: 两通道合计允许的观察条数，必须为正整数。

        Raises:
            ValueError: 任一上限不是非布尔正整数。
            OSError: 排他创建观察文件失败。
        """
        if any(type(value) is not int or value <= 0 for value in (max_bytes, max_records)):
            raise ValueError('trace limits must be positive integers')
        self.nav, self.ws = [], []
        self._size, self._count = 0, 0
        self._max_bytes, self._max_records = max_bytes, max_records
        self._output = (root / 'observations.jsonl').open('x', encoding='utf-8')

    def receive(self, channel: str, raw: str) -> Mapping[str, object]:
        """先校验、写入并 flush 一帧，再将同一对象加入对应通道列表.

        flush 不等于 fsync 或持久交付；写入失败可能留下部分行。
        只校验 JSON 结构及 nav 的 data 映射，不在此证明事件业务语义。

        Args:
            channel: nav 或 ws，两个通道保持各自接收顺序。
            raw: 不超过 64 KiB UTF-8 字节的 JSON 对象文本。

        Returns:
            列表中保留的同一可变对象，调用方不应事后修改，否则磁盘与内存证据会分歧。

        Raises:
            ValueError: 通道、输入或结构不合法、记录器已关闭，或超过数量/字节上限。
            UnicodeError: 输入无法编码为 UTF-8。
            OSError: 追加或刷新证据失败。
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
        """关闭观察文件，不清空内存记录；关闭失败向外传播，不承诺持久落盘."""
        self._output.close()

    def __enter__(self):
        """返回当前排他证据记录器，不重新打开文件."""
        return self

    def __exit__(self, *_args):
        """离开上下文时调用 close，不吞掉原异常；关闭失败可覆盖原异常."""
        self.close()


def reached(case: ExpectedCase, pose: Optional[Sequence[float]]) -> bool:
    """按目标容差检查独立观察的导航旋转中心位姿，不自行检查样本年龄.

    Args:
        case: 由上层校验的预期目标与容差。
        pose: 上层已检查新鲜度的 x/y/yaw，单位米/弧度；无观察时为 None。

    Returns:
        维数及数值检查通过，且平面距离与归一化航向差均不超过容差时为 True。

    Raises:
        OverflowError: 超大整数在有限性检查中无法转换；
            本函数不将这种异常统一转成 False。
    """
    if (pose is None or len(pose) != 3
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in pose)):
        return False
    return (math.hypot(pose[0] - case.goal[0], pose[1] - case.goal[1]) <= case.xy_ok
            and abs(math.remainder(pose[2] - case.goal[2], 2 * math.pi)) <= case.yaw_ok)


def audit_processes(scene: Scene, final: bool = False) -> bool:
    """核对私有图逐进程启动记录，或在最终阶段核对全部正常退出记录.

    Args:
        scene: 已验证的私有图及排他审计目录。
        final: True 时要求每个计划进程恰有一次匹配启动和正常退出，
            接受退出码 0、130 或 -SIGINT。

    Returns:
        启动记录是否齐全；非最终阶段缺失文件或尚缺启动记录时为 False，
        未结束的最后一行暂不读取。最终阶段缺失或不完整则抛异常。
        通过只证明审计记录符合判据，不是独立进程存活探测。

    Raises:
        ValueError: 记录格式、身份、重复、提前或异常退出不符合要求，
            最终审计缺失/不完整，或文件超过 1 MiB。
        UnicodeError: 完整记录行不是有效 UTF-8。
        OSError: 除文件尚不存在外的读取失败。
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
