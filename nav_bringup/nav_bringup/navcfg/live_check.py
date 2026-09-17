"""只读地评估运行时观测，并等待由现场人工完成的定位.

本模块自己不碰 ROS：输入是已渲染的期望值和一次观测快照，输出是发现列表。缺失的证据
一律不算通过；定位完成之前只给等待提示，不做其它判定，更不会代替现场发布初始位姿。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Callable, Mapping, Optional, Tuple


@dataclass(frozen=True)
class Finding:
    """一条运行期发现，同时可以直接作为运行事件.

    Attributes:
        severity: info、warn 或 error，本模块不再细分。
        code: 稳定的事件/检查标识，供上层按码处理。
        message: 人读的证据或操作提示。
    """

    severity: str
    code: str
    message: str


@dataclass(frozen=True)
class TopicRule:
    """某个已解析话题上期望的发布者数量.

    Attributes:
        topic: 绝对话题名。
        minimum: 要求的发布者数量下限。
        maximum: 发布者数量上限；None 表示不设上限。
        message_type: 期望的 ROS 接口名；留空表示只查数量、不查类型。
    """

    topic: str
    minimum: int = 1
    maximum: Optional[int] = None
    message_type: str = ''


@dataclass(frozen=True)
class LiveSpec:
    """来自已准备好的运行的输入，绝不写死机器人相关字面量.

    Attributes:
        map_frame: 全局定位坐标系。
        odom_frame: 定位里程计坐标系。
        nav_frame: 导航基准坐标系，与机身帧不是同一个。
        topics: 发布者数量约定。
        parameters: 已展平的渲染参数，按完整节点名分组。
        max_tf_age_sec: 超过该秒数的导航变换记为告警。
    """

    map_frame: str
    odom_frame: str
    nav_frame: str
    topics: Tuple[TopicRule, ...]
    parameters: Mapping[str, Mapping[str, Any]]
    max_tf_age_sec: float = 0.5

    def __post_init__(self) -> None:
        """在任何 ROS 采样之前先拒绝畸形或没有边界的配置.

        Raises:
            ValueError: 坐标系、话题规则、参数选择器或阈值不合法。
        """
        if not all(isinstance(frame, str) and frame.strip() for frame in (
                self.map_frame, self.odom_frame, self.nav_frame)):
            raise ValueError('live checks require explicit nonempty frames')
        if (isinstance(self.max_tf_age_sec, bool)
                or not isinstance(self.max_tf_age_sec, (int, float))
                or not math.isfinite(self.max_tf_age_sec) or self.max_tf_age_sec < 0):
            raise ValueError('max_tf_age_sec must be finite and nonnegative')
        for rule in self.topics:
            if (not rule.topic.startswith('/') or type(rule.minimum) is not int
                    or rule.minimum < 0 or (rule.maximum is not None and (
                        type(rule.maximum) is not int or rule.maximum < rule.minimum))):
                raise ValueError('invalid resolved topic publisher rule')
        if any(not name.startswith('/') or not isinstance(values, Mapping)
               for name, values in self.parameters.items()):
            raise ValueError('parameters require qualified node selectors and mappings')


@dataclass(frozen=True)
class LiveSnapshot:
    """一次有界观测得到的证据，查询失败与“确实没有”保持可区分.

    Attributes:
        localized: 全局到里程计的变换是否可查询。
        tf_age_sec: 导航变换的年龄；没有该变换时为 None。
        publishers: 观测到的各话题发布者数；键缺失表示没能查到。
        parameters: 成功取回的参数值；缺节点或缺键都算失败，不是空的成功。
        node_counts: 每个完整节点名当前发现到的数量。
        topic_types: 发现到的消息类型，包含同名话题类型冲突的情况。
    """

    localized: bool
    tf_age_sec: Optional[float]
    publishers: Mapping[str, int]
    parameters: Mapping[str, Mapping[str, Any]]
    node_counts: Mapping[str, int]
    topic_types: Mapping[str, Tuple[str, ...]] = field(default_factory=dict)


def _equal(expected: Any, actual: Any) -> bool:
    """比较参数类型和值，列表元素逐层核对以区分 bool 与 int.

    顶层类型必须相同；只对 list 特别递归，其它容器沿用自身等值运算。

    Args:
        expected: 期望的 ROS 参数值。
        actual: 实际读回的参数值。

    Returns:
        类型与当前规则下的值是否一致。
    """
    if type(expected) is not type(actual):
        return False
    if isinstance(expected, list):
        return len(expected) == len(actual) and all(
            _equal(left, right) for left, right in zip(expected, actual))
    return expected == actual


def assess(spec: LiveSpec, snapshot: LiveSnapshot) -> Tuple[Finding, ...]:
    """把在线图与渲染输入做对比，两边都不修改.

    定位完成之前只返回一条等待提示，不做其它判定。比较是单向的：只报缺失、不匹配、
    节点重名与话题类型不符，运行时多出来的参数不算问题。参数不匹配时只报名字，不打印
    实际值——值里可能整段带着机器人描述。

    Args:
        spec: 已校验的期望图与展平参数。
        snapshot: 观测结果；证据缺失一律不算匹配成功。

    Returns:
        全部发现；定位完成之前只有一条等待提示。
    """
    if not snapshot.localized:
        return (Finding('info', 'localization_waiting', '等待在 RViz 手动给初始位姿'),)
    findings = []
    age = snapshot.tf_age_sec
    if age is None:
        findings.append(Finding('error', 'navigation_tf_missing',
                                f'cannot transform {spec.map_frame} <- {spec.nav_frame}'))
    elif not math.isfinite(age) or age < 0:
        findings.append(Finding('warn', 'navigation_tf_clock', f'invalid TF age: {age} s'))
    elif age > spec.max_tf_age_sec:
        findings.append(Finding('warn', 'navigation_tf_stale',
                                f'TF age {age:.3f} s exceeds {spec.max_tf_age_sec:g} s'))
    for rule in spec.topics:
        count = snapshot.publishers.get(rule.topic)
        if count is None or count < rule.minimum or (
                rule.maximum is not None and count > rule.maximum):
            findings.append(Finding('error', 'topic_publishers',
                                    f'{rule.topic}: observed {count}; expected '
                                    f'{rule.minimum}..{rule.maximum} publishers'))
        if rule.message_type and set(snapshot.topic_types.get(rule.topic, ())) != {
                rule.message_type}:
            findings.append(Finding('error', 'topic_type',
                                    f'{rule.topic}: expected only {rule.message_type}'))
    for node, expected in spec.parameters.items():
        count = snapshot.node_counts.get(node, 0)
        if count != 1:
            findings.append(Finding('error', 'node_identity',
                                    f'{node}: expected one node, observed {count}'))
            continue
        actual = snapshot.parameters.get(node)
        if actual is None:
            findings.append(Finding('error', 'parameters_unavailable',
                                    f'{node}: parameter query failed'))
            continue
        for name, value in expected.items():
            if name not in actual:
                findings.append(Finding('error', 'parameter_missing',
                                        f'{node}/{name}: rendered but not declared'))
            elif not _equal(value, actual[name]):
                # Avoid dumping complete values, which may include robot descriptions.
                findings.append(Finding('error', 'parameter_mismatch',
                                        f'{node}/{name}: runtime type or value differs'))
    return tuple(findings)


def wait_localization(
    probe: Callable[[], bool], spin: Callable[[float], None], clock: Callable[[], float],
    stopped: Callable[[], bool], emit: Callable[[Finding], None], timeout_sec: float = 0.0,
) -> bool:
    """等待定位就绪：既不代为初始化定位，也不强加默认启动期限.

    定位由现场人工完成（在 RViz 给初始位姿），这里只反复做只读探测，并每 5 秒发一次
    等待提示。timeout_sec 为 0 表示一直等；正数只用于独立诊断，到期返回的是“还在等”，
    不是“定位失败”。探测已成功但调用方同时在收尾时返回 False，不报就绪。

    Args:
        probe: 只读判断全局到里程计的变换是否可用。
        spin: 处理 ROS 数据，或至多等待给定秒数。
        clock: 单调时钟，用于提示节奏与可选的诊断上限。
        stopped: 调用方是否正在收尾。
        emit: 打印或记录等待与就绪事件。
        timeout_sec: 0 为无限等待；正数为独立诊断的时间上限。

    Returns:
        定位可用返回 True；被中断或有界等待到期返回 False。

    Raises:
        ValueError: 期限为负数或非有限值。
    """
    if isinstance(timeout_sec, bool) or not math.isfinite(timeout_sec) or timeout_sec < 0:
        raise ValueError('localization wait timeout must be finite and nonnegative')
    started = clock()
    next_hint = started
    while not stopped():
        if probe():
            if stopped():
                return False
            emit(Finding('info', 'localization_ready', 'localization ready'))
            return True
        now = clock()
        if timeout_sec and now - started >= timeout_sec:
            return False
        if now >= next_hint:
            emit(Finding('info', 'localization_waiting', '等待在 RViz 手动给初始位姿'))
            next_hint = now + 5.0
        duration = min(0.2, next_hint - now)
        if timeout_sec:
            duration = min(duration, timeout_sec - (now - started))
        spin(duration)
    return False
