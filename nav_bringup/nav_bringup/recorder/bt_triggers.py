"""把行为树状态变化转换为脱困与恢复观测事件.

按节点名和目标状态匹配规则，生成记录数据和终端行；不修改行为树、
C++ 节点或恢复策略，也不下发导航指令。脱困结束只是一条观测，
不能据此声明导航 Goal succeeded。

本模块为纯 Python；ROS 订阅和运行日志写入由外层记录器负责。
"""

from __future__ import annotations

from dataclasses import dataclass
import datetime
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from nav_bringup.recorder.events import SEVERITIES

STATUSES = ('IDLE', 'RUNNING', 'SUCCESS', 'FAILURE')
ROOT_RECOVERY_NODE = 'NavigateRecovery'

_NAVIGATION_END_STATUSES = ('SUCCESS', 'FAILURE', 'IDLE')
_LINE_PREFIXES = {
    'escape_triggered': '⚠ 脱困触发',
    'recovery_triggered': '⚠ 恢复触发',
    'escape_finished': '· 脱困结束',
}


@dataclass(frozen=True)
class StatusChange:
    """表示一条行为树状态变化，构造时不执行字段校验.

    Attributes:
        node_name: 行为树节点名，通常来自 XML name 或节点类型名。
        previous_status: 上一个状态，调用方预期使用 STATUSES 中的值。
        current_status: 当前状态，调用方预期使用 STATUSES 中的值。
        stamp: 行为树事件秒级时间，不与本地墙钟自动同步或校验有限性。
    """

    node_name: str
    previous_status: str
    current_status: str
    stamp: float


@dataclass(frozen=True)
class TriggerRule:
    """描述按节点名与目标状态匹配的观测规则.

    Attributes:
        node_name: 精确匹配的行为树节点名。
        statuses: 可触发规则的当前状态元组。
        event_type: 输出事件类型，不在此处执行协议枚举验证。
        severity: 事件等级，由 TriggerDetector 检查是否为已知等级。
        label: 面向操作员的说明文本。
    """

    node_name: str
    statuses: Tuple[str, ...]
    event_type: str
    severity: str
    label: str


@dataclass(frozen=True)
class Pose2D:
    """保存观测到的平面位姿，不校验参考系或数值有限性.

    Attributes:
        x: x 方向位置，单位米。
        y: y 方向位置，单位米。
        yaw: 平面朝向，单位弧度。
        frame_id: 位姿所属参考系名称。
    """

    x: float
    y: float
    yaw: float
    frame_id: str = 'map'


@dataclass(frozen=True)
class Trigger:
    """保存一次规则触发与当前计数，不代表动作执行结果.

    Attributes:
        rule: 本次匹配的规则。
        change: 触发规则的状态变化。
        attempt: 该规则自最近根节点结束重置后的已报告次数，从一开始。
    """

    rule: TriggerRule
    change: StatusChange
    attempt: int

    def event_data(
        self, pose: Optional[Pose2D] = None, task_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """生成本地运行事件的数据字段，不执行完整线协议校验.

        Args:
            pose: 观察时获得的可选位姿，不保证与行为树时间同步。
            task_id: 可选任务号；空字符串与 None 均不输出该字段。

        Returns:
            包含节点、标签、状态变化、计数及行为树时间的字典；
            时间保留六位、位姿保留三位小数，不拒绝非有限数值。
        """
        data: Dict[str, Any] = {
            'node': self.change.node_name,
            'label': self.rule.label,
            'from': self.change.previous_status,
            'to': self.change.current_status,
            'attempt': self.attempt,
            'bt_stamp': round(self.change.stamp, 6),
        }
        if pose is not None:
            data['pose'] = {
                'frame': pose.frame_id,
                'x': round(pose.x, 3),
                'y': round(pose.y, 3),
                'yaw': round(pose.yaw, 3),
            }
        if task_id:
            data['task_id'] = task_id
        return data


# Node names of navigate_{to_pose,through_poses}_w_david_embedded_recovery.xml
# plus the stock Nav2 behaviors; test_recorder_bt_triggers checks them against
# the XML files so a renamed node cannot silently stop being reported.
DEFAULT_RULES: Tuple[TriggerRule, ...] = (
    TriggerRule(
        'RobotEmbeddedInObstacle', ('SUCCESS',), 'escape_triggered', 'warn',
        'RobotEmbeddedInObstacle 确认嵌入'),
    TriggerRule(
        'EmbeddedRecoveryLevel1', ('RUNNING',), 'escape_triggered', 'warn',
        '进入 EmbeddedRecoveryLevel1'),
    TriggerRule(
        'EmbeddedRecoveryLevel2', ('RUNNING',), 'escape_triggered', 'warn',
        '进入 EmbeddedRecoveryLevel2'),
    TriggerRule(
        'EscapeObstacle', ('SUCCESS', 'FAILURE'), 'escape_finished', 'info',
        'EscapeObstacle 结束'),
    TriggerRule(
        'RecoveryFallback', ('RUNNING',), 'recovery_triggered', 'warn',
        'NavigateRecovery 进入通用恢复'),
    TriggerRule('IsStuck', ('SUCCESS',), 'recovery_triggered', 'warn', 'IsStuck 判定无进展'),
    TriggerRule('Spin', ('RUNNING',), 'recovery_triggered', 'warn', 'Spin 开始'),
    TriggerRule('BackUp', ('RUNNING',), 'recovery_triggered', 'warn', 'BackUp 开始'),
)


class TriggerDetector:
    """顺序匹配状态变化并对同一规则限流、计数.

    相邻已报告触发的树时间差小于 min_interval_sec 时抑制；
    恰好等于间隔可通过。时间倒退不会重置历史，负差同样可能被抑制。
    根节点变为 SUCCESS、FAILURE 或 IDLE 时仅清空次数，
    不清空上次触发时间或累计抑制数。这里观察树状态，不判定导航动作终态。
    """

    def __init__(
        self,
        rules: Sequence[TriggerRule] = DEFAULT_RULES,
        min_interval_sec: float = 0.5,
        root_node: str = ROOT_RECOVERY_NODE,
    ) -> None:
        """建立规则索引与空的触发历史.

        Args:
            rules: 按输入顺序索引的规则，不去重；相等规则共享历史键。
            min_interval_sec: 同一规则允许再次报告的最小树时间间隔。
            root_node: 变化到结束状态时重置所有规则次数的节点名。

        Raises:
            ValueError: 状态集合为空、含未知状态，等级未知或间隔为负。
                不额外验证间隔有限性、事件类型或规则名。
        """
        if min_interval_sec < 0:
            raise ValueError('min_interval_sec must not be negative')
        self._rules: Dict[str, List[TriggerRule]] = {}
        for rule in rules:
            unknown = [status for status in rule.statuses if status not in STATUSES]
            if unknown or not rule.statuses:
                raise ValueError(f'rule {rule.node_name!r} has invalid statuses {rule.statuses}')
            if rule.severity not in SEVERITIES:
                raise ValueError(f'rule {rule.node_name!r} has invalid severity {rule.severity!r}')
            self._rules.setdefault(rule.node_name, []).append(rule)
        self._min_interval_sec = min_interval_sec
        self._root_node = root_node
        self._attempts: Dict[TriggerRule, int] = {}
        self._last_fired: Dict[TriggerRule, float] = {}
        self._suppressed = 0

    @property
    def suppressed(self) -> int:
        """int: 因间隔限制被抑制的累计规则匹配次数，根节点结束不清零."""
        return self._suppressed

    def process(self, changes: Sequence[StatusChange]) -> List[Trigger]:
        """按传入顺序处理状态变化，不排序或补齐缺失事件.

        前后状态相同的条目完全跳过，包括根节点重置。规则处理后才检查
        根节点结束；返回的计数只包含通过限流的触发，不是全部恢复尝试数。

        Args:
            changes: 一组状态变化，通常来自一条 BehaviorTreeLog。

        Returns:
            按条目和规则遍历顺序产生的触发列表；内部次数与限流历史同时更新。
        """
        triggers: List[Trigger] = []
        for change in changes:
            if change.previous_status == change.current_status:
                continue
            for rule in self._rules.get(change.node_name, ()):
                if change.current_status not in rule.statuses:
                    continue
                last = self._last_fired.get(rule)
                if last is not None and change.stamp - last < self._min_interval_sec:
                    self._suppressed += 1
                    continue
                self._last_fired[rule] = change.stamp
                attempt = self._attempts.get(rule, 0) + 1
                self._attempts[rule] = attempt
                triggers.append(Trigger(rule, change, attempt))
            if (change.node_name == self._root_node
                    and change.current_status in _NAVIGATION_END_STATUSES):
                self._attempts.clear()
        return triggers


def format_trigger_line(
    trigger: Trigger,
    wall_time: float,
    pose: Optional[Pose2D] = None,
    task_id: Optional[str] = None,
) -> str:
    """生成操作员可读的单条观测文本，保持既有日志格式.

    Args:
        trigger: 已匹配的触发记录，不重新校验规则。
        wall_time: Unix 秒级时间，按本地时区格式化为时分秒。
        pose: 可选位姿，以两位小数显示。
        task_id: 可选任务号，只有非空值才显示。

    Returns:
        不附加结尾换行的字符串；多目标状态规则额外显示当前状态，
        不对标签、任务号等输入做换行清理。
    """
    rule = trigger.rule
    default_prefix = '· 行为树' if rule.severity == 'info' else '⚠ 行为树'
    prefix = _LINE_PREFIXES.get(rule.event_type, default_prefix)
    label = rule.label
    if len(rule.statuses) > 1:
        label = f'{label} → {trigger.change.current_status}'
    details = [f'第 {trigger.attempt} 次']
    if task_id:
        details.append(f'task={task_id}')
    if pose is not None:
        details.append(f'位姿 x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.2f}')
    clock = datetime.datetime.fromtimestamp(wall_time).strftime('%H:%M:%S')
    return f"[{clock}] {prefix}：{label}（{'，'.join(details)}）"


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """按四元数分量公式计算平面朝向，不归一化或校验输入.

    Args:
        x: 四元数 x 分量。
        y: 四元数 y 分量。
        z: 四元数 z 分量。
        w: 四元数 w 分量。

    Returns:
        对合法有限单位四元数返回 [-pi, pi] 内的 z 轴转角，单位弧度；
        非单位或非有限输入不能据此保证有效姿态。
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
