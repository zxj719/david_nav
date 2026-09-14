"""Detect escape and recovery triggers in Nav2 behavior tree status changes.

Nav2's ``bt_navigator`` publishes the status changes of the running tree on
``/behavior_tree_log``. The David escape nodes already print ``WARN_THROTTLE``
when they fire, but that output is buried in launch logs. This module turns
the status changes of recovery-related nodes into run events and an operator
line, without changing the tree or its C++ nodes (refactor plan §10.3; a new
escape policy is TODO-E1).

The module is pure Python and imports no ROS packages; the subscriber lives in
:mod:`nav_bringup.recorder.bt_log_watcher`.
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
    """One entry of ``nav2_msgs/BehaviorTreeLog.event_log``.

    Attributes:
        node_name: Tree node name: the XML ``name`` attribute, or the node type
            when the attribute is absent.
        previous_status: One of :data:`STATUSES`.
        current_status: One of :data:`STATUSES`.
        stamp: Behavior tree event time in seconds since the epoch.
    """

    node_name: str
    previous_status: str
    current_status: str
    stamp: float


@dataclass(frozen=True)
class TriggerRule:
    """A node status that is reported to operators.

    Attributes:
        node_name: Tree node name to match exactly.
        statuses: Current statuses that fire the rule.
        event_type: Event type to record, for example ``escape_triggered``.
        severity: Event severity, one of ``info``, ``warn`` or ``error``.
        label: Short description shown to operators.
    """

    node_name: str
    statuses: Tuple[str, ...]
    event_type: str
    severity: str
    label: str


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose.

    Attributes:
        x: Position along x in meters.
        y: Position along y in meters.
        yaw: Heading in radians.
        frame_id: Frame the pose is expressed in.
    """

    x: float
    y: float
    yaw: float
    frame_id: str = 'map'


@dataclass(frozen=True)
class Trigger:
    """A fired rule.

    Attributes:
        rule: The rule that fired.
        change: The status change that fired it.
        attempt: How often the rule fired in the current navigation, from 1.
    """

    rule: TriggerRule
    change: StatusChange
    attempt: int

    def event_data(
        self, pose: Optional[Pose2D] = None, task_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Build the ``data`` of the run event.

        Args:
            pose: Robot pose when the trigger was seen, if known.
            task_id: Active navigation task, if known.

        Returns:
            JSON-serializable details: node, label, status change, attempt,
            behavior tree stamp, and the pose and task when given.
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
    """Turn behavior tree status changes into triggers.

    A rule fires when its node changes into one of the rule's statuses.
    Firings of the same rule less than ``min_interval_sec`` of tree time after
    the previous one are dropped, so a flapping node cannot flood the
    terminal. Attempt counters restart when the root recovery node finishes,
    fails or is halted, that is when a navigation ends.
    """

    def __init__(
        self,
        rules: Sequence[TriggerRule] = DEFAULT_RULES,
        min_interval_sec: float = 0.5,
        root_node: str = ROOT_RECOVERY_NODE,
    ) -> None:
        """Create a detector.

        Args:
            rules: Rules to evaluate.
            min_interval_sec: Minimum tree time between firings of one rule.
            root_node: Node whose end marks the end of a navigation.

        Raises:
            ValueError: If a rule has an unknown status or severity, or
                ``min_interval_sec`` is negative.
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
        """int: Firings dropped because of the minimum interval."""
        return self._suppressed

    def process(self, changes: Sequence[StatusChange]) -> List[Trigger]:
        """Evaluate status changes in order.

        Args:
            changes: Status changes, typically one ``BehaviorTreeLog`` message.

        Returns:
            The triggers fired by these changes, in order.
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
    """Format the operator line for a trigger.

    Example: ``[06:12:03] ⚠ 脱困触发：RobotEmbeddedInObstacle 确认嵌入（第 1 次，位姿
    x=14.20 y=22.31 yaw=-0.95）``.

    Args:
        trigger: The fired rule.
        wall_time: Local wall-clock time of the line in seconds since the epoch.
        pose: Robot pose, if known.
        task_id: Active navigation task, if known.

    Returns:
        One line without a trailing newline.
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
    """Extract the heading from a rotation quaternion.

    Args:
        x: Quaternion x component.
        y: Quaternion y component.
        z: Quaternion z component.
        w: Quaternion w component.

    Returns:
        Rotation about z in radians, in ``[-pi, pi]``.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
