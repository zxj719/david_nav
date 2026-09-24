"""Tests for behavior tree escape and recovery trigger detection."""

import datetime
import math
from pathlib import Path
import re

import pytest

from nav_bringup.recorder import bt_triggers
from nav_bringup.recorder.bt_triggers import StatusChange

_DAVID_TREES = (
    'navigate_to_pose_w_david_embedded_recovery.xml',
    'navigate_through_poses_w_david_embedded_recovery.xml',
)


def _rule(node_name):
    return next(rule for rule in bt_triggers.DEFAULT_RULES if rule.node_name == node_name)


def _embedded_escape(start):
    """Status changes of one embedded-obstacle escape as bt_navigator logs them."""
    return [
        StatusChange('NavigateRecovery', 'IDLE', 'RUNNING', start),
        StatusChange('RobotEmbeddedInObstacle', 'IDLE', 'FAILURE', start + 0.01),
        StatusChange('RobotEmbeddedInObstacle', 'FAILURE', 'IDLE', start + 0.02),
        StatusChange('RobotEmbeddedInObstacle', 'IDLE', 'SUCCESS', start + 0.30),
        StatusChange('EmbeddedRecoveryLevel1', 'IDLE', 'RUNNING', start + 0.31),
        StatusChange('EscapeObstacle', 'IDLE', 'RUNNING', start + 0.31),
        StatusChange('EscapeObstacle', 'RUNNING', 'SUCCESS', start + 1.50),
        StatusChange('EmbeddedRecoveryLevel1', 'RUNNING', 'SUCCESS', start + 1.60),
    ]


def test_embedded_escape_fires_escape_events():
    """Confirmed embedding, the recovery level and the escape result are reported."""
    triggers = bt_triggers.TriggerDetector().process(_embedded_escape(100.0))

    assert [(t.rule.event_type, t.change.node_name, t.attempt) for t in triggers] == [
        ('escape_triggered', 'RobotEmbeddedInObstacle', 1),
        ('escape_triggered', 'EmbeddedRecoveryLevel1', 1),
        ('escape_finished', 'EscapeObstacle', 1),
    ]


def test_attempts_restart_when_navigation_ends():
    """Attempts count up within a navigation and restart after the root node ends."""
    detector = bt_triggers.TriggerDetector(min_interval_sec=0.0)

    first = detector.process([StatusChange('IsStuck', 'IDLE', 'SUCCESS', 1.0)])
    second = detector.process([
        StatusChange('IsStuck', 'SUCCESS', 'IDLE', 1.1),
        StatusChange('IsStuck', 'IDLE', 'SUCCESS', 5.0),
    ])
    detector.process([StatusChange('NavigateRecovery', 'RUNNING', 'FAILURE', 6.0)])
    third = detector.process([StatusChange('IsStuck', 'IDLE', 'SUCCESS', 9.0)])

    triggers = first + second + third
    assert [t.attempt for t in triggers] == [1, 2, 1]
    assert {t.rule.event_type for t in triggers} == {'recovery_triggered'}


def test_min_interval_suppresses_flapping():
    """Repeated firings closer than the minimum interval are dropped and counted."""
    detector = bt_triggers.TriggerDetector(min_interval_sec=0.5)

    triggers = detector.process([
        StatusChange('Spin', 'IDLE', 'RUNNING', 10.0),
        StatusChange('Spin', 'RUNNING', 'IDLE', 10.1),
        StatusChange('Spin', 'IDLE', 'RUNNING', 10.2),
        StatusChange('Spin', 'RUNNING', 'IDLE', 10.3),
        StatusChange('Spin', 'IDLE', 'RUNNING', 10.8),
    ])

    assert [t.change.stamp for t in triggers] == [10.0, 10.8]
    assert detector.suppressed == 1


def test_unrelated_and_unchanged_statuses_are_ignored():
    """Other nodes, non-matching statuses and non-changes do not fire."""
    detector = bt_triggers.TriggerDetector()

    assert detector.process([
        StatusChange('FollowPath', 'IDLE', 'RUNNING', 1.0),
        StatusChange('Spin', 'RUNNING', 'RUNNING', 2.0),
        StatusChange('BackUp', 'RUNNING', 'SUCCESS', 3.0),
        StatusChange('RobotEmbeddedInObstacle', 'IDLE', 'FAILURE', 4.0),
    ]) == []


def test_invalid_configuration_is_rejected():
    """Unknown statuses, severities and negative intervals raise ValueError."""
    with pytest.raises(ValueError):
        bt_triggers.TriggerDetector(
            [bt_triggers.TriggerRule('X', ('DONE',), 'escape_triggered', 'warn', 'x')])
    with pytest.raises(ValueError):
        bt_triggers.TriggerDetector(
            [bt_triggers.TriggerRule('X', ('SUCCESS',), 'escape_triggered', 'loud', 'x')])
    with pytest.raises(ValueError):
        bt_triggers.TriggerDetector(min_interval_sec=-1.0)


def test_event_data_and_operator_line():
    """Event data carries the pose and task; the line matches plan §10.3."""
    trigger = bt_triggers.Trigger(
        _rule('RobotEmbeddedInObstacle'),
        StatusChange('RobotEmbeddedInObstacle', 'IDLE', 'SUCCESS', 100.25),
        2,
    )
    pose = bt_triggers.Pose2D(14.2, 22.314, -0.9512)
    wall_time = datetime.datetime(2026, 9, 14, 6, 12, 3).timestamp()

    assert trigger.event_data(pose, 'req_42') == {
        'node': 'RobotEmbeddedInObstacle',
        'label': 'RobotEmbeddedInObstacle 确认嵌入',
        'from': 'IDLE',
        'to': 'SUCCESS',
        'attempt': 2,
        'bt_stamp': 100.25,
        'pose': {'frame': 'map', 'x': 14.2, 'y': 22.314, 'yaw': -0.951},
        'task_id': 'req_42',
    }
    assert bt_triggers.format_trigger_line(trigger, wall_time, pose, 'req_42') == (
        '[06:12:03] ⚠ 脱困触发：RobotEmbeddedInObstacle 确认嵌入'
        '（第 2 次，task=req_42，位姿 x=14.20 y=22.31 yaw=-0.95）')
    assert bt_triggers.format_trigger_line(trigger, wall_time) == (
        '[06:12:03] ⚠ 脱困触发：RobotEmbeddedInObstacle 确认嵌入（第 2 次）')


def test_multi_status_rule_shows_result():
    """Rules firing on several statuses show which one happened."""
    trigger = bt_triggers.Trigger(
        _rule('EscapeObstacle'), StatusChange('EscapeObstacle', 'RUNNING', 'FAILURE', 1.0), 1)
    wall_time = datetime.datetime(2026, 9, 14, 6, 12, 3).timestamp()

    assert bt_triggers.format_trigger_line(trigger, wall_time) == (
        '[06:12:03] · 脱困结束：EscapeObstacle 结束 → FAILURE（第 1 次）')


def test_yaw_from_quaternion():
    """Heading is recovered from z rotations, including the half turn."""
    assert bt_triggers.yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0
    assert bt_triggers.yaw_from_quaternion(0.0, 0.0, math.sin(0.5), math.cos(0.5)) == (
        pytest.approx(1.0))
    assert abs(bt_triggers.yaw_from_quaternion(0.0, 0.0, 1.0, 0.0)) == pytest.approx(math.pi)


def test_default_rules_name_nodes_of_david_trees():
    """Every default rule names a node of the David recovery trees."""
    trees_dir = Path(__file__).resolve().parents[2] / 'nav_plugins' / 'behavior_trees'
    names = set()
    for tree in _DAVID_TREES:
        text = (trees_dir / (tree + '.tmpl')).read_text(encoding='utf-8')
        names.update(re.findall(r'\bname="([^"]+)"', text))
        names.update(re.findall(r'<([A-Za-z][A-Za-z0-9]*)\b', text))

    missing = [rule.node_name for rule in bt_triggers.DEFAULT_RULES if rule.node_name not in names]
    assert not missing
    assert bt_triggers.ROOT_RECOVERY_NODE in names
