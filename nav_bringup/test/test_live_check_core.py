"""Check live facts and manual-localization waiting without creating any ROS objects."""

from dataclasses import replace

import pytest

from nav_bringup.navcfg.live_check import (
    assess, LiveSnapshot, LiveSpec, TopicRule, wait_localization)


def _spec():
    return LiveSpec('world', 'wheel', 'axle', (
        TopicRule('/drive', 1, 1), TopicRule('/nav', 1, None)),
        {'/adapter': {'max_velocity': 0.2, 'active': True}})


def _snapshot():
    return LiveSnapshot(True, 0.1, {'/drive': 1, '/nav': 2},
                        {'/adapter': {'max_velocity': 0.2, 'active': True}},
                        {'/adapter': 1})


def test_matching_snapshot_has_no_findings():
    """Controller and behavior publishers on the Nav2 output are not false conflicts."""
    assert assess(_spec(), _snapshot()) == ()


def test_not_localized_is_waiting_not_configuration_failure():
    """Before initialization even an otherwise empty graph is only a waiting observation."""
    actual = LiveSnapshot(False, None, {}, {}, {})
    assert [item.code for item in assess(_spec(), actual)] == ['localization_waiting']
    assert assess(_spec(), actual)[0].severity == 'info'


@pytest.mark.parametrize('count', [0, 2])
def test_missing_or_conflicting_chassis_publisher_is_error(count):
    """The downstream drive topic must have exactly one publisher."""
    snapshot = replace(_snapshot(), publishers={'/drive': count, '/nav': 2})
    assert any(item.code == 'topic_publishers' and item.severity == 'error'
               for item in assess(_spec(), snapshot))


@pytest.mark.parametrize('age,code,severity', [
    (None, 'navigation_tf_missing', 'error'), (0.51, 'navigation_tf_stale', 'warn'),
    (-0.1, 'navigation_tf_clock', 'warn'), (float('nan'), 'navigation_tf_clock', 'warn')])
def test_tf_age_is_not_silently_treated_as_fresh(age, code, severity):
    """Missing transforms, stale stamps and clock anomalies have distinct findings."""
    findings = assess(_spec(), replace(_snapshot(), tf_age_sec=age))
    assert [(item.code, item.severity) for item in findings] == [(code, severity)]


@pytest.mark.parametrize('values', [
    {}, {'max_velocity': 0.3, 'active': True}, {'max_velocity': 0.2, 'active': 1}])
def test_parameter_missing_mismatch_and_bool_numeric_confusion_are_errors(values):
    """Python's True == 1 must not hide a ROS parameter type mismatch."""
    actual = replace(_snapshot(), parameters={'/adapter': values})
    assert any(item.code.startswith('parameter_') and item.severity == 'error'
               for item in assess(_spec(), actual))


@pytest.mark.parametrize('count', [0, 2])
def test_missing_or_duplicate_node_prevents_parameter_success(count):
    """A responding parameter service cannot prove which duplicate node answered."""
    actual = replace(_snapshot(), node_counts={'/adapter': count})
    assert any(item.code == 'node_identity' for item in assess(_spec(), actual))


def test_unavailable_parameter_service_is_error():
    """Failed retrieval is not an empty successful comparison."""
    actual = replace(_snapshot(), parameters={})
    assert any(item.code == 'parameters_unavailable' for item in assess(_spec(), actual))


@pytest.mark.parametrize('types', [
    (), ('std_msgs/msg/String',), ('geometry_msgs/msg/Twist', 'std_msgs/msg/String')])
def test_missing_wrong_or_conflicting_message_type_is_error(types):
    """Publisher count alone cannot establish that a command endpoint is compatible."""
    spec = replace(_spec(), topics=(
        TopicRule('/drive', 1, 1, 'geometry_msgs/msg/Twist'),))
    actual = replace(_snapshot(), topic_types={'/drive': types})
    assert any(item.code == 'topic_type' for item in assess(spec, actual))


def test_wait_prints_every_five_seconds_then_ready_once():
    """Manual localization has no default deadline and uses monotonic waiting cadence."""
    now, events = [0.0], []

    def spin(period):
        now[0] += period

    assert wait_localization(lambda: now[0] >= 12.0, spin, lambda: now[0],
                             lambda: False, events.append)
    assert [item.code for item in events] == [
        'localization_waiting', 'localization_waiting', 'localization_waiting',
        'localization_ready']
    assert all(item.severity == 'info' for item in events)


def test_wait_cancellation_does_not_report_ready_or_start_failure():
    """The caller retains shutdown ownership while initialization is pending."""
    events = []
    assert not wait_localization(lambda: False, lambda _: None, lambda: 0.0,
                                 lambda: True, events.append)
    assert not events


def test_explicit_wait_deadline_returns_pending_not_error():
    """A bounded diagnostic can finish waiting without claiming localization failed."""
    now, events = [0.0], []

    def spin(period):
        now[0] += period

    assert not wait_localization(lambda: False, spin, lambda: now[0], lambda: False,
                                 events.append, timeout_sec=0.3)
    assert now[0] == pytest.approx(0.3)
    assert [item.severity for item in events] == ['info']
