"""Generated event interface and CDR contracts; never initialize ROS or a node."""

import json

import pytest


def test_generated_nav_event_fields():
    """Keep the existing run-event fields and a transport correlation identity."""
    from david_nav_msgs.msg import NavEvent

    assert NavEvent.get_fields_and_field_types() == {
        'event_id': 'string', 'stamp': 'double', 'source': 'string',
        'event_type': 'string', 'severity': 'string', 'data_json': 'string',
    }


@pytest.mark.parametrize('source,event_type,severity,data', [
    ('tasks', 'task_started', 'info', {'task_id': 't1', 'generation': 2}),
    ('tasks', 'task_canceled', 'info', {'task_id': 't1', 'generation': 2}),
    ('tasks', 'task_failed', 'error', {'detail': '目标点不存在'}),
    ('dock', 'measured', 'info', {'exy': 0.03, 'eyaw': -0.02, 'settled': True}),
    ('navrun', 'run_started', 'info', {'run_id': 'test-run', 'mode': 'precise'}),
    ('localization', 'tf_stale', 'warn', {'age_sec': 0.6}),
])
def test_event_cdr_round_trip_without_ros_initialization(
        monkeypatch, source, event_type, severity, data):
    """Native serialization preserves types, Unicode and producer-owned identity."""
    from david_nav_msgs.msg import NavEvent
    import rclpy
    from rclpy.serialization import deserialize_message, serialize_message

    def forbidden(*args, **kwargs):
        pytest.fail('interface tests must not initialize ROS')

    monkeypatch.setattr(rclpy, 'init', forbidden)
    message = NavEvent(
        event_id='fixture-session:12', stamp=1789544395.125, source=source,
        event_type=event_type, severity=severity,
        data_json=json.dumps(data, ensure_ascii=False, allow_nan=False))
    decoded = deserialize_message(serialize_message(message), NavEvent)
    assert decoded == message
    assert json.loads(decoded.data_json) == data
    assert decoded.stamp == 1789544395.125


def test_existing_move_to_action_remains_available():
    """Adding observations must not rename or change the existing motion interface."""
    from david_nav_msgs.action import MoveTo

    assert MoveTo.Goal.get_fields_and_field_types() == {
        'mode': 'uint8', 'distance': 'double', 'yaw': 'double',
        'target': 'geometry_msgs/Point', 'relative': 'boolean',
    }
    assert (MoveTo.Goal.MODE_DISTANCE, MoveTo.Goal.MODE_ROTATE, MoveTo.Goal.MODE_POSE) == (0, 1, 2)
    assert MoveTo.Result.get_fields_and_field_types() == {
        'success': 'boolean', 'final_error': 'double',
    }
    assert MoveTo.Feedback.get_fields_and_field_types() == {
        'error_remaining': 'double', 'current_speed': 'double',
    }
