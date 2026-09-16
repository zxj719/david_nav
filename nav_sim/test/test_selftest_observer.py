"""Generated ROS-message observer callbacks without initializing ROS or sending commands."""

from types import SimpleNamespace

from david_nav_msgs.msg import NavEvent
from nav_msgs.msg import Odometry
import pytest

from nav_sim.selftest_ros import Observer
from nav_sim.selftest_trace import EvidenceTrace


@pytest.mark.parametrize('data', ['[]', '{"x":1,"x":2}', '{"x":NaN}', 'bad'])
def test_bad_ros_payload_invalidates_observer(tmp_path, data):
    """Malformed DDS payloads do not disappear as if no evidence had arrived."""
    with EvidenceTrace(tmp_path) as trace:
        observer = SimpleNamespace(_trace=trace, error=None)
        Observer._event(observer, NavEvent(
            event_id='id', source='tasks', event_type='event',
            severity='info', stamp=1.0, data_json=data))
        assert observer.error is not None


def test_generated_ros_envelope_preserves_identity_and_stamp(tmp_path):
    """The observer stores exactly the envelope shape consumed by the real oracle."""
    with EvidenceTrace(tmp_path) as trace:
        observer = SimpleNamespace(_trace=trace, error=None)
        Observer._event(observer, NavEvent(
            event_id='a' * 32 + ':1', source='tasks',
            event_type='task_admitted', severity='info', stamp=7.2,
            data_json='{"task_id":"one"}'))
        assert observer.error is None
        assert trace.nav[0]['event_id'] == 'a' * 32 + ':1'
        assert trace.nav[0]['stamp'] == 7.2 and trace.nav[0]['data'] == {'task_id': 'one'}


def test_odometry_uses_configured_body_and_parent_frames():
    """Different robots need no observer code change; wrong frame samples fail the run."""
    observer = SimpleNamespace(_frames={'odom': 'enc_2', 'body': 'body_2'}, error=None,
                               _odom=None, _seconds=Observer._seconds)
    message = Odometry()
    message.header.frame_id, message.child_frame_id = 'enc_2', 'body_2'
    message.pose.pose.position.x = 2.0
    message.header.stamp.sec = 3
    Observer._odometry(observer, message)
    assert observer._odom == (2, 0, 3) and observer.error is None
    message.child_frame_id = 'wrong'
    Observer._odometry(observer, message)
    assert observer.error == 'unexpected_odometry_frame'
