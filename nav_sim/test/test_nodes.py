"""Exercise real node callbacks with memory publishers and no rclpy context."""

from types import SimpleNamespace

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest

from nav_sim.fake_chassis import FakeChassis
from nav_sim.fake_localization import FakeLocalization
from nav_sim.model import PlanarPose, quaternion
from nav_sim import _ros


def _odometry():
    message = Odometry()
    message.header.frame_id = 'encoder_2'
    message.child_frame_id = 'body_2'
    message.header.stamp.sec = 27
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -0.4
    orientation = message.pose.pose.orientation
    orientation.x, orientation.y, orientation.z, orientation.w = quaternion(0.7)
    return message


def _localization():
    transforms, messages, warnings = [], [], []
    fake = SimpleNamespace(
        _map='world_2', _odom='encoder_2', _body='body_2',
        _broadcaster=SimpleNamespace(sendTransform=transforms.extend),
        _publisher=SimpleNamespace(publish=messages.append),
        get_logger=lambda: SimpleNamespace(warning=warnings.append))
    return fake, transforms, messages, warnings


def test_localization_publishes_two_edges_and_optional_odom_copy():
    """URDF remains the sole owner of the body-to-navigation-centre edge."""
    fake, transforms, messages, warnings = _localization()
    message = _odometry()
    FakeLocalization._odometry(fake, message)
    assert not warnings
    assert [(item.header.frame_id, item.child_frame_id) for item in transforms] == [
        ('world_2', 'encoder_2'), ('encoder_2', 'body_2')]
    assert transforms[0].transform.rotation.w == 1.0
    assert transforms[0].transform.translation.x == 0.0
    assert transforms[1].header.stamp == message.header.stamp
    assert transforms[1].transform.translation.x == 1.25
    assert transforms[1].transform.rotation == message.pose.pose.orientation
    assert messages[0] == message
    message.pose.pose.orientation.w = 0.0
    assert messages[0].pose.pose.orientation.w != 0.0
    assert transforms[1].transform.rotation.w != 0.0


@pytest.mark.parametrize('bad', ['parent', 'child', 'position', 'quaternion'])
def test_localization_rejects_invalid_samples(bad):
    """Malformed samples cannot poison TF or the mirrored odometry topic."""
    fake, transforms, messages, warnings = _localization()
    message = _odometry()
    if bad == 'parent':
        message.header.frame_id = 'wrong'
    elif bad == 'child':
        message.child_frame_id = 'wrong'
    elif bad == 'position':
        message.pose.pose.position.x = float('nan')
    else:
        message.pose.pose.orientation.w = 2.0
    FakeLocalization._odometry(fake, message)
    assert warnings
    assert not transforms and not messages


def test_actual_context_is_checked_before_creating_endpoints():
    """An explicit production-domain context cannot bypass environment isolation."""
    destroyed = []
    node = SimpleNamespace(context=SimpleNamespace(get_domain_id=lambda: 0),
                           destroy_node=lambda: destroyed.append(True))
    with pytest.raises(RuntimeError, match='domain 99'):
        _ros.require_node_sandbox(node)
    assert destroyed == [True]
    node.context.get_domain_id = lambda: 99
    _ros.require_node_sandbox(node)
    assert destroyed == [True]


def test_chassis_emits_only_integrated_odom_and_ignores_lateral_speed():
    """Callbacks use the legacy integration rule with configured message frames."""
    messages, warnings = [], []
    from builtin_interfaces.msg import Time
    fake = SimpleNamespace(
        _pose=PlanarPose(), _linear=0.0, _angular=0.0, _dt=0.02,
        _odom_frame='encoder_2', _body_frame='body_2',
        _publisher=SimpleNamespace(publish=messages.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
        get_logger=lambda: SimpleNamespace(warning=warnings.append))
    command = Twist()
    command.linear.x, command.linear.y, command.angular.z = 0.2, 0.8, 0.3
    FakeChassis._command(fake, command)
    FakeChassis._tick(fake)
    message = messages[0]
    assert message.header.frame_id == 'encoder_2'
    assert message.child_frame_id == 'body_2'
    assert message.pose.pose.position.x == 0.004
    assert message.pose.pose.position.y == 0.0
    assert message.twist.twist.linear.x == 0.2
    assert message.twist.twist.linear.y == 0.0
    command.linear.x = float('nan')
    FakeChassis._command(fake, command)
    assert fake._linear == fake._angular == 0.0
    assert warnings
