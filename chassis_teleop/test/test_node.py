"""Real keyboard methods on fake node infrastructure, with all commands kept in memory."""

from types import SimpleNamespace

from geometry_msgs.msg import Twist
import pytest

from chassis_teleop import node as runtime


@pytest.fixture
def endpoints(monkeypatch):
    """Replace Node services before constructing the actual keyboard class."""
    values, output, warnings = {}, [], []

    def initialize(node, name, **kwargs):
        node.overrides = kwargs.get('test_parameters', {})

    def declare(node, name, default):
        values[name] = node.overrides.get(name, default)

    def publisher(node, message_type, topic, depth):
        output.append(('publisher', topic))
        return SimpleNamespace(publish=output.append)

    monkeypatch.setattr(runtime.Node, '__init__', initialize)
    monkeypatch.setattr(runtime.Node, 'declare_parameter', declare)
    monkeypatch.setattr(runtime.Node, 'get_parameter',
                        lambda self, name: SimpleNamespace(value=values[name]))
    monkeypatch.setattr(runtime.Node, 'resolve_topic_name',
                        lambda self, name: '/' + name.lstrip('/'))
    monkeypatch.setattr(runtime.Node, 'get_logger',
                        lambda self: SimpleNamespace(warning=warnings.append))
    monkeypatch.setattr(runtime.Node, 'create_publisher', publisher)
    monkeypatch.setattr(runtime.rclpy, 'init', lambda **_: pytest.fail('ROS init forbidden'))
    return SimpleNamespace(output=output, warnings=warnings)


def test_construction_has_no_publisher_and_guard_includes_own_output(endpoints):
    """Declaring parameters does not authorize even a zero command publication."""
    node = runtime.KeyboardNode()
    assert endpoints.output == []
    assert node.guard_topics == ('/cmd_vel', '/cmd_vel_nav', '/cmd_vel_safe')
    node.publish_zero()
    node.publish_stop_burst()
    assert endpoints.output == []
    with pytest.raises(RuntimeError, match='not armed'):
        node.publish_current_command()


def test_motion_stop_timeout_and_quit_after_arming(endpoints, monkeypatch):
    """Nonzero calculations go only to a list; stop is immediate and quit publishes nothing."""
    now = [1.0]
    monkeypatch.setattr(runtime.time, 'monotonic', lambda: now[0])
    node = runtime.KeyboardNode()
    node.arm()
    assert node.handle_key('w') == 'motion'
    node.publish_current_command()
    assert endpoints.output[-1].linear.x == 0.2
    now[0] = 1.5
    node.publish_current_command()
    assert endpoints.output[-1] == Twist()
    node.handle_key('a')
    node.handle_key('k')
    assert endpoints.output[-1] == Twist()
    count = len(endpoints.output)
    assert node.handle_key('q') == 'quit'
    assert len(endpoints.output) == count
    with pytest.raises(RuntimeError, match='already armed'):
        node.arm()


def test_stop_burst_and_failed_transport_are_observable(endpoints, monkeypatch):
    """Exit attempts five zeros and reports transport failure without asserting hardware stop."""
    slept = []
    monkeypatch.setattr(runtime.time, 'sleep', slept.append)
    node = runtime.KeyboardNode()
    node.arm()
    node.publish_stop_burst()
    assert slept == [0.02] * 5
    assert endpoints.output[1:] == [Twist()] * 5

    def fail(_):
        raise RuntimeError('transport unavailable')

    node.cmd_publisher = SimpleNamespace(publish=fail)
    node.publish_zero()
    assert 'transport unavailable' in endpoints.warnings[-1]


@pytest.mark.parametrize('parameters', [
    {'guard_topics': []}, {'guard_topics': 'not a list'}, {'guard_topics': ['']},
    {'discovery_sec': 0.0}, {'discovery_sec': float('nan')},
])
def test_bad_guard_configuration_is_rejected_before_publisher(endpoints, parameters):
    """There is no empty-topic or zero-duration bypass of startup exclusion."""
    with pytest.raises(ValueError):
        runtime.KeyboardNode(test_parameters=parameters)
    assert endpoints.output == []
