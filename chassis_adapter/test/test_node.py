"""Exercise actual node methods with in-memory endpoints and no ROS context."""

from dataclasses import asdict
from pathlib import Path
import signal
from types import SimpleNamespace

from geometry_msgs.msg import Twist
import pytest
from rclpy.time import Time
import yaml

from chassis_adapter.core import AdapterConfig
from chassis_adapter import node as runtime


SOURCE = Path(__file__).resolve().parents[2]


@pytest.fixture
def endpoints(monkeypatch):
    """Replace Node infrastructure while executing the real adapter constructor."""
    outputs, subscriptions, timers, declarations = {}, [], [], {}
    clock = SimpleNamespace(seconds=100.0)

    def initialize(node, name, **kwargs):
        node.overrides = kwargs.get('test_parameters', {})

    def declare(node, name, value):
        declarations[name] = node.overrides.get(name, value)

    def publisher(node, message_type, topic, depth):
        outputs[topic] = []
        return SimpleNamespace(publish=outputs[topic].append)

    monkeypatch.setattr(runtime.Node, '__init__', initialize)
    monkeypatch.setattr(runtime.Node, 'declare_parameter', declare)
    monkeypatch.setattr(runtime.Node, 'get_parameter',
                        lambda self, key: SimpleNamespace(value=declarations[key]))
    monkeypatch.setattr(runtime.Node, 'get_clock',
                        lambda self: SimpleNamespace(now=lambda: Time(seconds=clock.seconds)))
    monkeypatch.setattr(runtime.Node, 'get_logger',
                        lambda self: SimpleNamespace(info=lambda _: None, warning=lambda _: None))
    monkeypatch.setattr(runtime.Node, 'get_fully_qualified_name', lambda self: '/chassis_adapter')
    monkeypatch.setattr(runtime.Node, 'create_publisher', publisher)
    monkeypatch.setattr(runtime.Node, 'create_subscription',
                        lambda self, *args: subscriptions.append(args))
    monkeypatch.setattr(runtime.Node, 'create_timer', lambda self, *args: timers.append(args))
    monkeypatch.setattr(runtime.rclpy, 'init', lambda **_: pytest.fail('ROS init forbidden'))
    return SimpleNamespace(outputs=outputs, subscriptions=subscriptions, timers=timers,
                           declarations=declarations, clock=clock)


def test_constructor_defaults_and_endpoint_binding(endpoints):
    """All ten old defaults are declared and configured topics feed the endpoints."""
    runtime.AdapterNode()
    assert endpoints.declarations == asdict(AdapterConfig())
    assert endpoints.subscriptions[0][1] == 'cmd_vel_safe'
    assert [timer[0] for timer in endpoints.timers] == [0.02, 1.0]


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_node_parameters_match_recorded_legacy_dump(endpoints, mode):
    """The exact navigation settings recorded before migration remain effective."""
    path = SOURCE / f'david_nav/test/golden/params/{mode}/david_move.yaml'
    expected = yaml.safe_load(path.read_text())['/david_move']['ros__parameters']
    expected.pop('use_sim_time')
    adapter = runtime.AdapterNode(test_parameters=expected)
    assert asdict(adapter.config) == expected
    assert endpoints.subscriptions[0][1] == expected['input_cmd_vel_topic']


def test_callback_timer_and_timeout_use_only_fake_publisher(endpoints):
    """Raw nonzero values stay in memory and stale output is zero exactly once."""
    adapter = runtime.AdapterNode()
    adapter.publish_current_command()
    assert not endpoints.outputs['/cmd_vel']
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = 1.0, 0.1, 0.026
    adapter.velocity_callback(message)
    assert not endpoints.outputs['/cmd_vel']
    adapter.publish_current_command()
    output = endpoints.outputs['/cmd_vel'][-1]
    assert output.linear.x == 0.20
    assert output.linear.y == output.linear.z == output.angular.x == output.angular.y == 0.0
    assert output.angular.z == 0.026
    endpoints.clock.seconds = 103.0
    adapter.publish_current_command()
    adapter.publish_current_command()
    assert len(endpoints.outputs['/cmd_vel']) == 2
    assert endpoints.outputs['/cmd_vel'][-1].linear.x == 0.0


def test_diagnostics_do_not_publish_or_refresh_commands(endpoints, monkeypatch):
    """Rate accounting is monotonic and separate from ROS-time freshness."""
    now = [50.0]
    monkeypatch.setattr(runtime.time, 'monotonic', lambda: now[0])
    adapter = runtime.AdapterNode()
    adapter.velocity_callback(Twist())
    adapter.velocity_callback(Twist())
    now[0] = 51.0
    adapter.publish_diagnostics()
    status = endpoints.outputs['/diagnostics'][-1].status[0]
    assert dict((item.key, item.value) for item in status.values)['input_hz'] == '2.0'
    assert endpoints.outputs['/cmd_vel'] == []
    endpoints.clock.seconds = 103.0
    now[0] = 52.0
    adapter.publish_diagnostics()
    assert endpoints.outputs['/diagnostics'][-1].status[0].level == runtime.DiagnosticStatus.WARN
    adapter.publish_current_command()
    assert len(endpoints.outputs['/cmd_vel']) == 1


def test_stop_burst_contains_only_zeros(endpoints, monkeypatch):
    """Shutdown attempts preserve the five-message/20ms legacy burst."""
    slept = []
    monkeypatch.setattr(runtime.time, 'sleep', slept.append)
    adapter = runtime.AdapterNode()
    adapter.publish_stop_burst()
    assert slept == [0.02] * 5
    assert len(endpoints.outputs['/cmd_vel']) == 5
    assert all(message == Twist() for message in endpoints.outputs['/cmd_vel'])


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_exit_stop_precedes_node_and_context_destruction(monkeypatch, signum):
    """Both signal paths keep publishers alive until their stop attempts finish."""
    history, handlers = [], {}
    alive = [False]

    def initialize(**kwargs):
        alive[0] = True

    def shutdown():
        history.append('shutdown')
        alive[0] = False

    def register(number, handler):
        previous = handlers.get(number, 'previous')
        handlers[number] = handler
        return previous

    monkeypatch.setattr(runtime.signal, 'signal', register)
    monkeypatch.setattr(runtime.rclpy, 'init', initialize)
    monkeypatch.setattr(runtime.rclpy, 'ok', lambda: alive[0])
    monkeypatch.setattr(runtime.rclpy, 'shutdown', shutdown)
    monkeypatch.setattr(runtime.rclpy, 'spin_once',
                        lambda *_, **__: handlers[signum](signum, None))
    monkeypatch.setattr(runtime, 'AdapterNode', lambda: SimpleNamespace(
        publish_stop_burst=lambda: history.append(('zero', alive[0])),
        destroy_node=lambda: history.append('destroy')))
    assert runtime.main([]) == 0
    assert history == [('zero', True), 'destroy', 'shutdown']
    assert all(value == 'previous' for value in handlers.values())
