"""Exercise the real health-node composition using fake ROS, never a DDS participant."""

import importlib
import math
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


def _rotation(yaw=0):
    return NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def _stamp(value):
    return NS(sec=int(value), nanosec=round((value - int(value)) * 1e9))


def _transform(stamp, x=0, y=0, yaw=0):
    return NS(header=NS(stamp=_stamp(stamp)), transform=NS(
        translation=NS(x=x, y=y, z=0), rotation=_rotation(yaw)))


def _odom(stamp, x=0, y=0, yaw=0, child='wheel_body', frame='encoder'):
    return NS(header=NS(stamp=_stamp(stamp), frame_id=frame), child_frame_id=child,
              pose=NS(pose=NS(position=NS(x=x, y=y, z=0), orientation=_rotation(yaw))))


@pytest.fixture
def runtime(monkeypatch):
    """Supply parameters, clock, TF and messages without importing real ROS at all."""
    state = NS(now=10.0, wall=1000.0, mono=0.0, localized=False, destroyed=0,
               unregistered=0, publishers={}, subscriptions=[], timers=[], logs=[],
               params={'map_frame': 'world', 'odom_frame': 'encoder', 'nav_frame': 'axle',
                       'chassis_odom_topic': '/wheel', 'chassis_odom_frame': 'encoder'},
               transforms={}, lookups=[])

    class Node:
        def __init__(self, name):
            assert name == 'localization_monitor'

        def declare_parameter(self, name, value):
            state.params.setdefault(name, value)

        def get_parameter(self, name):
            return NS(value=state.params[name])

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=round(state.now * 1e9),
                                     to_msg=lambda: _stamp(state.now)))

        def get_logger(self):
            return NS(info=lambda message: state.logs.append(('info', message)),
                      warning=lambda message: state.logs.append(('warn', message)),
                      error=lambda message: state.logs.append(('error', message)))

        def create_publisher(self, kind, topic, qos):
            state.publishers[topic] = []
            return NS(publish=state.publishers[topic].append)

        def create_subscription(self, kind, topic, callback, qos):
            state.subscriptions.append(NS(topic=topic, callback=callback, qos=qos))
            return object()

        def create_timer(self, period, callback):
            state.timers.append((period, callback))
            return object()

        def destroy_node(self):
            state.destroyed += 1
            return True

    class Buffer:
        def can_transform(self, target, source, stamp):
            assert (target, source) == ('world', 'encoder')
            return state.localized

        def lookup_transform(self, target, source, stamp):
            state.lookups.append((target, source, stamp))
            if (target, source) not in state.transforms:
                raise LookupError('private transform detail')
            return state.transforms[target, source]

    class Listener:
        def __init__(self, buffer, node, spin_thread=False):
            assert not spin_thread

        def unregister(self):
            state.unregistered += 1

    class Status(NS):
        OK, WARN, ERROR = bytes([0]), bytes([1]), bytes([2])

        def __init__(self, **values):
            assert isinstance(values['level'], bytes) and len(values['level']) == 1
            super().__init__(**values)

    modules = {
        'rclpy': {}, 'rclpy.node': dict(Node=Node),
        'rclpy.executors': dict(ExternalShutdownException=type('Shutdown', (Exception,), {})),
        'rclpy.time': dict(Time=type('Time', (), {'from_msg': staticmethod(lambda msg: msg)})),
        'rclpy.qos': dict(QoSProfile=NS, ReliabilityPolicy=NS(BEST_EFFORT=1),
                          DurabilityPolicy=NS(VOLATILE=2)),
        'nav_msgs': {}, 'nav_msgs.msg': dict(Odometry=NS),
        'david_nav_msgs': {}, 'david_nav_msgs.msg': dict(NavEvent=NS),
        'diagnostic_msgs': {}, 'diagnostic_msgs.msg': dict(
            DiagnosticArray=lambda: NS(header=NS(), status=[]),
            DiagnosticStatus=Status, KeyValue=NS),
        'tf2_ros': dict(Buffer=Buffer, TransformListener=Listener, TransformException=LookupError),
    }
    for name, values in modules.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    name = 'nav_perception_iface.health_node'
    sys.modules.pop(name, None)
    module = importlib.import_module(name)
    monkeypatch.setattr(module.time, 'time', lambda: state.wall)
    monkeypatch.setattr(module.time, 'monotonic', lambda: state.mono)
    yield module, state
    sys.modules.pop(name, None)


def test_actual_node_owns_only_observation_interfaces_and_waits_for_manual_pose(runtime):
    """There is exactly one 5 Hz timer; no command/action/IMU interface is constructed."""
    module, state = runtime
    node = module.LocalizationMonitor()
    assert set(state.publishers) == {'/diagnostics', '/nav/events'}
    assert [sub.topic for sub in state.subscriptions] == ['/wheel']
    assert len(state.timers) == 1 and state.timers[0][0] == 0.2
    state.timers[0][1]()
    assert state.publishers['/nav/events'][-1].event_type == 'localization_waiting'
    assert state.publishers['/nav/events'][-1].stamp == 1000.0
    assert any('等待在 RViz 手动给初始位姿' in message for _, message in state.logs)
    state.timers[0][1]()
    assert len(state.publishers['/nav/events']) == 1
    state.mono = 5.0
    state.timers[0][1]()
    assert len(state.publishers['/nav/events']) == 2
    assert len(state.publishers['/diagnostics']) == 3
    node.destroy_node()
    state.timers[0][1]()
    assert len(state.publishers['/diagnostics']) == 3 and state.unregistered == 1


def test_wheel_pose_is_shifted_to_navigation_point_before_divergence(runtime):
    """Distinct robot reference frames must not be silently treated as identical."""
    module, state = runtime
    state.localized = True
    state.transforms['wheel_body', 'axle'] = _transform(0, 0.2)
    state.transforms['world', 'axle'] = _transform(10, 0.2)
    node = module.LocalizationMonitor()
    state.subscriptions[0].callback(_odom(10))
    state.timers[0][1]()
    assert node._wheel.x == pytest.approx(0.2)
    state.now = 10.2
    state.mono = 0.2
    state.transforms['world', 'axle'] = _transform(10.2, 28.2)
    state.subscriptions[0].callback(_odom(10.2))
    state.timers[0][1]()
    event = state.publishers['/nav/events'][-1]
    assert event.event_type == 'localization_divergence' and event.severity == 'error'
    assert state.publishers['/diagnostics'][-1].status[0].level == bytes([2])
    assert event.source == 'localization_health'
    assert ('wheel_body', 'axle') in [(target, source) for target, source, _ in state.lookups]
    node.destroy_node()


def test_invalid_or_untransformable_odometry_never_reuses_old_good_sample(runtime):
    """Bad frames are missing evidence, not a fresh healthy observation."""
    module, state = runtime
    state.localized = True
    state.transforms['world', 'axle'] = _transform(10)
    node = module.LocalizationMonitor()
    receive = state.subscriptions[0].callback
    receive(_odom(10, child='axle'))
    assert node._wheel is not None
    receive(_odom(10, frame='wrong'))
    assert node._wheel is None
    state.timers[0][1]()
    assert state.publishers['/nav/events'][-1].event_type == 'localization_odom_missing'
    assert 'private' not in str(state.logs)
    node.destroy_node()


def test_stop_policy_is_rejected_before_publishers_or_subscriptions(runtime):
    """TODO-H1 cannot acquire cancellation authority through a warning-only node."""
    module, state = runtime
    state.params['action'] = 'stop'
    with pytest.raises(ValueError, match='TODO-H1'):
        module.LocalizationMonitor()
    assert not state.publishers and not state.subscriptions and state.destroyed == 1


def test_wheel_parent_frame_is_independent_of_localization_odometry_frame(runtime):
    """David's wheel integration frame is not the localization TF odometry frame."""
    module, state = runtime
    state.params['chassis_odom_frame'] = 'wheel_integration'
    node = module.LocalizationMonitor()
    state.subscriptions[0].callback(_odom(10, frame='wheel_integration', child='axle'))
    assert node._wheel is not None
    node.destroy_node()


def test_event_publication_failure_does_not_disable_diagnostic_heartbeat(runtime):
    """One failed observation sink cannot become a control action or hide the other sink."""
    module, state = runtime
    node = module.LocalizationMonitor()

    def failed(message):
        raise RuntimeError('private transport detail')

    node._events.publish = failed
    state.timers[0][1]()
    assert len(state.publishers['/diagnostics']) == 1
    assert 'private' not in str(state.logs)
    node.destroy_node()


def test_partial_constructor_failure_cleans_listener_and_node(runtime, monkeypatch):
    """A failed timer creation cannot leave an observer with only half its resources."""
    module, state = runtime

    def failed(*args):
        raise RuntimeError('fixture timer failure')

    monkeypatch.setattr(module.Node, 'create_timer', failed)
    with pytest.raises(RuntimeError):
        module.LocalizationMonitor()
    assert state.destroyed == state.unregistered == 1


def test_actual_main_owns_ros_shutdown_but_not_provider_lifecycle(runtime, monkeypatch):
    """The installed entry point cleans its own node/context after interruption."""
    module, state = runtime
    calls = []
    monkeypatch.setattr(module.rclpy, 'init', lambda **kw: calls.append(kw), raising=False)
    monkeypatch.setattr(module.rclpy, 'try_shutdown',
                        lambda: calls.append('closed'), raising=False)

    def interrupted(node):
        state.timers[0][1]()
        raise KeyboardInterrupt

    monkeypatch.setattr(module.rclpy, 'spin', interrupted, raising=False)
    assert module.main(['--ros-args']) == 0
    assert calls == [{'args': ['--ros-args']}, 'closed']
    assert state.destroyed == state.unregistered == 1


def test_health_severity_transitions_use_distinct_ros_logging_call_sites(runtime, monkeypatch):
    """Humble pins severity to a call site; warn-to-info recovery must not kill the node."""
    module, state = runtime
    contexts = {}

    def method(severity):
        def log(message):
            frame = sys._getframe(1)
            caller = (frame.f_code.co_filename, frame.f_lineno, frame.f_lasti)
            previous = contexts.setdefault(caller, severity)
            if previous != severity:
                raise ValueError('Logger severity cannot be changed between calls.')
            state.logs.append((severity, message))
        return log

    logger = NS(info=method('info'), warning=method('warn'), error=method('error'))
    monkeypatch.setattr(module.Node, 'get_logger', lambda self: logger)
    node = module.LocalizationMonitor()
    try:
        for severity in ('warn', 'info', 'error', 'info', 'warn'):
            node._event('localization_transition', severity, {})
        assert [level for level, _ in state.logs] == ['warn', 'info', 'error', 'info', 'warn']
        assert len(contexts) == 3
        assert len(state.publishers['/nav/events']) == 5
    finally:
        node.destroy_node()
