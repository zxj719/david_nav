"""Exercise the actual monitor composition using fake ROS modules and an in-memory graph."""

import io
import json
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from nav_bringup.recorder import monitor
from nav_bringup.recorder.run_dir import RunDirectory


@pytest.fixture
def fake_ros(monkeypatch):
    """Replace all ROS entry points; tests cannot create a DDS participant."""
    state = NS(subscriptions=[], timers=[], destroyed=False, shutdown=False,
               fail_subscription=False, during_spin=lambda node: None, publishers=[])

    def subscribe(kind, topic, callback, qos, **kwargs):
        if state.fail_subscription and topic == '/nav/events':
            raise RuntimeError('subscriber construction failed')
        subscription = NS(kind=kind, topic=topic, callback=callback, qos=qos)
        state.subscriptions.append(subscription)
        return subscription

    def spin(node):
        state.during_spin(node)
        raise KeyboardInterrupt

    node = NS(create_subscription=subscribe,
              create_timer=lambda interval, callback: state.timers.append((interval, callback)),
              destroy_node=lambda: setattr(state, 'destroyed', True))
    values = {
        'nav2_msgs.msg': dict(BehaviorTreeLog=type('BehaviorTreeLog', (), {})),
        'david_nav_msgs.msg': dict(NavEvent=NS),
        'rclpy': dict(init=lambda: None, create_node=lambda name: node, spin=spin,
                      try_shutdown=lambda: setattr(state, 'shutdown', True)),
        'rclpy.executors': dict(ExternalShutdownException=type('Shutdown', (Exception,), {})),
        'rclpy.qos': dict(QoSProfile=NS, ReliabilityPolicy=NS(BEST_EFFORT=1)),
        'rclpy.time': dict(Time=lambda: None),
        'rosidl_runtime_py.utilities': dict(get_message=lambda name: object),
        'tf2_ros': dict(Buffer=lambda: None, TransformListener=lambda buffer, node: object(),
                        TransformException=type('TransformError', (Exception,), {})),
    }
    for name, attributes in values.items():
        parts = name.split('.')
        for size in range(1, len(parts) + 1):
            module_name = '.'.join(parts[:size])
            if module_name not in values or size == len(parts):
                monkeypatch.setitem(sys.modules, module_name, ModuleType(module_name))
        module = sys.modules[name]
        for key, value in attributes.items():
            setattr(module, key, value)
    return state


def _run(tmp_path):
    return RunDirectory(tmp_path, '20260916_100000_precise_60a719c')


def _event():
    return NS(event_id='instance:1', stamp=1234.123456789, source='tasks',
              event_type='task_failed', severity='error', data_json='{"detail":"目标失败"}')


def test_real_monitor_records_events_and_seals_late_callbacks(tmp_path, fake_ros):
    """The existing monitor node owns one added subscription, not another participant."""
    run = _run(tmp_path)

    def deliver(node):
        assert len(fake_ros.subscriptions) == 1
        subscription = fake_ros.subscriptions[0]
        assert subscription.topic == '/nav/events' and subscription.qos == 10
        assert subscription.callback(_event())
        assert not subscription.callback(_event())

    fake_ros.during_spin = deliver
    assert monitor.spin(run, monitor.MonitorOptions(base_frame='robot', bt_topic=None),
                        io.StringIO()) == 0
    row = json.loads(run.events.read_text())
    assert row['stamp'] == _event().stamp and row['data']['detail'] == '目标失败'
    assert fake_ros.destroyed and fake_ros.shutdown
    assert not fake_ros.subscriptions[0].callback(_event())


def test_event_subscription_failure_warns_but_preserves_monitor_cleanup(tmp_path, fake_ros):
    """A diagnostic subscription failure does not prevent existing monitor work or teardown."""
    fake_ros.fail_subscription = True
    output = io.StringIO()
    assert monitor.spin(_run(tmp_path), monitor.MonitorOptions(base_frame='robot', bt_topic=None),
                        output) == 0
    assert 'navigation event' in output.getvalue()
    assert fake_ros.destroyed and fake_ros.shutdown
    assert len(fake_ros.timers) == 1


def test_no_run_directory_has_no_event_recording_subscription(fake_ros):
    """A print-only standalone monitor does not create an unused event queue."""
    assert monitor.spin(None, monitor.MonitorOptions(base_frame='robot', bt_topic=None),
                        io.StringIO()) == 0
    assert not fake_ros.subscriptions
