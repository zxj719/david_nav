"""Exercise the actual monitor composition using fake ROS modules and an in-memory graph."""

import io
import json
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from nav_bringup.recorder import monitor
from nav_bringup.recorder.events import EventLog
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

    def publish(kind, topic, qos):
        value = NS(topic=topic, qos=qos, messages=[])
        value.publish = value.messages.append
        state.publishers.append(value)
        return value

    transform_error = type('TransformError', (Exception,), {})

    def missing_transform(*args):
        raise transform_error()

    node = NS(create_subscription=subscribe,
              create_publisher=publish,
              count_publishers=lambda topic: 0,
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
        'tf2_ros': dict(Buffer=lambda: NS(lookup_transform=missing_transform),
                        TransformListener=lambda buffer, node: object(),
                        TransformException=transform_error),
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


def test_monitor_creates_no_publisher_and_keeps_navrun_history_local(tmp_path, fake_ros):
    """The monitor only subscribes: navrun lines stay local, in order, never written twice."""
    run = _run(tmp_path)
    run.path.mkdir()
    log = EventLog(run.events, clock=lambda: 12)
    log.append('navrun', 'run_started')

    def deliver(node):
        assert not fake_ros.publishers and len(fake_ros.timers) == 1
        assert fake_ros.subscriptions[0].callback(_event())
        log.append('navrun', 'startup_ready')
        fake_ros.timers[0][1]()

    fake_ros.during_spin = deliver
    assert monitor.spin(run, monitor.MonitorOptions(base_frame='robot', bt_topic=None),
                        io.StringIO()) == 0
    rows = [json.loads(line) for line in run.events.read_text().splitlines()]
    assert [(row['source'], row['type']) for row in rows] == [
        ('navrun', 'run_started'), ('tasks', 'task_failed'), ('navrun', 'startup_ready')]
    assert not fake_ros.publishers and fake_ros.destroyed and fake_ros.shutdown


def test_dump_start_failure_does_not_retry_on_next_sample(tmp_path, fake_ros, monkeypatch):
    """就绪已见标记在启动之前设置，Popen 失败只报告一次而不重试."""
    requested = []

    def fail_start(command, **kwargs):
        requested.append(command)
        raise OSError('fixture spawn failure')

    def sample_twice(node):
        node.count_publishers = lambda topic: 1
        fake_ros.timers[0][1]()
        fake_ros.timers[0][1]()

    monkeypatch.setattr(monitor.subprocess, 'Popen', fail_start)
    fake_ros.during_spin = sample_twice
    run = _run(tmp_path)
    options = monitor.MonitorOptions(
        base_frame='robot', bt_topic=None, ready_action='/navigate_to_pose',
        dump_params_on_ready=True)
    assert monitor.spin(run, options, io.StringIO()) == 0
    records = [json.loads(line) for line in run.events.read_text().splitlines()]
    assert [item['type'] for item in records] == [
        'ready_action_available', 'params_dump_start_failed']
    assert len(requested) == 1
    assert fake_ros.destroyed and fake_ros.shutdown
