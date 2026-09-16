"""Final event publication uses fake ROS entities and a bounded synthetic clock."""

import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from nav_bringup.recorder.events import EventLog
from nav_bringup.recorder.run_event_stream import RunEventStream


@pytest.fixture
def ros(monkeypatch):
    """Replace ROS before importing the final publisher; no real context can be created."""
    state = NS(now=0.0, initialized=0, destroyed=0, shutdown=0, sent=[], ack=True,
               fail_publish=False, fail_create=False, ack_timeouts=[])

    def advance(seconds):
        state.now += seconds

    def publish(message):
        if state.fail_publish:
            raise RuntimeError('private transport detail')
        state.sent.append(message)

    def acknowledge(duration):
        state.ack_timeouts.append(duration.seconds)
        return state.ack

    publisher = NS(publish=publish, wait_for_all_acked=acknowledge)

    def create_publisher(kind, topic, qos):
        assert topic == '/nav/events' and qos == 10
        if state.fail_create:
            raise RuntimeError('private creation detail')
        return publisher

    node = NS(create_publisher=create_publisher,
              destroy_node=lambda: setattr(state, 'destroyed', state.destroyed + 1))
    modules = {
        'rclpy': dict(init=lambda **kwargs: setattr(state, 'initialized', state.initialized + 1),
                      create_node=lambda *args, **kwargs: node,
                      spin_once=lambda node, timeout_sec: advance(timeout_sec),
                      try_shutdown=lambda **kw: setattr(state, 'shutdown', state.shutdown + 1)),
        'rclpy.context': dict(Context=object),
        'rclpy.duration': dict(Duration=NS),
        'david_nav_msgs': {},
        'david_nav_msgs.msg': dict(NavEvent=NS),
    }
    for name, values in modules.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    from nav_bringup.recorder import event_flush

    monkeypatch.setattr(event_flush.time, 'monotonic', lambda: state.now)
    monkeypatch.setattr(event_flush.time, 'sleep', advance)
    return state


def test_flush_resumes_monitor_cursor_and_includes_final_parent_event(tmp_path, ros):
    """The parent writes run_stopped only after the old monitor is gone."""
    from nav_bringup.recorder.event_flush import publish_tail

    log = EventLog(tmp_path / 'events.jsonl', clock=lambda: 12)
    log.append('navrun', 'run_started')
    previous = RunEventStream(log.path, lambda message: None, lambda message: None)
    assert previous.pump() == 1
    prefix = previous.identity_prefix
    previous.close()
    for _ in range(260):
        log.append('navrun', 'process_stopped', data={'exit_code': 0})
    log.append('navrun', 'run_stopped', data={'exit_code': 130, 'exit_reason': 'ctrl_c'})
    assert publish_tail(log.path)
    assert len(ros.sent) == 261 and ros.sent[-1].event_type == 'run_stopped'
    assert all(message.event_id.startswith(prefix) for message in ros.sent)
    assert ros.initialized == ros.destroyed == ros.shutdown == 1
    assert ros.ack_timeouts and 0 <= ros.ack_timeouts[-1] <= 3


def test_live_monitor_lease_prevents_any_second_ros_participant(tmp_path, ros):
    """A stale caller cannot overlap the final publisher with an existing monitor."""
    from nav_bringup.recorder.event_flush import publish_tail

    log = EventLog(tmp_path / 'events.jsonl')
    log.append('navrun', 'run_stopped')
    owner = RunEventStream(log.path, lambda message: None, lambda message: None)
    warnings = []
    assert not publish_tail(log.path, warn=warnings.append)
    assert ros.initialized == 0 and not ros.sent and warnings
    owner.close()


@pytest.mark.parametrize('failure', ['fail_publish', 'fail_create', 'ack'])
def test_failed_publication_or_ack_is_diagnostic_and_releases_lease(tmp_path, ros, failure):
    """No diagnostics failure is a navigation success/failure decision or a leaked owner."""
    from nav_bringup.recorder.event_flush import publish_tail

    log = EventLog(tmp_path / 'events.jsonl')
    log.append('navrun', 'run_stopped')
    setattr(ros, failure, failure != 'ack')
    warnings = []
    assert not publish_tail(log.path, timeout_sec=0.3, discovery_sec=0, warn=warnings.append)
    assert warnings and 'private' not in str(warnings)
    assert ros.destroyed == ros.shutdown == 1 and ros.now <= 0.35
    recovered = RunEventStream(log.path, lambda message: None, lambda message: None)
    recovered.close()


def test_incomplete_final_line_times_out_without_inventing_an_event(tmp_path, ros):
    """A partial append is retained, not silently declared delivered."""
    from nav_bringup.recorder.event_flush import publish_tail

    path = tmp_path / 'events.jsonl'
    path.write_text('{"partial":')
    assert not publish_tail(path, timeout_sec=0.2, discovery_sec=0)
    assert ros.now <= 0.25 and not ros.sent


def test_cleanup_failure_does_not_report_success_or_hold_lease(tmp_path, ros, monkeypatch):
    """A failed context shutdown is diagnostic, not a successful flush return value."""
    from nav_bringup.recorder.event_flush import publish_tail

    log = EventLog(tmp_path / 'events.jsonl')
    log.append('navrun', 'run_stopped')

    def failed(**kwargs):
        raise RuntimeError('private cleanup detail')

    monkeypatch.setattr(sys.modules['rclpy'], 'try_shutdown', failed)
    warnings = []
    assert not publish_tail(log.path, warn=warnings.append)
    assert warnings and 'private' not in str(warnings)
    recovered = RunEventStream(log.path, lambda message: None, lambda message: None)
    recovered.close()


@pytest.mark.parametrize('timeout,discovery', [
    (0, 0), (True, 0), (1, -1), (1, 2), (float('nan'), 0), (1, float('inf'))])
def test_invalid_time_budget_is_rejected_before_ros(tmp_path, ros, timeout, discovery):
    """Neither a negative Duration nor an unbounded final wait may reach the SDK."""
    from nav_bringup.recorder.event_flush import publish_tail

    with pytest.raises(ValueError):
        publish_tail(tmp_path / 'events.jsonl', timeout_sec=timeout, discovery_sec=discovery)
    assert ros.initialized == 0


def test_private_command_rejects_missing_run_identity_without_ros(tmp_path, ros, monkeypatch):
    """The helper must never implicitly choose a current directory or latest run."""
    from nav_bringup.recorder.event_flush import main

    monkeypatch.delenv('NAVRUN_RUN_DIR', raising=False)
    with pytest.raises(SystemExit) as raised:
        main([])
    assert raised.value.code == 2 and ros.initialized == 0


def test_private_command_uses_explicit_environment_identity(tmp_path, ros, monkeypatch):
    """Actual main argument binding reaches the publisher with the supplied run only."""
    from nav_bringup.recorder.event_flush import main

    EventLog(tmp_path / 'events.jsonl').append('navrun', 'run_stopped')
    monkeypatch.setenv('NAVRUN_RUN_DIR', str(tmp_path))
    assert main([]) == 0 and ros.sent[-1].event_type == 'run_stopped'


def test_publisher_only_discovery_does_not_use_global_executor(tmp_path, ros, monkeypatch):
    """Humble's default spin_once executor belongs to a different, uninitialized context."""
    from nav_bringup.recorder.event_flush import publish_tail

    EventLog(tmp_path / 'events.jsonl').append('navrun', 'run_stopped')

    def forbidden(*args, **kwargs):
        raise AssertionError('publisher-only discovery must not use the global executor')

    monkeypatch.setattr(sys.modules['rclpy'], 'spin_once', forbidden)
    assert publish_tail(tmp_path / 'events.jsonl')
    assert ros.now >= 0.5
