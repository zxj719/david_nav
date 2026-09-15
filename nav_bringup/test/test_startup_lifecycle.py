"""Startup ordering and failure cleanup use only fake or private non-ROS children."""

from dataclasses import replace
import os
import signal
import sys
import time

import pytest

from nav_bringup.navrun import startup_lifecycle as lifecycle
from nav_bringup.navrun.locking import RunLock
from nav_bringup.recorder.events import EventLog, read_events


@pytest.fixture
def rig(tmp_path, monkeypatch):
    """Provide fake children, explicit controls and an acquired private lease."""
    trace = []
    controls = {}

    class Child:
        def __init__(self, argv, environ, log, pass_fds=()):
            self.name = argv[0]
            self.pid = 123
            self.closed = False
            trace.append(('start', self.name))
            assert environ['ROS_LOG_DIR'] == str(tmp_path / 'logs/ros')
            assert log.parent == tmp_path / 'logs'
            if controls.get(self.name) == 'spawn_error':
                raise OSError('fixture spawn failed')

        def poll(self):
            if self.name in controls:
                return controls[self.name]
            return None if self.name in ('launch', 'record', 'monitor', 'keyboard') else 0

        def close(self, timeout_sec=10):
            trace.append(('close', self.name))
            self.closed = True
            if controls.get(self.name + '_close'):
                raise OSError('fixture close failed')
            return self.poll() or 0

    monkeypatch.setattr(lifecycle, 'OwnedProcess', Child)
    monkeypatch.setattr(lifecycle.time, 'sleep', lambda _: None)
    commands = lifecycle.StartupCommands(
        ('launch',),
        (lifecycle.StartupGate('ready', ('ready',), 120),
         lifecycle.StartupGate('live_check', ('live_check',))),
        ('record',), ('monitor',), ('zero',))
    (tmp_path / 'logs').mkdir()
    events = EventLog(tmp_path / 'events.jsonl')
    lease = RunLock(tmp_path / 'lock')
    with lease:
        yield commands, trace, controls, events, lease


def _run(rig, tmp_path, stopped=None):
    commands, trace, controls, events, lease = rig

    def default_stop():
        return signal.SIGINT if ('start', 'record') in trace else None

    return lifecycle.supervise(commands, tmp_path, events, os.environ, lease,
                               stopped or default_stop)


def test_order_and_two_zero_attempts_with_lock_retained(rig, tmp_path):
    """Readiness precedes recording and zero brackets launch shutdown."""
    result = _run(rig, tmp_path)
    assert result.exit_code == 130
    assert result.exit_reason == 'ctrl_c'
    assert rig[1] == [('start', 'launch'), ('start', 'ready'), ('close', 'ready'),
                      ('start', 'live_check'), ('close', 'live_check'),
                      ('start', 'monitor'), ('start', 'record'), ('start', 'zero'),
                      ('close', 'zero'), ('close', 'launch'), ('start', 'zero'),
                      ('close', 'zero'), ('close', 'record'), ('close', 'monitor')]
    rig[4].fileno()
    events, skipped = read_events(rig[3].path)
    assert skipped == 0
    assert any(event.event_type == 'startup_ready' for event in events)


@pytest.mark.parametrize('outcome', ['ready', 'failed', 'interrupted'])
def test_prelaunch_localization_precedes_all_navigation_and_zero(rig, tmp_path, outcome):
    """Neither Nav2 nor executor readiness timers run before manual localization."""
    commands, trace, controls, events, lease = rig
    commands = replace(commands, prelaunch_gates=(
        lifecycle.StartupGate('localization', ('localization',)),))
    if outcome != 'ready':
        controls['localization'] = 1 if outcome == 'failed' else None

    def stopped():
        if outcome == 'interrupted' and ('start', 'localization') in trace:
            return signal.SIGINT
        return signal.SIGINT if ('start', 'record') in trace else None

    result = lifecycle.supervise(commands, tmp_path, events, os.environ, lease, stopped)
    assert ('close', 'localization') in trace
    if outcome == 'ready':
        assert trace.index(('close', 'localization')) < trace.index(('start', 'launch'))
        assert result.exit_code == 130
    else:
        assert ('start', 'launch') not in trace and ('start', 'zero') not in trace
        assert result.exit_code == (1 if outcome == 'failed' else 130)
    lease.fileno()


@pytest.mark.parametrize('outcome', ['quit', 'reject', 'signal', 'revoke_error'])
def test_keyboard_readiness_precedes_launch_and_quit_is_normal(rig, tmp_path, outcome):
    """Only an accepted keyboard can bring up the adapter; q completes the owned run."""
    commands, trace, controls, events, lease = rig
    commands = replace(commands, keyboard=('keyboard',), tty_device='/fixture/tty', gates=())

    class Session:
        def command(self, argv):
            return argv, (30, 31, 32)

        def child_started(self):
            trace.append(('handoff',))

        def ready(self):
            trace.append(('keyboard_ready',))
            if outcome == 'reject':
                raise RuntimeError('conflict')
            return True

        def permit(self):
            trace.append(('permit',))

        def revoke(self):
            trace.append(('revoke',))
            if outcome == 'revoke_error':
                raise OSError('revoke failed')

    def stopped():
        if outcome == 'signal' and ('handoff',) in trace:
            return signal.SIGINT
        if outcome in ('quit', 'revoke_error') and ('start', 'record') in trace:
            controls['keyboard'] = 0
        return None

    result = lifecycle.supervise(commands, tmp_path, events, os.environ, lease, stopped,
                                 keyboard=Session())
    assert ('close', 'keyboard') in trace
    if outcome in ('quit', 'revoke_error'):
        assert (result.exit_code, result.exit_reason) == (
            (0, 'keyboard_quit') if outcome == 'quit' else (2, 'cleanup_failed'))
        assert ('close', 'launch') in trace
        assert trace.index(('keyboard_ready',)) < trace.index(('start', 'launch'))
        assert trace.index(('start', 'launch')) < trace.index(('permit',))
        assert trace.index(('revoke',)) < trace.index(('close', 'launch'))
    else:
        assert result.exit_code == (130 if outcome == 'signal' else 2)
        assert ('start', 'launch') not in trace
        assert ('start', 'zero') not in trace


@pytest.mark.parametrize('stage', ['ready', 'live_check'])
def test_failed_gate_never_starts_recording_and_still_cleans_launch(rig, tmp_path, stage):
    """A failed readiness or localization check cannot start recording."""
    rig[2][stage] = 1
    result = _run(rig, tmp_path)
    assert result.exit_code == 1
    assert result.exit_reason == stage + '_failed'
    assert ('start', 'record') not in rig[1]
    assert ('close', 'launch') in rig[1]
    assert rig[1].count(('start', 'zero')) == 2


def test_launcher_exit_during_localization_is_not_hidden_by_wait(rig, tmp_path):
    """Indefinite localization waiting still monitors launcher failure."""
    rig[2]['live_check'] = None

    def stop():
        if ('start', 'live_check') in rig[1]:
            rig[2]['launch'] = 0
        return None

    result = _run(rig, tmp_path, stop)
    assert result.exit_code == 1
    assert result.exit_reason == 'launch_exited'
    assert ('close', 'live_check') in rig[1]
    assert ('start', 'record') not in rig[1]


def test_signal_during_manual_localization_closes_pending_gate(rig, tmp_path):
    """Termination interrupts manual waiting and stops the owned checker."""
    rig[2]['live_check'] = None

    def stop():
        return signal.SIGTERM if ('start', 'live_check') in rig[1] else None

    assert _run(rig, tmp_path, stop).exit_code == 143
    assert ('close', 'live_check') in rig[1]
    assert ('start', 'record') not in rig[1]


def test_signal_before_start_does_not_launch_or_publish_zero(rig, tmp_path):
    """An interrupted request does not affect an unclaimed control graph."""
    assert _run(rig, tmp_path, lambda: signal.SIGINT).exit_code == 130
    assert rig[1] == []


def test_launch_spawn_error_does_not_publish_zero(rig, tmp_path):
    """No launch child means no authority to run a zero publisher."""
    rig[2]['launch'] = 'spawn_error'
    assert _run(rig, tmp_path).exit_code == 2
    assert rig[1] == [('start', 'launch')]


def test_record_exit_is_failure_not_goal_or_run_success(rig, tmp_path):
    """Even an early zero exit from the recorder fails continuous supervision."""
    rig[2]['record'] = 0
    result = _run(rig, tmp_path, lambda: None)
    assert result.exit_code == 1
    assert result.exit_reason == 'record_exited'


def test_monitor_failure_warns_without_aborting_run(rig, tmp_path):
    """Optional diagnostics retain the existing warn-only failure policy."""
    rig[2]['monitor'] = 'spawn_error'
    assert _run(rig, tmp_path).exit_code == 130
    assert ('start', 'record') in rig[1]
    assert any(event.event_type == 'monitor_failed' and event.severity == 'warn'
               for event in read_events(rig[3].path)[0])


def test_cleanup_error_does_not_skip_other_children_or_last_zero(rig, tmp_path):
    """A failed close retains its handle without skipping unrelated cleanup."""
    rig[2]['launch_close'] = True
    result = _run(rig, tmp_path)
    assert result.exit_code == 2
    assert result.exit_reason == 'cleanup_failed'
    assert rig[1].count(('start', 'zero')) == 2
    assert ('close', 'record') in rig[1]
    assert ('close', 'monitor') in rig[1]
    assert len(result.remaining) == 1
    assert result.remaining[0].name == 'launch'
    rig[2]['launch_close'] = False
    result.remaining[0].close()


def test_zero_failure_is_visible_and_does_not_skip_cleanup(rig, tmp_path):
    """Failed zero commands cannot turn a partial stop into claimed success."""
    rig[2]['zero'] = 1
    result = _run(rig, tmp_path)
    assert result.exit_code == 2
    assert result.exit_reason == 'cleanup_failed'
    assert ('close', 'launch') in rig[1]
    assert ('close', 'record') in rig[1]


def test_gate_timeout_uses_single_deadline(rig, tmp_path, monkeypatch):
    """Readiness uses one wall-clock deadline rather than resetting on each poll."""
    rig[2]['ready'] = None
    ticks = iter(range(1000))
    monkeypatch.setattr(lifecycle.time, 'monotonic', lambda: next(ticks))
    result = _run(rig, tmp_path, lambda: None)
    assert result.exit_code == 1
    assert result.exit_reason == 'ready_timeout'
    assert ('close', 'ready') in rig[1]


def test_no_lease_rejects_before_child_creation(rig, tmp_path):
    """Supervision cannot start without an acquired navigation lease."""
    rig[4].release()
    with pytest.raises(RuntimeError):
        _run(rig, tmp_path)
    assert not rig[1]


@pytest.mark.parametrize('name', ['../escape', 'launch', 'record', 'zero_before', 'ros'])
def test_unsafe_or_reserved_gate_names_rejected(rig, tmp_path, name):
    """Gate labels cannot overwrite another child or escape the logs directory."""
    changed = replace(rig[0], gates=(lifecycle.StartupGate(name, ('check',)),))
    with pytest.raises(ValueError):
        _run((changed, *rig[1:]), tmp_path)
    assert not rig[1]


def test_real_private_children_execute_pipeline_and_flush_logs(tmp_path):
    """The complete child lifecycle runs without ROS or any velocity message."""
    (tmp_path / 'logs').mkdir()

    def command(message, persistent=False):
        code = 'import time\nprint(' + repr(message) + ', flush=True)\n'
        if persistent:
            code += 'while True: time.sleep(1)\n'
        return (sys.executable, '-u', '-c', code)

    commands = lifecycle.StartupCommands(
        command('launcher', True),
        (lifecycle.StartupGate('ready', command('ready'), 5),
         lifecycle.StartupGate('live_check', command('localized'), 5)),
        command('recorder', True), command('monitor', True), command('zero-placeholder'))
    events = EventLog(tmp_path / 'events.jsonl')
    deadline = time.monotonic() + 10

    def stop():
        if time.monotonic() >= deadline:
            return signal.SIGTERM
        path = tmp_path / 'logs/record.log'
        return signal.SIGINT if path.exists() and path.stat().st_size else None

    with RunLock(tmp_path / 'lock') as lease:
        result = lifecycle.supervise(commands, tmp_path, events, os.environ, lease, stop,
                                     stop_timeout_sec=0.2)
    assert result.exit_code == 130
    assert (tmp_path / 'logs/ready.log').read_text().strip() == 'ready'
    assert (tmp_path / 'logs/live_check.log').read_text().strip() == 'localized'
    assert 'zero-placeholder' in (tmp_path / 'logs/zero_after.log').read_text()


@pytest.mark.parametrize('commands', [
    lifecycle.StartupCommands((), (), (), (), ('zero',)),
    lifecycle.StartupCommands(('launch',), (), (), (), ()),
    lifecycle.StartupCommands(('launch',), (
        lifecycle.StartupGate('ready', ('ready',), float('nan')),), (), (), ('zero',)),
    lifecycle.StartupCommands(('launch',), (
        lifecycle.StartupGate('ready', ('ready',)),
        lifecycle.StartupGate('ready', ('ready',))), (), (), ('zero',)),
])
def test_invalid_commands_fail_before_start(rig, tmp_path, commands):
    """Malformed vectors and deadlines fail before any child can claim control."""
    with pytest.raises(ValueError):
        _run((commands, *rig[1:]), tmp_path)
    assert not rig[1]


def test_event_write_error_does_not_prevent_shutdown(rig, tmp_path, monkeypatch):
    """Disk errors fail recording visibly but do not suppress stop attempts."""
    def broken(*args):
        raise OSError('fixture event disk full')

    monkeypatch.setattr(rig[3], 'append', broken)
    result = _run(rig, tmp_path)
    assert result.exit_code == 2
    assert not result.remaining
    assert ('close', 'launch') in rig[1]
    assert ('start', 'record') not in rig[1]


def test_successful_gate_with_unfinished_cleanup_cannot_advance(rig, tmp_path):
    """A zero exit code cannot hide a gate descendant that failed to stop."""
    rig[2]['ready_close'] = True
    result = _run(rig, tmp_path)
    assert result.exit_code == 2
    assert ('start', 'live_check') not in rig[1]
    assert ('start', 'record') not in rig[1]
    assert len(result.remaining) == 1
    rig[2]['ready_close'] = False
    result.remaining[0].close()


def test_parameter_dump_runs_independently_and_warns_on_failure(rig, tmp_path):
    """A one-shot dump failure does not block the persistent recorder or kill launch."""
    rig[2]['parameter_dump'] = 1
    selected = replace(rig[0], parameter_dump=('parameter_dump',))
    assert _run((selected, *rig[1:]), tmp_path).exit_code == 130
    assert ('start', 'record') in rig[1]
    assert ('close', 'parameter_dump') in rig[1]
    assert any(event.event_type == 'parameter_dump_finished' and event.severity == 'warn'
               for event in read_events(rig[3].path)[0])


def test_zero_attempt_precedes_waiting_for_pending_gate_cleanup(rig, tmp_path):
    """A slow diagnostic exit cannot delay the first zero attempt."""
    rig[2]['ready'] = None
    result = _run(rig, tmp_path,
                  lambda: signal.SIGINT if ('start', 'ready') in rig[1] else None)
    assert result.exit_code == 130
    assert rig[1].index(('start', 'zero')) < rig[1].index(('close', 'ready'))
