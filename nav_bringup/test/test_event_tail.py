"""The parent owns the final diagnostic child through proven group cleanup."""

import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

from nav_bringup.recorder.events import EventLog, read_events


def _owner(tmp_path, monkeypatch, *, code=0, failures=0):
    from nav_bringup.navrun import event_tail

    (tmp_path / 'logs').mkdir()
    state = NS(now=0.0, created=[], closed=0, failures=failures, code=code)

    class Child:
        def __init__(self, argv, environ, log):
            state.created.append((argv, environ, log))

        def poll(self):
            return state.code

        def close(self, timeout_sec):
            state.closed += 1
            if state.failures:
                state.failures -= 1
                raise OSError('private cleanup detail')
            return state.code

    monkeypatch.setattr(event_tail, 'OwnedProcess', Child)
    monkeypatch.setattr(event_tail.time, 'monotonic', lambda: state.now)
    monkeypatch.setattr(event_tail.time, 'sleep',
                        lambda value: setattr(state, 'now', state.now + value))
    return event_tail, state, EventLog(tmp_path / 'events.jsonl')


def test_final_child_has_exact_command_run_environment_and_private_log(tmp_path, monkeypatch):
    """The final publisher has no inherited standard input or broad process matching."""
    module, state, events = _owner(tmp_path, monkeypatch)
    env = {'ROS_DOMAIN_ID': '99'}
    module.flush_tail(('fixture',), tmp_path, env, events)
    assert state.closed == 1 and env == {'ROS_DOMAIN_ID': '99'}
    argv, inherited, log = state.created[0]
    assert argv == ('fixture',) and inherited['NAVRUN_RUN_DIR'] == str(tmp_path)
    assert log == tmp_path / 'logs/event_flush.log'
    rows = read_events(events.path)[0]
    assert rows[-1].source == 'event_flush' and rows[-1].event_type == 'finished'


@pytest.mark.parametrize('code', [3, None])
def test_final_helper_failure_or_timeout_cannot_abandon_its_group(tmp_path, monkeypatch, code):
    """Diagnostics failure is recorded locally, after cleanup, not as a navrun outcome."""
    module, state, events = _owner(tmp_path, monkeypatch, code=code, failures=1)
    module.flush_tail(('fixture',), tmp_path, {}, events, timeout_sec=0.2)
    assert state.closed == 2 and state.now < 2
    rows = read_events(events.path)[0]
    assert rows[-1].source == 'event_flush' and rows[-1].event_type == 'failed'
    assert 'private' not in events.path.read_text()


def test_disabled_tail_starts_nothing_and_preserves_event_format(tmp_path, monkeypatch):
    """Custom callers remain opted out unless they explicitly supply a final command."""
    module, state, events = _owner(tmp_path, monkeypatch)
    module.flush_tail((), tmp_path, {}, events)
    assert not state.created and not events.path.exists()


def test_close_interruption_is_raised_only_after_retry_has_drained(tmp_path, monkeypatch):
    """An interrupted caller cannot lose the final child handle during cleanup."""
    module, state, events = _owner(tmp_path, monkeypatch)
    original = module.OwnedProcess.close

    def interrupted(self, timeout_sec):
        if not state.closed:
            state.closed += 1
            raise KeyboardInterrupt
        return original(self, timeout_sec)

    monkeypatch.setattr(module.OwnedProcess, 'close', interrupted)
    with pytest.raises(KeyboardInterrupt):
        module.flush_tail(('fixture',), tmp_path, {}, events)
    assert state.closed == 2


def test_real_non_ros_helper_sees_final_record_and_is_reaped(tmp_path):
    """Only an owned plain Python process is launched; no SDK, DDS or motion exists."""
    from nav_bringup.navrun.event_tail import flush_tail

    (tmp_path / 'logs').mkdir()
    events = EventLog(tmp_path / 'events.jsonl')
    events.append('navrun', 'run_stopped', data={'exit_code': 130})
    code = (
        'import json, os, pathlib\n'
        'root = pathlib.Path(os.environ["NAVRUN_RUN_DIR"])\n'
        'record = json.loads((root / "events.jsonl").read_text().splitlines()[-1])\n'
        'assert record["type"] == "run_stopped" and record["data"]["exit_code"] == 130\n'
        '(root / "fixture_pid").write_text(str(os.getpid()))\n'
    )
    flush_tail((sys.executable, '-c', code), tmp_path, os.environ, events)
    pid = (tmp_path / 'fixture_pid').read_text()
    assert not Path('/proc', pid).exists()
    assert read_events(events.path)[0][-1].event_type == 'finished'


def test_broken_diagnostic_file_cannot_skip_child_cleanup(tmp_path, monkeypatch):
    """Disk failure does not override the required cleanup of the private helper."""
    module, state, events = _owner(tmp_path, monkeypatch, code=1, failures=1)

    def broken(*args, **kwargs):
        raise OSError('private disk detail')

    monkeypatch.setattr(events, 'append', broken)
    module.flush_tail(('fixture',), tmp_path, {}, events)
    assert state.closed == 2


def test_missing_helper_is_recorded_without_exposing_its_exception(tmp_path, monkeypatch):
    """A failed exec cannot change the already recorded navigation outcome."""
    module, state, events = _owner(tmp_path, monkeypatch)

    def unavailable(*args, **kwargs):
        raise OSError('private executable path')

    monkeypatch.setattr(module, 'OwnedProcess', unavailable)
    module.flush_tail(('fixture',), tmp_path, {}, events)
    rows = read_events(events.path)[0]
    assert rows[-1].event_type == 'failed' and state.closed == 0
    assert 'private' not in events.path.read_text()
