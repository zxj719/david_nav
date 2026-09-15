"""Tests for ``navrun wrap``."""

import datetime
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from nav_bringup.navrun import cli
from nav_bringup.navrun import wrap
from nav_bringup.recorder import events
from nav_bringup.recorder import manifest
from nav_bringup.recorder import run_dir

START = datetime.datetime(2026, 9, 14, 21, 30, 0)
COMMIT = 'fe1ac78d0c4b9a2e5f1d3c7b8a6e4f2d1c0b9a87'


def _no_git(argv):
    return 1, ''


def _clock(duration_sec=5.0):
    times = iter([START, START + datetime.timedelta(seconds=duration_sec)])
    return lambda: next(times)


def _options(tmp_path, command, **overrides):
    values = {
        'command': command,
        'mode': 'tour',
        'runs_root': tmp_path / 'runs',
        'repo_dir': tmp_path,
        'sample_interval_sec': 0.05,
    }
    values.update(overrides)
    return wrap.WrapOptions(**values)


def _python(code):
    return [sys.executable, '-c', code]


def _event_types(run):
    return [event.event_type for event in events.read_events(run.events)[0]]


def test_successful_command_is_recorded(tmp_path):
    """Output is teed, and manifest, events, resources and summary are written."""
    terminal = io.BytesIO()
    command = _python('import sys; print("hello"); sys.stderr.write("oops" + chr(10))')

    result = wrap.run_wrapped(
        _options(tmp_path, command, alias='4'),
        environ={'ROS_DOMAIN_ID': '99', 'PATH': os.environ.get('PATH', '')},
        now=_clock(),
        runner=_no_git,
        terminal=terminal,
    )

    assert (result.exit_code, result.exit_reason) == (0, 'completed')
    assert result.run.run_id == '20260914_213000_tour_nogit'
    console = (result.run.logs / wrap.CONSOLE_LOG_NAME).read_bytes()
    assert b'hello' in console and b'oops' in console
    assert terminal.getvalue() == console
    record = manifest.read_manifest(result.run.manifest)
    assert record['exit_code'] == 0
    assert record['exit_reason'] == 'completed'
    assert record['duration_sec'] == 5.0
    assert record['alias'] == '4'
    assert record['environment']['ROS_DOMAIN_ID'] == '99'
    assert _event_types(result.run) == [
        'run_started', 'process_started', 'process_exited', 'run_stopped']
    assert (result.run.metrics / wrap.RESOURCES_CSV_NAME).is_file()
    assert '| 退出 | completed（退出码 0） |' in result.run.summary.read_text(encoding='utf-8')
    assert not result.run.code_diff.exists()


def test_failing_command_propagates_exit_code(tmp_path):
    """A non-zero exit status is returned unchanged."""
    result = wrap.run_wrapped(
        _options(tmp_path, _python('import sys; sys.exit(3)')),
        now=_clock(), runner=_no_git, terminal=io.BytesIO())

    assert (result.exit_code, result.exit_reason) == (3, 'failed')


def test_command_killed_by_signal_maps_to_shell_status(tmp_path):
    """A command killed by SIGKILL exits the wrapper with 137."""
    result = wrap.run_wrapped(
        _options(tmp_path, _python('import os, signal; os.kill(os.getpid(), signal.SIGKILL)')),
        now=_clock(), runner=_no_git, terminal=io.BytesIO())

    assert (result.exit_code, result.exit_reason) == (137, 'killed_by_signal')


def test_missing_program_is_recorded(tmp_path):
    """A command that cannot start still produces a complete record."""
    terminal = io.BytesIO()

    result = wrap.run_wrapped(
        _options(tmp_path, ['surely-not-a-program-name']),
        now=_clock(), runner=_no_git, terminal=terminal)

    assert (result.exit_code, result.exit_reason) == (127, 'start_failed')
    assert b'cannot start' in terminal.getvalue()
    assert manifest.read_manifest(result.run.manifest)['exit_reason'] == 'start_failed'
    assert 'start_failed' in _event_types(result.run)
    assert result.run.summary.is_file()


def test_command_sees_run_environment(tmp_path):
    """The command can locate its run directory through the environment."""
    terminal = io.BytesIO()
    command = _python(
        'import os; print(os.environ["NAVRUN_RUN_ID"], os.environ["NAVRUN_RUN_DIR"])')

    result = wrap.run_wrapped(
        _options(tmp_path, command), now=_clock(), runner=_no_git, terminal=terminal)

    assert terminal.getvalue().decode().split() == [result.run.run_id, str(result.run.path)]


def test_dirty_repository_writes_code_diff(tmp_path):
    """A dirty checkout names the run after its commit and saves the diff."""
    repo = str(tmp_path)
    responses = {
        ('rev-parse', '--show-toplevel'): (0, repo),
        ('rev-parse', '--abbrev-ref', 'HEAD'): (0, 'refactor/nav-system'),
        ('rev-parse', 'HEAD'): (0, COMMIT),
        ('status', '--porcelain'): (0, ' M a.py'),
        ('diff', 'HEAD'): (0, 'diff --git a/a.py b/a.py'),
    }

    def runner(argv):
        return responses.get(tuple(argv[3:]), (1, ''))

    result = wrap.run_wrapped(
        _options(tmp_path, _python('pass')), now=_clock(), runner=runner, terminal=io.BytesIO())

    assert result.run.run_id == '20260914_213000_tour_fe1ac78'
    assert 'diff --git a/a.py b/a.py' in result.run.code_diff.read_text(encoding='utf-8')
    assert manifest.read_manifest(result.run.manifest)['git']['dirty'] is True


def test_gc_prunes_old_runs_but_not_the_new_one(tmp_path):
    """Old runs beyond the policy are pruned before the command starts."""
    root = tmp_path / 'runs'
    for stamp in ('20260910_120000', '20260911_120000'):
        old = run_dir.create_run_directory(root, f'{stamp}_tour_nogit')
        old.manifest.write_text('run_id: old\n', encoding='utf-8')
        (old.logs / 'launch.log').write_text('x' * 100, encoding='utf-8')
    policy = wrap.retention.RetentionPolicy(max_runs=2)

    result = wrap.run_wrapped(
        _options(tmp_path, _python('pass'), retention_policy=policy),
        now=_clock(), runner=_no_git, terminal=io.BytesIO())

    oldest = root / '20260910_120000_tour_nogit'
    assert sorted(entry.name for entry in oldest.iterdir()) == ['manifest.yaml']
    assert (root / '20260911_120000_tour_nogit' / 'logs' / 'launch.log').exists()
    assert result.run.logs.is_dir()
    assert 'gc_applied' in _event_types(result.run)


def _start_cli(tmp_path, extra_env=None):
    """Start ``navrun wrap`` in its own session around a long-running child."""
    runs_root = tmp_path / 'runs'
    child = (
        'import signal, time; signal.signal(signal.SIGINT, signal.SIG_DFL); '
        'print("ready", flush=True); time.sleep(30)'
    )
    env = dict(os.environ)
    env.update(extra_env or {})
    process = subprocess.Popen(
        [sys.executable, '-m', 'nav_bringup.navrun.cli', 'wrap', '--mode', 'tour',
         '--runs-root', str(runs_root), '--repo', str(tmp_path), '--no-gc',
         '--sample-interval', '0.05', '--', sys.executable, '-c', child],
        env=env,
        start_new_session=True,
        stderr=subprocess.PIPE,
        stdout=subprocess.PIPE,
    )
    console = runs_root / run_dir.LATEST_LINK_NAME / 'logs' / wrap.CONSOLE_LOG_NAME
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        if console.is_file() and b'ready' in console.read_bytes():
            return process, runs_root
        time.sleep(0.05)
    process.kill()
    raise AssertionError('wrapped child did not start')


def test_ctrl_c_reaches_command_and_run_is_finalized(tmp_path):
    """SIGINT to the process group stops the child; the wrapper records ctrl_c."""
    process, runs_root = _start_cli(tmp_path)

    os.killpg(process.pid, signal.SIGINT)
    _, stderr = process.communicate(timeout=30)

    assert process.returncode == 130, stderr
    record = manifest.read_manifest(runs_root / 'latest' / 'manifest.yaml')
    assert record['exit_reason'] == 'ctrl_c'
    assert record['exit_code'] == 130
    assert b'navrun: run record' in stderr


def test_sigterm_to_wrapper_is_forwarded_to_command(tmp_path):
    """SIGTERM sent only to the wrapper still stops the child."""
    process, runs_root = _start_cli(tmp_path)

    os.kill(process.pid, signal.SIGTERM)
    process.communicate(timeout=30)

    assert process.returncode == 128 + signal.SIGTERM
    record = manifest.read_manifest(Path(runs_root) / 'latest' / 'manifest.yaml')
    assert record['exit_reason'] == 'terminated'


FAKE_MONITOR = (
    'import os, pathlib, signal, sys, time\n'
    'run = pathlib.Path(os.environ["NAVRUN_RUN_DIR"])\n'
    'def stop(*args):\n'
    '    (run / "monitor_stopped").touch()\n'
    '    sys.exit(0)\n'
    'signal.signal(signal.SIGINT, stop)\n'
    'print("monitor up", flush=True)\n'
    '(run / "monitor_ready").touch()\n'
    'time.sleep(30)\n'
)
WAIT_FOR_MONITOR = (
    'import os, pathlib, time\n'
    'marker = pathlib.Path(os.environ["NAVRUN_RUN_DIR"]) / "monitor_ready"\n'
    'deadline = time.monotonic() + 20\n'
    'while not marker.exists() and time.monotonic() < deadline:\n'
    '    time.sleep(0.05)\n'
    'print("child saw monitor", marker.exists(), flush=True)\n'
)


def _monitor_stopped_event(run):
    return next(
        event for event in events.read_events(run.events)[0]
        if event.event_type == 'monitor_stopped')


def test_monitor_runs_next_to_command_and_is_stopped(tmp_path):
    """The monitor sees the run, shares the output, and is interrupted after the command."""
    terminal = io.BytesIO()

    result = wrap.run_wrapped(
        _options(tmp_path, _python(WAIT_FOR_MONITOR), monitor_command=_python(FAKE_MONITOR)),
        now=_clock(), runner=_no_git, terminal=terminal)

    output = terminal.getvalue()
    assert (result.exit_code, result.exit_reason) == (0, 'completed')
    assert b'monitor up' in output
    assert b'child saw monitor True' in output
    assert b'monitor up' in (result.run.logs / wrap.CONSOLE_LOG_NAME).read_bytes()
    assert (result.run.path / 'monitor_stopped').exists()
    assert _event_types(result.run) == [
        'run_started', 'process_started', 'monitor_started', 'monitor_stopped',
        'process_exited', 'run_stopped']
    stopped = _monitor_stopped_event(result.run)
    assert (stopped.severity, stopped.data['return_code'], stopped.data['exited_early']) == (
        'info', 0, False)


def test_crashing_monitor_does_not_change_the_result(tmp_path):
    """A monitor that dies is a warning; the command result is unchanged."""
    result = wrap.run_wrapped(
        _options(tmp_path, _python('import time; time.sleep(1.0)'),
                 monitor_command=_python('import sys; sys.exit(3)')),
        now=_clock(), runner=_no_git, terminal=io.BytesIO())

    stopped = _monitor_stopped_event(result.run)
    assert (result.exit_code, result.exit_reason) == (0, 'completed')
    assert (stopped.severity, stopped.data['return_code']) == ('warn', 3)


def test_monitor_that_cannot_start_is_recorded(tmp_path):
    """A missing monitor program is recorded and the command still runs."""
    terminal = io.BytesIO()

    result = wrap.run_wrapped(
        _options(tmp_path, _python('print("child ran")'),
                 monitor_command=['surely-not-a-monitor-program']),
        now=_clock(), runner=_no_git, terminal=terminal)

    assert result.exit_code == 0
    assert b'child ran' in terminal.getvalue()
    assert b'cannot start the monitor' in terminal.getvalue()
    assert 'monitor_start_failed' in _event_types(result.run)


def test_monitor_command():
    """The monitor command carries frame, topics, period and readiness settings."""
    assert wrap.monitor_command(
        'robot_base', topics=['/cmd_vel'], period_sec=2.0, python='python3') == [
        'python3', '-m', 'nav_bringup.navrun.cli', 'monitor', '--base-frame', 'robot_base',
        '--period', '2', '--topic', '/cmd_vel', '--ready-action', '/navigate_to_pose',
        '--dump-params-on-ready']
    assert wrap.monitor_command('robot_base', ready_action=None, python='python3')[-2:] == [
        '--period', '5']
    assert '--dump-params-on-ready' not in wrap.monitor_command(
        'robot_base', dump_params=False, python='python3')


def test_cli_wrap_builds_monitor_command(tmp_path, monkeypatch):
    """--monitor-base-frame enables the monitor with the given settings."""
    captured = []

    def fake_run_wrapped(options):
        captured.append(options)
        return wrap.WrapResult(
            run_dir.RunDirectory(tmp_path, '20260915_031500_tour_nogit'), 0, 'completed')

    monkeypatch.setattr(wrap, 'run_wrapped', fake_run_wrapped)

    assert cli.main([
        'wrap', '--mode', 'tour', '--runs-root', str(tmp_path), '--no-gc',
        '--monitor-base-frame', 'robot_base', '--monitor-topic', '/cmd_vel',
        '--no-dump-params', '--', 'true',
    ]) == 0
    assert cli.main(['wrap', '--mode', 'tour', '--runs-root', str(tmp_path), '--', 'true']) == 0

    command = captured[0].monitor_command
    assert command[3:6] == ['monitor', '--base-frame', 'robot_base']
    assert command[command.index('--topic'):command.index('--topic') + 2] == [
        '--topic', '/cmd_vel']
    assert '--dump-params-on-ready' not in command
    assert captured[1].monitor_command is None
