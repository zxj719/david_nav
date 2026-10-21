"""Recorded startup uses actual bundles and only private non-ROS child commands."""

from dataclasses import fields, replace
import datetime
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import termios
import time
import tty

import pytest
import yaml

from nav_bringup.navcfg import schema
from nav_bringup.navrun import recorded_startup as startup
from nav_bringup.navrun.locking import LockBusy, RunLock
from nav_bringup.navrun.preflight import ProcessInfo
from nav_bringup.navrun.startup_lifecycle import LifecycleResult, StartupCommands, StartupGate
from nav_bringup.navrun.up_planning import discover_inputs, plan_up, UpRequest
from nav_bringup.recorder.events import read_events
from nav_bringup.recorder.manifest import read_manifest
from nav_bringup.recorder.retention import RetentionPolicy
from nav_bringup.recorder.run_dir import create_run_directory


SOURCE = Path(__file__).resolve().parents[2]
START = datetime.datetime(2026, 9, 16, 1, 0, 0)


def _options(tmp_path):
    return startup.StartupOptions('chassis', 'example_diffdrive', 'none',
                                  tmp_path / 'runs', tmp_path, tmp_path / 'lease',
                                  command=('navrun', 'up', '--mode', 'chassis'))


def _prepare(path):
    request = UpRequest('chassis', 'example_diffdrive', 'none', path, no_executor=True)
    own = ProcessInfo(os.getpid(), os.getppid(), 1.0, ('navrun',), os.getuid())
    return plan_up(request, discover_inputs(request, lambda package: SOURCE / package),
                   schema.load_catalog(schema.find_source_schemas(SOURCE)), (own,), own.pid,
                   lambda package, name: '/fake/' + name)


def _commands(plan):
    def command(code):
        return (sys.executable, '-u', '-c', code)

    return StartupCommands(
        command('import time\nprint("launch", flush=True)\nwhile True: time.sleep(1)'),
        (StartupGate('ready', command('print("等待定位 fixture", flush=True)'), 5),),
        (), (), command('print("zero placeholder", flush=True)'))


def _run(options, prepare=_prepare, commands=_commands, steps=()):
    terminal = io.BytesIO()
    result = startup.run_recorded_startup(
        options, prepare, commands, steps, runner=lambda _: (1, ''),
        now=lambda: START, terminal=terminal)
    return result, terminal.getvalue()


@pytest.fixture
def successful_lifecycle(monkeypatch):
    """Replace child execution while asserting prepared files and lock ownership."""
    def run(commands, run_dir, events, environ, lease, stopped, **kwargs):
        assert (run_dir / 'manifest.yaml').is_file()
        assert (run_dir / 'launch.json').is_file()
        with pytest.raises(LockBusy):
            with RunLock(lease.path):
                pass
        assert environ['ROS_LOG_DIR'] == str(run_dir / 'logs/ros')
        return LifecycleResult(130, 'ctrl_c')

    monkeypatch.setattr(startup, 'supervise', run)


def test_run_stopped_is_the_last_local_navrun_record(tmp_path, successful_lifecycle):
    """The navrun parent writes its terminal record locally; no helper runs or writes after it."""
    result, _ = _run(_options(tmp_path))
    events = read_events(result.run.events)[0]
    assert [e.event_type for e in events if e.source == 'navrun'][-1] == 'run_stopped'
    assert all(e.source != 'event_flush' for e in events)
    assert not (result.run.logs / 'event_flush.log').exists()
    assert 'event_flush' not in {item.name for item in fields(StartupCommands)}
    assert result.exit_code == 130


def test_host_preparation_failure_never_starts_final_ros_helper(tmp_path):
    """A rejected preflight is recorded locally without any ROS helper process."""
    def reject(*args):
        raise RuntimeError('preflight rejected')

    result, _ = _run(_options(tmp_path), steps=(startup.StartupStep('preflight', reject),))
    events = read_events(result.run.events)[0]
    assert result.exit_reason == 'preflight_failed'
    assert all(e.source != 'event_flush' for e in events)
    assert not (result.run.logs / 'event_flush.log').exists()


def test_run_record_written_before_prepare_and_finalized_before_unlock(
        tmp_path, monkeypatch, successful_lifecycle):
    """The lease spans initial manifest, bundle creation and completed summary."""
    options = _options(tmp_path)
    original_write = startup.manifest_lib.write_manifest
    writes = []

    def write(path, record):
        with pytest.raises(LockBusy):
            with RunLock(options.lock_path):
                pass
        writes.append(record['exit_reason'])
        original_write(path, record)

    def prepare(path):
        assert read_manifest(path / 'manifest.yaml')['ended_at'] is None
        return _prepare(path)

    monkeypatch.setattr(startup.manifest_lib, 'write_manifest', write)
    result, _ = _run(options, prepare)
    assert result.exit_code == 130
    assert writes[0] is None and writes[-1] == 'ctrl_c'
    assert result.run.summary.is_file()
    assert result.run.params_inputs.joinpath('fingerprints.json').is_file()
    assert read_manifest(result.run.manifest)['files']
    assert result.run.metrics.joinpath('resources.csv').is_file()
    assert options.runs_root.joinpath('latest').resolve() == result.run.path
    with RunLock(options.lock_path):
        pass


def test_prepare_failure_is_recorded_without_executing_host_or_children(tmp_path):
    """Offline errors still produce a final manifest and summary."""
    called = []

    def prepare(path):
        raise ValueError('fixture bad configuration')

    def forbidden(*args):
        called.append(True)
        raise AssertionError('must not advance')

    result, _ = _run(_options(tmp_path), prepare, forbidden,
                     (startup.StartupStep('platform', forbidden),))
    assert result.exit_code == 2
    assert not called
    assert not result.run.path.joinpath('launch.json').exists()
    assert read_manifest(result.run.manifest)['exit_reason'] == 'prepare_failed'
    assert result.run.summary.is_file()


def test_host_failure_stops_later_steps_and_records_partial_progress(tmp_path):
    """An applied platform step is not represented as rolled back after later failure."""
    called = []

    def platform(*args):
        called.append('platform')

    def preflight(*args):
        called.append('preflight')
        raise RuntimeError('fixture residual survived')

    def dds(*args):
        called.append('dds')

    steps = tuple(startup.StartupStep(name, call) for name, call in
                  [('platform', platform), ('preflight', preflight), ('dds', dds)])
    result, _ = _run(_options(tmp_path), steps=steps)
    assert called == ['platform', 'preflight']
    assert result.exit_reason == 'preflight_failed'
    assert not result.run.logs.joinpath('launch.log').exists()
    events = read_events(result.run.events)[0]
    assert any(event.event_type == 'startup_step_finished' and
               event.data['name'] == 'platform' for event in events)


def test_invalid_commands_rejected_before_host_mutation(tmp_path):
    """Every command is validated before applying any host step."""
    called = []
    result, _ = _run(_options(tmp_path), commands=lambda p: replace(_commands(p), zero=()),
                     steps=(startup.StartupStep('platform', lambda *args: called.append(1)),))
    assert result.exit_code == 2
    assert not called


def test_cleanup_retry_keeps_unfinished_manifest_and_lock(tmp_path, monkeypatch):
    """Remaining child handles are drained before summary or ownership release."""
    options = _options(tmp_path)
    calls = []

    class Remaining:
        pid = 999

        def close(self, timeout_sec):
            with pytest.raises(LockBusy):
                with RunLock(options.lock_path):
                    pass
            run = options.runs_root.joinpath('latest').resolve()
            assert read_manifest(run / 'manifest.yaml')['ended_at'] is None
            assert not (run / 'summary.md').exists()
            calls.append(1)
            if len(calls) == 1:
                raise OSError('fixture not drained yet')
            return 0

    monkeypatch.setattr(startup, 'supervise', lambda *a, **kw: LifecycleResult(
        2, 'cleanup_failed', ('fixture cleanup incomplete',), (Remaining(),)))
    monkeypatch.setattr(startup, '_retry_pause', lambda: None)
    result, _ = _run(options)
    assert len(calls) == 2
    assert result.exit_code == 2
    assert result.run.summary.is_file()


def test_terminal_restore_waits_until_keyboard_cleanup_retry_finishes(tmp_path, monkeypatch):
    """A force-killed keyboard cannot leave cbreak mode behind or restore it too early."""
    master, slave = os.openpty()
    original = termios.tcgetattr(slave)
    options = _options(tmp_path)
    attempts = []

    class Remaining:
        pid = 999

        def close(self, timeout_sec):
            attempts.append(1)
            assert not termios.tcgetattr(slave)[3] & termios.ICANON
            with pytest.raises(LockBusy):
                RunLock(options.lock_path).acquire()
            if len(attempts) == 1:
                raise OSError('not drained')

    def run(*args, **kwargs):
        assert kwargs['keyboard'] is not None
        tty.setcbreak(slave)
        return LifecycleResult(2, 'cleanup_failed', ('fixture',), (Remaining(),))

    monkeypatch.setattr(startup, 'supervise', run)
    monkeypatch.setattr(startup, '_retry_pause', lambda: None)
    try:
        result, _ = _run(options, commands=lambda plan: replace(
            _commands(plan), keyboard=('fake_keyboard',), tty_device=os.ttyname(slave)))
        assert result.exit_code == 2 and len(attempts) == 2
        assert termios.tcgetattr(slave) == original
    finally:
        os.close(master)
        os.close(slave)


def test_missing_terminal_prevents_host_steps(tmp_path):
    """TTY discovery occurs under the lease but before platform or process mutations."""
    called = []
    result, _ = _run(_options(tmp_path), commands=lambda plan: replace(
        _commands(plan), keyboard=('fake_keyboard',), tty_device=str(tmp_path / 'missing')),
        steps=(startup.StartupStep('platform', lambda *args: called.append(1)),))
    assert (result.exit_code, result.exit_reason) == (2, 'terminal_failed')
    assert not called
    assert not any(event.event_type == 'terminal_restored'
                   for event in read_events(result.run.events)[0])


def test_lock_busy_does_not_create_run_or_call_prepare(tmp_path):
    """A rejected concurrent startup has no recording or host side effects."""
    options = _options(tmp_path)
    with RunLock(options.lock_path):
        with pytest.raises(LockBusy):
            _run(options, prepare=lambda path: pytest.fail('must not prepare'))
    assert not options.runs_root.exists()


def test_signal_during_host_step_is_observed_before_next_stage(tmp_path, monkeypatch):
    """The parent owns interruption without raising inside cleanup or releasing early."""
    handlers = {}
    previous = signal.getsignal

    def install(signum, handler):
        old = handlers.get(signum, previous(signum))
        handlers[signum] = handler
        return old

    monkeypatch.setattr(signal, 'signal', install)

    def platform(plan, events, stopped):
        handlers[signal.SIGTERM](signal.SIGTERM, None)
        assert stopped() == signal.SIGTERM

    result, _ = _run(_options(tmp_path), steps=(startup.StartupStep('platform', platform),))
    assert result.exit_code == 143
    assert not result.run.logs.joinpath('launch.log').exists()
    assert handlers[signal.SIGTERM] == previous(signal.SIGTERM)


def test_real_child_failure_has_console_resource_and_summary_evidence(tmp_path):
    """A private launch and failing gate exercise recording end to end without ROS."""
    def commands(plan):
        selected = _commands(plan)
        gate = StartupGate('ready', (
            sys.executable, '-u', '-c', 'print("等待定位 fixture"); raise SystemExit(7)'), 5)
        return replace(selected, gates=(gate,))

    result, output = _run(_options(tmp_path), commands=commands)
    assert (result.exit_code, result.exit_reason) == (7, 'ready_failed')
    assert '等待定位 fixture'.encode() in output
    assert b'zero placeholder' in output
    assert output == result.run.logs.joinpath('console.log').read_bytes()
    assert read_manifest(result.run.manifest)['exit_code'] == 7
    assert 'ready_failed' in result.run.summary.read_text()


def test_final_write_error_preserves_previous_manifest_and_releases_empty_run(
        tmp_path, monkeypatch, successful_lifecycle):
    """Failed atomic finalization cannot erase the initial unfinished record."""
    options = _options(tmp_path)
    write = startup.manifest_lib.write_manifest

    def fail_final(path, data):
        if data['ended_at'] is not None:
            path.write_text('partial write', encoding='utf-8')
            raise OSError('fixture final disk full')
        write(path, data)

    monkeypatch.setattr(startup.manifest_lib, 'write_manifest', fail_final)
    with pytest.raises(OSError, match='fixture final disk full'):
        _run(options)
    run = options.runs_root.joinpath('latest').resolve()
    assert read_manifest(run / 'manifest.yaml')['ended_at'] is None
    assert not (run / 'summary.md').exists()
    assert not list(run.glob('.manifest-*'))
    with RunLock(options.lock_path):
        pass


def test_retention_only_prunes_completed_private_history(tmp_path, successful_lifecycle):
    """Current, unfinished and unreadable records cannot become retention targets."""
    options = replace(_options(tmp_path), retention_policy=RetentionPolicy(max_runs=0))
    roots = []
    for stamp, text in [('000001', 'ended_at: "2026-09-15T23:00:00"\n'),
                        ('000002', 'ended_at: null\n'), ('000003', '[broken:')]:
        run = create_run_directory(options.runs_root, '20260916_' + stamp + '_chassis_nogit')
        if stamp != '000003':
            text = f'schema_version: 1\nrun_id: {run.run_id}\n' + text
        run.manifest.write_text(text, encoding='utf-8')
        run.summary.write_text('fixture summary', encoding='utf-8')
        (run.logs / 'history.log').write_text('private fixture', encoding='utf-8')
        roots.append(run)
    result, _ = _run(options)
    assert result.exit_code == 130
    assert not roots[0].logs.exists()
    assert roots[0].manifest.is_file() and roots[0].summary.is_file()
    assert roots[1].logs.is_dir() and roots[2].logs.is_dir()
    assert result.run.params_inputs.joinpath('robot.yaml').exists()


def test_plan_identity_mismatch_rejects_before_bundle_write(tmp_path):
    """A plan cannot silently redirect the caller-owned run record."""
    def prepare(path):
        plan = _prepare(path)
        return replace(plan, bundle=replace(plan.bundle, robot='wrong_robot'))

    result, _ = _run(_options(tmp_path), prepare=prepare)
    assert result.exit_code == 2
    assert not (result.run.path / 'launch.json').exists()


@pytest.mark.parametrize('field,value', [
    ('mode', 'bad'), ('site', '../site'),
    ('runs_root', Path('/')), ('repo_dir', Path('relative'))])
def test_invalid_options_do_not_acquire_lock(tmp_path, field, value):
    """Malformed selectors fail before file or ownership mutation."""
    options = replace(_options(tmp_path), **{field: value})
    with pytest.raises(ValueError):
        _run(options)
    assert not options.lock_path.exists()


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_actual_parent_signal_finalizes_after_stopping_private_children(tmp_path, signum):
    """Only the self-created wrapper is signaled; its own children must drain."""
    waiting = 'import time; print("waiting fixture", flush=True); time.sleep(60)'
    code = (
        'import sys\n'
        f'sys.path.insert(0, {str(Path(__file__).parent)!r})\n'
        'from pathlib import Path\n'
        'from dataclasses import replace\n'
        'from test_recorded_startup import _options, _commands, _run, StartupGate\n'
        'def commands(plan):\n'
        f'    gate = StartupGate("ready", (sys.executable, "-u", "-c", {waiting!r}))\n'
        '    return replace(_commands(plan), gates=(gate,))\n'
        f'result, output = _run(_options(Path({str(tmp_path)!r})), commands=commands)\n'
        'raise SystemExit(result.exit_code)\n')
    process = subprocess.Popen(
        [sys.executable, '-c', code], start_new_session=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=dict(os.environ, ROS_DOMAIN_ID='99'))
    run = tmp_path / 'runs/20260916_010000_chassis_nogit'
    deadline = time.monotonic() + 15
    try:
        while True:
            path = run / 'logs/ready.log'
            if path.is_file() and b'waiting fixture' in path.read_bytes():
                break
            if process.poll() is not None:
                pytest.fail(process.stdout.read().decode(errors='replace'))
            assert time.monotonic() < deadline, 'private startup did not enter its gate'
            time.sleep(0.02)
        process.send_signal(signum)
        assert process.wait(timeout=15) == 128 + signum
        assert read_manifest(run / 'manifest.yaml')['exit_code'] == 128 + signum
        assert (run / 'summary.md').is_file()
        for event in read_events(run / 'events.jsonl')[0]:
            if event.event_type == 'process_started':
                assert not Path('/proc', str(event.data['pid'])).exists()
        with RunLock(tmp_path / 'lease'):
            pass
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=20)
        process.stdout.close()


@pytest.mark.parametrize('missing', [False, True])
def test_localization_link_is_read_only_and_not_a_startup_prerequisite(
        tmp_path, monkeypatch, successful_lifecycle, missing):
    """Snapshot failure only warns; excerpts target this run, never the provider."""
    options = replace(_options(tmp_path), mode='tour', site='offline')
    calls = []

    def prepare(path):
        request = UpRequest('tour', 'example_diffdrive', 'offline', path, no_executor=True)
        own = ProcessInfo(os.getpid(), os.getppid(), 1.0, ('navrun',), os.getuid())
        plan = plan_up(request, discover_inputs(request, lambda package: SOURCE / package),
                       schema.load_catalog(schema.find_source_schemas(SOURCE)), (own,), own.pid,
                       lambda package, name: '/fake/' + name)
        artifacts = dict(plan.bundle.artifacts)
        robot = yaml.safe_load(artifacts['params/inputs/robot.yaml'])
        robot['localization']['process_name'] = 'fixture_localization'
        robot['localization']['log_globs'] = [str(tmp_path / 'provider_{pid}.log')]
        artifacts['params/inputs/robot.yaml'] = yaml.safe_dump(robot).encode()
        return replace(plan, bundle=replace(plan.bundle, artifacts=tuple(artifacts.items())))

    def snapshot(spec):
        calls.append(('snapshot', spec.executable))
        if missing:
            raise OSError('fixture provider unreadable')
        return startup.linked_process.LinkedProcess(
            123, (), None, ('fixture_localization',), {}, {})

    def collect(spec, linked, directory, since):
        calls.append(('collect', linked.pid if linked else None))
        assert directory == options.runs_root.joinpath('latest').resolve() / 'logs/loc'
        assert since == START.timestamp()
        return []

    monkeypatch.setattr(startup.linked_process, 'snapshot', snapshot)
    monkeypatch.setattr(startup.linked_process, 'collect_logs', collect)
    result, _ = _run(options, prepare=prepare)
    assert result.exit_code == 130
    assert calls == [('snapshot', 'fixture_localization'), ('collect', None if missing else 123)]
    recorded = read_manifest(result.run.manifest)['linked']['loc']
    assert recorded is None if missing else recorded['pid'] == 123
