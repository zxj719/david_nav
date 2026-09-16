"""Lock-scoped recorded startup, including partial failures and cleanup retries.

Host operations and command binding are explicit callbacks: importing this module
does not configure hardware, discover ROS, clean DDS, or start a process. The live
CLI supplies these bindings explicitly; tests substitute private or fake adapters.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import datetime
import json
import os
from pathlib import Path
import re
import signal
import sys
import tempfile
import threading
import time
from typing import Callable, IO, Mapping, Optional, Sequence, Tuple

import yaml

from nav_bringup.navcfg import render
from nav_bringup.navrun import event_tail
from nav_bringup.navrun.locking import RunLock
from nav_bringup.navrun.keyboard_session import KeyboardSession
from nav_bringup.navrun.preparation import write_bundle
from nav_bringup.navrun.startup_lifecycle import LifecycleResult, StartupCommands
from nav_bringup.navrun.startup_lifecycle import supervise, validate_commands
from nav_bringup.navrun.startup_observer import StartupObserver
from nav_bringup.navrun.up_planning import UpPlan
from nav_bringup.recorder import manifest as manifest_lib
from nav_bringup.recorder import linked_process, retention, run_dir as run_dir_lib
from nav_bringup.recorder.events import EventLog, read_events


@dataclass(frozen=True)
class StartupOptions:
    """Run identity and recording destinations selected by the outer CLI.

    Attributes:
        mode: Launch mode and run-id component.
        robot: Expected robot identity in the prepared bundle.
        site: Expected site, or none for chassis tools.
        runs_root: Absolute root for new run records and retention.
        repo_dir: Explicit repository to fingerprint, never an implicit production path.
        lock_path: Navigation lease; tests must use a private path.
        command: Original user-facing invocation to record.
        gc: Apply retention before launch, preserving this incomplete run.
        retention_policy: Limits for previously completed records.
    """

    mode: str
    robot: str
    site: str
    runs_root: Path
    repo_dir: Path
    lock_path: Path
    command: Tuple[str, ...] = ()
    gc: bool = True
    retention_policy: retention.RetentionPolicy = field(default_factory=retention.RetentionPolicy)


@dataclass(frozen=True)
class StartupStep:
    """One externally bound startup operation after command validation.

    Attributes:
        name: Unique event label, such as platform, preflight or dds.
        execute: Apply this step; receive the plan, event writer and signal probe.
            It must observe interruption, own any temporary resources, and raise
            on failure. Applied host changes are recorded, not implicitly rolled back.
    """

    name: str
    execute: Callable[[UpPlan, EventLog, Callable[[], Optional[int]]], None]


@dataclass(frozen=True)
class RecordedStartupResult:
    """A finalized record whose owned child groups have all drained.

    Attributes:
        run: Completed run directory, including failed startup evidence.
        exit_code: Shell-compatible result of startup and cleanup.
        exit_reason: Stage, signal or failure that ended the run.
    """

    run: run_dir_lib.RunDirectory
    exit_code: int
    exit_reason: str


class _Signals:
    def __init__(self):
        self.received = None
        self.previous = {}

    def __enter__(self):
        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError('startup supervision must run on the main thread')
        try:
            for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                self.previous[number] = signal.signal(number, self._handle)
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def _handle(self, number, _frame):
        if self.received is None:
            self.received = number

    def __call__(self):
        return self.received

    def __exit__(self, *args):
        for number, handler in self.previous.items():
            signal.signal(number, handler)


class StartupInterrupted(Exception):
    """An explicit shutdown request observed by a startup operation."""

    def __init__(self, signum: int) -> None:
        """Preserve the first signal even across a host-operation boundary.

        Args:
            signum: Signal number received by the owning supervisor.
        """
        self.signum = signum
        super().__init__(f'startup interrupted by signal {signum}')


def run_recorded_startup(
    options: StartupOptions, prepare: Callable[[Path], UpPlan],
    commands: Callable[[UpPlan], StartupCommands], steps: Sequence[StartupStep] = (),
    environ: Optional[Mapping[str, str]] = None,
    now: Callable[[], datetime.datetime] = datetime.datetime.now,
    runner: manifest_lib.CommandRunner = manifest_lib.run_command,
    terminal: Optional[IO[bytes]] = None,
) -> RecordedStartupResult:
    """Record and execute startup while retaining ownership through finalization.

    A new manifest precedes configuration preparation. Plan and command errors
    prevent host operations; every completed host step is recorded. SIGINT,
    SIGTERM and SIGHUP only request shutdown, including during cleanup. Remaining
    owned process groups are retried under the lock without a false completion
    deadline. Final manifest/summary are written only after all groups drain.
    No retries create new children or broaden signal targets.

    Args:
        options: Explicit identity, repository, lease and recording settings.
        prepare: Build the immutable plan for the newly created absolute run path.
        commands: Bind all runtime commands before any host mutation.
        steps: Ordered explicit host operations; empty means none are applied.
        environ: Inherited environment copied for recording and child processes.
        now: Wall clock used for run identity and duration.
        runner: Read-only git command runner, replaceable by tests.
        terminal: Binary console sink; defaults to stdout when available.

    Returns:
        The finalized record and exit status; runtime failures are recorded.

    Raises:
        ValueError: Top-level settings are invalid before ownership is acquired.
        RuntimeError: Called outside the main thread or recording cannot stop.
        OSError: Lock or run creation, initial recording or finalization fails.
        LockBusy: Another run owns the requested lease.
    """
    _validate(options, steps)
    env = dict(os.environ if environ is None else environ)
    output = terminal if terminal is not None else getattr(sys.stdout, 'buffer', None)
    with _Signals() as stopped, RunLock(options.lock_path) as lease:
        started = now()
        git = manifest_lib.collect_git_info(options.repo_dir, runner)
        run_id = run_dir_lib.make_run_id(options.mode, git.commit, started)
        run = run_dir_lib.create_run_directory(options.runs_root, run_id)
        env.update(ROS_LOG_DIR=str(run.logs / 'ros'), NAVRUN_RUN_DIR=str(run.path),
                   NAVRUN_RUN_ID=run_id, PYTHONUNBUFFERED='1')
        (run.logs / 'ros').mkdir()
        record = manifest_lib.build_manifest(
            run_id=run_id, mode=options.mode, robot=options.robot, site=options.site,
            command=options.command, started_at=started, git=git,
            versions=manifest_lib.collect_versions(),
            environment=manifest_lib.collect_environment(env), alias=env.get('NAVRUN_ALIAS'))
        _write_manifest(run.manifest, record)
        events = EventLog(run.events)
        observer = StartupObserver(run, output)
        result = LifecycleResult(2, 'prepare_failed')
        stage = 'prepare'
        linked = None
        link_spec = None
        keyboard = None
        final_events = ()
        try:
            events.append('navrun', 'run_started')
            observer.start()
            _check_signal(stopped)
            if git.dirty:
                manifest_lib.write_code_diff(options.repo_dir, run.code_diff, runner)
            if options.gc:
                _gc(options, run, started, events)
            plan = prepare(run.path)
            _check_plan(options, plan, run.path)
            write_bundle(plan.bundle)
            link_spec = _link_spec(plan)
            if link_spec is not None:
                try:
                    linked = linked_process.snapshot(link_spec)
                except (OSError, ValueError) as error:
                    events.append('navrun', 'linked_snapshot_failed', 'warn',
                                  {'error': str(error)})
                record['linked'] = {'loc': linked.to_dict() if linked else None}
                events.append('navrun', 'linked_process', 'info' if linked else 'warn',
                              {'name': 'loc', 'found': linked is not None})
            record['files'] = json.loads(dict(plan.bundle.artifacts)[
                'params/inputs/fingerprints.json'])
            record['launch'] = json.loads(dict(plan.bundle.artifacts)['launch.json'])
            _write_manifest(run.manifest, record)
            selected = commands(plan)
            validate_commands(selected, run.path)
            if selected.keyboard:
                stage = 'terminal'
                keyboard = KeyboardSession(lease, selected.tty_device).__enter__()
            for step in steps:
                _check_signal(stopped)
                stage = step.name
                events.append('navrun', 'startup_step_started', data={'name': stage})
                step.execute(plan, events, stopped)
                events.append('navrun', 'startup_step_finished', data={'name': stage})
            _check_signal(stopped)
            stage = 'launch'
            final_events = selected.event_flush
            if keyboard is None:
                result = supervise(selected, run.path, events, env, lease, stopped)
            else:
                result = supervise(selected, run.path, events, env, lease, stopped,
                                   keyboard=keyboard)
        except StartupInterrupted as interrupted:
            result = LifecycleResult(128 + interrupted.signum,
                                     'ctrl_c' if interrupted.signum == signal.SIGINT
                                     else 'terminated')
        except KeyboardInterrupt:
            result = LifecycleResult(130, 'ctrl_c')
        except Exception as error:
            result = LifecycleResult(2, stage + '_failed')
            _event(events, 'startup_failed', 'error', {'stage': stage, 'error': str(error)})
        finally:
            _drain_remaining(result, events)
            if keyboard is not None:
                try:
                    keyboard.close()
                    _event(events, 'terminal_restored', 'info', {})
                except Exception as error:
                    result = LifecycleResult(2, 'terminal_restore_failed')
                    _event(events, 'terminal_restore_failed', 'error', {'error': str(error)})
            observer.close()
        for error in observer.errors:
            _event(events, 'record_observer_failed', 'warn', {'error': error})
        if link_spec is not None:
            try:
                copied = linked_process.collect_logs(
                    link_spec, linked, run.logs / 'loc', since=started.timestamp())
                _event(events, 'linked_logs_collected', 'info', {
                    'files': [item.destination for item in copied],
                    'bytes': sum(item.copied_bytes for item in copied)})
            except (OSError, ValueError) as error:
                _event(events, 'linked_logs_failed', 'warn', {'error': str(error)})
        _event(events, 'run_stopped', 'info', {
            'exit_code': result.exit_code, 'exit_reason': result.exit_reason})
        event_tail.flush_tail(final_events, run.path, env, events)
        finalized = manifest_lib.finalize_manifest(record, now(), result.exit_code,
                                                   result.exit_reason)
        _write_manifest(run.manifest, finalized)
        _summary(run, finalized)
        return RecordedStartupResult(run, result.exit_code, result.exit_reason)


def _validate(options, steps):
    if options.mode not in ('tour', 'precise', 'bench_slamgt', 'chassis', 'teleop'):
        raise ValueError('unknown startup mode')
    for name in (options.robot, options.site):
        if not re.fullmatch('[a-z][a-z0-9_]*', name):
            raise ValueError('invalid robot/site name')
    for path in (options.runs_root, options.repo_dir, options.lock_path):
        if not path.is_absolute() or '..' in path.parts or path == Path('/'):
            raise ValueError('startup paths must be explicit normalized absolute paths')
    names = [step.name for step in steps]
    if len(names) != len(set(names)) or any(not re.fullmatch('[a-z][a-z0-9_]*', n) for n in names):
        raise ValueError('host step names must be safe unique labels')


def _check_plan(options, plan, path):
    if (plan.bundle.options.run_dir != str(path) or plan.bundle.robot != options.robot
            or plan.bundle.site != options.site or plan.bundle.options.mode != options.mode):
        raise ValueError('prepared bundle identity does not match this run')
    if plan.errors:
        raise ValueError('startup plan rejected: ' + '; '.join(plan.errors))


def _check_signal(stopped):
    if stopped() is not None:
        raise StartupInterrupted(stopped())


def _link_spec(plan):
    if plan.bundle.options.mode not in ('tour', 'precise', 'bench_slamgt'):
        return None
    artifacts = dict(plan.bundle.artifacts)
    profile = yaml.safe_load(artifacts['params/inputs/robot.yaml'])
    mode = render.parse_mode(yaml.safe_load(artifacts['params/inputs/mode.yaml']),
                             Path(plan.bundle.options.run_dir) / 'params/inputs/mode.yaml')
    localization = render.deep_merge(profile, mode.profile).get('localization', {})
    executable = localization.get('process_name')
    if not executable:
        return None
    return linked_process.LinkSpec(
        name='loc', executable=executable,
        config_flags=tuple(localization.get('config_flags', ('--config',))),
        log_globs=tuple(localization.get('log_globs', ())))


def _write_manifest(path, record):
    # A failed update must not truncate the previous, readable manifest.
    with tempfile.NamedTemporaryFile(prefix='.manifest-', dir=path.parent,
                                     delete=False) as temporary:
        temporary_path = Path(temporary.name)
    try:
        manifest_lib.write_manifest(temporary_path, record)
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def _event(events, name, severity, data):
    try:
        events.append('navrun', name, severity, data)
    except OSError as error:
        print(f'navrun: event recording failed: {error}', file=sys.stderr)


def _drain_remaining(result, events):
    remaining = list(result.remaining)
    while remaining:
        for child in tuple(remaining):
            try:
                child.close(10.0)
                remaining.remove(child)
                _event(events, 'cleanup_retry_finished', 'info', {'pid': child.pid})
            except Exception as error:
                _event(events, 'cleanup_retry_pending', 'error',
                       {'pid': child.pid, 'error': str(error)})
        if remaining:
            _retry_pause()


def _retry_pause():
    time.sleep(1.0)


def _gc(options, run, started, events):
    complete = []
    for previous in run_dir_lib.list_runs(run.root):
        if previous.run_id == run.run_id:
            continue
        try:
            if manifest_lib.read_manifest(previous.manifest).get('ended_at') is not None:
                complete.append(previous)
        except (OSError, ValueError, yaml.YAMLError):
            continue
    local_time = started.astimezone().replace(tzinfo=None) if started.tzinfo else started
    plan = retention.plan_gc(complete, options.retention_policy, local_time,
                             protected_run_id=run.run_id)
    removed = retention.apply_gc(plan, run.root)
    events.append('navrun', 'gc_finished', data={'removed_bytes': removed})


def _summary(run, finalized):
    from nav_bringup.recorder import summary, topic_stats
    from nav_bringup.recorder.resources import summarize_resources

    events, skipped = read_events(run.events)
    summary.write_summary(run.summary, summary.render_summary(
        finalized, events, summarize_resources(run.metrics / 'resources.csv'), skipped,
        topic_stats.summarize_topics(run.metrics / topic_stats.TOPICS_CSV_NAME)))
