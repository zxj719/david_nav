"""``navrun wrap``: record a run around an existing launch command.

The wrapped command stays in charge (refactor plan §16.2 Phase 1). It keeps
the terminal's standard input and the wrapper's process group, so Ctrl-C and
keyboard input reach it exactly as before. Around it the wrapper

1. creates the run directory and writes the manifest and ``code.diff``,
2. prunes old runs with the retention policy (never the new run),
3. tees the command's output to the terminal and ``logs/console.log``,
4. samples CPU and memory of the command's process tree,
5. records process and signal events, and
6. finalizes the manifest and writes ``summary.md`` when the command exits.

The wrapper exits with the command's exit status; a command killed by signal
``N`` yields ``128 + N`` like a shell does.
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
import datetime
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
from typing import Any, Callable, Dict, IO, List, Mapping, Optional, Sequence, Tuple

from nav_bringup.recorder import manifest as manifest_lib
from nav_bringup.recorder import retention
from nav_bringup.recorder import run_dir as run_dir_lib
from nav_bringup.recorder import summary as summary_lib
from nav_bringup.recorder import topic_stats
from nav_bringup.recorder.events import EventLog
from nav_bringup.recorder.events import read_events
from nav_bringup.recorder.resources import ResourceSampler
from nav_bringup.recorder.resources import summarize_resources

CONSOLE_LOG_NAME = 'console.log'
RESOURCES_CSV_NAME = 'resources.csv'
EVENT_SOURCE = 'navrun'

_RELAYED_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
_JOIN_TIMEOUT_SEC = 5.0
_READ_CHUNK_BYTES = 65536


@dataclass(frozen=True)
class WrapOptions:
    """Settings of one wrapped run.

    Attributes:
        command: Program and arguments to run.
        mode: Run mode used in the run id, for example ``tour``.
        runs_root: Directory that holds all runs.
        repo_dir: Repository whose git state is recorded.
        alias: Shell alias that started the run; ``$NAVRUN_ALIAS`` when unset.
        robot: Robot profile name, if known.
        site: Site profile name, if known.
        files: Named input files to fingerprint in the manifest.
        sample_interval_sec: Seconds between resource samples.
        gc: Whether to apply the retention policy before the command starts.
        retention_policy: Limits used when ``gc`` is enabled.
    """

    command: Sequence[str]
    mode: str
    runs_root: Path
    repo_dir: Path
    alias: Optional[str] = None
    robot: Optional[str] = None
    site: Optional[str] = None
    files: Mapping[str, Path] = field(default_factory=dict)
    sample_interval_sec: float = 1.0
    gc: bool = True
    retention_policy: retention.RetentionPolicy = field(
        default_factory=retention.RetentionPolicy)


@dataclass(frozen=True)
class WrapResult:
    """Outcome of a wrapped run.

    Attributes:
        run: The run directory.
        exit_code: Status to exit with; ``128 + N`` if the command died from signal N.
        exit_reason: One of ``completed``, ``failed``, ``killed_by_signal``,
            ``ctrl_c``, ``terminated`` or ``start_failed``.
    """

    run: run_dir_lib.RunDirectory
    exit_code: int
    exit_reason: str


def run_wrapped(
    options: WrapOptions,
    environ: Optional[Mapping[str, str]] = None,
    now: Callable[[], datetime.datetime] = datetime.datetime.now,
    runner: manifest_lib.CommandRunner = manifest_lib.run_command,
    terminal: Optional[IO[bytes]] = None,
) -> WrapResult:
    """Run a command inside a recorded run.

    Args:
        options: What to run and how to record it.
        environ: Environment for the manifest and the command; defaults to
            ``os.environ``.
        now: Clock for the start and end times, called exactly twice.
        runner: Command runner for git queries, replaceable in tests.
        terminal: Binary stream that receives the command's output; defaults
            to the wrapper's standard output.

    Returns:
        The run directory, the exit code and the exit reason.
    """
    env = dict(os.environ if environ is None else environ)
    started = now()
    git = manifest_lib.collect_git_info(options.repo_dir, runner)
    run_id = run_dir_lib.make_run_id(options.mode, git.commit, started)
    run = run_dir_lib.create_run_directory(options.runs_root, run_id)
    if git.dirty:
        manifest_lib.write_code_diff(options.repo_dir, run.code_diff, runner)
    record = manifest_lib.build_manifest(
        run_id=run_id,
        mode=options.mode,
        command=options.command,
        started_at=started,
        git=git,
        versions=manifest_lib.collect_versions(),
        environment=manifest_lib.collect_environment(env),
        files=options.files,
        robot=options.robot,
        site=options.site,
        alias=options.alias or env.get('NAVRUN_ALIAS'),
    )
    manifest_lib.write_manifest(run.manifest, record)

    events = EventLog(run.events)
    events.append(EVENT_SOURCE, 'run_started', data={'command': list(options.command)})
    if options.gc:
        _collect_garbage(options, run, _naive_local(started), events)

    child_env = dict(env)
    child_env.update({
        'PYTHONUNBUFFERED': '1',
        'NAVRUN_RUN_ID': run_id,
        'NAVRUN_RUN_DIR': str(run.path),
    })
    sinks: List[Any] = [terminal if terminal is not None else getattr(sys.stdout, 'buffer', None)]
    relay = _SignalRelay()
    relay.install()
    try:
        exit_code, exit_reason = _supervise(options, run, child_env, sinks, relay, events)
    finally:
        relay.restore()

    if relay.received:
        events.append(EVENT_SOURCE, 'signal_received', 'warn', {
            'signals': [signal.Signals(signum).name for signum in relay.received]})
    events.append(EVENT_SOURCE, 'run_stopped', data={
        'exit_code': exit_code, 'exit_reason': exit_reason})

    finalized = manifest_lib.finalize_manifest(record, now(), exit_code, exit_reason)
    manifest_lib.write_manifest(run.manifest, finalized)
    run_events, skipped = read_events(run.events)
    usage = summarize_resources(run.metrics / RESOURCES_CSV_NAME)
    topics = topic_stats.summarize_topics(run.metrics / topic_stats.TOPICS_CSV_NAME)
    summary_lib.write_summary(
        run.summary, summary_lib.render_summary(finalized, run_events, usage, skipped, topics))
    return WrapResult(run=run, exit_code=exit_code, exit_reason=exit_reason)


class _SignalRelay:
    """Keeps the wrapper alive on signals and forwards termination to the child.

    SIGINT from a terminal is delivered to the whole foreground process group,
    so the child already has it; SIGTERM and SIGHUP are sent to the wrapper
    only and must be forwarded.
    """

    def __init__(self) -> None:
        self.child: Optional[subprocess.Popen] = None
        self.received: List[int] = []
        self._previous: Dict[int, Any] = {}

    def install(self) -> None:
        if threading.current_thread() is not threading.main_thread():
            return
        for signum in _RELAYED_SIGNALS:
            self._previous[signum] = signal.signal(signum, self._handle)

    def restore(self) -> None:
        for signum, handler in self._previous.items():
            signal.signal(signum, handler)
        self._previous.clear()

    def _handle(self, signum: int, frame: Any) -> None:
        self.received.append(signum)
        child = self.child
        if signum != signal.SIGINT and child is not None and child.poll() is None:
            child.send_signal(signum)


def _supervise(
    options: WrapOptions,
    run: run_dir_lib.RunDirectory,
    child_env: Mapping[str, str],
    sinks: List[Any],
    relay: _SignalRelay,
    events: EventLog,
) -> Tuple[int, str]:
    with (run.logs / CONSOLE_LOG_NAME).open('ab') as console_log:
        outputs = [sink for sink in sinks if sink is not None] + [console_log]
        try:
            child = subprocess.Popen(
                list(options.command),
                env=dict(child_env),
                stderr=subprocess.STDOUT,
                stdout=subprocess.PIPE,
            )
        except OSError as error:
            message = f'navrun: cannot start {options.command[0]!r}: {error}\n'.encode()
            _write_all(outputs, message)
            events.append(EVENT_SOURCE, 'start_failed', 'error', {'error': str(error)})
            return 127, 'start_failed'

        relay.child = child
        events.append(EVENT_SOURCE, 'process_started', data={'pid': child.pid})
        stop_sampling = threading.Event()
        sampler = ResourceSampler(lambda: [child.pid], run.metrics / RESOURCES_CSV_NAME)
        sampler_thread = threading.Thread(
            target=sampler.run, args=(stop_sampling, options.sample_interval_sec), daemon=True)
        tee_thread = threading.Thread(target=_tee, args=(child.stdout, outputs), daemon=True)
        sampler_thread.start()
        tee_thread.start()

        return_code = child.wait()
        tee_thread.join(timeout=_JOIN_TIMEOUT_SEC)
        stop_sampling.set()
        sampler_thread.join(timeout=_JOIN_TIMEOUT_SEC)

    events.append(EVENT_SOURCE, 'process_exited', data={'return_code': return_code})
    return _exit_status(return_code, relay.received)


def _exit_status(return_code: int, received: Sequence[int]) -> Tuple[int, str]:
    exit_code = 128 - return_code if return_code < 0 else return_code
    if signal.SIGINT in received:
        return exit_code, 'ctrl_c'
    if signal.SIGTERM in received or signal.SIGHUP in received:
        return exit_code, 'terminated'
    if return_code == 0:
        return exit_code, 'completed'
    if return_code < 0:
        return exit_code, 'killed_by_signal'
    return exit_code, 'failed'


def _collect_garbage(
    options: WrapOptions,
    run: run_dir_lib.RunDirectory,
    now: datetime.datetime,
    events: EventLog,
) -> None:
    plan = retention.plan_gc(
        run_dir_lib.list_runs(options.runs_root),
        options.retention_policy,
        now,
        protected_run_id=run.run_id,
    )
    if not plan.prune_runs and not plan.delete_bags:
        return
    freed = retention.apply_gc(plan, options.runs_root)
    events.append(EVENT_SOURCE, 'gc_applied', data={
        'pruned_runs': [item.run_id for item in plan.prune_runs],
        'deleted_bags': [str(path) for path in plan.delete_bags],
        'freed_bytes': freed,
    })


def _naive_local(value: datetime.datetime) -> datetime.datetime:
    if value.tzinfo is None:
        return value
    return value.astimezone().replace(tzinfo=None)


def _tee(source: IO[bytes], outputs: Sequence[Any]) -> None:
    read = getattr(source, 'read1', source.read)
    while True:
        chunk = read(_READ_CHUNK_BYTES)
        if not chunk:
            return
        _write_all(outputs, chunk)


def _write_all(outputs: Sequence[Any], data: bytes) -> None:
    for output in outputs:
        try:
            output.write(data)
            output.flush()
        except (OSError, ValueError):
            continue
