"""``navrun wrap``: record a run around an existing launch command.

The wrapped command stays in charge (refactor plan §16.2 Phase 1). It keeps
the terminal's standard input and the wrapper's process group, so Ctrl-C and
keyboard input reach it exactly as before. Around it the wrapper

1. creates the run directory and writes the manifest and ``code.diff``,
2. prunes old runs with the retention policy (never the new run),
3. tees the command's output to the terminal and ``logs/console.log``,
4. optionally runs ``navrun monitor`` next to the command in its own session,
   tees its output the same way and stops it after the command exits,
5. samples CPU and memory of the command and the monitor,
6. records process and signal events, and
7. finalizes the manifest and writes ``summary.md`` when the command exits.

The wrapper exits with the command's exit status; a command killed by signal
``N`` yields ``128 + N`` like a shell does. A failing monitor never changes
that status; it is recorded as a warning event.
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

from nav_bringup.navrun import event_tail
from nav_bringup.recorder import linked_process
from nav_bringup.recorder import manifest as manifest_lib
from nav_bringup.recorder import retention
from nav_bringup.recorder import run_dir as run_dir_lib
from nav_bringup.recorder import sessions
from nav_bringup.recorder import summary as summary_lib
from nav_bringup.recorder import topic_stats
from nav_bringup.recorder.events import EventLog
from nav_bringup.recorder.events import read_events
from nav_bringup.recorder.resources import ResourceSampler
from nav_bringup.recorder.resources import summarize_resources

CONSOLE_LOG_NAME = 'console.log'
RESOURCES_CSV_NAME = 'resources.csv'
EVENT_SOURCE = 'navrun'
DEFAULT_READY_ACTION = '/navigate_to_pose'
MONITOR_STOP_TIMEOUT_SEC = 10.0

_RELAYED_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
_JOIN_TIMEOUT_SEC = 5.0
_READ_CHUNK_BYTES = 65536
_MONITOR_INTERRUPTED_CODES = (0, -int(signal.SIGINT), 128 + int(signal.SIGINT))


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
        monitor_command: ``navrun monitor`` command to run next to the
            command, or ``None``; see :func:`monitor_command`.
        linked: Separately started processes, such as localization, whose
            state is recorded at start and whose logs are excerpted at the end.
        event_flush_command: Optional bounded final publisher after monitor cleanup.
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
    monitor_command: Optional[Sequence[str]] = None
    linked: Sequence[linked_process.LinkSpec] = ()
    event_flush_command: Optional[Sequence[str]] = None


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


def monitor_command(
    base_frame: str,
    topics: Sequence[str] = (),
    period_sec: float = topic_stats.DEFAULT_PERIOD_SEC,
    ready_action: Optional[str] = DEFAULT_READY_ACTION,
    dump_params: bool = True,
    python: str = sys.executable,
) -> List[str]:
    """Build the ``navrun monitor`` command for a wrapped run.

    The monitor finds the run directory through ``NAVRUN_RUN_DIR``, which the
    wrapper exports to it.

    Args:
        base_frame: Robot frame for pose and TF lag (robot profile ``frames.nav_base``).
        topics: Topics whose rates are recorded.
        period_sec: Seconds between ``topics.csv`` samples.
        ready_action: Action whose server marks the stack as ready, or ``None``.
        dump_params: Whether to dump effective parameters once ``ready_action``
            has a server.
        python: Interpreter that runs ``navrun``.

    Returns:
        Program and arguments.
    """
    command = [python, '-m', 'nav_bringup.navrun.cli', 'monitor',
               '--base-frame', base_frame, '--period', f'{period_sec:g}']
    for topic in topics:
        command.extend(['--topic', topic])
    if ready_action:
        command.extend(['--ready-action', ready_action])
        if dump_params:
            command.append('--dump-params-on-ready')
    return command


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
    linked = {spec.name: linked_process.snapshot(spec) for spec in options.linked}
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
        linked={name: state.to_dict() if state else None for name, state in linked.items()},
    )
    manifest_lib.write_manifest(run.manifest, record)

    events = EventLog(run.events)
    events.append(EVENT_SOURCE, 'run_started', data={'command': list(options.command)})
    for name, state in linked.items():
        events.append(EVENT_SOURCE, 'linked_process', 'info' if state else 'warn', {
            'name': name, 'found': state is not None, 'pid': state.pid if state else None})
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
    for spec in options.linked:
        copies = linked_process.collect_logs(
            spec, linked[spec.name], run.logs / spec.name, since=_naive_local(started).timestamp())
        events.append(EVENT_SOURCE, 'linked_logs_collected', data={
            'name': spec.name,
            'files': [item.destination for item in copies],
            'bytes': sum(item.copied_bytes for item in copies),
            'truncated': [item.source for item in copies if item.truncated],
        })
    events.append(EVENT_SOURCE, 'run_stopped', data={
        'exit_code': exit_code, 'exit_reason': exit_reason})
    event_tail.flush_tail(options.event_flush_command or (), run.path, child_env, events)

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
        """记录所收信号，仅把非 SIGINT 信号转交仍存活的直接子进程.

        终端 Ctrl+C 已发给共享前台进程组，因此不重复转发 SIGINT；
        这里不是对子进程树逐个发信号的接口。

        Args:
            signum: 收到的信号编号。
            frame: 信号处理器提供的栈帧，此处不使用。

        Raises:
            OSError: 子进程状态检查或信号转交失败。
        """
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
    """包装一个前台命令并观察其输出和资源，沿用既有有界线程收尾策略.

    Args:
        options: 已校验的命令及采样/监测选项。
        run: 本次运行记录目录。
        child_env: 显式传给子进程的环境。
        sinks: 输出接收器列表，None 项忽略。
        relay: 只绑定本次主子进程的信号转发器。
        events: 本次运行事件日志。

    Returns:
        规范退出码与原因；主命令无法启动时返回 127/start_failed。
        tee/采样线程的有界 join 不构成任意后代全部退出的证明。

    Raises:
        Exception: 记录、观察器创建或收尾失败；这里不是按名称清理外部栈的接口。
    """
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
        tee_thread = threading.Thread(target=_tee, args=(child.stdout, outputs), daemon=True)
        tee_thread.start()
        monitor, monitor_tee = _start_monitor(options, child_env, outputs, events)
        sampled = [child.pid] + ([monitor.pid] if monitor is not None else [])
        stop_sampling = threading.Event()
        sampler = ResourceSampler(lambda: sampled, run.metrics / RESOURCES_CSV_NAME)
        sampler_thread = threading.Thread(
            target=sampler.run, args=(stop_sampling, options.sample_interval_sec), daemon=True)
        sampler_thread.start()

        return_code = child.wait()
        tee_thread.join(timeout=_JOIN_TIMEOUT_SEC)
        _stop_monitor(monitor, monitor_tee, events)
        stop_sampling.set()
        sampler_thread.join(timeout=_JOIN_TIMEOUT_SEC)

    events.append(EVENT_SOURCE, 'process_exited', data={'return_code': return_code})
    return _exit_status(return_code, relay.received)


def _start_monitor(
    options: WrapOptions,
    child_env: Mapping[str, str],
    outputs: Sequence[Any],
    events: EventLog,
) -> Tuple[Optional[subprocess.Popen], Optional[threading.Thread]]:
    """在独立会话启动可选监测进程，保留其句柄与输出线程供外层收尾.

    Args:
        options: 包含可选 monitor_command 的包装配置。
        child_env: 监测进程环境。
        outputs: 共享字节输出接收器。
        events: 启动结果事件记录器。

    Returns:
        自有进程和 tee 线程；未配置或进程创建 OSError 时返回两个 None，
        启动失败记录警告，不改变主命令策略。

    Raises:
        Exception: 输出、事件记录或线程创建失败。
    """
    if not options.monitor_command:
        return None, None
    command = list(options.monitor_command)
    try:
        process = subprocess.Popen(
            command,
            env=dict(child_env),
            start_new_session=True,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
        )
    except OSError as error:
        _write_all(outputs, f'navrun: cannot start the monitor: {error}\n'.encode())
        events.append(EVENT_SOURCE, 'monitor_start_failed', 'warn', {'error': str(error)})
        return None, None
    events.append(EVENT_SOURCE, 'monitor_started', data={'pid': process.pid, 'command': command})
    thread = threading.Thread(target=_tee, args=(process.stdout, outputs), daemon=True)
    thread.start()
    return process, thread


def _stop_monitor(
    process: Optional[subprocess.Popen],
    thread: Optional[threading.Thread],
    events: EventLog,
) -> None:
    """关闭本次持有的监测会话，按是否提前退出记录严重程度.

    Args:
        process: _start_monitor 创建的进程句柄；None 时无需清理。
        thread: 对应 tee 线程，或 None；只按既有预算等待。
        events: 监测结束事件接收器。

    Raises:
        Exception: 会话关闭、线程等待或事件记录失败。
    """
    if process is None:
        return
    exited_early = process.poll() is not None
    return_code = sessions.stop_session(process, MONITOR_STOP_TIMEOUT_SEC)
    if thread is not None:
        thread.join(timeout=_JOIN_TIMEOUT_SEC)
    expected = not exited_early and return_code in _MONITOR_INTERRUPTED_CODES
    events.append(EVENT_SOURCE, 'monitor_stopped', 'info' if expected else 'warn', {
        'return_code': return_code, 'exited_early': exited_early})


def _exit_status(return_code: int, received: Sequence[int]) -> Tuple[int, str]:
    """规范化子进程退出码，并按已收信号优先判定运行结束原因.

    Args:
        return_code: Popen 退出状态，负数表示被信号终止。
        received: 包装器记录的信号序列。

    Returns:
        shell 风格退出码与原因；SIGINT 优先于 TERM/HUP，负退出值转 128+信号号。
    """
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
    """按保留策略规划并执行旧运行记录清理，始终保护本次 run_id.

    Args:
        options: 指定记录根目录和保留策略的包装选项。
        run: 本次运行，不能作为候选被删除。
        now: 用于计算保留期限的日期时间。
        events: 实际清理后记录目标清单与释放字节数的事件接收器。

    本函数会调用 apply_gc，并非预览；调用方须先确认清理授权与目标范围。

    Raises:
        Exception: 计划、清理或记录失败；清理可能已经部分完成。
    """
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
