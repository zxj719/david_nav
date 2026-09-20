"""``navrun wrap``：在既有启动命令外面套一层运行记录.

被包裹的命令仍然是主角（方案 §16.2 阶段一）：它保留终端的标准输入和包装器的进程组，
因此 Ctrl-C 与键盘输入和以前一样直接到它。包装器只在外围做这些事：

1. 建立运行目录，写清单与 ``code.diff``；
2. 按保留策略清理旧运行（绝不清理本次新建的这条）；
3. 把命令输出同时送到终端和 ``logs/console.log``；
4. 可选地在命令旁边、以独立会话运行 ``navrun monitor``，输出同样分流，命令退出后停掉它；
5. 采样命令与 monitor 的 CPU 和内存；
6. 记录进程与信号事件；
7. 命令退出后收尾清单并写 ``summary.md``。

包装器以命令自身的退出状态退出；命令被信号 ``N`` 杀死时按 shell 惯例返回 ``128 + N``。
monitor 失败绝不改变这个状态，只作为告警事件记录——也就是说记录不全时命令仍算成功。
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
    """一次被包裹运行的设置.

    Attributes:
        command: 要运行的程序与参数。
        mode: 运行模式，构成运行 ID 的一部分，例如 ``tour``。
        runs_root: 存放所有运行记录的目录。
        repo_dir: 要记录 git 状态的仓库。
        alias: 启动本次运行的 shell 别名；未给出时取 ``$NAVRUN_ALIAS``。
        robot: 已知时的机器人档案名。
        site: 已知时的场地档案名。
        files: 需要在清单里记指纹的具名输入文件。
        sample_interval_sec: 资源采样间隔秒数。
        gc: 命令启动前是否执行保留策略。
        retention_policy: 启用 ``gc`` 时使用的上限。
        monitor_command: 与命令并行运行的 ``navrun monitor`` 命令，或 ``None``；
            见 :func:`monitor_command`。
        linked: 另行启动的进程（例如定位），只在开始时记录其状态、结束时摘录其日志，
            包装器不启动也不停止它们。
        event_flush_command: monitor 清理之后可选的有界末尾发布命令。
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
    """一次被包裹运行的结果.

    Attributes:
        run: 运行目录。
        exit_code: 应当用来退出的状态；命令被信号 N 杀死时为 ``128 + N``。
        exit_reason: ``completed``、``failed``、``killed_by_signal``、``ctrl_c``、
            ``terminated`` 或 ``start_failed`` 之一；它描述命令怎么结束的，
            不描述导航任务成没成功。
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
    """为一次被包裹运行构造 ``navrun monitor`` 命令.

    monitor 通过包装器导出的 ``NAVRUN_RUN_DIR`` 找到运行目录。本函数只拼命令，
    不启动它。

    Args:
        base_frame: 求位姿与 TF 延迟用的机器人帧（机器人档案的 ``frames.nav_base``）。
        topics: 需要记录频率的话题。
        period_sec: ``topics.csv`` 的采样间隔秒数。
        ready_action: 其服务端出现即视为整栈就绪的动作，或 ``None``。
        dump_params: ``ready_action`` 有服务端之后是否转储实际生效参数。
        python: 运行 ``navrun`` 的解释器。

    Returns:
        程序与参数。
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
    """在一条运行记录内执行命令.

    记录失败不会改变命令的退出状态，只会被记为告警。

    Args:
        options: 要运行什么、怎么记录。
        environ: 清单与命令使用的环境；默认取 ``os.environ``。
        now: 起止时间所用的时钟，恰好被调用两次。
        runner: git 查询所用的命令执行器，测试可替换。
        terminal: 接收命令输出的二进制流；默认是包装器自己的标准输出。

    Returns:
        运行目录、退出码与结束原因。
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
    """收到信号时让包装器继续活着，并把终止信号转发给子进程.

    终端发出的 SIGINT 会送达整个前台进程组，子进程已经收到了，包装器不必重复转发；
    SIGTERM 与 SIGHUP 只发给包装器，必须由它转发下去。包装器坚持活到收尾写完为止，
    因此"包装器还在"不代表被包裹的命令还在运行。
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
    """统一到本地无时区时间，供既有运行保留策略比较.

    无时区输入原样返回；有时区输入先换算本地时区再移除时区信息，
    不是直接丢弃原时区偏移。

    Args:
        value: 运行开始等日历时间。

    Returns:
        表示本地时间的无时区 datetime。
    """
    if value.tzinfo is None:
        return value
    return value.astimezone().replace(tzinfo=None)


def _tee(source: IO[bytes], outputs: Sequence[Any]) -> None:
    """持续读取子进程字节输出并分发，优先使用 read1 避免等待填满缓冲.

    空读表示 EOF；不解码文本或关闭借入的流，输出失败处理由 _write_all 负责。

    Args:
        source: 已打开的二进制输入流。
        outputs: 要复制到的二进制输出流序列。

    Raises:
        Exception: 输入读取或未被输出助手处理的写入异常。
    """
    read = getattr(source, 'read1', source.read)
    while True:
        chunk = read(_READ_CHUNK_BYTES)
        if not chunk:
            return
        _write_all(outputs, chunk)


def _write_all(outputs: Sequence[Any], data: bytes) -> None:
    """逐个写入并刷新输出流，单个终端或日志关闭不阻断其它输出.

    OSError 和 ValueError 按既有策略忽略；不是可靠持久化确认，
    也不补写 write 返回的部分字节。

    Args:
        outputs: 当前需要接收字节的输出流序列。
        data: 原样分发的字节块。

    Raises:
        Exception: 输出流抛出未被忽略的其它异常。
    """
    for output in outputs:
        try:
            output.write(data)
            output.flush()
        except (OSError, ValueError):
            continue
