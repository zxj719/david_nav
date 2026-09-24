"""在锁的作用域内记录并执行启动，含部分失败与清理重试.

对主机的操作和命令绑定都是显式回调：仅仅导入本模块不会配置硬件、不做 ROS 发现、
不清理 DDS、也不启动任何进程。真实 CLI 显式提供这些绑定，测试换成私有或假的适配器。
记录是尽力而为的证据，不是事务：已经生效的主机改动只会被记录，不会被自动回滚。
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
from nav_bringup.navrun.control_server import RunControl
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
    """由外层 CLI 选定的运行身份与记录去处.

    Attributes:
        mode: 启动模式，同时作为运行 ID 的组成部分。
        robot: 已备好的运行包里应当出现的机器人身份。
        site: 期望的场地；底盘类工具为空。
        runs_root: 新运行记录与保留策略的绝对根目录。
        repo_dir: 要取指纹的仓库，必须显式给出，绝不隐含指向生产目录。
        lock_path: 导航租约；测试必须使用私有路径。
        command: 需要记录的原始用户命令。
        gc: 启动前先执行保留策略，并保住本次尚未完成的运行。
        retention_policy: 对此前已完成记录的保留上限。
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
    """命令校验之后、由外部绑定的一个启动操作.

    Attributes:
        name: 唯一的事件标签，例如 platform、preflight 或 dds。
        execute: 执行该步骤，入参为计划、事件写入器与信号探针。实现必须自己观察
            中断请求、自己管理临时资源，失败时抛异常。已经生效的主机改动只会被记录，
            不会被隐式回滚。
    """

    name: str
    execute: Callable[[UpPlan, EventLog, Callable[[], Optional[int]]], None]


@dataclass(frozen=True)
class RecordedStartupResult:
    """一份已收尾的记录，其自有子进程组全部已退干净.

    Attributes:
        run: 已完成的运行目录；启动失败时同样保留现场证据。
        exit_code: 启动与清理的最终状态码，语义与 shell 一致。
        exit_reason: 结束本次运行的阶段、信号或失败原因。
    """

    run: run_dir_lib.RunDirectory
    exit_code: int
    exit_reason: str


class _Signals:
    def __init__(self):
        self.received = None
        self.previous = {}
        self.remote_stop = threading.Event()

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
        """只保存首个终止信号，后续信号不覆盖最初的退出原因.

        这里只记录，不在信号回调里清理资源或转发给外部进程。

        Args:
            number: 收到的信号编号。
            _frame: Python 信号处理器传入的栈帧，此处不用。
        """
        if self.received is None:
            self.received = number

    def __call__(self):
        return self.received if self.received is not None else (
            signal.SIGTERM if self.remote_stop.is_set() else None)

    def __exit__(self, *args):
        for number, handler in self.previous.items():
            signal.signal(number, handler)


class StartupInterrupted(Exception):
    """启动操作观察到的一次显式关停请求.

    信号只是"请求"：抛出本异常表示协作式退出已经开始，不代表任何进程已经结束。

    Attributes:
        signum: 导致启动流程协作退出的信号编号。
    """

    def __init__(self, signum: int) -> None:
        """保住第一个信号，跨主机操作边界也不丢失.

        后续再来的信号不会覆盖它，退出原因始终指向最早的那次请求。

        Args:
            signum: 持有方监督进程收到的信号编号。
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
    """记录并执行启动，直到收尾为止始终保持对自有资源的持有.

    先写新清单再准备配置。计划或命令错误会挡在主机操作之前；每个已完成的主机步骤
    都会被记录。SIGINT、SIGTERM、SIGHUP 一律只是请求关停（清理期间也是如此），
    不强杀。残余的自有进程组在锁内反复重试回收，不设"假装已完成"的截止时间；
    最终清单与摘要只在所有进程组都退干净之后才写。重试不会创建新子进程，
    也不会扩大发信号的目标范围——因此本函数不保证能回收不属于本次启动的进程。

    Args:
        options: 显式的身份、仓库、租约与记录设置。
        prepare: 为新建的绝对运行路径构建不可变计划。
        commands: 在任何主机改动之前绑定全部运行时命令。
        steps: 有序的显式主机操作；为空表示一个都不做。
        environ: 供记录与子进程使用的继承环境副本。
        now: 用于运行身份与时长的墙钟。
        runner: 只读的 git 命令执行器，测试可替换。
        terminal: 二进制控制台输出端；可用时默认接标准输出。

    Returns:
        已收尾的记录与退出状态；运行期失败同样被记录下来。

    Raises:
        ValueError: 取得资源归属之前发现顶层设置非法。
        RuntimeError: 不在主线程调用，或记录无法停止。
        OSError: 加锁、创建运行目录、初始记录或收尾失败。
        LockBusy: 请求的租约已被另一次运行持有。
    """
    _validate(options, steps)
    env = dict(os.environ if environ is None else environ)
    output = terminal if terminal is not None else getattr(sys.stdout, 'buffer', None)
    with _Signals() as stopped, RunLock(options.lock_path) as lease, \
            RunControl(lease, stopped.remote_stop.set) as control:
        started = now()
        git = manifest_lib.collect_git_info(options.repo_dir, runner)
        run_id = run_dir_lib.make_run_id(options.mode, git.commit, started)
        run = run_dir_lib.create_run_directory(options.runs_root, run_id)
        control.identify(run_id, options.mode)
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
        observer = StartupObserver(run, output, poll=control.poll)
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
    """验证受管启动身份、绝对路径和唯一主机步骤标签.

    Args:
        options: 含模式、机器人/场地、记录根目录、仓库及锁路径的选项。
        steps: 尚未执行的主机准备步骤。

    Raises:
        ValueError: 名称、模式、路径或步骤标签不符合约束。
    """
    if options.mode not in ('tour', 'precise', 'chassis', 'teleop'):
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
    """拒绝与当前运行身份不匹配或已含错误的准备计划.

    这里只核运行目录字符串、机器人、场地、模式和已有错误列表；
    不替代资产哈希、ROS 就绪或运动安全验证。

    Args:
        options: 当前启动请求中的身份设置。
        plan: 已准备的启动计划及其错误列表。
        path: 本次运行目录，用字符串与准备记录比对。

    Raises:
        ValueError: 准备记录身份不一致，或计划含错误。
    """
    if (plan.bundle.options.run_dir != str(path) or plan.bundle.robot != options.robot
            or plan.bundle.site != options.site or plan.bundle.options.mode != options.mode):
        raise ValueError('prepared bundle identity does not match this run')
    if plan.errors:
        raise ValueError('startup plan rejected: ' + '; '.join(plan.errors))


def _check_signal(stopped):
    """把已记录的停止请求转为启动中断异常，不自行发信号.

    停止读取器会被调用两次，应在记录首个信号后保持返回值稳定。

    Args:
        stopped: 返回已记录信号编号或 None 的无参读取函数。

    Raises:
        StartupInterrupted: 已收到停止请求，由外层负责自有资源收尾。
    """
    if stopped() is not None:
        raise StartupInterrupted(stopped())


def _link_spec(plan):
    """从冻结档案构造定位侧只读关联说明，不启动或控制定位提供者.

    Args:
        plan: 已准备的启动计划，含机器人及模式输入快照。

    Returns:
        导航模式且配置了进程名时的 LinkSpec；其它情况为 None。
        仅描述配置参数标记与日志路径规则，不读取日志负载。
    """
    if plan.bundle.options.mode not in ('tour', 'precise'):
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
    """先写同目录临时清单，再替换目标，避免写失败截断旧清单.

    最终清理临时路径；这是替换式发布，不是断电持久性保证，也不阻止并发写入。

    Args:
        path: 运行清单的目标路径，父目录已存在。
        record: 交给清单序列化器的完整记录。

    Raises:
        OSError: 临时文件创建、清单写入、替换或清理失败。
        Exception: 清单序列化失败，尚未执行目标替换。
    """
    with tempfile.NamedTemporaryFile(prefix='.manifest-', dir=path.parent,
                                     delete=False) as temporary:
        temporary_path = Path(temporary.name)
    try:
        manifest_lib.write_manifest(temporary_path, record)
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def _event(events, name, severity, data):
    """尽力追加启动事件，文件 IO 错误改报 stderr 但不改变启动控制流.

    只捕获 OSError，不把所有编程或数据错误悄悄吞掉；
    stderr 报告本身失败仍可能向外传播。

    Args:
        events: 当前运行的事件记录器。
        name: 事件名称。
        severity: 事件严重程度。
        data: 原样交给记录器的结构化载荷。
    """
    try:
        events.append('navrun', name, severity, data)
    except OSError as error:
        print(f'navrun: event recording failed: {error}', file=sys.stderr)


def _drain_remaining(result, events):
    """持续重试本次仍持有的子进程关闭，确认成功前不放弃引用.

    Args:
        result: 前一清理阶段返回的 remaining 自有子进程集合。
        events: 清理重试事件接收器。

    每次 close 使用十秒预算，失败记录并暂停后重试；没有把总等待时间到期
    当作资源已释放的退出分支，不查杀原集合外的进程。
    """
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
    """为历史运行规划并实际执行保留策略，排除当前运行后记录结果.

    这是有删除副作用的执行步骤，不是预览；调用方负责授权并选定记录根。
    有时区的开始时间转为本地无时区时间，以匹配保留策略的时间口径。

    Args:
        options: 含保留策略的运行设置。
        run: 当前运行目录与记录根，当前 ID 始终受保护。
        started: 本次运行开始时间。
        events: 删除执行后追加 gc_finished 的事件记录器。

    Raises:
        Exception: 历史读取、规划、删除或事件写入失败；此处不回滚已完成删除。
    """
    history = [previous for previous in run_dir_lib.list_runs(run.root)
               if previous.run_id != run.run_id]
    local_time = started.astimezone().replace(tzinfo=None) if started.tzinfo else started
    plan = retention.plan_gc(history, options.retention_policy, local_time,
                             protected_run_id=run.run_id)
    removed = retention.apply_gc(plan, run.root)
    events.append('navrun', 'gc_finished', data={'removed_bytes': removed})


def _summary(run, finalized):
    """从已落盘事件和采样文件生成最终摘要，不重新查询运行进程.

    解析时跳过的事件数量仍传入摘要，不把缺失数据改写成成功。

    Args:
        run: 本次运行的事件、指标及摘要目标路径集合。
        finalized: 已收尾的运行清单记录。

    Raises:
        Exception: 记录读取、汇总或摘要写入失败。
    """
    from nav_bringup.recorder import summary, topic_stats
    from nav_bringup.recorder.resources import summarize_resources

    events, skipped = read_events(run.events)
    summary.write_summary(run.summary, summary.render_summary(
        finalized, events, summarize_resources(run.metrics / 'resources.csv'), skipped,
        topic_stats.summarize_topics(run.metrics / topic_stats.TOPICS_CSV_NAME)))
