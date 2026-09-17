"""监督已准备且持有独占租约的前台导航运行，按既定命令执行与收尾.

不自行发现或修改平台、DDS、定位、别名或配置。命令是否适合现场由外层
准备阶段决定；租约必须保留到记录最终完成。测试只使用假件或私有非 ROS 子进程。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
import signal
import time
from typing import Callable, Mapping, Optional, Tuple

from nav_bringup.navrun.locking import RunLock
from nav_bringup.navrun.keyboard_session import KeyboardSession
from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.recorder.events import EventLog


@dataclass(frozen=True)
class StartupGate:
    """描述须成功退出且完成进程清理后才能通过的启动门禁命令.

    数据类构造不执行校验，实际验证由 validate_commands 完成。

    Attributes:
        name: 唯一且安全的事件/日志标签，不是任意文件路径。
        argv: 原样执行的参数向量，不解释 shell 字符串。
        timeout_sec: 可选秒制期限；人工定位检查可不设期限，但仍轮询信号和关键进程。
    """

    name: str
    argv: Tuple[str, ...]
    timeout_sec: Optional[float] = None


@dataclass(frozen=True)
class StartupCommands:
    """保存外层入口已经解析的启动命令，不在构造时启动或验证它们.

    Attributes:
        launch: 应持续运行的前台导航启动命令。
        gates: launch 之后依次执行的就绪与定位/检查门禁。
        record: 可选持续录包命令；空向量显式禁用。
        monitor: 可选持续诊断命令，其运行失败通常只记录警告。
        zero: 必需停止辅助命令，外层须保证只发布零速；这里不验证其实现。
        parameter_dump: 可选一次性参数诊断，独立受管以便回收子进程组。
        keyboard: 可选直接受管键盘进程，不作为 launch 子进程。
        tty_device: 外层捕获并持有的配置终端路径。
        prelaunch_gates: 所有 launch 与 executor 计时器启动前完成的门禁。
        executor_launch: 所有门禁完成后才启动的可选 WS 阶段。
        event_flush: 清理后由外层记录器执行的可选最终诊断，本监督器不运行它。
    """

    launch: Tuple[str, ...]
    gates: Tuple[StartupGate, ...]
    record: Tuple[str, ...]
    monitor: Tuple[str, ...]
    zero: Tuple[str, ...]
    parameter_dump: Tuple[str, ...] = ()
    keyboard: Tuple[str, ...] = ()
    tty_device: str = ''
    prelaunch_gates: Tuple[StartupGate, ...] = ()
    executor_launch: Tuple[str, ...] = ()
    event_flush: Tuple[str, ...] = ()


@dataclass(frozen=True)
class LifecycleResult:
    """保存外层记录器最终落盘前必须处理的运行状态与未回收句柄.

    冻结字段不深冻结进程对象，构造本身不验证退出码或所有权关系。

    Attributes:
        exit_code: shell 兼容状态；监督器不把关键进程意外零退出视为成功。
        exit_reason: 结束阶段或信号原因。
        cleanup_errors: 已记录的停止失败说明，需进一步检查。
        remaining: 未完成回收的自有进程句柄；调用方必须保留租约并重试清理，
            不能提前发布运行完成或释放导航所有权。
    """

    exit_code: int
    exit_reason: str
    cleanup_errors: Tuple[str, ...] = ()
    remaining: Tuple[OwnedProcess, ...] = ()


class _Finished(Exception):
    """在监督器内部携带已确定的运行退出状态，不直接回收资源.

    Attributes:
        result: 供外层 supervise 捕获后返回或被清理错误覆盖的结果。
    """

    def __init__(self, code: int, reason: str) -> None:
        """保存退出码及原因，不校验输入或发布完成记录.

        Args:
            code: shell 兼容退出码。
            reason: 运行结束原因标签。
        """
        self.result = LifecycleResult(code, reason)


class _Supervisor:
    """持有一次已准备运行的子进程和顺序清理状态，不取得外层租约.

    Attributes:
        commands: 外层传入并已验证的命令计划。
        run_dir: 本次运行目录。
        events: 追加式事件接收器，只有 OSError 在 emit 内隔离。
        environ: 补充运行目录变量后的独立环境字典。
        stopped: 信号请求读取器，不是本对象安装的信号处理器。
        stop_timeout: 每组清理宽限秒数。
        children: 按阶段名保留的自有进程，关闭失败仍保留句柄。
        cleanup_errors: 清理或事件记录失败说明。
        launched: 主 launch 已由 start 成功返回，不代表已就绪。
        record_broken: 事件记录发生过 OSError。
        monitor_failed: 已处理诊断进程退出，避免重复报告。
        dump_finished: 已处理参数导出退出。
        keyboard: 可选父进程键盘能力，外层负责其完整生命周期。
        keyboard_permitted: 已向键盘发出允许控制标志。
    """

    def __init__(self, commands, run_dir, events, environ, stopped, stop_timeout, keyboard):
        """复制环境并初始化持有记录，不启动进程或执行参数验证.

        Args:
            commands: 已验证命令计划。
            run_dir: 当前运行目录路径。
            events: 事件追加接收器。
            environ: 要复制的继承环境。
            stopped: 返回信号编号或 None 的回调。
            stop_timeout: 每组清理宽限秒数。
            keyboard: 可选父进程键盘能力。
        """
        self.commands = commands
        self.run_dir = run_dir
        self.events = events
        self.environ = dict(environ)
        self.environ.update(ROS_LOG_DIR=str(run_dir / 'logs/ros'),
                            NAVRUN_RUN_DIR=str(run_dir), NAVRUN_RUN_ID=run_dir.name,
                            PYTHONUNBUFFERED='1')
        self.stopped = stopped
        self.stop_timeout = stop_timeout
        self.children = {}
        self.cleanup_errors = []
        self.launched = False
        self.record_broken = False
        self.monitor_failed = False
        self.dump_finished = False
        self.keyboard = keyboard
        self.keyboard_permitted = False

    def emit(self, event, severity='info', **data):
        """追加事件并隔离磁盘类错误，首条 OSError 才加入清理错误列表.

        记录失效标志供后续 check 终止运行；不吞任意事件接收器异常。

        Args:
            event: 事件类型。
            severity: 严重程度标签。
            data: 要随事件保存的字段。

        Raises:
            Exception: 事件接收器的非 OSError 异常。
        """
        # A broken event file must not prevent stop attempts on other children.
        try:
            self.events.append('navrun', event, severity, data)
        except OSError as error:
            if not self.record_broken:
                self.cleanup_errors.append(f'event log: {error}')
            self.record_broken = True

    def start(self, name, argv):
        """创建私有受管子进程，先保存句柄再记录启动事件.

        阶段名由已验证的命令计划保证不冲突；此辅助函数不重复检查重名。

        Args:
            name: 阶段名，同时决定运行内日志文件名。
            argv: 原样执行的参数向量，不使用 shell 解释。

        Returns:
            已登记的 OwnedProcess 句柄。

        Raises:
            Exception: 进程创建或未隔离的事件记录失败，已登记句柄仍被持有。
        """
        child = OwnedProcess(argv, self.environ, self.run_dir / 'logs' / (name + '.log'))
        self.children[name] = child
        self.emit('process_started', name=name, pid=child.pid)
        return child

    def check(self):
        """轮询信号、关键进程及诊断状态，以内部结果异常结束主流程.

        信号优先于记录错误；获准键盘的正常退出才算主动结束，导航/录包
        意外零退出仍判失败。诊断和参数导出退出各报告一次并尝试关闭。

        Raises:
            _Finished: 信号、记录失效、键盘结束或关键子进程退出。
            Exception: 状态读取或未隔离的事件记录异常。
        """
        signum = self.stopped()
        if signum is not None:
            reason = 'ctrl_c' if signum == signal.SIGINT else 'terminated'
            self.emit('signal_received', 'warn', signal=int(signum))
            raise _Finished(128 + int(signum), reason)
        if self.record_broken:
            raise _Finished(2, 'record_failed')
        keyboard = self.children.get('keyboard')
        if keyboard is not None and keyboard.poll() is not None:
            code = keyboard.poll()
            if code == 0 and self.keyboard_permitted:
                raise _Finished(0, 'keyboard_quit')
            raise _Finished(code if code > 0 else 128 - code if code < 0 else 1,
                            'keyboard_exited')
        for name in ('launch', 'executor_launch', 'record'):
            child = self.children.get(name)
            code = child.poll() if child is not None else None
            if code is not None:
                self.emit('process_exited', 'error', name=name, exit_code=code)
                raise _Finished(code if code > 0 else 128 - code if code < 0 else 1,
                                name + '_exited')
        monitor = self.children.get('monitor')
        if monitor is not None and not self.monitor_failed and monitor.poll() is not None:
            self.monitor_failed = True
            self.emit('monitor_failed', 'warn', exit_code=monitor.poll())
            self.close('monitor')
        dump = self.children.get('parameter_dump')
        if dump is not None and not self.dump_finished and dump.poll() is not None:
            self.dump_finished = True
            code = dump.poll()
            self.emit('parameter_dump_finished', 'info' if code == 0 else 'warn', exit_code=code)
            self.close('parameter_dump')

    def gate(self, gate):
        """运行一个门禁，同时检查全局状态，成功且清理完成后才放行.

        子进程退出检查先于本门禁期限；无期限仍每约 0.05 秒检查信号。
        超时不在此直接清理，外层 finally 负责已登记子进程。

        Args:
            gate: 已验证的命令、标签与可选秒制期限。

        Raises:
            _Finished: 全局结束、门禁失败/超时或句柄清理尚未完成。
            Exception: 创建、轮询或未隔离的事件记录失败。
        """
        self.check()
        child = self.start(gate.name, gate.argv)
        deadline = (time.monotonic() + gate.timeout_sec
                    if gate.timeout_sec is not None else None)
        self.emit('startup_gate_started', name=gate.name)
        while True:
            self.check()
            code = child.poll()
            if code is not None:
                self.close(gate.name)
                if gate.name in self.children:
                    raise _Finished(2, gate.name + '_cleanup_failed')
                self.emit('startup_gate_finished', 'info' if code == 0 else 'error',
                          name=gate.name, exit_code=code)
                if code != 0:
                    raise _Finished(code if code > 0 else 128 - code, gate.name + '_failed')
                return
            if deadline is not None and time.monotonic() >= deadline:
                raise _Finished(1, gate.name + '_timeout')
            time.sleep(0.05)

    def execute(self):
        """按门禁、键盘、导航及诊断录包顺序启动，再持续监督.

        prelaunch 门禁最先完成；键盘就绪后启动导航并允许键盘控制，
        后置门禁完成后才启动 WS 阶段。参数导出和监控创建的 OSError
        只告警，录包创建失败则由外层处理。event_flush 不在本流程执行。

        Raises:
            _Finished: 正常键盘退出、信号或任一关键阶段结束。
            Exception: 启动、键盘能力或未隔离观察操作失败，外层仍须收尾。
        """
        self.check()
        for gate in self.commands.prelaunch_gates:
            self.gate(gate)
        if self.keyboard is not None:
            argv, descriptors = self.keyboard.command(self.commands.keyboard)
            child = OwnedProcess(argv, self.environ, self.run_dir / 'logs/keyboard.log',
                                 pass_fds=descriptors)
            self.children['keyboard'] = child
            self.keyboard.child_started()
            self.emit('process_started', name='keyboard', pid=child.pid)
            deadline = time.monotonic() + 30.0
            while True:
                self.check()
                if self.keyboard.ready():
                    break
                if time.monotonic() >= deadline:
                    raise _Finished(1, 'keyboard_ready_timeout')
                time.sleep(0.05)
            self.emit('keyboard_guard_ready')
        self.check()
        self.start('launch', self.commands.launch)
        self.launched = True
        if self.keyboard is not None:
            self.check()
            self.keyboard.permit()
            self.keyboard_permitted = True
            self.emit('keyboard_permitted')
        for gate in self.commands.gates:
            self.gate(gate)
        self.check()
        if self.commands.executor_launch:
            self.start('executor_launch', self.commands.executor_launch)
        self.check()
        if self.commands.parameter_dump:
            try:
                self.start('parameter_dump', self.commands.parameter_dump)
            except OSError as error:
                self.emit('parameter_dump_failed', 'warn', error=str(error))
        self.check()
        if self.commands.monitor:
            try:
                self.start('monitor', self.commands.monitor)
            except OSError as error:
                self.emit('monitor_failed', 'warn', error=str(error))
        self.check()
        if self.commands.record:
            self.start('record', self.commands.record)
        else:
            self.emit('recording_disabled')
        self.emit('startup_ready')
        while True:
            self.check()
            time.sleep(0.05)

    def close(self, name):
        """尝试关闭指定自有进程，失败记录后保留未完成句柄.

        无句柄时直接返回。成功关闭后先移除再写事件，因此后续事件错误
        不表示进程仍存在；非隔离事件异常也可能从错误报告路径传播。

        Args:
            name: children 中的阶段名。

        Raises:
            Exception: 错误报告本身抛出非 OSError 异常。
        """
        child = self.children.get(name)
        if child is None:
            return
        try:
            code = child.close(self.stop_timeout)
            del self.children[name]
            self.emit('process_stopped', name=name, exit_code=code)
        except Exception as error:
            self.cleanup_errors.append(f'{name}: {error}')
            self.emit('cleanup_failed', 'error', name=name, error=str(error))

    def zero(self, name):
        """启动外层提供的零速辅助命令，有限轮询后始终尝试回收它.

        等待预算为五秒，但子进程创建/清理与同步回调不受该预算抢占。
        零退出只证明命令返回成功，不证明订阅者收到或实体停止。

        Args:
            name: zero_before 或 zero_after 阶段标签。

        Raises:
            Exception: 未隔离的事件记录或最终清理错误报告失败。
        """
        try:
            child = self.start(name, self.commands.zero)
            deadline = time.monotonic() + 5.0
            while child.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            code = child.poll()
            if code != 0:
                raise RuntimeError(f'zero command failed or timed out (status={code})')
            self.emit('stop_zero_sent', phase=name)
        except Exception as error:
            self.cleanup_errors.append(f'{name}: {error}')
            self.emit('stop_zero_failed', 'error', phase=name, error=str(error))
        finally:
            self.close(name)

    def cleanup(self):
        """撤销键盘权限，并围绕导航退出尝试两次零速后顺序清理自有进程.

        只有 launch 已成功登记才调用零速辅助器；先零速、再等待键盘/门禁，
        关闭 WS 和导航后再零速，最后处理录包及诊断。常见关闭错误记账后
        继续，但任意 BaseException 或未隔离事件异常仍可能中断本序列。
        不操作外部定位或底盘进程，未回收句柄仍由 children 持有。

        Raises:
            Exception: 未隔离的能力或事件接收器异常。
        """
        if self.keyboard is not None:
            try:
                self.keyboard.revoke()
            except Exception as error:
                self.cleanup_errors.append(f'keyboard revoke: {error}')
                self.emit('keyboard_revoke_failed', 'error', error=str(error))
        if self.launched:
            # Attempt stopping before waiting on any diagnostic helper to exit.
            self.zero('zero_before')
        self.close('keyboard')
        for gate in self.commands.prelaunch_gates + self.commands.gates:
            self.close(gate.name)
        self.close('executor_launch')
        if self.launched:
            self.close('launch')
            # An external driver survives launch. Try again after navigation
            # publishers have stopped, so a final nonzero cannot overtake zero.
            self.zero('zero_after')
        self.close('record')
        self.close('parameter_dump')
        self.close('monitor')


def supervise(
    commands: StartupCommands, run_dir: Path, events: EventLog,
    environ: Mapping[str, str], lease: RunLock,
    stopped: Callable[[], Optional[int]], stop_timeout_sec: float = 10.0,
    keyboard: Optional[KeyboardSession] = None,
) -> LifecycleResult:
    """验证并运行已准备命令，退出时按顺序尝试回收自有子进程.

    调用方负责 manifest 及最终记录，并持有租约贯穿这些步骤。子进程使用
    私有组和运行目录内日志；不修改调用方环境或安装信号处理器。
    门禁等待仍轮询信号；诊断退出通常只警告，门禁、导航和录包失败终止运行。
    记录的清理错误覆盖原退出状态；非 OSError 的事件接收器错误或 BaseException
    仍可能打断清理，不能将此函数理解为任意故障下都能完整回收的保证。

    Args:
        commands: 显式命令计划，在启动前验证参数向量等结构。
        run_dir: 已存在的绝对运行目录，包含 logs 子目录。
        events: 本次运行的追加事件接收器。
        environ: 继承环境，复制后补运行内日志变量，不修改原映射。
        lease: 调用方已取得并继续持有的单实例锁。
        stopped: 返回首个待处理信号编号，或 None 表示继续。
        stop_timeout_sec: 每个进程组有限非负的清理宽限秒数，不是整轮总期限。
        keyboard: 可选父进程终端/socket 能力，所有返回句柄回收前须继续持有。

    Returns:
        运行状态及已知清理错误；如有 remaining，外层仍负有回收责任。

    Raises:
        ValueError: 已检查命令、路径、标签、期限或键盘配对无效。
        RuntimeError: 调用方未取得租约。
        OSError: 无法创建运行内 ROS 日志目录。
        Exception: 未归一化类型错误、事件接收器或清理过程异常。
    """
    lease.fileno()
    validate_commands(commands, run_dir, stop_timeout_sec)
    if bool(commands.keyboard) != (keyboard is not None):
        raise ValueError('keyboard command and parent-owned capabilities must agree')
    (run_dir / 'logs/ros').mkdir(exist_ok=True)
    owner = _Supervisor(commands, run_dir, events, environ, stopped, stop_timeout_sec, keyboard)
    result = LifecycleResult(2, 'startup_failed')
    try:
        owner.execute()
    except _Finished as finished:
        result = finished.result
    except KeyboardInterrupt:
        result = LifecycleResult(130, 'ctrl_c')
    except Exception as error:
        owner.emit('startup_failed', 'error', error=str(error))
    finally:
        owner.cleanup()
    if owner.cleanup_errors:
        return LifecycleResult(2, 'cleanup_failed', tuple(owner.cleanup_errors),
                               tuple(owner.children.values()))
    return result


def validate_commands(
    commands: StartupCommands, run_dir: Path, stop_timeout: float = 10.0,
) -> None:
    """在启动进程前验证命令结构和运行目录，不执行主机配置变更.

    只读检查路径；不解析命令语义、不验证可执行文件存在、是否只发零速，
    也不禁止所有空字符串参数。门禁名必须安全、唯一且不占保留名称。
    键盘命令与终端选择须配对；终端要求绝对路径但不在此打开。

    Args:
        commands: 已解析的命令向量及启动门禁集合。
        run_dir: 包含 logs 子目录的已存在绝对运行目录。
        stop_timeout: 每组清理宽限秒数，有限非负且拒绝布尔值。

    Raises:
        ValueError: 已检查路径、标签、命令结构或期限无效。
        Exception: 未归一化类型、数值转换或文件系统访问异常。
    """
    if not run_dir.is_absolute() or not (run_dir / 'logs').is_dir():
        raise ValueError('expected an absolute existing run directory with logs')
    _timeout(stop_timeout)
    reserved = {'launch', 'executor_launch', 'record', 'monitor', 'parameter_dump', 'keyboard',
                'zero_before', 'zero_after', 'ros', 'event_flush'}
    argv_list = [commands.launch, commands.zero]
    for gate in commands.prelaunch_gates + commands.gates:
        if not re.fullmatch('[a-z][a-z0-9_]*', gate.name) or gate.name in reserved:
            raise ValueError('gate names must be unique safe nonreserved labels')
        reserved.add(gate.name)
        if gate.timeout_sec is not None:
            _timeout(gate.timeout_sec)
        argv_list.append(gate.argv)
    argv_list.extend(argv for argv in (
        commands.record, commands.monitor, commands.parameter_dump, commands.keyboard,
        commands.executor_launch, commands.event_flush) if argv)
    if bool(commands.keyboard) != bool(commands.tty_device):
        raise ValueError('keyboard and terminal must be selected together')
    if commands.tty_device and (
            not isinstance(commands.tty_device, str) or '\0' in commands.tty_device
            or not Path(commands.tty_device).is_absolute()):
        raise ValueError('keyboard terminal must be an absolute path without NUL')
    if any(not argv or any(not isinstance(arg, str) or '\0' in arg for arg in argv)
           for argv in argv_list):
        raise ValueError('commands must be nonempty argument vectors without NUL')


def _timeout(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError('time limits must be nonnegative and finite')
