"""根据进程快照规划残留清理，并在导航启动前确定底盘所有权.

规划不读取真实宿主或发送信号；执行必须显式注入快照与身份安全的信号端口，
没有默认 kill 实现。只允许处理最初选定身份，每次重查定位、外部底盘及
不同用户的保护边界。计划不是对任意进程的清理授权，也不是事务。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import PurePath
import re
import signal
import time
from typing import Any, Callable, Dict, Mapping, Sequence, Tuple


@dataclass(frozen=True)
class ProcessInfo:
    """表示不依赖 ROS 发现的单次进程身份观察.

    Attributes:
        pid: 正操作系统进程号。
        ppid: 快照时的父进程号，重新归属父进程不单独构成身份改变。
        started_at: 用于识别 PID 复用的创建时间。
        argv: 真实参数元组，不是其它进程携带的 shell 文本。
        uid: 进程所属的真实用户编号。
    """

    pid: int
    ppid: int
    started_at: float
    argv: Tuple[str, ...]
    uid: int


@dataclass(frozen=True)
class _Policy:
    kill: Tuple[str, ...]
    foreign: Tuple[str, ...]
    driver: Tuple[str, ...]
    protected: Tuple[str, ...]
    kill_foreign: bool


@dataclass(frozen=True)
class PreflightPlan:
    """表示底盘选择及唯一允许参与本次清理的初始进程身份集合.

    Attributes:
        start_chassis: 启动图是否应拥有新底盘驱动。
        targets: 初始候选，祖先排在后代之前；执行时不得扩大集合。
        protected_pids: 监督者祖先及外部受保护进程树的 PID。
        warnings: 匹配策略但明确保留的进程说明。
        own_pid: 必须保留其祖先和用户边界的监督者 PID。
    """

    start_chassis: bool
    targets: Tuple[ProcessInfo, ...]
    protected_pids: Tuple[int, ...]
    warnings: Tuple[str, ...]
    own_pid: int
    _policy: _Policy


class PreflightError(RuntimeError):
    """表示无法安全完成残留清理，启动方不能据此假定进程图已干净."""


def _patterns(values: Any, label: str) -> Tuple[str, ...]:
    """检查并预编译进程匹配正则，返回原样模式元组.

    允许空模式集合；字符串必须非空但不会 strip，不改写任何匹配规则。

    Args:
        values: 正则字符串列表或元组。
        label: 无效规则错误中的配置字段名称。

    Returns:
        保留输入次序与文本的模式元组。

    Raises:
        ValueError: 容器、模式类型或正则语法不合法。
    """
    if not isinstance(values, (list, tuple)) or any(
            not isinstance(item, str) or not item for item in values):
        raise ValueError(f'{label} must be a list of nonempty regex strings')
    try:
        for item in values:
            re.compile(item)
    except re.error as error:
        raise ValueError(f'{label}: invalid regex: {error}') from error
    return tuple(values)


def _policy(profile: Mapping[str, Any]) -> _Policy:
    """从档案解析预检策略，并把定位提供者加入保护匹配规则.

    Args:
        profile: 已读入的机器人档案，不在此访问进程或更改配置。

    Returns:
        含清理、外部栈、驱动和保护正则及显式开关的不可变策略；
        未配置清理开关时沿用 False。

    Raises:
        ValueError: 配置节、布尔开关、正则或定位可执行文件名不合法。
    """
    preflight = profile.get('preflight', {})
    chassis = profile.get('chassis', {})
    if not isinstance(chassis, Mapping):
        raise ValueError('chassis must be a mapping')
    driver = chassis.get('driver', {})
    localization = profile.get('localization', {})
    if not all(isinstance(value, Mapping) for value in (preflight, driver, localization)):
        raise ValueError('preflight, chassis.driver and localization must be mappings')
    kill_foreign = preflight.get('kill_foreign_stacks', False)
    if type(kill_foreign) is not bool:
        raise ValueError('preflight switches must be booleans')
    protected = _patterns(preflight.get('protected_patterns', []), 'protected_patterns')
    provider = localization.get('process_name')
    if provider is not None:
        if (not isinstance(provider, str) or not provider or '/' in provider
                or re.search(r'\s', provider)):
            raise ValueError('localization.process_name must be an executable basename')
        protected += (r'(?:^|[/\s])' + re.escape(provider) + r'(?:\s|$)',)
    return _Policy(
        _patterns(preflight.get('kill_patterns', []), 'kill_patterns'),
        _patterns(preflight.get('foreign_patterns', []), 'foreign_patterns'),
        _patterns(driver.get('detect_patterns', []), 'detect_patterns'),
        protected, kill_foreign)


def _command(process: ProcessInfo) -> str:
    """提取进程启动身份前缀供匹配，避免把普通参数中的名称当作启动目标.

    Args:
        process: 含不可变 argv 快照的进程记录。

    Returns:
        支持的 Python/脚本/ros2 包装器身份前缀；不明确的解释器选项或
        Python -c 返回空串，不执行命令或输出完整参数内容。
    """
    argv = process.argv
    if not argv:
        return ''
    index = 0
    executable = PurePath(argv[0]).name
    if re.fullmatch(r'python(?:[0-9]+(?:\.[0-9]+)?)?', executable):
        index = 1
        while index < len(argv) and argv[index] in ('-u', '-B', '-s', '-E', '-I'):
            index += 1
        if index >= len(argv) or argv[index] == '-c':
            return ''
        if argv[index] == '-m':
            return ' '.join(argv[:index + 2])
        if argv[index].startswith('-'):
            return ''
        executable = PurePath(argv[index]).name
    elif executable in ('bash', 'sh', 'dash', 'zsh'):
        if len(argv) < 2 or argv[1].startswith('-'):
            return ''
        return ' '.join(argv[:2])
    if executable == 'ros2' and len(argv) > index + 1 and argv[index + 1] in ('run', 'launch'):
        return ' '.join(argv[:index + 4])
    return ' '.join(argv[:index + 1])


def _matches(command: str, patterns: Sequence[str]) -> bool:
    return bool(command) and any(re.search(pattern, command) for pattern in patterns)


def is_task_manager(process: ProcessInfo) -> bool:
    """按可执行入口识别共享任务管理器，排除普通 shell 文本与编辑器参数.

    Args:
        process: 只读命令行快照，不在此探测宿主。

    Returns:
        实际程序或 ROS run 目标是否匹配受支持入口；
        分类结果本身不授予发送信号的权限。
    """
    return _matches(_command(process), (
        r'(?:^|[/\s])nav_task_manager$',
        r'(?:^|/)lib/nav_executor/navigation_executor$',
        r'(?:^|/)ros2 run nav_executor navigation_executor$',
    ))


def _table(processes: Sequence[ProcessInfo], own_pid: int) -> Dict[int, ProcessInfo]:
    """验证进程快照身份字段并构建唯一 PID 索引.

    Args:
        processes: 同一次只读扫描得到的进程记录。
        own_pid: 必须存在于快照中的监督者 PID。

    Returns:
        PID 到原始记录的字典，保留 UID、父 PID 和启动时间供后续身份复核。

    Raises:
        ValueError: 字段不完整/非法、PID 重复或缺少监督者身份。
    """
    table = {}
    for process in processes:
        if (type(process.pid) is not int or process.pid <= 0 or type(process.ppid) is not int
                or process.ppid < 0 or type(process.uid) is not int or process.uid < 0
                or isinstance(process.started_at, bool)
                or not isinstance(process.started_at, (int, float))
                or not math.isfinite(process.started_at) or process.started_at <= 0
                or not isinstance(process.argv, tuple)
                or any(not isinstance(arg, str) for arg in process.argv)):
            raise ValueError('invalid or incomplete process identity')
        if process.pid in table:
            raise ValueError('duplicate PID in process snapshot')
        table[process.pid] = process
    if own_pid not in table:
        raise ValueError('snapshot must contain the supervisor identity')
    return table


def _ancestry(pid: int, table: Mapping[int, ProcessInfo]) -> set[int]:
    """沿父 PID 链收集自身及祖先，缺失快照节点处停止.

    Args:
        pid: 开始追溯的进程号。
        table: 当前 PID 到进程身份的快照映射。

    Returns:
        访问过的正 PID 集合，包含最后一个快照缺失的祖先号。

    Raises:
        ValueError: 祖先链出现循环，不能据此判断保护边界。
    """
    result = set()
    while pid > 0:
        if pid in result:
            raise ValueError('cycle in process ancestry')
        result.add(pid)
        process = table.get(pid)
        if process is None:
            break
        pid = process.ppid
    return result


def _is_launch_wrapper(process: ProcessInfo) -> bool:
    """按命令行形状识别 ROS 启动包装器与 shell 包装器.

    用于现有进程分类启发式，不证明进程归属或安全可终止；不执行命令。
    shell 识别沿用 -c 或非选项脚本参数的规则。

    Args:
        process: 已读取的进程身份与 argv 快照。

    Returns:
        命令是否匹配已知包装器形状。
    """
    command = _command(process)
    if re.search(r'(?:^|/)ros2 (?:launch|run) ', command):
        return True
    argv = process.argv
    return (bool(argv) and PurePath(argv[0]).name in ('bash', 'sh', 'dash', 'zsh')
            and len(argv) > 1 and (argv[1] == '-c' or not argv[1].startswith('-')))


def _protected_tree(seed: int, table: Mapping[int, ProcessInfo], barriers: set[int]) -> set[int]:
    # Ascend only inside the external branch, stopping before shared supervisor
    # ancestors. Protect sibling sensor/RSP processes under that external launch.
    """保护外部启动分支及相关祖先，不把共享终端的其它分支算作自己持有.

    Args:
        seed: 定位/驱动等需要保留的外部进程 PID。
        table: 已验证的进程快照。
        barriers: 监督者自身及祖先集合，向上扩展前在此停止。

    Returns:
        外部包装器子树和非屏障祖先的保护 PID 集合，不执行任何信号操作。

    Raises:
        ValueError: 外部进程祖先链存在循环。
    """
    root = seed
    visited = set()
    while root in table and root not in visited:
        visited.add(root)
        parent = table[root].ppid
        if (parent <= 1 or parent in barriers or parent not in table
                or not _is_launch_wrapper(table[parent])):
            break
        root = parent
    if root in table and table[root].ppid in visited:
        raise ValueError('cycle in external process ancestry')
    protected = {root}
    while True:
        children = {process.pid for process in table.values() if process.ppid in protected}
        expanded = protected | children
        if expanded == protected:
            # Do not signal higher ancestors either, but do not claim ownership
            # of all their siblings (e.g. every tab of a terminal server).
            return protected | (_ancestry(seed, table) - barriers)
        protected = expanded


def _make_plan(
    policy: _Policy, table: Mapping[int, ProcessInfo], own_pid: int, chassis: str,
) -> PreflightPlan:
    """基于快照生成预检计划，保护外部定位分支和不同用户进程.

    Args:
        policy: 已解析的匹配规则与显式清理开关。
        table: 含监督者自身的已验证快照。
        own_pid: 本次监督者 PID。
        chassis: on、off 或 auto 的底盘启动选择。

    Returns:
        启动底盘决策、祖先优先排序的候选、保护集合和警告。
        这是未执行的计划，不发送信号；执行时仍须通过身份复核入口。

    Raises:
        ValueError: 进程祖先/保护树关系无法安全确定。
    """
    own = table[own_pid]
    commands = {pid: _command(process) for pid, process in table.items()}
    barriers = _ancestry(own_pid, table)
    drivers = {pid for pid, command in commands.items() if pid not in barriers
               and _matches(command, policy.driver)}
    start_chassis = chassis == 'on' or (chassis == 'auto' and not drivers)
    protected = set(barriers)
    seeds = {pid for pid, command in commands.items() if _matches(command, policy.protected)}
    if not start_chassis:
        seeds.update(drivers)
    for seed in seeds:
        protected.update(_protected_tree(seed, table, barriers))
    targets, warnings = [], []
    for pid, process in sorted(table.items()):
        command = commands[pid]
        foreign = _matches(command, policy.foreign)
        # 2026-09-20 用户决定：本流程永不结束 robot_state_publisher。
        # 现场的机械臂 moveit 栈自带一个，误杀会直接打断上层作业；
        # 重复的 URDF 发布者改由 doctor 的只读检查报告，不再自动处理。
        selected = (_matches(command, policy.kill)
                    or (start_chassis and pid in drivers)
                    or (policy.kill_foreign and foreign))
        if pid in barriers:
            continue
        if foreign and not policy.kill_foreign:
            warnings.append(f'pid {pid}: foreign stack retained (warn-only policy)')
        elif selected and pid in protected:
            warnings.append(f'pid {pid}: protected external process retained')
        elif selected and process.uid != own.uid:
            warnings.append(f'pid {pid}: different user retained')
        elif selected:
            targets.append(process)
    targets.sort(key=lambda process: (len(_ancestry(process.pid, table)), process.pid))
    return PreflightPlan(start_chassis, tuple(targets), tuple(sorted(protected)),
                         tuple(warnings), own_pid, policy)


def plan_preflight(
    profile: Mapping[str, Any], processes: Sequence[ProcessInfo], own_pid: int,
    chassis: str = 'auto',
) -> PreflightPlan:
    """纯粹分类进程快照，生成固定候选及保护边界，不读取宿主或发信号.

    Args:
        profile: 模式叠加后的档案，含残留、外部栈、驱动和保护匹配规则。
        processes: 覆盖监督者及相关外部树的完整身份快照。
        own_pid: 快照中必须存在的监督者编号。
        chassis: auto 按已有驱动决定，on 计划拥有驱动，off 保留外部驱动。

    Returns:
        冻结的初始清理身份、保护 PID、警告及底盘启动决策。

    Raises:
        ValueError: 策略、选择器、进程身份或祖先关系无效。
    """
    if chassis not in ('auto', 'on', 'off'):
        raise ValueError('chassis must be auto, on or off')
    return _make_plan(_policy(profile), _table(processes, own_pid), own_pid, chassis)


def _identity(process: ProcessInfo) -> tuple:
    return process.pid, process.started_at, process.uid, process.argv


def _remaining(plan: PreflightPlan, snapshot: Sequence[ProcessInfo]) -> Tuple[ProcessInfo, ...]:
    """重扫原计划候选，只允许身份未变且仍符合原边界的残留进程.

    Args:
        plan: 原始预检计划，不允许扩大其 targets。
        snapshot: 操作前的新进程身份快照。

    Returns:
        按原计划顺序仍存活且可处理的记录元组；不执行清理。

    Raises:
        PreflightError: 出现新候选、PID 被复用或目标变成受保护/不再符合策略。
        ValueError: 新快照或祖先关系不合法。
    """
    table = _table(snapshot, plan.own_pid)
    fresh = _make_plan(plan._policy, table, plan.own_pid, 'on' if plan.start_chassis else 'off')
    original = {process.pid: process for process in plan.targets}
    eligible = {process.pid: process for process in fresh.targets}
    for process in fresh.targets:
        if process.pid not in original or _identity(original[process.pid]) != _identity(process):
            raise PreflightError(
                'new residual process or changed identity; refusing wider cleanup')
    remaining = []
    for process in plan.targets:
        current = table.get(process.pid)
        if current is None:
            continue
        if _identity(current) != _identity(process):
            raise PreflightError(f'pid {process.pid}: identity changed')
        if current.pid not in eligible:
            raise PreflightError(f'pid {process.pid}: now protected or no longer eligible')
        remaining.append(current)
    return tuple(remaining)


def cleanup_preflight(
    plan: PreflightPlan,
    snapshot: Callable[[], Sequence[ProcessInfo]],
    send_if_same: Callable[[ProcessInfo, int], bool],
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    dry_run: bool = False,
    term_timeout_sec: float = 8.0,
    kill_timeout_sec: float = 0.5,
    poll_sec: float = 0.3,
) -> Tuple[int, ...]:
    """仅对初始清理集合执行 TERM，并在等待后按需 KILL 和再次确认.

    每个信号与轮询前重新检查快照；发送后端仍须在实际发送时固定创建身份
    （例如 pidfd），不能用裸 PID 发送代替。回调返回 False 时再查是否已退出。
    等待预算不限制同步快照、信号回调或被阻塞的时钟/休眠；不是整体硬期限。
    失败可发生在部分进程已退出之后，不回滚也不扩大清理集合。

    Args:
        plan: 初始只读清理决策。
        snapshot: 每次取得完整新身份视图的端口。
        send_if_same: 必需的身份安全信号端口，不隐式提供 os.kill。
        clock: 单调时钟，测试可使用虚拟时间。
        sleep: 轮询等待端口，测试可替换。
        dry_run: 只返回计划 PID；仍校验开关和预算，但不调用运行端口。
        term_timeout_sec: TERM 后的等待预算，默认保留原八秒策略。
        kill_timeout_sec: KILL 后确认预算。
        poll_sec: 两次观察之间请求的最大休眠时间。

    Returns:
        已确认消失的初始目标 PID 元组；dry-run 返回计划 PID，不证明已消失。

    Raises:
        ValueError: 开关、预算或进程快照无效。
        PreflightError: 身份改变、新候选、保护边界变化、信号失败或仍有残留。
        Exception: 注入快照、时钟、休眠或信号端口的其它异常。
    """
    if type(dry_run) is not bool:
        raise ValueError('dry_run must be a boolean')
    for value in (term_timeout_sec, kill_timeout_sec, poll_sec):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0):
            raise ValueError('preflight deadlines and polling must be finite and positive')
    if dry_run:
        return tuple(process.pid for process in plan.targets)

    def send(signum: int) -> None:
        """每个信号前重新核对原计划身份，只处理仍存活的初始候选.

        真正发送仍要求注入后端固定身份，不以 PID 快照代替 pidfd 等保证。
        后端返回假值时重扫：确认消失可继续，仍存在或身份变化则拒绝。
        普通信号错误中的 OSError 包装为 PreflightError，其它异常原样传播。

        Args:
            signum: 本轮 TERM 或 KILL 信号编号。

        Raises:
            PreflightError: 目标变化、发送失败或无法确认发送身份。
            ValueError: 新进程快照不合法。
            Exception: 注入快照或信号端口的其它异常。
        """
        for original in plan.targets:
            current = {item.pid: item for item in _remaining(plan, snapshot())}
            if original.pid not in current:
                continue
            try:
                sent = send_if_same(current[original.pid], signum)
            except OSError as error:
                raise PreflightError(f'pid {original.pid}: signal failed: {error}') from error
            if not sent:
                # A normal exit between snapshot and send is harmless; a reused
                # or newly protected identity is diagnosed by the fresh view.
                remaining = _remaining(plan, snapshot())
                if any(item.pid == original.pid for item in remaining):
                    raise PreflightError(
                        f'pid {original.pid}: signal identity could not be verified')

    def wait(timeout: float) -> Tuple[ProcessInfo, ...]:
        deadline = clock() + timeout
        while True:
            remaining = _remaining(plan, snapshot())
            if not remaining:
                return ()
            delay = deadline - clock()
            if delay <= 0:
                return remaining
            sleep(min(poll_sec, delay))

    send(signal.SIGTERM)
    if wait(term_timeout_sec):
        send(signal.SIGKILL)
        remaining = wait(kill_timeout_sec)
        if remaining:
            raise PreflightError('residual processes remain after KILL: ' + ', '.join(
                str(process.pid) for process in remaining))
    return tuple(process.pid for process in plan.targets)
