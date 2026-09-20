"""由机器人档案控制的 Jetson 平台设置，命令执行非交互且有时限.

规划阶段是纯函数：不探测主机、不执行任何命令。执行阶段不使用密码、不经过 shell、
不从标准输入读任何提示，也不修改 sudoers 等权限配置——权限不足时只会失败，
不会自动放宽。单元测试注入自己的执行器，从不改动真实功耗模式。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import subprocess
from typing import Any, Callable, Mapping, Sequence, Tuple


CommandRunner = Callable[[Sequence[str], float], Tuple[int, str]]


@dataclass(frozen=True)
class PlatformStep:
    """一条有序的平台命令.

    Attributes:
        name: 稳定的步骤名，运行事件按此名记录。
        argv: 精确的参数向量，不做任何 shell 插值或变量展开。
        timeout_sec: 正的墙钟超时；超时按失败处理，不重试。
    """

    name: str
    argv: Tuple[str, ...]
    timeout_sec: float


@dataclass(frozen=True)
class PlatformResult:
    """一次已规划或已尝试的平台操作记录.

    Attributes:
        name: 稳定的操作名。
        argv: 实际（或计划）的调用向量。
        status: planned、succeeded 或 failed 三者之一；succeeded 只表示该命令
            自己以 0 退出，不证明功耗模式或时钟真的生效。
        return_code: 退出码；dry-run 时为 None。
        detail: 命令输出或有界的失败描述。
    """

    name: str
    argv: Tuple[str, ...]
    status: str
    return_code: int | None
    detail: str


class PlatformError(RuntimeError):
    """平台设置失败；该失败之后的平台步骤一律未尝试，已成功的步骤也未回滚.

    Attributes:
        results: 按尝试顺序记录的结果，包含最后一次失败；未执行后续操作。
    """

    def __init__(self, results: Tuple[PlatformResult, ...]) -> None:
        """保留已成功的步骤与首次失败，供运行记录器使用.

        只组装信息：不重试、不回滚已经成功的步骤，也不改动任何权限配置。

        Args:
            results: 按尝试顺序排列的结果，最后一项是失败的那次操作。
        """
        self.results = results
        last = results[-1]
        super().__init__(
            f'platform {last.name} failed ({last.return_code}): {last.detail}; '
            'check tools and the operator-managed sudoers policy; no permissions were changed')


def plan_platform(profile: Mapping[str, Any]) -> Tuple[PlatformStep, ...]:
    """仅按档案解析出平台操作，不探测也不改动主机.

    纯函数：不执行命令、不读取硬件状态，因此返回的步骤只代表档案的意图，
    不代表这台机器真的支持它们。

    Args:
        profile: 已按模式合并的机器人档案，platform 段可缺省。

    Returns:
        有序的功耗模式与时钟命令；档案关闭或非 Jetson 时为空元组。

    Raises:
        ValueError: 类型、模式号、超时非法，或在非 Jetson 档案上打开了 Jetson 操作。
    """
    values = profile.get('platform', {})
    if not isinstance(values, Mapping):
        raise ValueError('platform must be a mapping')
    switches = {name: values.get(name, False) for name in ('jetson', 'maxn', 'jetson_clocks')}
    if any(type(value) is not bool for value in switches.values()):
        raise ValueError('platform switches must be booleans')
    mode_id = values.get('maxn_mode_id', 0)
    timeout = values.get('command_timeout_sec', 30.0)
    if type(mode_id) is not int or mode_id < 0:
        raise ValueError('platform.maxn_mode_id must be a nonnegative integer')
    if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
            or not math.isfinite(timeout) or timeout <= 0):
        raise ValueError('platform.command_timeout_sec must be finite and positive')
    if not switches['jetson'] and (switches['maxn'] or switches['jetson_clocks']):
        raise ValueError('Jetson platform operations require platform.jetson: true')
    steps = []
    prefix = ('/usr/bin/sudo', '-n', '--')
    if switches['maxn']:
        steps.append(PlatformStep(
            'maxn', prefix + ('/usr/sbin/nvpmodel', '-m', str(mode_id)), float(timeout)))
    if switches['jetson_clocks']:
        steps.append(PlatformStep(
            'jetson_clocks', prefix + ('/usr/bin/jetson_clocks',), float(timeout)))
    return tuple(steps)


def run_platform_command(argv: Sequence[str], timeout_sec: float) -> Tuple[int, str]:
    """以墙钟期限执行一条非交互命令.

    标准输入接空设备，因此需要交互确认的命令会立刻读到 EOF 而失败，不会挂住启动流程；
    不经过 shell，argv 逐项传给内核。

    Args:
        argv: 程序及其精确参数。
        timeout_sec: 传给 subprocess.run 的期限。

    Returns:
        退出码与合并后的输出。命令不存在、被拒绝、超时分别用 127、126、124 表示，
        既不重试也不提示；退出码 0 只说明该进程自己正常退出。
    """
    try:
        result = subprocess.run(
            list(argv), stdin=subprocess.DEVNULL, capture_output=True, text=True,
            check=False, timeout=timeout_sec)
        return result.returncode, result.stdout + result.stderr
    except FileNotFoundError as error:
        return 127, str(error)
    except subprocess.TimeoutExpired:
        return 124, f'command timed out after {timeout_sec:g} seconds'
    except OSError as error:
        return 126, str(error)


def configure_platform(
    profile: Mapping[str, Any], runner: CommandRunner = run_platform_command,
    dry_run: bool = False,
) -> Tuple[PlatformResult, ...]:
    """执行全部已校验的平台步骤，或只描述而不执行.

    按顺序逐条执行，首次失败立即抛出，后续步骤不再尝试，已成功的步骤也不回滚。
    dry-run 分支根本不调用执行器。

    Args:
        profile: 已按模式合并的机器人档案。
        runner: 可注入的命令执行器；测试必须传入假件。
        dry_run: 只返回计划记录，不调用执行器。

    Returns:
        有序的操作记录；平台设置关闭时为空元组。

    Raises:
        ValueError: 平台设置或 dry-run 开关非法。
        PlatformError: 首次命令失败，异常内带已尝试的部分结果。
    """
    if type(dry_run) is not bool:
        raise ValueError('dry_run must be a boolean')
    steps = plan_platform(profile)
    results = []
    for step in steps:
        if dry_run:
            results.append(PlatformResult(step.name, step.argv, 'planned', None, ''))
            continue
        code, output = runner(step.argv, step.timeout_sec)
        results.append(PlatformResult(
            step.name, step.argv, 'succeeded' if code == 0 else 'failed', code, output))
        if code != 0:
            raise PlatformError(tuple(results))
    return tuple(results)
