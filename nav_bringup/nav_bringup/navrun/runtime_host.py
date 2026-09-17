"""Explicitly bound platform, residual-process and DDS startup operations.

Construction performs no observation or mutation. Tests pass fake platform and
process adapters plus a private DDS directory. Deployment must supply the real
directory deliberately; no import or default argument cleans shared memory.
"""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple

from nav_bringup.navrun.dds_hygiene import cleanup_hygiene, MappingView, plan_hygiene
from nav_bringup.navrun.dds_hygiene import snapshot_mappings
from nav_bringup.navrun.platform import CommandRunner, PlatformError, PlatformResult
from nav_bringup.navrun.platform import run_platform_command
from nav_bringup.navrun.preflight import cleanup_preflight, plan_preflight, PreflightError
from nav_bringup.navrun.preflight import ProcessInfo
from nav_bringup.navrun.recorded_startup import StartupInterrupted, StartupStep
from nav_bringup.navrun.runtime_commands import effective_profile


def bind_host_steps(
    dds_directory: Path, platform_runner: CommandRunner = run_platform_command,
    snapshot: Optional[Callable[[], Sequence[ProcessInfo]]] = None,
    send: Optional[Callable[[ProcessInfo, int], bool]] = None,
    observe: Callable[[], MappingView] = snapshot_mappings,
) -> Tuple[StartupStep, ...]:
    """Bind ordered host operations for the lock-scoped recorded startup.

    Every process operation rechecks identity; final chassis ownership must
    still agree with the rendered graph. DDS cleanup retains incomplete evidence
    and never stops a daemon or changes a transport whitelist. Successful host
    mutations are logged immediately, including when a later operation fails.

    Args:
        dds_directory: Explicit absolute cleanup directory; tests use private fixtures.
        platform_runner: Bounded noninteractive command runner; tests supply a fake.
        snapshot: Complete process view, or the lazy read-only Linux backend.
        send: Identity-pinned signal backend, or the lazy Linux pidfd backend.
        observe: Fresh read-only mapping evidence; tests supply controlled views.

    Returns:
        Platform, preflight and DDS callbacks, not executed during binding.

    Raises:
        ValueError: The cleanup directory is not explicit and normalized.
    """
    if (not dds_directory.is_absolute() or dds_directory == Path('/')
            or '..' in dds_directory.parts):
        raise ValueError('DDS directory must be an explicit normalized absolute path')

    def processes():
        if snapshot is not None:
            return snapshot()
        from nav_bringup.navrun.process_backend import snapshot_processes

        return snapshot_processes()

    def signal_process(process, signum):
        if send is not None:
            return send(process, signum)
        from nav_bringup.navrun.process_backend import send_if_same

        return send_if_same(process, signum)

    def platform(plan, events, stopped):
        """按序执行显式平台步骤，每步记录结果且失败不撤销已完成变更.

        每步前及全程后检查停止请求，正在执行的同步 runner 不在此被抢占。

        Args:
            plan: 已准备的平台操作计划。
            events: 本次运行事件接收器。
            stopped: 返回停止信号或 None 的回调。

        Raises:
            StartupInterrupted: 步骤边界发现停止请求。
            PlatformError: 某步骤非零退出，附带此前全部步骤结果。
            Exception: runner 或事件记录失败，之前操作可能已经生效。
        """
        results = []
        for step in plan.platform:
            _interrupt(stopped)
            code, detail = platform_runner(step.argv, step.timeout_sec)
            result = PlatformResult(step.name, step.argv,
                                    'succeeded' if code == 0 else 'failed', code, detail)
            results.append(result)
            events.append('navrun', 'platform_result', 'info' if code == 0 else 'error',
                          asdict(result))
            if code != 0:
                raise PlatformError(tuple(results))
        _interrupt(stopped)

    def preflight(plan, events, stopped):
        """清理原计划内残留候选后重新核对进程与底盘所有权.

        每次快照和发送前检查停止请求，实际身份固定由注入后端负责。
        清理不是回滚事务；重新规划发现新候选或底盘选择变化就要求重新准备。

        Args:
            plan: 含初始预检计划、配置及启动图的准备结果。
            events: 警告与已清理 PID 的事件接收器。
            stopped: 返回停止信号或 None 的回调。

        Raises:
            StartupInterrupted: 观察或信号操作前发现停止请求。
            PreflightError: 清理不安全、未完成或最终所有权与启动图不一致。
            Exception: 配置、进程端口或事件记录失败。
        """
        def checked_snapshot():
            _interrupt(stopped)
            return processes()

        def checked_send(process, signum):
            _interrupt(stopped)
            return signal_process(process, signum)

        for warning in plan.preflight.warnings:
            events.append('navrun', 'preflight_warning', 'warn', {'message': warning})
        removed = cleanup_preflight(plan.preflight, checked_snapshot, checked_send)
        events.append('navrun', 'preflight_cleaned', data={'pids': list(removed)})
        final = plan_preflight(effective_profile(plan), checked_snapshot(),
                               plan.preflight.own_pid, chassis=plan.request.chassis)
        if final.targets or final.start_chassis != plan.bundle.options.start_chassis:
            raise PreflightError('process or chassis ownership changed; prepare a new run')

    def dds(plan, events, stopped):
        """按显式目录与新鲜映射证据执行 DDS 清理，记录部分结果再判失败.

        每次证据查询及结束时检查停止请求；证据不完整的项继续保留。
        不停止 daemon 或更改 DDS 白名单，已删除项不能因后续错误自动恢复。

        Args:
            plan: 统一启动步骤签名中的计划，本闭包不读取它。
            events: DDS 清理结果事件接收器。
            stopped: 返回停止信号或 None 的回调。

        Raises:
            StartupInterrupted: 证据查询或结束时发现停止请求。
            RuntimeError: 结果中存在清理错误。
            Exception: 证据查询、目录操作或事件记录失败。
        """
        def observation():
            _interrupt(stopped)
            return observe()

        selected = plan_hygiene(dds_directory, observation())
        result = cleanup_hygiene(selected, observation)
        severity = 'error' if result.errors else 'warn' if result.retained else 'info'
        events.append('navrun', 'dds_hygiene', severity, asdict(result))
        if result.errors:
            raise RuntimeError('DDS cleanup failed; inspect recorded partial results')
        _interrupt(stopped)

    return (StartupStep('platform', platform), StartupStep('preflight', preflight),
            StartupStep('dds', dds))


def _interrupt(stopped):
    signum = stopped()
    if signum is not None:
        raise StartupInterrupted(signum)
