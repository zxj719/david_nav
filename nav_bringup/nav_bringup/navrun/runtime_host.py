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
