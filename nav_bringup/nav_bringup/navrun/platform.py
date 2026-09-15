"""Profile-controlled Jetson setup with noninteractive, bounded command execution.

Planning is pure. Execution uses no password, shell, stdin prompt or permission
configuration edit. Unit tests inject runners and never change real power modes.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import subprocess
from typing import Any, Callable, Mapping, Sequence, Tuple


CommandRunner = Callable[[Sequence[str], float], Tuple[int, str]]


@dataclass(frozen=True)
class PlatformStep:
    """One ordered platform command.

    Attributes:
        name: Stable step name used by run events.
        argv: Exact argument vector; no shell interpolation.
        timeout_sec: Positive wall-clock command deadline.
    """

    name: str
    argv: Tuple[str, ...]
    timeout_sec: float


@dataclass(frozen=True)
class PlatformResult:
    """A planned or attempted platform operation.

    Attributes:
        name: Stable operation name.
        argv: Exact invocation.
        status: planned, succeeded or failed.
        return_code: Exit status, or None for dry-run.
        detail: Command output or a bounded failure description.
    """

    name: str
    argv: Tuple[str, ...]
    status: str
    return_code: int | None
    detail: str


class PlatformError(RuntimeError):
    """Platform setup failed; no later platform step has been attempted."""

    def __init__(self, results: Tuple[PlatformResult, ...]) -> None:
        """Retain successful steps and the first failure for the run recorder.

        Args:
            results: Ordered attempts ending in one failed operation.
        """
        self.results = results
        last = results[-1]
        super().__init__(
            f'platform {last.name} failed ({last.return_code}): {last.detail}; '
            'check tools and the operator-managed sudoers policy; no permissions were changed')


def plan_platform(profile: Mapping[str, Any]) -> Tuple[PlatformStep, ...]:
    """Resolve platform operations without probing or changing the host.

    Args:
        profile: Mode-merged robot profile, optionally containing platform settings.

    Returns:
        Ordered MAXN and clock commands, empty for disabled or non-Jetson profiles.

    Raises:
        ValueError: Invalid types, mode id, timeout or non-Jetson enabled operations.
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
    """Execute one noninteractive command with a wall-clock deadline.

    Args:
        argv: Program and its exact arguments.
        timeout_sec: Deadline passed to subprocess.run.

    Returns:
        Exit code and combined output. Missing, denied or timed-out commands use
        statuses 127, 126 and 124 respectively, without retrying or prompting.
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
    """Apply all validated platform steps or describe them without executing.

    Args:
        profile: Mode-merged robot profile.
        runner: Injectable command executor; tests must supply a fake.
        dry_run: Return planned records without calling the runner.

    Returns:
        Ordered operation records, empty when platform setup is disabled.

    Raises:
        ValueError: Invalid platform settings or dry-run switch.
        PlatformError: The first command failure, carrying partial results.
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
