"""Plan residual cleanup and resolve chassis ownership before launch.

Planning consumes process snapshots and never probes the host. Execution requires
explicit snapshot and identity-safe signal callbacks; there is no default kill
function. Only the initially selected identities may receive signals, and fresh
localization, external-chassis and user-ownership protection is checked each time.
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
    """One process as observed without ROS discovery.

    Attributes:
        pid: Positive operating-system process id.
        ppid: Parent id at snapshot time; reparenting does not change identity.
        started_at: Creation time used to detect PID reuse.
        argv: Exact command line, not a shell-joined command from another process.
        uid: Real user id owning the process.
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
    kill_rsp: bool


@dataclass(frozen=True)
class PreflightPlan:
    """Resolved chassis choice and the only identities eligible for cleanup.

    Attributes:
        start_chassis: Whether launch should own a new driver.
        targets: Initial cleanup generation, with ancestors before descendants.
        protected_pids: Protected self ancestry and external process trees.
        warnings: Explanations for matched processes intentionally not targeted.
        own_pid: Supervisor identity whose ancestry and user must be preserved.
    """

    start_chassis: bool
    targets: Tuple[ProcessInfo, ...]
    protected_pids: Tuple[int, ...]
    warnings: Tuple[str, ...]
    own_pid: int
    _policy: _Policy


class PreflightError(RuntimeError):
    """Cleanup cannot safely finish; navigation startup must not assume a clean graph."""


def _patterns(values: Any, label: str) -> Tuple[str, ...]:
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
    preflight = profile.get('preflight', {})
    chassis = profile.get('chassis', {})
    if not isinstance(chassis, Mapping):
        raise ValueError('chassis must be a mapping')
    driver = chassis.get('driver', {})
    localization = profile.get('localization', {})
    if not all(isinstance(value, Mapping) for value in (preflight, driver, localization)):
        raise ValueError('preflight, chassis.driver and localization must be mappings')
    switches = [preflight.get(name, False) for name in (
        'kill_foreign_stacks', 'kill_stale_robot_state_publishers')]
    if any(type(value) is not bool for value in switches):
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
        protected, switches[0], switches[1])


def _command(process: ProcessInfo) -> str:
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


def _table(processes: Sequence[ProcessInfo], own_pid: int) -> Dict[int, ProcessInfo]:
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
    command = _command(process)
    if re.search(r'(?:^|/)ros2 (?:launch|run) ', command):
        return True
    argv = process.argv
    return (bool(argv) and PurePath(argv[0]).name in ('bash', 'sh', 'dash', 'zsh')
            and len(argv) > 1 and (argv[1] == '-c' or not argv[1].startswith('-')))


def _protected_tree(seed: int, table: Mapping[int, ProcessInfo], barriers: set[int]) -> set[int]:
    # Ascend only inside the external branch, stopping before shared supervisor
    # ancestors. Protect sibling sensor/RSP processes under that external launch.
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
        selected = (_matches(command, policy.kill)
                    or (policy.kill_rsp and _matches(command, [r'(?:^|/)robot_state_publisher$']))
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
    """Classify a process snapshot without sending signals or reading the host.

    Args:
        profile: Mode-merged robot profile and residual/foreign/driver match patterns.
        processes: Complete process identities relevant to the supervisor and external trees.
        own_pid: Supervisor id, present in the snapshot.
        chassis: auto detects an existing driver; on owns one; off preserves external drivers.

    Returns:
        A frozen cleanup generation, protected PIDs, warnings and resolved driver ownership.

    Raises:
        ValueError: Invalid policy, selector, process identity or ambiguous ancestry.
    """
    if chassis not in ('auto', 'on', 'off'):
        raise ValueError('chassis must be auto, on or off')
    return _make_plan(_policy(profile), _table(processes, own_pid), own_pid, chassis)


def _identity(process: ProcessInfo) -> tuple:
    return process.pid, process.started_at, process.uid, process.argv


def _remaining(plan: PreflightPlan, snapshot: Sequence[ProcessInfo]) -> Tuple[ProcessInfo, ...]:
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
    """TERM and, after a bounded wait, KILL only the initial cleanup generation.

    The signal callback must pin/recheck the supplied creation identity at the
    actual send (for example using pidfd); returning False reports a race or exit.
    No real signal callback is provided implicitly. All tests inject fake worlds.

    Args:
        plan: Initial read-only cleanup decision.
        snapshot: Fresh complete process view, consulted before every signal and poll.
        send_if_same: Identity-safe signal backend, mandatory and never a bare os.kill.
        clock: Monotonic wall clock, replaceable by virtual time in tests.
        sleep: Bounded polling delay, replaceable in tests.
        dry_run: Return planned PIDs without calling runtime hooks.
        term_timeout_sec: Graceful shutdown wait, preserving the old eight-second policy.
        kill_timeout_sec: Confirmation wait after the final kill.
        poll_sec: Maximum delay between observations.

    Returns:
        Initial target PIDs confirmed absent, or planned PIDs for dry-run.

    Raises:
        ValueError: Invalid deadlines, switch or process snapshots.
        PreflightError: New/protected/reused identities, failed signals or live survivors.
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
