"""Plan and exercise residual cleanup using synthetic processes and virtual time only."""

from dataclasses import replace
import signal

import pytest

from nav_bringup.navrun.preflight import cleanup_preflight, plan_preflight
from nav_bringup.navrun.preflight import PreflightError, ProcessInfo


SELF = ProcessInfo(100, 1, 10.0, ('navrun', 'up'), 1000)


def _process(pid, command, parent=1, started=None, uid=1000):
    return ProcessInfo(
        pid, parent, float(pid if started is None else started), tuple(command), uid)


def _profile():
    return {
        'preflight': {
            'kill_patterns': ['navigation_executor.py', 'nav2_controller/controller_server'],
            'foreign_patterns': ['move_group', 'arm_node'],
            'kill_foreign_stacks': False,
            'kill_stale_robot_state_publishers': True,
        },
        'chassis': {'driver': {'detect_patterns': [
            'wheeltec_robot_node', 'wheeltec_driver_node']}},
        'localization': {'process_name': 'run_loc_online'},
    }


def _plan(processes=(), chassis='auto', profile=None):
    return plan_preflight(profile or _profile(), (SELF, *processes), SELF.pid, chassis)


@pytest.mark.parametrize('choice, existing, expected', [
    ('auto', False, True), ('auto', True, False), ('on', False, True),
    ('on', True, True), ('off', False, False), ('off', True, False),
])
def test_chassis_switches_resolve_against_existing_driver(choice, existing, expected):
    """Auto skips an existing chassis; explicit off never targets its process."""
    driver = _process(200, ['/install/lib/wheeltec_driver/wheeltec_driver_node'])
    plan = _plan([driver] if existing else [], choice)
    assert plan.start_chassis is expected
    assert [item.pid for item in plan.targets] == ([200] if existing and choice == 'on' else [])


def test_matching_ignores_editor_arguments_and_inline_shell_text():
    """A file being edited or mentioned in an SSH command is not an executable candidate."""
    processes = [
        _process(201, ['vim', '/install/lib/node/navigation_executor.py']),
        _process(202, ['bash', '-c', 'echo navigation_executor.py']),
        _process(203, ['python3', '-c', 'print("navigation_executor.py")']),
        _process(204, ['python3', '/install/lib/node/navigation_executor.py']),
        _process(205, ['/install/lib/nav2_controller/controller_server', '--ros-args']),
    ]
    assert [item.pid for item in _plan(processes).targets] == [204, 205]


def test_ros2_launcher_matching_survives_python_wrapper():
    """ROS CLI wrappers expose only command/package/launch-file, not arbitrary arguments."""
    profile = _profile()
    profile['preflight']['kill_patterns'] = ['launch nav_bringup nav.launch.py']
    launcher = _process(200, ['python3', '/opt/ros/bin/ros2', 'launch',
                              'nav_bringup', 'nav.launch.py', 'robot:=example'])
    assert _plan([launcher], profile=profile).targets == (launcher,)


def test_localization_ros2_run_is_protected_before_its_child_exists():
    """A starting localization launcher is protected even before its node becomes visible."""
    profile = _profile()
    profile['preflight']['kill_patterns'] = ['.*']
    launcher = _process(200, ['python3', '/opt/ros/bin/ros2', 'run',
                              'localization', 'run_loc_online'])
    plan = _plan([launcher], profile=profile)
    assert not plan.targets
    assert 200 in plan.protected_pids


def test_localization_launcher_siblings_and_descendants_are_protected():
    """A provider's launch parent and sibling RSP/IMU processes remain outside cleanup."""
    launcher = _process(200, ['ros2', 'launch', 'localization', 'online.launch.py'])
    provider = _process(201, ['/install/lib/localization/run_loc_online'], parent=200)
    sibling = _process(
        202, ['/install/lib/robot_state_publisher/robot_state_publisher'], parent=200)
    child = _process(203, ['imu_reader'], parent=202)
    unrelated = _process(300, ['/install/lib/node/navigation_executor.py'])
    plan = _plan([launcher, provider, sibling, child, unrelated])
    assert plan.targets == (unrelated,)
    assert {200, 201, 202, 203}.issubset(plan.protected_pids)
    assert any('protected' in warning and '202' in warning for warning in plan.warnings)


def test_shared_desktop_ancestor_does_not_hide_unrelated_navigation():
    """Protection covers a launch branch, not every tab under the user's terminal server."""
    terminal = _process(190, ['terminal-server'])
    launcher = _process(200, ['ros2', 'launch', 'localization', 'online.launch.py'], parent=190)
    provider = _process(201, ['run_loc_online'], parent=200)
    stale = _process(202, ['navigation_executor.py'], parent=190)
    plan = _plan([terminal, launcher, provider, stale])
    assert plan.targets == (stale,)
    assert 190 in plan.protected_pids


def test_self_ancestors_are_excluded_without_protecting_unrelated_siblings():
    """The supervisor's own ancestry is never signaled, while stale sibling nodes match."""
    parent = _process(99, ['python3', '/install/lib/node/navigation_executor.py'])
    own = replace(SELF, ppid=99)
    stale = _process(201, ['python3', '/install/lib/node/navigation_executor.py'], parent=99)
    plan = plan_preflight(_profile(), [own, parent, stale], own.pid, 'auto')
    assert plan.targets == (stale,)


def test_foreign_stack_is_warn_only_even_if_a_broad_kill_pattern_matches():
    """Default MoveIt/arm policy wins over the generic residual match list."""
    profile = _profile()
    profile['preflight']['kill_patterns'] = ['.*']
    foreign = _process(201, ['/install/lib/moveit_ros_move_group/move_group'])
    plan = _plan([foreign], profile=profile)
    assert not plan.targets
    assert any('foreign' in warning for warning in plan.warnings)
    profile['preflight']['kill_foreign_stacks'] = True
    assert _plan([foreign], profile=profile).targets == (foreign,)


def test_external_chassis_subtree_and_its_launch_parent_are_preserved():
    """Auto/off must not stop an external driver's launch parent or its RSP sibling."""
    launcher = _process(200, ['ros2', 'launch', 'wheeltec', 'driver.launch.py'])
    driver = _process(201, ['wheeltec_driver_node'], parent=200)
    sibling = _process(202, ['robot_state_publisher'], parent=200)
    plan = _plan([launcher, driver, sibling], 'auto')
    assert not plan.start_chassis
    assert not plan.targets
    assert {200, 201, 202}.issubset(plan.protected_pids)


def test_different_user_is_never_a_cleanup_target():
    """A matching name does not grant ownership over another account's process."""
    foreign_uid = _process(201, ['navigation_executor.py'], uid=2000)
    plan = _plan([foreign_uid])
    assert not plan.targets
    assert any('different user' in warning for warning in plan.warnings)


@pytest.mark.parametrize('change', [
    {'kill_patterns': 'executor'}, {'kill_patterns': ['[']},
    {'kill_foreign_stacks': 'false'}, {'kill_stale_robot_state_publishers': 1},
    {'foreign_patterns': [False]},
])
def test_bad_policy_is_rejected_before_any_cleanup(change):
    """No regex or boolean coercion changes the cleanup scope."""
    profile = _profile()
    profile['preflight'].update(change)
    with pytest.raises(ValueError):
        _plan(profile=profile)


@pytest.mark.parametrize('choice', ['yes', '', None])
def test_unknown_chassis_selector_is_rejected(choice):
    """Chassis ownership must be explicit or resolved by auto detection."""
    with pytest.raises(ValueError):
        _plan(chassis=choice)


class _World:
    def __init__(self, processes, ignore_term=False):
        self.processes = {item.pid: item for item in (SELF, *processes)}
        self.signals = []
        self.now = 0.0
        self.ignore_term = ignore_term

    def snapshot(self):
        return tuple(self.processes.values())

    def send(self, process, signum):
        self.signals.append((process.pid, signum, self.now))
        current = self.processes.get(process.pid)
        if current != process:
            return False
        if signum == signal.SIGKILL or not self.ignore_term:
            del self.processes[process.pid]
        return True

    def sleep(self, duration):
        self.now += duration

    def run(self, plan, **kwargs):
        return cleanup_preflight(plan, self.snapshot, self.send,
                                 clock=lambda: self.now, sleep=self.sleep, **kwargs)


def test_cleanup_term_success_needs_no_kill_or_wait():
    """Graceful exit ends the cleanup promptly."""
    target = _process(201, ['navigation_executor.py'])
    world = _World([target])
    result = world.run(_plan([target]))
    assert world.signals == [(201, signal.SIGTERM, 0.0)]
    assert result == (201,)


def test_launcher_is_signaled_before_child_and_reparenting_is_allowed():
    """Parent-first cleanup remains valid when orphaned children are adopted by init."""
    profile = _profile()
    profile['preflight']['kill_patterns'].append('launch test stack.launch.py')
    launcher = _process(300, ['ros2', 'launch', 'test', 'stack.launch.py'])
    child = _process(201, ['navigation_executor.py'], parent=300)
    plan = _plan([launcher, child], profile=profile)
    world = _World([launcher, child])
    original = world.send

    def send(process, signum):
        result = original(process, signum)
        if process.pid == 300:
            world.processes[201] = replace(child, ppid=1)
        return result

    world.send = send
    world.run(plan)
    assert [pid for pid, _, _ in world.signals] == [300, 201]


def test_exit_between_snapshot_and_send_is_harmless():
    """A sender reporting an already exited target does not cause a false startup failure."""
    target = _process(201, ['navigation_executor.py'])
    world = _World([target])

    def exited(process, _signum):
        del world.processes[process.pid]
        return False

    world.send = exited
    assert world.run(_plan([target])) == (201,)


def test_new_residual_with_an_initially_empty_plan_blocks_without_signals():
    """The final view must still be clean when a new program appeared after planning."""
    world = _World([_process(201, ['navigation_executor.py'])])
    with pytest.raises(PreflightError, match='new residual'):
        world.run(_plan())
    assert not world.signals


def test_term_deadline_is_eight_seconds_before_kill():
    """Unresponsive initial targets get one TERM, bounded waiting, then one KILL."""
    target = _process(201, ['navigation_executor.py'])
    world = _World([target], ignore_term=True)
    world.run(_plan([target]))
    assert world.signals[0] == (201, signal.SIGTERM, 0.0)
    assert world.signals[1][:2] == (201, signal.SIGKILL)
    assert world.signals[1][2] == pytest.approx(8.0)


def test_pid_reuse_is_not_signaled_and_blocks_startup():
    """A new process with the same PID is not a stale member of this cleanup generation."""
    target = _process(201, ['navigation_executor.py'])
    world = _World([replace(target, started_at=999.0)])
    with pytest.raises(PreflightError, match='new|identity'):
        world.run(_plan([target]))
    assert not world.signals


def test_replacement_appearing_during_term_wait_is_not_killed():
    """Retries cannot broaden the initial target set to fresh executor generations."""
    target = _process(201, ['navigation_executor.py'])
    world = _World([target], ignore_term=True)

    def spawn(_duration):
        world.now = 8.0
        world.processes[201] = replace(target, started_at=999.0)

    world.sleep = spawn
    with pytest.raises(PreflightError):
        world.run(_plan([target]))
    assert [sig for _, sig, _ in world.signals] == [signal.SIGTERM]


def test_new_localization_descendant_prevents_signaling_old_parent():
    """Fresh protection evidence is applied before every signal, not just at planning."""
    profile = _profile()
    profile['preflight']['kill_patterns'] = ['launch shared stack.launch.py']
    target = _process(201, ['ros2', 'launch', 'shared', 'stack.launch.py'])
    plan = _plan([target], profile=profile)
    world = _World([target, _process(202, ['run_loc_online'], parent=201)])
    with pytest.raises(PreflightError, match='protected'):
        world.run(plan)
    assert not world.signals


def test_sender_identity_race_is_a_failure_not_a_retry():
    """The signal backend can reject an identity change after the fresh snapshot."""
    target = _process(201, ['navigation_executor.py'])
    world = _World([target])
    world.send = lambda *_: False
    with pytest.raises(PreflightError, match='identity'):
        world.run(_plan([target]))


def test_dry_run_does_not_observe_wait_or_signal():
    """A prepared dry-run result does not call any runtime hook."""
    target = _process(201, ['navigation_executor.py'])

    def forbidden(*_args):
        pytest.fail('runtime hook called')

    assert cleanup_preflight(_plan([target]), forbidden, forbidden, clock=forbidden,
                             sleep=forbidden, dry_run=True) == (201,)


def test_survivor_after_kill_does_not_report_success():
    """Permission errors or ineffective signals cannot let startup race a live residual."""
    target = _process(201, ['navigation_executor.py'])
    world = _World([target], ignore_term=True)
    world.send = lambda *_: True
    with pytest.raises(PreflightError, match='remain'):
        world.run(_plan([target]))
    assert world.now == pytest.approx(8.5)
