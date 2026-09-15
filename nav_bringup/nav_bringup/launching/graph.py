"""Describe the five navigation launch modes without creating processes or files.

The supervisor prepares run-local parameter files. This module names those
files and describes ownership, restart budgets and readiness dependencies;
the launch adapter consumes the result. It never probes or starts alias 3.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import PurePath
from typing import Tuple


MODES = ('tour', 'precise', 'bench_slamgt', 'chassis', 'teleop')
NAVIGATION_MODES = MODES[:3]
CORE_NODES = (
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
)


@dataclass(frozen=True)
class GraphOptions:
    """Resolved graph switches passed by navrun, not robot tuning parameters.

    Attributes:
        mode: One of the five supported modes.
        run_dir: Absolute directory holding pre-rendered parameter files.
        start_chassis: Whether this launch owns a newly started driver.
        start_executor: Whether the WS executor is enabled after Nav2 readiness.
        map_server: Whether this launch owns the map publisher.
        scan_bridge: Whether pointcloud preprocessing is enabled.
        robot_state: Whether to publish the supplied robot description.
        rviz: Whether to start the supplied RViz configuration.
        chassis_delay_sec: Delay before starting navigation after our driver.
        driver_restarts: Number of allowed driver restarts before stack shutdown.
        rviz_delay_sec: Delay after navigation launch before RViz startup.
        driver_package: Package selected by the robot profile.
        driver_executable: Driver entry point selected by the profile.
        nav_command_topic: Controller and behavior output when the smoother is enabled.
        smooth_command_topic: Adapter input, fed by the smoother or directly by Nav2.
        chassis_command_topic: Adapter output consumed by the driver.
        chassis_odom_topic: Wheel odometry published by the driver.
        cloud_topic: Optional preprocessing input.
        scan_topic: Optional preprocessing output.
        executor_arguments: Frozen legacy executor CLI arguments until Phase 6.
        ready_timeout_sec: Maximum wall-clock wait before refusing to start the executor.
        ready_action: Navigation action server checked without submitting any goal.
        supervisor_keyboard: Teleop keyboard is owned directly by navrun, not launch.
        velocity_smoother: Keep the legacy smoothing hop unless explicitly disabled.
    """

    mode: str
    run_dir: str
    start_chassis: bool = True
    start_executor: bool = False
    map_server: bool = True
    scan_bridge: bool = False
    robot_state: bool = True
    rviz: bool = False
    chassis_delay_sec: float = 0.0
    driver_restarts: int = 1
    rviz_delay_sec: float = 0.0
    driver_package: str = 'wheeltec_driver'
    driver_executable: str = 'wheeltec_driver_node'
    nav_command_topic: str = 'cmd_vel_nav'
    smooth_command_topic: str = 'cmd_vel_smoothed'
    chassis_command_topic: str = 'cmd_vel'
    chassis_odom_topic: str = 'odom'
    cloud_topic: str = 'cloud_in'
    scan_topic: str = 'scan'
    executor_arguments: Tuple[str, ...] = ()
    ready_timeout_sec: float = 90.0
    ready_action: str = '/navigate_to_pose'
    supervisor_keyboard: bool = False
    velocity_smoother: bool = True


@dataclass(frozen=True)
class ProcessSpec:
    """One owned process and its operational contract.

    Attributes:
        name: Stable process/node identity within the graph.
        package: ROS package providing the executable.
        executable: Executable inside that package.
        parameter_file: Run-local ROS parameter file, or empty for a plain program.
        remappings: Topic remappings, without robot-specific frame literals.
        arguments: Non-parameter CLI arguments.
        delay_sec: Start delay relative to launch start.
        restart_limit: Allowed respawns before global shutdown.
        critical: Whether an unexpected final exit shuts down the whole graph.
        wait_for_navigation: Whether a readiness gate precedes process execution.
        ros_node: Whether ROS node name/parameter arguments may be appended.
        ready_timeout_sec: Deadline of the optional navigation readiness gate.
        ready_action: Action name used by the optional readiness gate.
    """

    name: str
    package: str
    executable: str
    parameter_file: str = ''
    remappings: Tuple[Tuple[str, str], ...] = ()
    arguments: Tuple[str, ...] = ()
    delay_sec: float = 0.0
    restart_limit: int = 0
    critical: bool = True
    wait_for_navigation: bool = False
    ros_node: bool = True
    ready_timeout_sec: float = 90.0
    ready_action: str = '/navigate_to_pose'


@dataclass(frozen=True)
class LaunchGraph:
    """Complete owned graph for one mode.

    Attributes:
        mode: Selected mode.
        processes: Owned processes, in declaration order.
        lifecycle_nodes: Nav2 activation order, with an owned map server first.
        supervisor_keyboard: Exclude the explicitly supervised keyboard from launch actions.
    """

    mode: str
    processes: Tuple[ProcessSpec, ...]
    lifecycle_nodes: Tuple[str, ...]
    supervisor_keyboard: bool = False


def _validate(options: GraphOptions) -> None:
    if options.mode not in MODES:
        raise ValueError(f'unknown launch mode: {options.mode!r}')
    if not PurePath(options.run_dir).is_absolute() or '..' in PurePath(options.run_dir).parts:
        raise ValueError('run_dir must be an absolute normalized run directory')
    for name in ('start_chassis', 'start_executor', 'map_server', 'scan_bridge',
                 'robot_state', 'rviz', 'supervisor_keyboard', 'velocity_smoother'):
        if not isinstance(getattr(options, name), bool):
            raise ValueError(f'{name} must be a resolved boolean')
    for delay in (options.chassis_delay_sec, options.rviz_delay_sec):
        if isinstance(delay, bool) or not math.isfinite(delay) or delay < 0:
            raise ValueError('launch delays must be finite and nonnegative')
    if type(options.driver_restarts) is not int or options.driver_restarts < 0:
        raise ValueError('driver_restarts must be a nonnegative integer')
    if isinstance(options.ready_timeout_sec, bool) or not math.isfinite(
            options.ready_timeout_sec) or options.ready_timeout_sec <= 0:
        raise ValueError('ready_timeout_sec must be finite and positive')
    if not isinstance(options.executor_arguments, tuple) or not all(
            isinstance(argument, str) for argument in options.executor_arguments):
        raise ValueError('executor_arguments must be a tuple of strings')
    if options.start_executor and options.mode not in ('tour', 'precise'):
        raise ValueError('the executor is only valid in tour or precise mode')
    if options.supervisor_keyboard and options.mode != 'teleop':
        raise ValueError('supervisor keyboard ownership is only valid in teleop mode')
    if options.scan_bridge and options.mode not in NAVIGATION_MODES:
        raise ValueError('preprocessing is only valid in a navigation mode')
    for name in ('driver_package', 'driver_executable', 'nav_command_topic',
                 'smooth_command_topic', 'chassis_command_topic', 'chassis_odom_topic',
                 'cloud_topic', 'scan_topic', 'ready_action'):
        if not isinstance(getattr(options, name), str) or not getattr(options, name).strip():
            raise ValueError(f'{name} must be a nonempty string')


def build_graph(options: GraphOptions) -> LaunchGraph:
    """Resolve the owned process graph without importing runtime implementations.

    Args:
        options: Resolved mode switches and profile-owned topic names.

    Returns:
        The immutable process graph and lifecycle activation order.

    Raises:
        ValueError: If switches, mode, paths or restart/delay values are invalid.
    """
    _validate(options)
    root = PurePath(options.run_dir)

    def params(name: str) -> str:
        return str(root / 'params' / 'rendered' / f'{name}.yaml')

    processes = []
    if options.start_chassis:
        processes.append(ProcessSpec(
            'wheeltec_robot', options.driver_package, options.driver_executable,
            params('driver'),
            (('cmd_vel', options.chassis_command_topic), ('odom', options.chassis_odom_topic)),
            restart_limit=options.driver_restarts))
    delay = options.chassis_delay_sec if options.start_chassis else 0.0
    processes.append(ProcessSpec(
        'chassis_adapter', 'chassis_adapter', 'chassis_adapter', params('adapter'),
        delay_sec=delay))
    if options.mode == 'chassis':
        return LaunchGraph(options.mode, tuple(processes), ())
    if options.mode == 'teleop':
        processes.append(ProcessSpec(
            'chassis_teleop', 'chassis_teleop',
            'chassis_teleop_managed' if options.supervisor_keyboard else 'chassis_teleop',
            params('teleop'),
            delay_sec=delay))
        return LaunchGraph(options.mode, tuple(processes), (), options.supervisor_keyboard)

    tf = (('/tf', 'tf'), ('/tf_static', 'tf_static'))
    lifecycle = []
    if options.map_server:
        lifecycle.append('map_server')
        processes.append(ProcessSpec(
            'map_server', 'nav2_map_server', 'map_server', params('map_server'),
            tf, delay_sec=delay))
    for package, executable in CORE_NODES:
        if executable == 'velocity_smoother' and not options.velocity_smoother:
            continue
        lifecycle.append(executable)
        remappings = tf
        if executable != 'planner_server':
            remappings += (('cmd_vel', options.nav_command_topic if options.velocity_smoother
                            else options.smooth_command_topic),)
        if executable == 'velocity_smoother':
            remappings += (('cmd_vel_smoothed', options.smooth_command_topic),)
        processes.append(ProcessSpec(
            executable, package, executable, params('nav2'), remappings, delay_sec=delay))
    processes.append(ProcessSpec(
        'lifecycle_manager_navigation', 'nav2_lifecycle_manager', 'lifecycle_manager',
        params('lifecycle'), delay_sec=delay))
    if options.robot_state:
        processes.append(ProcessSpec(
            'robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
            params('robot_state'), delay_sec=delay))
    if options.scan_bridge:
        processes.append(ProcessSpec(
            'pointcloud_to_laserscan', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
            params('scan_bridge'),
            (('cloud_in', options.cloud_topic), ('scan', options.scan_topic)), delay_sec=delay))
    if options.mode == 'precise':
        processes.append(ProcessSpec(
            'dock_navigator', 'david_nav', 'dock_navigator', params('dock'), delay_sec=delay))
    if options.start_executor:
        processes.append(ProcessSpec(
            'navigation_executor', 'david_nav', 'navigation_executor.py',
            arguments=options.executor_arguments, delay_sec=delay,
            wait_for_navigation=True, ros_node=False,
            ready_timeout_sec=options.ready_timeout_sec, ready_action=options.ready_action))
    if options.rviz:
        processes.append(ProcessSpec(
            'rviz2', 'rviz2', 'rviz2', params('rviz'),
            arguments=('-d', str(root / 'inputs' / 'navigation.rviz')),
            delay_sec=delay + options.rviz_delay_sec, critical=False))
    return LaunchGraph(options.mode, tuple(processes), tuple(lifecycle))


def exit_decision(process: ProcessSpec, prior_restarts: int, shutting_down: bool) -> str:
    """Decide restart versus shutdown for an owned process exit.

    Args:
        process: The exited process contract.
        prior_restarts: Respawns already used by this process instance lineage.
        shutting_down: Whether global shutdown is already underway.

    Returns:
        One of ignore, restart or shutdown; shutdown never restarts a process.

    Raises:
        ValueError: If the restart counter is invalid.
    """
    if type(prior_restarts) is not int or prior_restarts < 0:
        raise ValueError('prior_restarts must be a nonnegative integer')
    if shutting_down:
        return 'ignore'
    if prior_restarts < process.restart_limit:
        return 'restart'
    return 'shutdown' if process.critical else 'ignore'
