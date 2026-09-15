"""Bind node parameter names while preserving the legacy chassis and dock contracts."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping

from nav_bringup.launching.graph import build_graph, GraphOptions, NAVIGATION_MODES
from nav_bringup.navcfg import render


_ADAPTER_KEYS = {
    'input_cmd_vel_topic': 'input_topic', 'cmd_vel_timeout': 'cmd_timeout_sec',
    'min_linear_x': 'min_linear_x', 'max_linear_x': 'hard_max_linear_x',
    'min_angular_z': 'min_angular_z', 'max_angular_z': 'hard_max_angular_z',
    'min_stationary_angular_z': 'min_stationary_angular_z',
    'stationary_angular_threshold': 'stationary_angular_threshold', 'publish_hz': 'publish_hz',
}
_TELEOP_KEYS = {
    'cmd_vel_topic': 'cmd_topic', 'linear_speed': 'linear_speed', 'angular_speed': 'angular_speed',
    'max_linear_x': 'max_linear_x', 'max_angular_z': 'max_angular_z',
    'publish_hz': 'publish_hz', 'key_timeout': 'key_timeout_sec',
    'linear_deadzone': 'linear_deadzone', 'angular_deadzone': 'angular_deadzone',
    'layout': 'layout', 'tty_device': 'tty_device',
}
_SCAN_KEYS = (
    'transform_tolerance', 'min_height', 'max_height', 'angle_min', 'angle_max',
    'angle_increment', 'scan_time', 'range_min', 'range_max', 'use_inf')


def _node(name: str, parameters: Mapping[str, Any]) -> Dict[str, Any]:
    return {name: {'ros__parameters': dict(parameters)}}


def node_parameters(
    options: GraphOptions, profile: Mapping[str, Any], navigation: Mapping[str, Any],
) -> Dict[str, Any]:
    """Build parameter trees for the processes selected by an already resolved graph.

    Chassis and teleop parameter names remain compatible with david_move and
    david_keyboard during migration. Dock tolerances come from the mode profile;
    its remaining defaults preserve the precise launch policy and user stop rate.

    Args:
        options: Resolved graph options with profile topic bindings.
        profile: Robot profile after its mode overlay.
        navigation: Validated Nav2 tree and URDF, or empty for chassis tools.

    Returns:
        Parameter trees keyed by the run-local parameter file stem.

    Raises:
        ValueError: If required profile values are absent or have an invalid shape.
    """
    graph = build_graph(options)
    output: Dict[str, Any] = {}
    if options.start_chassis:
        parameters = render.profile_value(profile, 'chassis.driver.parameters')
        if not isinstance(parameters, dict):
            raise ValueError('chassis.driver.parameters must be a mapping')
        output['driver'] = _node('wheeltec_robot', parameters)
    adapter = {target: render.profile_value(profile, 'chassis.adapter.' + source)
               for target, source in _ADAPTER_KEYS.items()}
    adapter['output_cmd_vel_topic'] = options.chassis_command_topic
    output['adapter'] = _node('chassis_adapter', adapter)
    if options.mode == 'chassis':
        return output
    if options.mode == 'teleop':
        teleop = {target: render.profile_value(profile, 'teleop.' + source)
                  for target, source in _TELEOP_KEYS.items()}
        teleop['guard_topics'] = [options.chassis_command_topic, options.nav_command_topic]
        teleop['discovery_sec'] = render.profile_value(profile, 'teleop').get('discovery_sec', 1.0)
        output['teleop'] = _node('chassis_teleop', teleop)
        return output
    if options.mode not in NAVIGATION_MODES:
        raise ValueError('unsupported navigation mode')
    output['nav2'] = navigation['nav2']
    output['lifecycle'] = _node('lifecycle_manager_navigation', {
        'use_sim_time': False, 'autostart': True, 'node_names': list(graph.lifecycle_nodes)})
    if options.map_server:
        output['map_server'] = _node('map_server', {
            'use_sim_time': False,
            'yaml_filename': str(Path(options.run_dir) / 'params/rendered/map.yaml'),
            'topic_name': render.profile_value(profile, 'nav2.map_topic'),
            'frame_id': render.profile_value(profile, 'frames.map')})
    if options.robot_state:
        output['robot_state'] = _node('robot_state_publisher', {
            'use_sim_time': False, 'robot_description': navigation['robot_description']})
    if options.scan_bridge:
        scan = {key: render.profile_value(profile, 'sensors.scan_from_cloud.' + key)
                for key in _SCAN_KEYS}
        scan['target_frame'] = render.profile_value(profile, 'frames.scan_target')
        output['scan_bridge'] = _node('pointcloud_to_laserscan', scan)
    if options.mode == 'precise':
        dock = {
            'approach_dist': 0.5, 'max_yaw_retries': 0, 'goal_topic': '/dock_goal',
            'action_name': 'dock_to_pose', 'nav_timeout_sec': 120.0, 'spin_timeout_sec': 30.0,
            'static_occupied_threshold': 50, 'settle_max_rate_rad_s': 0.05,
            'settle_window_sec': 0.4, 'settle_timeout_sec': 8.0,
            'xy_ok': render.profile_value(profile, 'goal.xy_tolerance'),
            'yaw_ok': render.profile_value(profile, 'goal.yaw_tolerance'),
            'robot_base_frame': render.profile_value(profile, 'frames.nav_base'),
            'global_frame': render.profile_value(profile, 'frames.map'),
            'static_map_topic': render.profile_value(profile, 'nav2.map_topic'),
            'footprint': render.VALUE_FORMATS['footprint_string'](
                render.profile_value(profile, 'footprint')),
        }
        output['dock'] = _node('dock_navigator', dock)
    if options.rviz:
        output['rviz'] = _node('rviz2', {'use_sim_time': False})
    return output
