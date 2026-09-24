"""把唯一启动图中的节点绑定到已验证档案参数.

运行期不读取其它参数源；底盘、遥控和入位均从同一有效档案生成。
"""

from __future__ import annotations

import math
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


def _node(name: str, parameters: Mapping[str, Any]) -> Dict[str, Any]:
    """把参数封装为单节点 ROS 参数文件片段，仅浅复制顶层参数表.

    嵌套值仍引用输入对象；是否需要更深复制由构建流程负责。

    Args:
        name: ROS 参数文件中的节点键。
        parameters: 节点的有效参数映射。

    Returns:
        以节点名和 ros__parameters 两层键包装的新字典。
    """
    return {name: {'ros__parameters': dict(parameters)}}


def node_parameters(
    options: GraphOptions, profile: Mapping[str, Any], navigation: Mapping[str, Any],
) -> Dict[str, Any]:
    """为已解析启动图选中的进程构建参数树.

    底盘与遥控参数名由各自节点的公共接口定义。入位容差取自模式档案，
    其余默认值沿用精准模式的既有启动策略与用户设定的停止速率。
    入位参数只在精准模式生成——现场的导览模式本来就没有入位节点。

    Args:
        options: 已解析的启动图选项，含档案给出的话题绑定。
        profile: 叠加模式差异之后的机器人档案。
        navigation: 已校验的 Nav2 参数树与 URDF；底盘类工具为空。

    Returns:
        以本次运行参数文件名（不含扩展名）为键的参数树。

    Raises:
        ValueError: 缺少必需的档案取值，或取值形状非法。
    """
    graph = build_graph(options)
    output: Dict[str, Any] = {}
    if options.start_executor or options.mode == 'precise':
        # A CLI input, not a ROS parameter tree. Keep the mode-merged robot values
        # instead of making the task manager fall back to its installed David profile.
        output['executor_profile'] = dict(profile)
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
    health_keys = ('max_tf_age_sec', 'max_divergence_m', 'divergence_window_sec', 'action')
    health = {name: render.profile_value(profile, 'localization_health.' + name)
              for name in health_keys}
    if any(type(health[name]) not in (int, float) or not math.isfinite(health[name])
           or health[name] <= 0 for name in (
               'max_tf_age_sec', 'max_divergence_m', 'divergence_window_sec')):
        raise ValueError('localization_health thresholds must be positive finite numbers')
    if health['action'] != 'warn':
        raise ValueError('localization_health automatic stop is uncommissioned TODO-H1')
    health.update(map_frame=render.profile_value(profile, 'frames.map'),
                  odom_frame=render.profile_value(profile, 'frames.odom'),
                  nav_frame=render.profile_value(profile, 'frames.nav_base'),
                  chassis_odom_frame=render.profile_value(profile, 'chassis.odom_frame'),
                  chassis_odom_topic=render.profile_value(profile, 'chassis.odom_topic'))
    if any(not isinstance(health[name], str) or not health[name].strip() for name in (
            'map_frame', 'odom_frame', 'nav_frame', 'chassis_odom_frame', 'chassis_odom_topic')):
        raise ValueError('localization_health frame/topic names must be explicit')
    output['localization_health'] = _node('localization_monitor', health)
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
    if options.mode == 'precise':
        dock = {
            'approach_dist': render.profile_value(profile, 'dock.approach_dist'),
            'max_yaw_retries': render.profile_value(profile, 'dock.max_yaw_retries'),
            'goal_topic': '/dock_goal',
            'action_name': 'dock_to_pose', 'nav_timeout_sec': 120.0, 'spin_timeout_sec': 30.0,
            'static_occupied_threshold': 50, 'settle_max_rate_rad_s': 0.05,
            'settle_window_sec': 0.4, 'settle_timeout_sec': 8.0,
            'xy_ok': render.profile_value(profile, 'dock.xy_ok'),
            'yaw_ok': render.profile_value(profile, 'dock.yaw_ok'),
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
