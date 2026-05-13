#!/usr/bin/env python3
"""2D Nav2 bringup for Unitree David with external Lightning SLAM topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_david_nav = get_package_share_directory('david_nav')
    urdf_file = os.path.join(pkg_david_nav, 'urdf', 'david.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_robot_state = LaunchConfiguration('publish_robot_state')
    params_file = LaunchConfiguration('params_file')
    enable_exploration = LaunchConfiguration('enable_exploration')
    explorer_params_file = LaunchConfiguration('explorer_params_file')
    collision_monitor_params_file = LaunchConfiguration('collision_monitor_params_file')
    enable_scan_bridge = LaunchConfiguration('enable_scan_bridge')
    enable_realsense_scan_bridge = LaunchConfiguration('enable_realsense_scan_bridge')
    enable_map_warmup_spin = LaunchConfiguration('enable_map_warmup_spin')
    map_warmup_min_live_area_m2 = LaunchConfiguration('map_warmup_min_live_area_m2')
    map_warmup_angular_speed = LaunchConfiguration('map_warmup_angular_speed')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml')
    log_level = LaunchConfiguration('log_level')
    scan_observation_active = PythonExpression([
        "'true' if ('",
        enable_scan_bridge,
        "' == 'true' or '",
        enable_realsense_scan_bridge,
        "' == 'true') else 'false'",
    ])
    configured_nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_bt_xml_filename': nav_to_pose_bt_xml,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
            'local_costmap.local_costmap.ros__parameters.scan_priority_layer.require_live_scan':
                scan_observation_active,
        },
        convert_types=True,
    )
    configured_nav2_params_file = ParameterFile(
        configured_nav2_params,
        allow_substs=True,
    )

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings,
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings,
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings,
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes,
        }],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', urdf_file]),
                value_type=str,
            ),
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(publish_robot_state),
    )

    base_alias_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base_link_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'base', 'base_link'],
        output='screen',
    )

    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/lightning/loc/aligned_scan'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base',
            'transform_tolerance': 0.1,
            'min_height': -0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 12.0,
            'use_inf': True,
        }],
        condition=IfCondition(enable_scan_bridge),
    )

    realsense_depth_to_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_david_nav, 'launch', 'realsense_depth_to_scan.launch.py')
        ),
        condition=IfCondition(enable_realsense_scan_bridge),
    )

    map_warmup_spin_node = Node(
        package='david_nav',
        executable='map_warmup_spin',
        name='map_warmup_spin',
        output='screen',
        parameters=[{
            'map_topic': '/lightning/grid_map',
            'live_map_ready_topic': '/map_live_ready',
            'cmd_vel_nav_topic': '/cmd_vel_nav',
            'resume_topic': 'explore/resume',
            'spin_angular_speed': map_warmup_angular_speed,
            'min_live_map_area_m2': map_warmup_min_live_area_m2,
        }],
        condition=IfCondition(enable_map_warmup_spin),
    )

    frontier_explorer = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='david_nav',
                executable='frontier_explorer_cpp',
                name='frontier_explorer',
                output='screen',
                parameters=[explorer_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(enable_exploration),
            ),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_monitor_params_file, {'use_sim_time': use_sim_time}],
    )

    collision_monitor_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    david_move_node = Node(
        package='david_cmd',
        executable='david_move',
        name='david_move',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz. Requires a valid graphical display.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_david_nav, 'rviz', 'nav2_go2_view.rviz'),
            description='RViz config file',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='true',
            description='Launch robot_state_publisher for the RViz RobotModel display.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_david_nav, 'config', 'nav2_params_2d.yaml'),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=os.path.join(
                pkg_david_nav, 'behavior_trees',
                'navigate_to_pose_w_david_embedded_recovery.xml'),
            description='Behavior tree XML used by bt_navigator for NavigateToPose.',
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_xml',
            default_value=os.path.join(
                pkg_david_nav, 'behavior_trees',
                'navigate_through_poses_w_david_embedded_recovery.xml'),
            description='Behavior tree XML used by bt_navigator for NavigateThroughPoses.',
        ),
        DeclareLaunchArgument(
            'enable_exploration',
            default_value='false',
            description='Launch frontier explorer after Nav2 bringup',
        ),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=os.path.join(
                pkg_david_nav, 'config', 'frontier_explorer_params.yaml'),
            description='Frontier explorer parameters file',
        ),
        DeclareLaunchArgument(
            'collision_monitor_params_file',
            default_value=os.path.join(
                pkg_david_nav, 'config', 'collision_monitor_params.yaml'),
            description='Collision monitor parameters file',
        ),
        DeclareLaunchArgument(
            'enable_scan_bridge',
            default_value='false',
            description='Launch pointcloud_to_laserscan to publish /scan from lidar.',
        ),
        DeclareLaunchArgument(
            'enable_realsense_scan_bridge',
            default_value='false',
            description='Launch depthimage_to_laserscan to publish /scan from Realsense depth.',
        ),
        DeclareLaunchArgument(
            'enable_map_warmup_spin',
            default_value='false',
            description='Rotate in place until the first live SLAM map grows to a usable size.',
        ),
        DeclareLaunchArgument(
            'map_warmup_min_live_area_m2',
            default_value='12.0',
            description='Minimum live /map area before exploration starts.',
        ),
        DeclareLaunchArgument(
            'map_warmup_angular_speed',
            default_value='0.45',
            description='Angular speed used while warming up the live SLAM map.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 log level',
        ),
        robot_state_publisher_node,
        base_alias_tf_node,
        pointcloud_to_scan_node,
        realsense_depth_to_scan_launch,
        controller_server_node,
        smoother_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        nav2_lifecycle_manager,
        map_warmup_spin_node,
        frontier_explorer,
        rviz_node,
        collision_monitor_node,
        collision_monitor_lifecycle_manager,
        david_move_node,
    ])
