#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_g1_nav = get_package_share_directory('g1_nav')
    explore_nav_to_pose_bt_xml = os.path.join(
        pkg_g1_nav, 'behavior_trees', 'navigate_to_pose_w_g1_explore_fail_fast.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_robot_state = LaunchConfiguration('publish_robot_state')
    params_file = LaunchConfiguration('params_file')
    explorer_params_file = LaunchConfiguration('explorer_params_file')
    enable_exploration = LaunchConfiguration('enable_exploration')
    enable_scan_bridge = LaunchConfiguration('enable_scan_bridge')
    enable_realsense_scan_bridge = LaunchConfiguration('enable_realsense_scan_bridge')
    enable_map_warmup_spin = LaunchConfiguration('enable_map_warmup_spin')
    map_warmup_min_live_area_m2 = LaunchConfiguration('map_warmup_min_live_area_m2')
    map_warmup_angular_speed = LaunchConfiguration('map_warmup_angular_speed')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g1_nav, 'launch', '2d_g1_nav2_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'rviz_config': rviz_config,
            'publish_robot_state': publish_robot_state,
            'params_file': params_file,
            'nav_to_pose_bt_xml': explore_nav_to_pose_bt_xml,
            'enable_exploration': enable_exploration,
            'explorer_params_file': explorer_params_file,
            'enable_scan_bridge': enable_scan_bridge,
            'enable_realsense_scan_bridge': enable_realsense_scan_bridge,
            'enable_map_warmup_spin': enable_map_warmup_spin,
            'map_warmup_min_live_area_m2': map_warmup_min_live_area_m2,
            'map_warmup_angular_speed': map_warmup_angular_speed,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz together with Nav2 and frontier exploration.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_g1_nav, 'rviz', 'nav2_go2_view.rviz'),
            description='RViz config file',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='true',
            description='Publish robot_description so RViz can show the robot model.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_g1_nav, 'config', 'nav2_params_2d.yaml'),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=os.path.join(
                pkg_g1_nav, 'config', 'frontier_explorer_params.yaml'),
            description='Frontier explorer parameters file',
        ),
        DeclareLaunchArgument(
            'enable_exploration',
            default_value='true',
            description='Launch frontier exploration together with Nav2.',
        ),
        DeclareLaunchArgument(
            'enable_scan_bridge',
            default_value='false',
            description='Launch pointcloud_to_laserscan to publish /scan from lidar.',
        ),
        DeclareLaunchArgument(
            'enable_realsense_scan_bridge',
            default_value='true',
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
            description='Minimum live /map area before exploration resumes.',
        ),
        DeclareLaunchArgument(
            'map_warmup_angular_speed',
            default_value='0.45',
            description='Angular speed used while warming up the live SLAM map.',
        ),
        bringup_launch,
    ])
