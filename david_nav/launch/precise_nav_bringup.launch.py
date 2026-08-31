#!/usr/bin/env python3
"""alias 0 专用：高精度两段式入位导航（2026-08-31）。

= 现有 2d_david_nav2_bringup.launch.py（传高精度参数快照）
+ dock_navigator（两段式对齐入位编排节点）

生产导览 alias 4 完全不受影响：本文件不修改任何生产文件，只是换一份
params_file 并额外起一个节点。

话题分工（关键，别搞混）：
  /dock_goal   -> dock_navigator   两段式入位（rviz 的 "2D Goal Pose" 工具发这个）
  /goal_pose   -> bt_navigator     单段直达（CLI 直接发，用于对照测试）
故意分开两个话题：bt_navigator 自己订阅 /goal_pose，若 dock_navigator 也收同一个
话题，一个目标会同时触发两条链路。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('david_nav')

    precise_params = LaunchConfiguration('params_file')
    goal_topic = LaunchConfiguration('dock_goal_topic')
    approach_dist = LaunchConfiguration('approach_dist')
    xy_ok = LaunchConfiguration('xy_ok')
    yaw_ok = LaunchConfiguration('yaw_ok')
    max_yaw_retries = LaunchConfiguration('max_yaw_retries')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', '2d_david_nav2_bringup.launch.py')),
        launch_arguments={
            'params_file': precise_params,
            'cmd_shaping': 'false',
        }.items(),
    )

    dock_navigator = Node(
        package='david_nav',
        executable='dock_navigator',
        name='dock_navigator',
        output='screen',
        parameters=[{
            'goal_topic': goal_topic,
            'approach_dist': approach_dist,
            'xy_ok': xy_ok,
            'yaw_ok': yaw_ok,
            'max_yaw_retries': max_yaw_retries,
            'robot_base_frame': 'base_footprint',
            'global_frame': 'map',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params_2d_precise.yaml'),
            description='高精度参数快照。与生产 nav2_params_2d.yaml 的差异见该文件头。',
        ),
        DeclareLaunchArgument(
            'dock_goal_topic',
            default_value='/dock_goal',
            description='dock_navigator 的输入话题。刻意不用 /goal_pose —— '
                        'bt_navigator 自己订阅 /goal_pose，同名会双链路抢同一目标。',
        ),
        DeclareLaunchArgument(
            'approach_dist', default_value='0.5',
            description='预备点沿目标航向后退的距离(m)。',
        ),
        DeclareLaunchArgument(
            'xy_ok', default_value='0.10',
            description='入位判定的位置误差阈值(m)，与 general_goal_checker 一致。',
        ),
        DeclareLaunchArgument(
            'yaw_ok', default_value='0.087',
            description='入位判定的朝向误差阈值(rad)，与 general_goal_checker 一致。',
        ),
        DeclareLaunchArgument(
            'max_yaw_retries', default_value='2',
            description='入位后朝向超差时的纯旋转补偿次数上限。',
        ),
        bringup,
        dock_navigator,
    ])
