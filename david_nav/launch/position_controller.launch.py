#!/usr/bin/env python3
"""Bring up the position loop + cmd_vel mux.

  ros2 launch david_nav position_controller.launch.py

Brings up:
  * position_controller  → /cmd_vel_position   (only while a MoveTo goal runs)
  * cmd_vel_mux          → /cmd_vel             (position prio 100, nav prio 10)

To actually insert the mux ahead of the chassis, remap the existing Nav2
chain output (cmd_vel_separator) from /cmd_vel to /cmd_vel_nav so it feeds the
mux instead of the chassis directly. See README "与 Nav2 切换".
Set start_mux:=false to run the controller alone (e.g. for sim / bench tests).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('david_nav')
    default_params = os.path.join(pkg, 'config', 'position_controller.yaml')

    params_file = LaunchConfiguration('params_file')
    start_mux = LaunchConfiguration('start_mux')
    controller_cmd_topic = LaunchConfiguration('controller_cmd_topic')
    output_cmd_topic = LaunchConfiguration('output_cmd_topic')

    controller = Node(
        package='david_nav',
        executable='position_controller',
        name='position_controller',
        output='screen',
        parameters=[params_file, {'cmd_topic': controller_cmd_topic}],
    )

    mux = Node(
        package='david_nav',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
        condition=IfCondition(start_mux),
        parameters=[{
            # launch_ros 不能把 LaunchConfiguration 塞进 list 参数(会被拼成单个
            # STRING → STRING_ARRAY 类型错)。nav 输入话题写死;要改直接改这行。
            'input_topics': ['/cmd_vel_position', '/cmd_vel_nav'],
            'input_priorities': [100, 10],
            'output_topic': output_cmd_topic,
            'input_timeout': 0.3,
            'rate': 50.0,
            'idle_grace_sec': 0.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='position_controller params yaml'),
        DeclareLaunchArgument('start_mux', default_value='true',
                              description='also start the cmd_vel priority mux'),
        DeclareLaunchArgument('controller_cmd_topic', default_value='/cmd_vel_position',
                              description='position_controller 输出话题;bench 直连底盘用 '
                                          '/cmd_vel(配 start_mux:=false,不经 mux)'),
        DeclareLaunchArgument('output_cmd_topic', default_value='/cmd_vel',
                              description='final chassis cmd topic'),
        controller,
        mux,
    ])
