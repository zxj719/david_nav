#!/usr/bin/env python3
"""精简 DWB bringup(口径 A,纯 DWB) —— feature/dwb-control-opt。

只起 controller_server(DWB + local_costmap)+ lifecycle_manager,cmd **直连 /cmd_vel**
(不经 velocity_smoother / cmd_vel_separator),用于隔离测/调 DWB 本体,和 controller_bench
的 position_controller 直连口径一致。

不起:planner / bt_navigator / smoother / separator / RSP。参考路径由 controller_bench 的
DwbFollowPathAdapter 通过 FollowPath action 直接喂(绕 planner)。

前置(全 ROS_DOMAIN_ID=0):
  - david_slam_gt localization:/map + map->odom->base_footprint TF(+ /scan_slam)
  - wheeltec:/odom + 吃 /cmd_vel
  - controller_bench:  python3 run_bench.py --controller dwb ...

用法:
  ros2 launch david_nav dwb_bench_bringup.launch.py
  ros2 launch david_nav dwb_bench_bringup.launch.py params_file:=<你的调参版>.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg = get_package_share_directory('david_nav')
    default_params = os.path.join(pkg, 'config', 'dwb_bench_params.yaml')

    params_file = LaunchConfiguration('params_file')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    log_level = LaunchConfiguration('log_level')

    nav2_params = ParameterFile(params_file, allow_substs=True)
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        arguments=['--ros-args', '--log-level', log_level],
        # 口径 A:controller_server 默认发 'cmd_vel' → 直接映射到底盘 /cmd_vel(不经 separator)。
        remappings=tf_remaps + [('cmd_vel', cmd_vel_topic)],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_dwb_bench',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['controller_server'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='DWB bench 参数(controller_server + local_costmap)'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='controller_server 速度输出话题(口径A=/cmd_vel 直连底盘)'),
        DeclareLaunchArgument('log_level', default_value='info'),
        controller_server,
        lifecycle_manager,
    ])
