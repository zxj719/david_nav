#!/usr/bin/env python3
"""slam_gt 建图(mapping)。先把这套跑起来、遥控机器人转一圈建图，再 save_map.sh 存图。

前置(本 launch 不起，用 scripts/launch_slam_gt.sh 或手动起)：
  - alias 2: LSLIDAR C16 驱动 -> /c16/lslidar_point_cloud (frame laser_link)
  - alias 1: wit IMU 驱动 (slam_toolbox 不吃，按需起，仅记录/未来 EKF)
  - wheeltec 底盘 -> /odom 话题(本 launch 用 odom_to_base_tf 转 TF) + 收 /cmd_vel 遥控

本 launch 起的 TF / 处理链(自洽，不依赖 david_nav 的 RSP，绕过 90° 坑)：
  map ->(slam_toolbox) odom ->(odom_to_base_tf, 轮速计) base_link
       -> (static) laser_link / base_footprint
  /c16/lslidar_point_cloud -(p2l 360°,高度+自身过滤)-> /scan_slam -> slam_toolbox
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('david_slam_gt')
    slam_params = os.path.join(pkg, 'config', 'slam_gt_mapping.yaml')
    p2l_params = os.path.join(pkg, 'config', 'pointcloud_to_laserscan.yaml')
    self_filter_params = os.path.join(pkg, 'config', 'self_filter.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'slam_gt_view.rviz')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params),
        DeclareLaunchArgument('p2l_params_file', default_value=p2l_params),
        DeclareLaunchArgument('cloud_topic', default_value='/c16/lslidar_point_cloud'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'with_robot_tf', default_value='true',
            description='起本包自洽的静态 TF(base_link->laser_link/base_footprint)。'
                        '若已有别处提供这些帧，设 false。'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='起本包的 SLAM 可视化 rviz(Fixed Frame=map, Map=Transient '
                        'Local, /scan_slam)。与 lslidar 驱动自带的黑屏 rviz 无关。'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')
    with_tf = LaunchConfiguration('with_robot_tf')
    tf_cond = IfCondition(PythonExpression(["'", with_tf, "' == 'true'"]))
    rviz_cond = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('use_rviz'), "' == 'true'"]))

    # —— 自洽静态 TF：base_link(前向落地系) -> laser_link / base_footprint ——
    static_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_laser_link',
        # x y z yaw pitch roll  parent child  (纯平移，雷达前 0.352m、离地 0.3775m)
        arguments=['0.352', '0', '0.3775', '0', '0', '0', 'base_link', 'laser_link'],
        condition=tf_cond)
    static_footprint = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        condition=tf_cond)

    odom_tf = Node(
        package='david_slam_gt', executable='odom_to_base_tf.py',
        name='odom_to_base_tf', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'odom_topic': LaunchConfiguration('odom_topic'),
                     'parent_frame': 'odom', 'child_frame': 'base_link'}])

    self_filter = Node(
        package='david_slam_gt', executable='self_filter.py',
        name='self_filter', output='screen',
        parameters=[self_filter_params, {'use_sim_time': use_sim_time,
                    'input_topic': LaunchConfiguration('cloud_topic'),
                    'output_topic': '/c16/points_self_filtered'}])

    p2l = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan', output='screen',
        parameters=[p2l_params, {'use_sim_time': use_sim_time}],
        remappings=[('cloud_in', '/c16/points_self_filtered'),
                    ('scan', '/scan_slam')])

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[LaunchConfiguration('slam_params_file'),
                    {'use_sim_time': use_sim_time}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='slam_gt_rviz',
        arguments=['-d', rviz_cfg], output='screen',
        condition=rviz_cond)

    return LaunchDescription(
        args + [static_laser, static_footprint, odom_tf, self_filter, p2l, slam, rviz])
