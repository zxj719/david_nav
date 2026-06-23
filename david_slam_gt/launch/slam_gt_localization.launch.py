#!/usr/bin/env python3
"""slam_gt 定位(localization) + 真值。锁 mapping 存的图，跑导航调参测试时用。

与 nav2 同跑的约定(重要)：
  本 launch 自洽地拥有机器人 TF(base_link->laser_link/base_footprint) 和 /scan_slam。
  跑 nav2 调参时，nav2 那边务必【不要】再起自己的 robot_state_publisher / p2l，
  以免和这里重复/冲突(david_nav 的 2d_david_nav2_bringup 用 use_rsp:=false、
  enable_scan_bridge:=false)。当然，【不要】起 lightning。

TF / 数据链：
  map ->(slam_toolbox localization, 锁图) odom ->(odom_to_base_tf 轮速计) base_link
       -> (static) laser_link / base_footprint
  groundtruth_pose_publisher: 查 map->base_footprint -> /groundtruth/pose (给 trial_runner)
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
    slam_params = os.path.join(pkg, 'config', 'slam_gt_localization.yaml')
    p2l_params = os.path.join(pkg, 'config', 'pointcloud_to_laserscan.yaml')
    self_filter_params = os.path.join(pkg, 'config', 'self_filter.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'slam_gt_view.rviz')
    default_map = os.path.join(pkg, 'maps', 'david_map')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params),
        DeclareLaunchArgument('p2l_params_file', default_value=p2l_params),
        DeclareLaunchArgument('cloud_topic', default_value='/c16/lslidar_point_cloud'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'map_file', default_value=default_map,
            description='序列化地图路径(无扩展名，需有 <map_file>.posegraph + .data)'),
        DeclareLaunchArgument('with_robot_tf', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')
    with_tf = LaunchConfiguration('with_robot_tf')
    tf_cond = IfCondition(PythonExpression(["'", with_tf, "' == 'true'"]))
    rviz_cond = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('use_rviz'), "' == 'true'"]))

    static_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_laser_link',
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
        package='slam_toolbox', executable='localization_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[LaunchConfiguration('slam_params_file'),
                    {'use_sim_time': use_sim_time,
                     'map_file_name': LaunchConfiguration('map_file')}])

    gt_pose = Node(
        package='david_slam_gt', executable='groundtruth_pose_publisher.py',
        name='groundtruth_pose_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'map_frame': 'map', 'base_frame': 'base_footprint',
                     'rate_hz': 20.0, 'pose_topic': '/groundtruth/pose'}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='slam_gt_rviz',
        arguments=['-d', rviz_cfg], output='screen',
        condition=rviz_cond)

    return LaunchDescription(
        args + [static_laser, static_footprint, odom_tf, self_filter, p2l, slam, gt_pose, rviz])
