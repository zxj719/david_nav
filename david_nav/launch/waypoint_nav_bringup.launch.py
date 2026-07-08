#!/usr/bin/env python3
"""Planner-only Nav2 + position-controller waypoint following + RViz.

This is the bringup for the "Nav2 only plans, position_controller drives"
stack (see docs/waypoint_follower_README.md). Unlike 2d_david_nav2_bringup,
it does NOT start the Nav2 controller / smoother / bt_navigator / david_move
chain — only:

  * planner_server  (hosts /global_costmap/costmap, ComputePathToPose)
  * lifecycle_manager (planner_server only)
  * position_controller  -> /cmd_vel_position
  * cmd_vel_mux          -> /cmd_vel   (SOLE /cmd_vel publisher here)
  * waypoint_follower    (plan -> sparsify -> drive each segment)
  * RViz (delayed), showing the global plan + waypoint markers

RViz's "2D Nav Goal" (publishes /goal_pose) is remapped into the
waypoint_follower, so you drive by clicking a goal in RViz.

Prereqs (start these first, same ROS_DOMAIN_ID): Lightning SLAM/loc
(/lightning/grid_map + map->odom->base TF) and the wheeltec chassis driver
(/odom TOPIC only + consumes /cmd_vel — 注意:这套 wheeltec build 不发任何
TF,旧注释说"底盘提供 odom->base_footprint TF"是错的,2026-06-09 实查
wheeltec_robot.cpp 无 TransformBroadcaster)。完整 TF 链:
map ->(Lightning) odom ->(Lightning) base ->(本 launch 静态tf) base_link
->(URDF/RSP) base_footprint / laser_link。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("david_nav")
    urdf_file = os.path.join(pkg, "urdf", "david.urdf")

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    params_file = LaunchConfiguration("params_file")
    wf_params = LaunchConfiguration("wf_params")
    pc_params = LaunchConfiguration("pc_params")
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    rviz_delay_sec = LaunchConfiguration("rviz_delay_sec")
    log_level = LaunchConfiguration("log_level")
    map_topic = LaunchConfiguration("map_topic")

    nav2_params = ParameterFile(params_file, allow_substs=True)
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
        arguments=["--ros-args", "--log-level", log_level],
        # global_costmap 的 static_layer 订 nav2_params 里写死的 /lightning/grid_map;
        # 跑在 david_slam_gt 上时把它重映射到 /map(slam_toolbox 发的图)。
        # 默认 no-op(/lightning/grid_map->/lightning/grid_map),不影响 lightning 用法。
        remappings=tf_remaps + [("/lightning/grid_map", map_topic)],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_waypoint_nav",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["planner_server"],
        }],
    )

    # 2026-06-11:节点名/robot_description 话题都加 nav 前缀,不和抓取栈的
    # RSP 抢默认名(抓取的 jsp 订 /robot_description,被底盘 URDF 覆盖会判
    # invalid → /joint_states 停发,机械臂瘫;见抓取组 2026-06-11 说明 §四)。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="nav_robot_state_publisher",
        output="screen",
        condition=IfCondition(publish_robot_state),
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", urdf_file]) if urdf_file.endswith(".xacro")
                else Command(["cat ", urdf_file]),
                value_type=str),
        }],
        remappings=[("robot_description", "/nav/robot_description")],
    )

    # 2026-06-12:SLAM 已直接发 map→odom→base_link(base_link = 正向本体帧),
    # 原来的 base→world→base_link 桥**已移除** —— 再发会让 base_link 双父
    # (SLAM 的 odom + 桥的 world),又裂树。base_link 的父现在唯一由 SLAM 发。
    # 底盘 URDF(本 launch 的 RSP)只发 base_link 的子帧,不发其父。
    # 抓取栈把上半身挂在 base_link 下,交界仍在 base_link,一棵树。

    position_controller = Node(
        package="david_nav",
        executable="position_controller",
        name="position_controller",
        output="screen",
        parameters=[pc_params],
    )

    cmd_vel_mux = Node(
        package="david_nav",
        executable="cmd_vel_mux",
        name="cmd_vel_mux",
        output="screen",
    )

    waypoint_follower = Node(
        package="david_nav",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[wf_params],
        # RViz "2D Nav Goal" publishes /goal_pose -> drive the follower.
        remappings=[("/waypoint_follower/goal_pose", "/goal_pose")],
    )

    rviz = TimerAction(
        period=rviz_delay_sec,
        condition=IfCondition(use_rviz),
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(pkg, "rviz", "waypoint_follower.rviz")),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg, "config", "nav2_params_2d.yaml")),
        DeclareLaunchArgument(
            "wf_params",
            default_value=os.path.join(pkg, "config", "waypoint_follower.yaml")),
        DeclareLaunchArgument(
            "pc_params",
            default_value=os.path.join(pkg, "config", "position_controller.yaml")),
        DeclareLaunchArgument("publish_robot_state", default_value="true"),
        DeclareLaunchArgument(
            "map_topic", default_value="/lightning/grid_map",
            description="global_costmap 的图源;跑 david_slam_gt 时设 /map"),
        DeclareLaunchArgument("rviz_delay_sec", default_value="8.0"),
        DeclareLaunchArgument("log_level", default_value="info"),
        planner_server,
        lifecycle_manager,
        robot_state_publisher,
        position_controller,
        cmd_vel_mux,
        waypoint_follower,
        rviz,
    ])
