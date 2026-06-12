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

    nav2_params = ParameterFile(params_file, allow_substs=True)
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=tf_remaps,
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

    # Lightning 的 TF 只发到 `base` 为止;底盘 URDF 的根是 `base_link`。
    #
    # 2026-06-11 拓扑改造(与机械臂 TF 树统一):桥过 world——
    #     base →(本launch) world →(本launch) base_link
    # 历史:抓取栈全身 URDF 曾以 world 为根、发 world→base_link,与导航旧的
    # base→base_link 形成 base_link 双父 → 两棵树 → ConnectivityException
    # (15:17 实锅)。同日抓取侧 v2 重构:删 world、base_link 改为其子树根、
    # 删底盘段(轮/lidar/imu),只把腰+双臂+手+相机挂在 base_link 下。
    # 两条桥都是本 launch 自发,抓取新版(不发 base_link 父)/旧版(发同值
    # world→base_link)都兼容,机械臂 RSP 在不在都是一棵树。
    # 注意:旧式 base→base_link 静态 tf 残留会跟这套打架(base_link 双父),
    # pre-flight 必须清掉(见 launch_waypoint_nav.sh)。
    base_to_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_world_tf",
        output="screen",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base", "--child-frame-id", "world"],
    )
    world_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link_tf",
        output="screen",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "world", "--child-frame-id", "base_link"],
    )

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
        DeclareLaunchArgument("rviz_delay_sec", default_value="8.0"),
        DeclareLaunchArgument("log_level", default_value="info"),
        planner_server,
        lifecycle_manager,
        robot_state_publisher,
        base_to_world_tf,
        world_to_base_link_tf,
        position_controller,
        cmd_vel_mux,
        waypoint_follower,
        rviz,
    ])
