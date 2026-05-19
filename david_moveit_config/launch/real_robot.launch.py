"""David MoveIt 2 + real robot via SDK bridge.

Architecture:
  - SDK bridge publishes 14 real arm joints to /joint_states_arms (50Hz)
  - joint_state_publisher subscribes /joint_states_arms (source_list) and
    publishes complete /joint_states by filling passive joints (waist/hands/wheels) with 0
  - RSP listens /joint_states for TF, MoveIt listens /joint_states for planning_scene
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    dry_run_arg = DeclareLaunchArgument("dry_run", default_value="false")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")

    urdf_path = PathJoinSubstitution([
        FindPackageShare("david_moveit_config"), "config", "david.urdf.xacro"
    ])
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_path]), value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder("waist_robot", package_name="david_moveit_config")
        .robot_description(file_path="config/david.urdf.xacro")
        .robot_description_semantic(file_path="config/david.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 关键：joint_state_publisher 把 SDK 桥的 14 关节 + URDF 中其余 48 关节（默认 0）合并发到 /joint_states
    jsp = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        name="joint_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"rate": 30.0},
            {"source_list": ["/joint_states_arms"]},
        ],
    )

    sdk_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("david_sdk_bridge"), "launch", "sdk_bridge.launch.py"
            ])
        ),
        launch_arguments={"dry_run": LaunchConfiguration("dry_run")}.items(),
    )

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("david_moveit_config"), "config", "moveit.rviz"
    ])
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # 延迟 3 秒启动 jsp/move_group/rviz, 等 sdk_bridge 先发出 /joint_states_arms
    delayed = TimerAction(period=3.0, actions=[jsp, move_group, rviz])

    return LaunchDescription([
        dry_run_arg, rviz_arg,
        rsp, sdk_bridge, delayed,
    ])
