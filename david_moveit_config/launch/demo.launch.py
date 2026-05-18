"""David MoveIt 2 demo launch with explicit robot_description for RSP."""
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ---- robot_description: 显式 Command + ParameterValue ----
    urdf_path = PathJoinSubstitution([
        FindPackageShare("david_moveit_config"), "config", "david.urdf.xacro"
    ])
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_path]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # ---- moveit_config（含 SRDF/kinematics/controllers/planning） ----
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

    # ---- ros2_control node ----
    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare("david_moveit_config"), "config", "ros2_controllers.yaml"
    ])
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )

    # ---- robot_state_publisher ----
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ---- spawners ----
    spawn_jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    spawn_left = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
    )
    spawn_right = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # ---- move_group ----
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # ---- RViz ----
    rviz_config = PathJoinSubstitution([
        FindPackageShare("david_moveit_config"), "config", "moveit.rviz"
    ])
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        rsp,
        ros2_control_node,
        spawn_jsb,
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_left])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_left, on_exit=[spawn_right])),
        move_group,
        rviz,
    ])
