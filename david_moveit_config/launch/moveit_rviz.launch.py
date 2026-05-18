from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("waist_robot", package_name="david_moveit_config")
        .robot_description(file_path="config/david.urdf.xacro")
        .robot_description_semantic(file_path="config/david.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )
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
    return LaunchDescription([rviz])
