from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("waist_robot", package_name="david_moveit_config")
        .robot_description(file_path="config/david.urdf.xacro")
        .to_moveit_configs()
    )
    return LaunchDescription([Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[moveit_config.robot_description],
    )])
