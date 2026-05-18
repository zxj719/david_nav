"""Launch the scene_loader to push obstacles to MoveIt."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="double_passage"),
        DeclareLaunchArgument("clear", default_value="false"),
        Node(
            package="david_world",
            executable="scene_loader",
            output="screen",
            parameters=[{
                "scene": LaunchConfiguration("scene"),
                "clear": LaunchConfiguration("clear"),
            }],
        ),
    ])
