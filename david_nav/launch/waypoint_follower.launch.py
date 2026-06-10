import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("david_nav")
    wf_params = os.path.join(pkg, "config", "waypoint_follower.yaml")
    pc_params = os.path.join(pkg, "config", "position_controller.yaml")

    return LaunchDescription([
        Node(
            package="david_nav",
            executable="position_controller",
            name="position_controller",
            output="screen",
            parameters=[pc_params],
        ),
        Node(
            package="david_nav",
            executable="cmd_vel_mux",
            name="cmd_vel_mux",
            output="screen",
        ),
        Node(
            package="david_nav",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[wf_params],
        ),
    ])
