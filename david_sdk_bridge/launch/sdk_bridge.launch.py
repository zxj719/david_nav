"""Launch both arm SDK bridges. Each arm runs in its own process (SDK isolation)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("left_ip",  default_value="192.168.160.162"),
        DeclareLaunchArgument("right_ip", default_value="192.168.160.161"),
        DeclareLaunchArgument("left_local_ip",  default_value="192.168.160.201"),
        DeclareLaunchArgument("right_local_ip", default_value="192.168.160.200"),
        DeclareLaunchArgument("enable_left",  default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
    ]

    left_node = Node(
        package="david_sdk_bridge", executable="arm_node",
        name="left_arm_bridge", output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_left")),
        parameters=[{
            "side": "left",
            "robot_ip": LaunchConfiguration("left_ip"),
            "local_ip": LaunchConfiguration("left_local_ip"),
            "controller_name": "left_arm_controller",
            "joint_name_prefix": "AR5-5_08L-W4C4A6-ZY2_joint_",
            "sdk_speed": 600,
            "sdk_zone": 80,
            "state_pub_hz": 50.0,
            "dry_run": LaunchConfiguration("dry_run"),
            # 左臂偏移（暂用 0，等用户测后填）
            "joint_offsets_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # 左臂 home 位姿（SDK 坐标，度）：与 SRDF home 匹配
            "home_pose_deg": [145.00, 90.00, -90.00, 100.00, 90.00, 0.00, 0.00],
            "hand_slave_id": 0x28,  # left = 40
        }],
    )

    right_node = Node(
        package="david_sdk_bridge", executable="arm_node",
        name="right_arm_bridge", output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_right")),
        parameters=[{
            "side": "right",
            "robot_ip": LaunchConfiguration("right_ip"),
            "local_ip": LaunchConfiguration("right_local_ip"),
            "controller_name": "right_arm_controller",
            "joint_name_prefix": "AR5-5_08R-W4C4A6-ZY2_joint_",
            "sdk_speed": 600,
            "sdk_zone": 80,
            "state_pub_hz": 50.0,
            "dry_run": LaunchConfiguration("dry_run"),
            # 右臂偏移：joint_1 需要 -90°，其他 0
            "joint_offsets_deg": [-90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # 右臂 home 位姿（SDK 坐标，度）：与 SRDF home 匹配
            "home_pose_deg": [-60.01, 90.00, 90.00, 99.99, -90.01, 5.00, -0.00],
            "hand_slave_id": 0x27,  # right = 39
        }],
    )

    return LaunchDescription(args + [left_node, right_node])
