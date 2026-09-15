"""按有效档案启动可选点云预处理；默认配置不启动任何节点."""

from pathlib import Path
from typing import List

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav_perception_iface.configuration import load_scan_configuration


def _configure(context: LaunchContext) -> List[Node]:
    """只读取档案并返回节点图，不操作进程、设备或系统配置."""
    enabled = LaunchConfiguration('enabled').perform(context).lower()
    if enabled not in {'profile', 'true', 'false'}:
        raise ValueError('enabled must be profile, true or false')
    override = None if enabled == 'profile' else enabled == 'true'
    config = load_scan_configuration(
        Path(LaunchConfiguration('profile_file').perform(context)).expanduser(), override)
    if not config.enabled:
        return []
    return [Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan', output='screen',
        parameters=[dict(config.parameters)],
        remappings=[('cloud_in', config.cloud_topic), ('scan', config.scan_topic)],
    )]


def generate_launch_description() -> LaunchDescription:
    """构造没有默认机器人路径的感知 launch.

    Returns:
        仅启用时包含一个预处理节点的描述。
    """
    return LaunchDescription([
        DeclareLaunchArgument('profile_file', description='Effective robot profile YAML'),
        DeclareLaunchArgument('enabled', default_value='profile',
                              description='profile | true | false (legacy override)'),
        OpaqueFunction(function=_configure),
    ])
