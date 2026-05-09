#!/usr/bin/env python3
"""Standalone Realsense camera + depth-to-LaserScan launch for Nav2 local obstacle use."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _config_path():
    return Path(__file__).resolve().parents[1] / 'config' / 'realsense_depth_to_scan.yaml'


def _load_realsense_bridge_config():
    config_path = _config_path()
    with config_path.open() as stream:
        config = yaml.safe_load(stream) or {}

    try:
        return (
            config['realsense_camera']['launch_arguments'],
            config['depth_to_scan']['remappings'],
            config['depth_to_scan']['parameters'],
        )
    except KeyError as exc:
        raise KeyError(f'Missing required key {exc.args[0]!r} in {config_path}') from exc


def _stringify_launch_argument_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value == '':
        return "''"
    return str(value)


def _rs_launch_path():
    return (
        Path(get_package_share_directory('realsense2_camera'))
        / 'launch'
        / 'rs_launch.py'
    )


def generate_launch_description():
    camera_launch_arguments, depth_remappings, depth_parameters = (
        _load_realsense_bridge_config()
    )

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(_rs_launch_path())),
        launch_arguments={
            key: _stringify_launch_argument_value(value)
            for key, value in camera_launch_arguments.items()
        }.items(),
    )

    realsense_depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='realsense_depth_to_scan',
        output='screen',
        remappings=list(depth_remappings.items()),
        parameters=[{
            'use_sim_time': False,
            **depth_parameters,
        }],
    )

    return LaunchDescription([
        rs_launch,
        realsense_depth_to_scan_node,
    ])
