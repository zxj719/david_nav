"""Launch a prepared run bundle; never render files, probe processes or start localization."""

from typing import List

from launch import LaunchContext
from launch import LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import build_graph
from nav_bringup.launching.runtime import launch_actions


def _setup(context: LaunchContext) -> List[Action]:
    root = LaunchConfiguration('run_dir').perform(context)
    selectors = {name: LaunchConfiguration(name).perform(context)
                 for name in ('robot', 'site', 'mode', 'chassis', 'executor', 'composition')}
    return launch_actions(build_graph(load_options(root, selectors)),
                          LaunchConfiguration('phase').perform(context))


def generate_launch_description() -> LaunchDescription:
    """Declare selectors and a required pre-rendered run directory.

    Returns:
        A launch description that validates the whole bundle before startup.
    """
    return LaunchDescription([
        DeclareLaunchArgument('run_dir', description='Absolute navrun-prepared run directory'),
        DeclareLaunchArgument(
            'phase', default_value='all', description='Owned launch phase: all/core/executor'),
        DeclareLaunchArgument(
            'robot', default_value='', description='Assert prepared robot identity'),
        DeclareLaunchArgument(
            'site', default_value='', description='Assert prepared site identity'),
        DeclareLaunchArgument(
            'mode', default_value='', description='Assert prepared mode identity'),
        DeclareLaunchArgument(
            'chassis', default_value='', description='Override ownership: on/off'),
        DeclareLaunchArgument('executor', default_value='', description='Override WS: true/false'),
        DeclareLaunchArgument(
            'composition', default_value='',
            description='Override frozen composition: true/false (bundle default false)'),
        OpaqueFunction(function=_setup),
    ])
