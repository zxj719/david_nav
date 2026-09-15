"""Start the prepared driver and adapter, never probing hardware or localization."""

from typing import List

from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import build_graph
from nav_bringup.launching.runtime import launch_actions


def _setup(context: LaunchContext) -> List[Action]:
    selectors = {name: LaunchConfiguration(name).perform(context)
                 for name in ('robot', 'chassis')}
    selectors['mode'] = 'chassis'
    options = load_options(LaunchConfiguration('run_dir').perform(context), selectors)
    return launch_actions(build_graph(options))


def generate_launch_description() -> LaunchDescription:
    """Declare run-local inputs; navrun already resolved chassis auto-detection.

    Returns:
        A launch graph limited to the adapter and any explicitly owned driver.
    """
    return LaunchDescription([
        DeclareLaunchArgument('run_dir', description='Prepared chassis-mode run directory'),
        DeclareLaunchArgument('robot', default_value='', description='Assert robot identity'),
        DeclareLaunchArgument('chassis', default_value='', description='Owned driver: on/off'),
        OpaqueFunction(function=_setup),
    ])
