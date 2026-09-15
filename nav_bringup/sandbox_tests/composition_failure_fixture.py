"""Real container ownership faults with no hardware, goals or velocity publishers."""

import argparse
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnShutdown
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

from nav_bringup.launching.component_loader import CheckedComponentLoader
from nav_bringup.launching.executables import resolve_executable
from nav_bringup.launching.graph import LaunchGraph, ProcessSpec
from nav_bringup.launching.supervision import managed_actions


def main() -> int:
    """Start an empty container and inert child to verify launch-owned cleanup.

    Returns:
        The actual ROS launch exit code after a component or owned-process fault.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario', choices=('rejected', 'timeout', 'container_exit'))
    args = parser.parse_args()
    specs = (ProcessSpec('nav2_container', 'rclcpp_components', 'component_container_isolated'),
             ProcessSpec('inert_child', '', '', ros_node=False))
    container = Node(executable=resolve_executable(specs[0].package, specs[0].executable),
                     name='nav2_container', output='screen')
    child = ExecuteProcess(cmd=[sys.executable, '-c', 'import signal; signal.pause()'],
                           name='inert_child', output='screen')
    actions = managed_actions(LaunchGraph('tour', specs, ()),
                              lambda spec: container if spec.name == 'nav2_container' else child)
    actions.insert(0, RegisterEventHandler(OnShutdown(
        on_shutdown=lambda event, context: [LogInfo(msg=event.reason)])))
    if args.scenario != 'container_exit':
        loader = CheckedComponentLoader(
            '/absent_container' if args.scenario == 'timeout' else '/nav2_container',
            [ComposableNode(package='nav2_map_server', plugin='not_registered::Test',
                            name='rejected_component')],
            timeout_sec=1.0 if args.scenario == 'timeout' else 10.0)
        actions.insert(0, RegisterEventHandler(OnProcessStart(
            target_action=container, on_start=[loader], handle_once=True)))
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    return service.run()


if __name__ == '__main__':
    sys.exit(main())
