"""Adapt a validated graph to ROS launch, resolving all entry points before startup."""

from __future__ import annotations

import os
from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_prefix
from launch.action import Action
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from nav_bringup.launching.graph import LaunchGraph
from nav_bringup.launching.graph import ProcessSpec
from nav_bringup.launching.supervision import managed_actions


def _resolve(package: str, executable: str) -> str:
    if Path(executable).name != executable:
        raise ValueError('package executable must be a basename')
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ValueError(f'missing executable: {path}')
    return str(path)


def launch_actions(graph: LaunchGraph) -> List[Action]:
    """Resolve every selected executable before constructing owned launch actions.

    Args:
        graph: Validated graph whose run-local parameters were checked by the loader.

    Returns:
        Managed ROS nodes and plain executor processes; no process starts here.

    Raises:
        ValueError: If an executable is invalid, missing or not executable.
    """
    executables = {spec.name: _resolve(spec.package, spec.executable) for spec in graph.processes}
    gate = _resolve('nav_bringup', 'nav_ready_exec') if any(
        spec.wait_for_navigation for spec in graph.processes) else ''

    def factory(spec: ProcessSpec) -> ExecuteProcess:
        executable = executables[spec.name]
        if spec.wait_for_navigation:
            return ExecuteProcess(
                cmd=[gate, '--timeout', str(spec.ready_timeout_sec), '--action', spec.ready_action,
                     '--', executable, *spec.arguments], name=spec.name, output='screen')
        if not spec.ros_node:
            return ExecuteProcess(
                cmd=[executable, *spec.arguments], name=spec.name, output='screen')
        return Node(
            executable=executable, name=spec.name, output='screen',
            parameters=[spec.parameter_file] if spec.parameter_file else [],
            remappings=list(spec.remappings), arguments=list(spec.arguments))

    return managed_actions(graph, factory)
