"""Adapt a validated graph to ROS launch, resolving all entry points before startup."""

from __future__ import annotations

from dataclasses import replace
from typing import List

from launch.action import Action
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

from nav_bringup.launching.component_loader import CheckedComponentLoader
from nav_bringup.launching.composition import component_specs, CONTAINER_NAME, physical_graph
from nav_bringup.launching.executables import resolve_component
from nav_bringup.launching.executables import resolve_executable as _resolve
from nav_bringup.launching.graph import LaunchGraph
from nav_bringup.launching.graph import ProcessSpec
from nav_bringup.launching.supervision import managed_actions


def launch_actions(graph: LaunchGraph, phase: str = 'all') -> List[Action]:
    """Resolve every selected executable before constructing owned launch actions.

    Args:
        graph: Validated graph whose run-local parameters were checked by the loader.
        phase: Entire graph, core only, or the deferred launch-managed executor.

    Returns:
        Managed ROS nodes and plain executor processes; no process starts here.

    Raises:
        ValueError: If an executable is invalid, missing or not executable.
    """
    if phase not in ('all', 'core', 'executor'):
        raise ValueError('launch phase must be all, core or executor')
    if phase != 'all':
        processes = tuple(spec for spec in graph.processes
                          if (spec.name == 'navigation_executor') == (phase == 'executor'))
        if not processes:
            raise ValueError('selected launch phase has no processes')
        graph = replace(graph, processes=processes)
    if graph.mode == 'teleop':
        if not graph.supervisor_keyboard:
            raise ValueError('teleop requires navrun supervisor-owned keyboard startup')
        graph = replace(graph, processes=tuple(
            spec for spec in graph.processes if spec.name != 'chassis_teleop'))
    members = component_specs(graph)
    for spec, plugin in members:
        resolve_component(spec.package, plugin)
    graph = physical_graph(graph)
    executables = {spec.name: _resolve(spec.package, spec.executable) for spec in graph.processes}
    gate = _resolve('nav_bringup', 'nav_ready_exec') if any(
        spec.wait_for_navigation for spec in graph.processes) else ''

    loaders = []

    def factory(spec: ProcessSpec) -> ExecuteProcess:
        """把已解析进程规格构造成受管 launch 动作，不在这里启动进程.

        组件容器保留去重后的全局参数文件，并登记启动后的加载处理器。
        需导航就绪的进程使用 gate 包装；其余按是否 ROS 节点选择动作类型。
        loaders 是外层共享列表，构造失败不自动回滚此前追加的处理器。

        Args:
            spec: 当前物理图中的进程规格，入口路径已由外层统一解析。

        Returns:
            Node 或 ExecuteProcess；组件加载事件处理器另加入 loaders。

        Raises:
            KeyError: 规格名没有对应的已解析入口。
            Exception: launch 动作或组件描述构造失败。
        """
        executable = executables[spec.name]
        if spec.name == CONTAINER_NAME and members:
            # Child costmaps read global YAML sections in the container's argv;
            # component-local overrides alone would lose those nested node sections.
            parameters = list(dict.fromkeys(item.parameter_file for item, _ in members
                                            if item.parameter_file))
            container = Node(executable=executable, name=spec.name, output='screen',
                             parameters=parameters)
            descriptions = [ComposableNode(
                package=item.package, plugin=plugin, name=item.name,
                parameters=[item.parameter_file] if item.parameter_file else [],
                remappings=list(item.remappings)) for item, plugin in members]
            loaders.append(RegisterEventHandler(OnProcessStart(
                target_action=container, on_start=[CheckedComponentLoader(
                    '/' + CONTAINER_NAME, descriptions)], handle_once=True)))
            return container
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

    actions = managed_actions(graph, factory)
    return loaders + actions
