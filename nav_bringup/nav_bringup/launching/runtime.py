"""将已验证进程图转换为受管 ROS launch 动作，构造前统一解析所需入口.

本模块只构造动作及事件处理器，不直接启动子进程。
组件模式先验证插件，再转换物理进程图；真实启动与收尾由 launch 接管。
"""

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
    """按所选阶段解析可执行入口并构造受管动作，不在此启动进程.

    core 与 executor 阶段按 navigation_executor 名称切分。
    teleop 要求监督者已拥有键盘，避免在 launch 图中重复启动键盘进程。
    组件加载处理器排在受管动作之前；构造失败不代表真实进程曾被启动。

    Args:
        graph: 参数文件已由外层 loader 核对的逻辑图。
        phase: all 为全图、core 排除执行器、executor 仅选延后执行器。

    Returns:
        组件加载处理器与受管 Node/ExecuteProcess 动作列表。

    Raises:
        ValueError: 阶段无效/为空、teleop 所有权不符或入口/组件校验失败。
        Exception: launch 动作或描述对象构造失败。
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
