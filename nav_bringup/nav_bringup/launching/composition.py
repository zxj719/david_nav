"""把逻辑节点映射到物理归属，不改动它们的任何参数.

组合只改变"谁和谁同进程"，节点名、参数文件与重映射逐项保持不变。
"""

from dataclasses import replace
from typing import Tuple

from nav_bringup.launching.graph import LaunchGraph, ProcessSpec


CONTAINER_NAME = 'nav2_container'
PLUGINS = {
    ('nav2_controller', 'controller_server'): 'nav2_controller::ControllerServer',
    ('nav2_planner', 'planner_server'): 'nav2_planner::PlannerServer',
    ('nav2_behaviors', 'behavior_server'): 'behavior_server::BehaviorServer',
    ('nav2_bt_navigator', 'bt_navigator'): 'nav2_bt_navigator::BtNavigator',
    ('nav2_velocity_smoother', 'velocity_smoother'): 'nav2_velocity_smoother::VelocitySmoother',
    ('nav2_map_server', 'map_server'): 'nav2_map_server::MapServer',
    ('nav2_lifecycle_manager', 'lifecycle_manager'): 'nav2_lifecycle_manager::LifecycleManager',
    ('chassis_adapter', 'chassis_adapter_cpp'): 'chassis_adapter::AdapterNode',
}
NODE_NAMES = {'lifecycle_manager': 'lifecycle_manager_navigation',
              'chassis_adapter_cpp': 'chassis_adapter'}


def component_specs(graph: LaunchGraph) -> Tuple[Tuple[ProcessSpec, str], ...]:
    """仅在显式要求组合时，挑出已注册的组件.

    未要求组合时返回空元组；注册表里没有的节点一律保持独立进程，不做猜测。

    Args:
        graph: 逻辑图，可能已被限制到某个延后执行的阶段。

    Returns:
        原节点规格与其注册插件标识的配对。
    """
    if not graph.composition:
        return ()
    # Profile-selected drivers remain independently owned even if their entry
    # point happens to match a registered component implementation.
    return tuple((spec, PLUGINS[(spec.package, spec.executable)]) for spec in graph.processes
                 if (spec.package, spec.executable) in PLUGINS
                 and spec.name == NODE_NAMES.get(spec.executable, spec.executable))


def physical_graph(graph: LaunchGraph) -> LaunchGraph:
    """把可组合的逻辑进程换成一个关键且不自动重启的容器.

    容器被标为 critical 且重启预算为零：同进程的节点共享故障域，一个组件崩溃
    等于整组消失，因此不做局部重启，直接交由整栈关停处理。

    Args:
        graph: 逻辑图，其中每个预期 ROS 节点与参数文件都仍然保留。

    Returns:
        供进程解析、监督与 dry-run 报告使用的物理图。

    Raises:
        ValueError: 各组件的启动时机不一致，或带了不支持的命令行参数。
    """
    members = component_specs(graph)
    if not members:
        return graph
    delay = members[0][0].delay_sec
    if any(spec.delay_sec != delay or spec.arguments or spec.wait_for_navigation
           for spec, _ in members):
        raise ValueError('composition requires common timing and parameter-only components')
    names = {spec.name for spec, _ in members}
    processes = []
    inserted = False
    for spec in graph.processes:
        if spec.name not in names:
            processes.append(spec)
        elif not inserted:
            processes.append(ProcessSpec(
                CONTAINER_NAME, 'rclcpp_components', 'component_container_isolated',
                delay_sec=delay))
            inserted = True
    return replace(graph, processes=tuple(processes))
