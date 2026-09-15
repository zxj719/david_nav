"""Map logical nodes to physical ownership without changing their parameters."""

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
    """Select registered components only when composition was explicitly requested.

    Args:
        graph: Logical graph, possibly restricted to a deferred execution phase.

    Returns:
        Original node specifications paired with their registered plugin identifiers.
    """
    if not graph.composition:
        return ()
    # Profile-selected drivers remain independently owned even if their entry
    # point happens to match a registered component implementation.
    return tuple((spec, PLUGINS[(spec.package, spec.executable)]) for spec in graph.processes
                 if (spec.package, spec.executable) in PLUGINS
                 and spec.name == NODE_NAMES.get(spec.executable, spec.executable))


def physical_graph(graph: LaunchGraph) -> LaunchGraph:
    """Replace eligible logical processes by one critical, non-respawning container.

    Args:
        graph: Logical graph retaining every expected ROS node and parameter file.

    Returns:
        Physical graph for process resolution, supervision and dry-run reporting.

    Raises:
        ValueError: Components disagree on startup timing or carry unsupported CLI arguments.
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
