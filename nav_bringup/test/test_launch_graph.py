"""Pure process-graph contracts for PLAN 7.1 and 7.2; never launch any node."""

from dataclasses import replace
from pathlib import Path

import pytest

from nav_bringup.launching.graph import build_graph
from nav_bringup.launching.graph import CORE_NODES
from nav_bringup.launching.graph import exit_decision
from nav_bringup.launching.graph import GraphOptions


def test_tour_keeps_nav2_lifecycle_and_command_chain() -> None:
    """Map activates first; controller/behavior output feeds the retained smoother."""
    graph = build_graph(GraphOptions('tour', '/run/test', start_executor=True))
    nodes = {node.name: node for node in graph.processes}
    assert graph.lifecycle_nodes == ('map_server',) + tuple(name for _, name in CORE_NODES)
    assert 'dock_navigator' not in nodes
    assert ('cmd_vel', 'cmd_vel_nav') in nodes['controller_server'].remappings
    assert ('cmd_vel', 'cmd_vel_nav') in nodes['behavior_server'].remappings
    assert ('cmd_vel_smoothed', 'cmd_vel_smoothed') in nodes['velocity_smoother'].remappings
    assert nodes['navigation_executor'].wait_for_navigation
    assert not nodes['navigation_executor'].ros_node


def test_precise_keeps_local_dock_with_optional_in_process_websocket() -> None:
    """ROS-only precise mode has one shared manager under the compatible Dock node name."""
    graph = build_graph(GraphOptions('precise', '/run/test'))
    nodes = {node.name: node for node in graph.processes}
    assert nodes['dock_navigator'].parameter_file == '/run/test/params/rendered/dock.yaml'
    assert nodes['dock_navigator'].package == 'nav_executor'
    assert nodes['dock_navigator'].executable == 'nav_task_manager'
    assert 'navigation_executor' not in nodes
    assert len([node for node in graph.processes if node.name == 'bt_navigator']) == 1


def test_bench_external_map_does_not_start_or_manage_localization() -> None:
    """An externally owned map is absent from both processes and lifecycle management."""
    graph = build_graph(GraphOptions('bench_slamgt', '/run/test', map_server=False))
    assert 'map_server' not in graph.lifecycle_nodes
    assert all(node.package not in ('lightning', 'uslam', 'david_slam_gt')
               for node in graph.processes)
    assert not any(node.wait_for_navigation for node in graph.processes)


def test_chassis_and_teleop_modes_never_start_nav2() -> None:
    """Chassis includes its adapter; teleop additionally includes the keyboard."""
    chassis = build_graph(GraphOptions('chassis', '/run/test'))
    teleop = build_graph(GraphOptions('teleop', '/run/test'))
    assert [node.name for node in chassis.processes] == ['wheeltec_robot', 'chassis_adapter']
    assert [node.name for node in teleop.processes] == [
        'wheeltec_robot', 'chassis_adapter', 'chassis_teleop']
    assert chassis.lifecycle_nodes == teleop.lifecycle_nodes == ()


def test_external_driver_is_not_owned_restarted_or_delayed() -> None:
    """Skipping an existing driver removes its restart policy and startup delay."""
    options = GraphOptions('tour', '/run/test', chassis_delay_sec=3.0)
    owned = build_graph(options)
    external = build_graph(replace(options, start_chassis=False))
    assert next(node for node in owned.processes if node.name == 'bt_navigator').delay_sec == 3.0
    assert all(node.name != 'wheeltec_robot' and node.restart_limit == 0
               and node.delay_sec == 0.0 for node in external.processes)


def test_driver_respawns_once_then_stops_graph_and_shutdown_never_respawns() -> None:
    """A finite restart budget does not resurrect the driver during normal exit."""
    graph = build_graph(GraphOptions('tour', '/run/test', rviz=True))
    nodes = {node.name: node for node in graph.processes}
    driver = nodes['wheeltec_robot']
    assert exit_decision(driver, 0, False) == 'restart'
    assert exit_decision(driver, 1, False) == 'shutdown'
    assert exit_decision(driver, 0, True) == 'ignore'
    assert exit_decision(nodes['controller_server'], 0, False) == 'shutdown'
    assert exit_decision(nodes['rviz2'], 0, False) == 'ignore'


def test_profile_topics_and_optional_processes_are_respected() -> None:
    """A non-David graph can use alternate topics and an injected driver package."""
    graph = build_graph(GraphOptions(
        'precise', '/run/other', driver_package='nav_sim', driver_executable='fake_chassis',
        nav_command_topic='/demo/nav', smooth_command_topic='/demo/smooth',
        chassis_command_topic='/demo/drive', chassis_odom_topic='/demo/encoders',
        scan_bridge=True, cloud_topic='/demo/cloud', scan_topic='/demo/scan',
        rviz=True, rviz_delay_sec=12.0, robot_state=False))
    nodes = {node.name: node for node in graph.processes}
    assert nodes['wheeltec_robot'].package == 'nav_sim'
    assert ('cmd_vel', '/demo/drive') in nodes['wheeltec_robot'].remappings
    assert nodes['pointcloud_to_laserscan'].remappings == (
        ('cloud_in', '/demo/cloud'), ('scan', '/demo/scan'))
    assert 'robot_state_publisher' not in nodes
    assert nodes['rviz2'].delay_sec == 12.0


@pytest.mark.parametrize('changes', [
    {'mode': 'unknown'}, {'run_dir': 'relative'}, {'run_dir': '/runs/../prod'},
    {'start_chassis': 'false'}, {'driver_restarts': -1}, {'driver_restarts': True},
    {'chassis_delay_sec': float('nan')}, {'rviz_delay_sec': -1.0},
    {'driver_package': ''}, {'chassis_command_topic': ''},
    {'mode': 'teleop', 'start_executor': True},
    {'mode': 'chassis', 'scan_bridge': True},
    {'ready_timeout_sec': 0.0}, {'ready_timeout_sec': float('inf')},
    {'executor_arguments': 'not argv'}, {'ready_action': ''},
    {'supervisor_keyboard': True}, {'supervisor_keyboard': 'true'},
])
def test_invalid_graphs_are_rejected(changes) -> None:
    """Invalid switches fail while the graph is still pure data."""
    with pytest.raises(ValueError):
        build_graph(replace(GraphOptions('tour', '/run/test'), **changes))


def test_graph_construction_creates_no_run_directory(tmp_path: Path) -> None:
    """The launch model neither renders parameters nor makes directories."""
    missing = tmp_path / 'never_created'
    build_graph(GraphOptions('tour', str(missing)))
    assert not missing.exists()


def test_chassis_external_driver_still_starts_adapter() -> None:
    """An external driver is preserved while this graph owns only its adapter."""
    graph = build_graph(GraphOptions('chassis', '/run/test', start_chassis=False))
    assert [node.name for node in graph.processes] == ['chassis_adapter']
