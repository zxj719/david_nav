"""Test eager runtime resolution and process wiring with unexecuted actions only."""

from launch.actions import ExecuteProcess
import pytest

from nav_bringup.launching.graph import build_graph
from nav_bringup.launching.graph import GraphOptions
from nav_bringup.launching import runtime


def test_combined_teleop_refuses_before_starting_hardware(monkeypatch) -> None:
    """Standalone guard cannot be bypassed by starting all teleop children concurrently."""
    monkeypatch.setattr(runtime, '_resolve',
                        lambda *_: pytest.fail('must reject before resolution'))
    with pytest.raises(ValueError, match='supervisor-owned'):
        runtime.launch_actions(build_graph(GraphOptions('teleop', '/run/test')))


def test_managed_teleop_launch_only_owns_driver_and_adapter(monkeypatch) -> None:
    """The keyboard stays in the complete plan but must not be spawned twice."""
    nodes = []
    monkeypatch.setattr(runtime, '_resolve', lambda package, name: '/unused/' + name)
    monkeypatch.setattr(runtime, 'Node', lambda **kwargs: (
        nodes.append(kwargs['name']) or ExecuteProcess(cmd=['not-executed'])))
    graph = build_graph(GraphOptions('teleop', '/run/test', supervisor_keyboard=True))
    assert any(spec.executable == 'chassis_teleop_managed' for spec in graph.processes)
    runtime.launch_actions(graph)
    assert nodes == ['wheeltec_robot', 'chassis_adapter']


def test_missing_package_fails_before_any_node_constructor(monkeypatch) -> None:
    """A late missing dependency cannot leave the early hardware driver started."""
    def resolve(package, executable):
        if package == 'nav2_controller':
            raise ValueError('missing controller')
        return '/unexecuted/' + executable

    monkeypatch.setattr(runtime, '_resolve', resolve)
    monkeypatch.setattr(runtime, 'Node', lambda **_: pytest.fail('must resolve everything first'))
    with pytest.raises(ValueError, match='missing controller'):
        runtime.launch_actions(build_graph(GraphOptions('tour', '/run/test')))


def test_node_mapping_keeps_prepared_parameters_and_topics(monkeypatch) -> None:
    """The runtime does not re-render or override the prepared parameter values."""
    nodes = []

    def node(**kwargs):
        nodes.append(kwargs)
        return ExecuteProcess(cmd=['not-executed'])

    monkeypatch.setattr(
        runtime, '_resolve', lambda package, executable: '/unexecuted/' + executable)
    monkeypatch.setattr(runtime, 'Node', node)
    runtime.launch_actions(build_graph(GraphOptions('precise', '/run/test')))
    controller = next(item for item in nodes if item['name'] == 'controller_server')
    assert controller['parameters'] == ['/run/test/params/rendered/nav2.yaml']
    assert ('cmd_vel', 'cmd_vel_nav') in controller['remappings']
    assert next(item for item in nodes if item['name'] == 'dock_navigator')['parameters'] == [
        '/run/test/params/rendered/dock.yaml']


def test_executor_has_readiness_gate_and_no_ros_argument_injection(monkeypatch) -> None:
    """The legacy CLI is a plain program, not a renamed ROS Node action."""
    plain = []

    def process(**kwargs):
        plain.append(kwargs)
        return ExecuteProcess(cmd=['not-executed'])

    monkeypatch.setattr(
        runtime, '_resolve', lambda package, executable: '/unexecuted/' + executable)
    monkeypatch.setattr(runtime, 'Node', lambda **_: ExecuteProcess(cmd=['not-executed']))
    monkeypatch.setattr(runtime, 'ExecuteProcess', process)
    options = GraphOptions(
        'tour', '/run/test', start_executor=True, executor_arguments=('--pose-file', '/a b.json'))
    runtime.launch_actions(build_graph(options))
    assert plain[0]['cmd'] == [
        '/unexecuted/nav_ready_exec', '--timeout', '90.0', '--action', '/navigate_to_pose',
        '--', '/unexecuted/navigation_executor.py',
        '--pose-file', '/a b.json']
