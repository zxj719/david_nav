"""Exercise real launch event handlers without starting processes or a ROS context."""

from dataclasses import replace

from launch import LaunchContext
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.events.process import ProcessExited
import pytest

from nav_bringup.launching.graph import build_graph
from nav_bringup.launching.graph import GraphOptions
from nav_bringup.launching.graph import LaunchGraph
from nav_bringup.launching.supervision import managed_actions


def _exit(registration, process, context, returncode=1):
    registration.execute(context)
    event = ProcessExited(
        action=process, name='memory-only', cmd=['not-executed'], cwd=None,
        env=None, pid=123, returncode=returncode)
    assert registration.event_handler.matches(event)
    return list(registration.event_handler.handle(event, context))


def test_launch_handler_restarts_driver_once_then_emits_global_shutdown() -> None:
    """The actual event callback cannot create a third driver action."""
    constructed = []

    def factory(spec):
        process = ExecuteProcess(cmd=['not-executed'], name=spec.name)
        constructed.append((spec.name, process))
        return process

    graph = build_graph(GraphOptions('chassis', '/run/test'))
    actions = managed_actions(graph, factory)
    context = LaunchContext()
    restarted = _exit(actions[0], actions[1], context)
    assert [name for name, _ in constructed] == [
        'wheeltec_robot', 'chassis_adapter', 'wheeltec_robot']
    assert restarted[1] is not actions[1]
    exhausted = _exit(restarted[0], restarted[1], context)
    assert len(constructed) == 3
    assert len(exhausted) == 1 and isinstance(exhausted[0], EmitEvent)


def test_shutdown_does_not_construct_a_replacement_process() -> None:
    """An exiting driver during global shutdown consumes no restart budget."""
    constructed = []

    def factory(spec):
        constructed.append(spec)
        return ExecuteProcess(cmd=['not-executed'])

    actions = managed_actions(build_graph(GraphOptions('chassis', '/run/test')), factory)
    context = LaunchContext()
    context._set_is_shutdown(True)
    assert _exit(actions[0], actions[1], context) == []
    assert [spec.name for spec in constructed] == ['wheeltec_robot', 'chassis_adapter']


def test_critical_clean_exit_shuts_down_but_optional_rviz_exit_does_not() -> None:
    """Critical ownership is not weakened by a zero process return code."""
    graph = build_graph(GraphOptions('tour', '/run/test', rviz=True))
    context = LaunchContext()
    for name, should_stop in (('controller_server', True), ('rviz2', False)):
        spec = next(node for node in graph.processes if node.name == name)
        single = LaunchGraph(graph.mode, (spec,), ())
        actions = managed_actions(single, lambda _: ExecuteProcess(cmd=['not-executed']))
        result = _exit(actions[0], actions[1], context, 0)
        assert bool(result) == should_stop


def test_delayed_process_is_registered_before_its_timer() -> None:
    """Startup delay is represented by launch actions, not blocking sleeps."""
    graph = build_graph(GraphOptions('tour', '/run/test', chassis_delay_sec=3.0))
    actions = managed_actions(graph, lambda _: ExecuteProcess(cmd=['not-executed']))
    assert isinstance(actions[3], TimerAction)
    assert not isinstance(actions[1], TimerAction)


def test_duplicate_processes_fail_before_any_factory_call() -> None:
    """Duplicate identities cannot get independent restart budgets or double drivers."""
    graph = build_graph(GraphOptions('chassis', '/run/test'))
    graph = replace(graph, processes=graph.processes * 2)
    with pytest.raises(ValueError):
        managed_actions(graph, lambda _: pytest.fail('factory must not run'))
