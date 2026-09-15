"""Bounded launch-owned process restarts, without process discovery or external signals."""

from __future__ import annotations

from dataclasses import replace
from typing import Callable, List

from launch import LaunchContext
from launch.action import Action
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited

from nav_bringup.launching.graph import exit_decision
from nav_bringup.launching.graph import LaunchGraph
from nav_bringup.launching.graph import ProcessSpec


ProcessFactory = Callable[[ProcessSpec], ExecuteProcess]


def managed_actions(graph: LaunchGraph, factory: ProcessFactory) -> List[Action]:
    """Build launch actions with a finite restart budget for each owned process.

    All initial process objects are constructed before any action is returned.
    The production factory must resolve executables and parameters eagerly, so
    a missing dependency fails before a driver is started. Restarts construct
    a fresh action: no unlimited respawn coroutine can outlive the budget.

    Args:
        graph: Validated process graph with no externally owned processes.
        factory: Constructor of an unexecuted process action; injectable in tests.

    Returns:
        Handlers registered before the corresponding processes or start timers.

    Raises:
        ValueError: If process identities are duplicated.
    """
    names = [process.name for process in graph.processes]
    if len(names) != len(set(names)):
        raise ValueError('owned process identities must be unique')
    actions = []
    for spec in graph.processes:
        actions.extend(_owned_actions(spec, factory, 0))
    return actions


def _owned_actions(spec: ProcessSpec, factory: ProcessFactory, used: int) -> List[Action]:
    process = factory(spec)

    def on_exit(event: ProcessExited, context: LaunchContext) -> List[Action]:
        decision = exit_decision(spec, used, context.is_shutdown)
        if decision == 'restart':
            return _owned_actions(replace(spec, delay_sec=0.0), factory, used + 1)
        if decision == 'shutdown':
            return [EmitEvent(event=Shutdown(reason=(
                f'{spec.name} exited ({event.returncode}); restart budget {used} exhausted')))]
        return []

    handler = RegisterEventHandler(OnProcessExit(
        target_action=process, on_exit=on_exit, handle_once=True))
    start = TimerAction(period=spec.delay_sec, actions=[process]) if spec.delay_sec else process
    return [handler, start]
