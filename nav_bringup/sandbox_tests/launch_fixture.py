"""Run launch ownership checks using only short-lived Python child processes."""

from __future__ import annotations

from dataclasses import replace
import os
import sys
from typing import Optional, Sequence

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import Shutdown

from nav_bringup.launching.graph import build_graph
from nav_bringup.launching.graph import GraphOptions
from nav_bringup.launching.graph import LaunchGraph
from nav_bringup.launching.supervision import managed_actions


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run one no-hardware launch scenario, refusing non-sandbox environments.

    Args:
        argv: Either restart, shutdown or critical; defaults to process arguments.

    Returns:
        Zero if construction counts and actual child starts satisfy the contract.
    """
    args = list(sys.argv[1:] if argv is None else argv)
    if os.environ.get('ROS_DOMAIN_ID') != '99' or args not in (
            ['restart'], ['shutdown'], ['critical']):
        return 2
    scenario = args[0]
    graph = build_graph(GraphOptions('chassis', '/unused/launch-fixture'))
    spec = replace(graph.processes[0], name='ownership_fixture')
    if scenario == 'critical':
        spec = replace(spec, restart_limit=0)
    graph = LaunchGraph('chassis', (spec,), ())
    constructed = []
    starts = []
    worker = 'import sys; print("owned_child_started", flush=True); sys.exit(1)'
    if scenario == 'shutdown':
        worker = 'import time; time.sleep(30)'

    def factory(process_spec):
        child = ExecuteProcess(cmd=[sys.executable, '-c', worker], name=process_spec.name)
        constructed.append(child)
        return child

    actions = managed_actions(graph, factory)

    def started(event, context):
        starts.append(event.pid)
        if scenario == 'shutdown':
            return [EmitEvent(event=Shutdown(reason='fixture requests owned shutdown'))]
        return []

    description = LaunchDescription([
        RegisterEventHandler(OnProcessStart(on_start=started)), *actions])
    service = LaunchService()
    service.include_launch_description(description)
    service.run()
    expected = 2 if scenario == 'restart' else 1
    passed = len(constructed) == len(starts) == expected and len(set(starts)) == expected
    print(f'ownership {scenario}: constructed={len(constructed)} started={len(starts)} '
          f'passed={passed}', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
