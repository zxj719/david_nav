"""Launch only a digest-verified private graph; this process never submits goals.

The outer selftest runner owns this entire process group. It must explicitly
authorize and implement its own goal dispatch, observation and result checks.
Starting this module alone is not a completed navigation experiment.
"""

import argparse
import asyncio
import json
import math
import os
from pathlib import Path
import signal
import time
from typing import Optional, Sequence

from nav_sim.model import require_sandbox
from nav_sim.selftest_plan import load_scene
from nav_bringup.navrun.owned_process import OwnedProcess


async def close_fixture(child: OwnedProcess, timeout_sec: float = 15.0) -> int:
    """Allow owned launch/WS teardown while the parent's event loop remains responsive.

    Signal the still-reserved owned group, just like a terminal interrupt.
    Humble's interactive launch service does not forward a leader-only SIGINT.
    OwnedProcess finally drains its own group, even on
    timeout; a leader timeout is never reported as graceful completion. Callers
    must also inspect the child-exit audit before claiming an entirely graceful
    stack shutdown; a leader status alone does not prove every child succeeded.

    Args:
        child: Launcher created by the caller on this event-loop thread.
        timeout_sec: Positive finite grace period before owned-group cleanup.

    Returns:
        Launcher exit status after its group has been drained and reaped.

    Raises:
        ValueError: The grace period is invalid.
        TimeoutError: Graceful termination timed out; owned-group cleanup was attempted.
        OSError: Ownership verification or group cleanup failed.
    """
    if type(timeout_sec) not in (int, float) or not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError('fixture shutdown timeout must be positive and finite')
    if child.poll() is None:
        os.killpg(child.pid, signal.SIGINT)
    deadline = time.monotonic() + timeout_sec
    while child.poll() is None and time.monotonic() < deadline:
        await asyncio.sleep(0.02)
    graceful = child.poll() is not None
    status = child.close(timeout_sec=0)
    if not graceful:
        raise TimeoutError('fixture required forced group cleanup')
    return status


def main(args: Optional[Sequence[str]] = None) -> int:
    """Verify private inputs and run the shared managed launch adapter.

    The explicit start switch is distinct from permission to submit goals. No
    Nav2 client, velocity publisher or WebSocket server is created here. Domain
    and digest checks run before any ROS import or process construction.

    Args:
        args: Internal launcher arguments, or the current process command line.

    Returns:
        Launch status, or 130 for an interrupted launch service.

    Raises:
        RuntimeError: The caller did not explicitly select ROS domain 99.
        ValueError: The experiment or a frozen asset failed integrity checks.
        OSError: A private asset or exclusive launch audit cannot be opened.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--plan-sha256', required=True)
    parser.add_argument('--start-verified-fixture', required=True, action='store_true')
    options = parser.parse_args(args)
    require_sandbox(os.environ)
    scene = load_scene(options.run_dir, options.plan_sha256)

    from launch import LaunchDescription, LaunchService
    from launch.actions import GroupAction, RegisterEventHandler
    from launch.event_handlers import OnProcessExit, OnProcessStart
    from launch_ros.actions import PushRosNamespace

    from nav_bringup.launching.runtime import launch_actions

    # Every child ROS Node inherits the namespace. The plain readiness-gated
    # executor already has explicit ROS remappings and a private action name.
    actions = launch_actions(scene.graph)
    with (scene.root / 'process_events.jsonl').open('x', encoding='utf-8') as audit:
        def observe(event, _context):
            record = {'name': event.process_name, 'pid': event.pid,
                      'node': getattr(event.action, 'node_name', None),
                      'stamp': time.monotonic(),
                      'event': 'exit' if hasattr(event, 'returncode') else 'start'}
            if record['event'] == 'exit':
                record['returncode'] = event.returncode
            audit.write(json.dumps(record, allow_nan=False) + '\n')
            audit.flush()

        description = LaunchDescription([
            RegisterEventHandler(OnProcessStart(on_start=observe)),
            RegisterEventHandler(OnProcessExit(on_exit=observe)),
            GroupAction([PushRosNamespace(scene.namespace), *actions]),
        ])
        service = LaunchService(argv=[])
        service.include_launch_description(description)
        try:
            return service.run()
        except KeyboardInterrupt:
            return 130


if __name__ == '__main__':
    raise SystemExit(main())
