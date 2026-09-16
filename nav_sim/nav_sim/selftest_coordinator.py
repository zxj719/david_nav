"""Cooperative POI/Dock sequencing using independent observations, never fake success."""

import asyncio
import json
import math
import time
from typing import Any, Mapping, Sequence

from nav_sim.fake_agent_server import navigate_to
from nav_sim.model import finite_number
from nav_sim.selftest_evidence import check_evidence
from nav_sim.selftest_scene import Scene
from nav_sim.selftest_trace import audit_processes, EvidenceTrace, reached


class Peer:
    """One non-replayable WebSocket connection with bounded evidence intake."""

    def __init__(self) -> None:
        """Initialize a disconnected peer; bind trace before starting any child."""
        self.trace = self.socket = self.error = None
        self.stopping = False
        self._accepted = False

    async def handle(self, socket: Any) -> None:
        """Observe one client and answer heartbeat only; commands belong to drive().

        Args:
            socket: The loopback server's single accepted connection.
        """
        if self._accepted or self.trace is None:
            self.error = 'unexpected_or_reconnected_client'
            await socket.close(code=1008, reason='single experiment connection')
            return
        self._accepted, self.socket = True, socket
        try:
            async for raw in socket:
                frame = self.trace.receive('ws', raw)
                if (frame.get('event_type') == 'on_error'
                        or frame.get('event_type') == 'on_progress'
                        and frame.get('status') == 'paused'):
                    self.error = 'task_failed_or_paused'
                if frame.get('type') == 'ping':
                    await socket.send(json.dumps({'type': 'pong'}))
        except Exception as error:
            self.error = 'websocket_' + type(error).__name__
        finally:
            self.socket = None
            if not self.stopping and self.error is None:
                self.error = 'client_disconnected'


async def drive(scene: Scene, trace: EvidenceTrace, peer: Peer, observer: Any,
                child: Any, timeout: float, startup_only: bool) -> Sequence[Mapping[str, object]]:
    """Wait for real readiness, then perform exactly one POI and one Dock experiment.

    Args:
        scene: Private immutable expectations and real launch graph.
        trace: Bounded WS and NavEvent evidence.
        peer: Single-use connected command transport.
        observer: Read-only ROS observer; injected memory ports are only for pure tests.
        child: Still-owned launcher with non-reaping poll().
        timeout: Positive finite total experiment deadline, including startup.
        startup_only: Never dispatch commands when True; returns no completion evidence.

    Returns:
        Independent final TF/odometry measurements for each completed case.
        These alone are not overall success: runtime must still verify cleanup.

    Raises:
        ValueError: Timeout or process/evidence records are invalid.
        RuntimeError: A process, transport or observation fails.
        TimeoutError: Readiness, pose or completion evidence does not arrive in time.
    """
    if finite_number(timeout, 'timeout') <= 0:
        raise ValueError('timeout must be positive')
    deadline = time.monotonic() + timeout
    inspected = 0

    async def until(predicate):
        nonlocal inspected
        while True:
            if time.monotonic() >= deadline:
                raise TimeoutError('selftest deadline exceeded')
            if child.poll() is not None:
                raise RuntimeError('launcher exited before experiment completion')
            if peer.error:
                raise RuntimeError(peer.error)
            audit_processes(scene)
            observer.step()
            for event in trace.nav[inspected:]:
                source = event.get('source')
                permitted = {
                    'tasks': ('task_admitted', 'task_phase', 'task_progress', 'task_succeeded'),
                    'dock': ('dock_started', 'dock_approach_checked', 'dock_stage_started',
                             'dock_measured'),
                }
                if source in permitted and (event.get('event_type') not in permitted[source]
                                            or event.get('severity') != 'info'):
                    raise RuntimeError('unhealthy task observation')
            inspected = len(trace.nav)
            if predicate():
                return
            await asyncio.sleep(0.02)

    await until(lambda: peer.socket is not None and observer.ready() and audit_processes(scene))
    if startup_only:
        return []
    measurements = []
    for index, case in enumerate(scene.cases):
        await until(observer.ready)
        before = observer.odometry()
        if _arrivals(trace, scene.cases) != [item.task_id for item in scene.cases[:index]]:
            raise RuntimeError('premature or duplicate arrival')
        command = navigate_to(case.task_id, case.target_id, case.sub_id)
        await asyncio.wait_for(peer.socket.send(json.dumps(command)),
                               max(0.001, deadline - time.monotonic()))

        def arrived():
            arrivals = _arrivals(trace, scene.cases)
            if arrivals not in ([item.task_id for item in scene.cases[:index]],
                                [item.task_id for item in scene.cases[:index + 1]]):
                raise RuntimeError('unexpected arrival sequence')
            return (len(arrivals) == index + 1 and reached(case, observer.pose())
                    and observer.odometry() is not None and any(
                        event.get('event_type') == 'task_succeeded'
                        and event.get('data', {}).get('task_id') == case.task_id
                        and event['data'].get('ownership_released') is True
                        for event in trace.nav))

        await until(arrived)
        pose, after = observer.pose(), observer.odometry()
        displacement = math.hypot(after[0] - before[0], after[1] - before[1])
        if displacement <= 0.05:
            raise RuntimeError('arrival without independent chassis displacement')
        measurements.append(dict(task_id=case.task_id, pose=pose,
                                 odometry_start=before, odometry_end=after,
                                 odometry_displacement=displacement))
    await until(lambda: check_evidence(scene.cases, trace.nav, trace.ws).complete)
    return measurements


def _arrivals(trace, cases):
    expected = {case.task_id: case for case in cases}
    arrivals = []
    for row in trace.ws:
        name = row.get('event_type')
        if name not in ('on_arrived', 'on_progress', 'on_error'):
            continue
        task = row.get('request_id')
        if (not isinstance(task, str) or task not in expected
                or type(row.get('sub_id')) is not int or row['sub_id'] != expected[task].sub_id):
            raise RuntimeError('mismatched task correlation')
        if task in arrivals:
            raise RuntimeError('task frame after terminal arrival')
        if name == 'on_error' or name == 'on_progress' and row.get('status') == 'paused':
            raise RuntimeError('task failed or paused')
        if name == 'on_arrived':
            arrivals.append(task)
    return arrivals
