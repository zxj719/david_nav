"""Blocking worker facade marshaling stages onto the sole Nav2 action owner loop."""

import asyncio
from concurrent.futures import Future
import logging
import threading
from typing import Callable

from nav_tasks.adapters.ros.action_session import ActionState
from nav_tasks.core.supervisor import TaskSupervisor
from nav_tasks.domain.errors import BusyError
from nav_tasks.domain.tasks import SpinGoal
from nav_tasks.domain.types import PoseRecord
from nav_tasks.nav2_client import Nav2Client
from nav_tasks.skills.docking.ports import DockActionResult, DockLogger, PlanarPose

_LOGGER = logging.getLogger(__name__)


class _DockMotion:
    """One worker's blocking port; all backend operations run on the owning loop."""

    def __init__(
        self, backend: Nav2Client, frame: str, stop: threading.Event,
        supervisor: TaskSupervisor, logger: DockLogger,
        feedback: Callable[[object, float], None] | None,
        server_timeout: float, send_timeout: float,
    ) -> None:
        self._loop = asyncio.get_running_loop()
        self._loop_thread = threading.get_ident()
        self._backend = backend
        self._frame = frame
        self._stop = stop
        self._supervisor = supervisor
        self._logger = logger
        self._feedback = feedback
        self._server_timeout = server_timeout
        self._send_timeout = send_timeout
        self._stage: asyncio.Task | None = None
        self._label: str | None = None
        self._cancel_logged = False

    def navigate(
        self, pose: PlanarPose, label: str, timeout_sec: float, extra_distance: float,
    ) -> DockActionResult:
        """Block the worker until the shared navigation stage has settled.

        Args:
            pose: Global-frame planar target, without snapping.
            label: Frozen stage label.
            timeout_sec: Result budget after admission.
            extra_distance: Approach distance for complete feedback forwarding.

        Returns:
            Settled stage result; cleanup exceptions are never converted to False.
        """
        return self._call('navigate', pose, label, timeout_sec, extra_distance)

    def spin(
        self, yaw: float, label: str, time_allowance_sec: float, timeout_sec: float,
    ) -> DockActionResult:
        """Block for a signed rotation through the same backend and action slot.

        Args:
            yaw: Signed relative angle.
            label: Frozen correction label.
            time_allowance_sec: Nav2 server allowance.
            timeout_sec: Independent local result budget.

        Returns:
            Settled result, without treating angular feedback as remaining meters.
        """
        return self._call('spin', SpinGoal(yaw, time_allowance_sec), label, timeout_sec, 0)

    def request_cancel(self) -> None:
        """Fence future stages and cancel the active stage from the owner loop."""
        self._stop.set()
        stage = self._stage
        if stage is not None and not stage.done() and not self._cancel_logged:
            try:
                self._cancel_logged = True
                self._logger.warn('[dock] %s: 收到取消请求，取消当前目标' % self._label)
            except Exception:
                # Logging cannot be allowed to disable motion cancellation.
                _LOGGER.exception('Dock cancellation audit failed')
            finally:
                stage.cancel()

    def _call(self, kind, goal, label, timeout, extra):
        if threading.get_ident() == self._loop_thread:
            raise RuntimeError('blocking Dock motion called from the owner loop')
        answer = Future()
        self._loop.call_soon_threadsafe(self._dispatch, answer, kind, goal, label, timeout, extra)
        # This is our own cross-thread future, not a ROS future. No executor spins
        # or state mutation occur on the worker while it waits for the loop owner.
        return answer.result()

    def _dispatch(self, answer, kind, goal, label, timeout, extra):
        try:
            if self._stop.is_set():
                self._logger.warn('[dock] %s: 已收到取消请求，不再下发' % label)
                answer.set_result(DockActionResult(False, True))
                return
            if self._stage is not None and not self._stage.done():
                raise BusyError('Dock worker already has a stage in flight')
            self._label = label
            self._cancel_logged = False
            self._stage = self._supervisor.spawn(
                self._run_stage(kind, goal, label, timeout, extra), 'dock-action-stage')
            self._stage.add_done_callback(lambda task: self._finish_stage(task, answer))
        except Exception as error:
            answer.set_exception(error)

    def _finish_stage(self, task, answer):
        if self._stage is task:
            self._stage = None
            self._label = None
        try:
            answer.set_result(task.result())
        except asyncio.CancelledError:
            answer.set_result(DockActionResult(False, True))
        except BaseException as error:
            answer.set_exception(error)

    async def _run_stage(self, kind, goal, label, timeout, extra):
        def deadline(state):
            if state == ActionState.SEND_TIMEOUT:
                self._logger.error('[dock] %s: 发目标后 10s 没有应答' % label)
            else:
                self._logger.error('[dock] %s: 超时 %.0fs，取消' % (label, timeout))

        budgets = dict(server_timeout_sec=self._server_timeout,
                       send_timeout_sec=self._send_timeout,
                       result_timeout_sec=timeout, on_timeout=deadline)
        if kind == 'navigate':
            x, y, yaw = goal
            pose = PoseRecord(self._frame, x, y, 0.0, yaw, 0.0)

            def feedback(raw):
                if not self._stop.is_set() and self._feedback is not None:
                    self._feedback(raw, extra)

            result = await self._backend.navigate_pose(pose, feedback, **budgets)
        else:
            result = await self._backend.spin_action(goal, **budgets)
        if self._stop.is_set():
            return DockActionResult(False, True)
        if result.state == ActionState.UNAVAILABLE:
            self._logger.error('[dock] %s: action server 不可用' % label)
        elif result.state == ActionState.REJECTED:
            self._logger.error('[dock] %s: 目标被拒绝' % label)
        elif result.state == ActionState.TERMINAL:
            status = result.response.status
            ok = status == 4
            self._logger.info('[dock] %s: %s' % (
                label, 'SUCCEEDED' if ok else 'status=%d' % status))
            return DockActionResult(ok)
        return DockActionResult(False)
