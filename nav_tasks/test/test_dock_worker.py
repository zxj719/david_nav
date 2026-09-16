"""Real blocking Dock workers over fake action futures, with no ROS initialization."""

import asyncio
from concurrent.futures import Future
from dataclasses import replace
import threading
from types import SimpleNamespace as NS

import pytest

from nav_tasks.adapters.ros.dock_worker import DockWorker
from nav_tasks.domain.errors import BusyError, ClosedError
from nav_tasks.domain.footprint import FootprintCheck
from nav_tasks.domain.tasks import NavStatus
from nav_tasks.skills.docking.finish import DockTolerance
from nav_tasks.skills.docking.settle import SettleSpec
from nav_tasks.skills.docking.workflow import DockConfig
from test_action_session import _Client, _done, _Handle, _settle
from test_dock_workflow import _Clock
from test_nav2_client import _goal
from test_spin_backend import _backend


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warn(self, text):
        self.lines.append(('warn', text))

    def error(self, text):
        self.lines.append(('error', text))


def _worker(backend, *, pose_fn=None, logger=None, **kwargs):
    config = DockConfig(
        0.5, DockTolerance(0.1, 0.44, 0), SettleSpec(), 120, 30, 0)
    return DockWorker(
        backend, 'fixture_global', config, pose_fn or (lambda: (1, 2, 0)),
        lambda pose: FootprintCheck(True, 'clear'), logger or _Log(), **kwargs)


async def _bounded(awaitable):
    return await asyncio.wait_for(awaitable, 2)


def test_workflow_runs_on_worker_but_actions_and_feedback_use_owner_loop():
    """Two stages use the shared backend, preserving raw feedback plus approach distance."""
    async def scenario():
        loop_thread = threading.get_ident()
        action_threads = []
        pose_threads = []
        first = _Handle(cancel_status=None)

        class Client(_Client):
            def send_goal_async(self, goal, feedback_callback):
                action_threads.append(threading.get_ident())
                result = super().send_goal_async(goal, feedback_callback)
                self.send = _done(_Handle(status=4))
                return result

        def pose():
            pose_threads.append(threading.get_ident())
            return (1, 2, 0)

        client = Client(_done(first))
        backend = _backend(nav=client)
        feedback = asyncio.Queue()
        log = _Log()
        worker = _worker(backend, pose_fn=pose, logger=log)
        operation = asyncio.create_task(worker.execute(
            (1, 2, 0), lambda raw, extra: feedback.put_nowait(
                (raw, extra, threading.get_ident()))))
        await _bounded(client.sent.wait())
        raw = NS(feedback=NS(distance_remaining=2, navigation_time='untouched'))
        client.feedback(raw)
        assert await _bounded(feedback.get()) == (raw, 0.5, loop_thread)
        first.result_future.set_result(NS(status=4))
        # One pose sample at a zero settle budget cannot establish a settled window.
        assert (await _bounded(operation)).status == NavStatus.FAILED
        assert action_threads == [loop_thread, loop_thread]
        assert pose_threads and all(value != loop_thread for value in pose_threads)
        assert len(client.goals) == 2
        assert [item.pose.pose.position.x for item in client.goals] == [0.5, 1]
        assert all(item.pose.header.frame_id == 'fixture_global' for item in client.goals)
        assert ('info', '[dock] 阶段1(预备点): SUCCEEDED') in log.lines
        assert ('info', '[dock] 阶段2(入位): SUCCEEDED') in log.lines
        assert not worker.busy
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


def test_success_is_only_returned_after_both_stages_and_settled_window():
    """The real workflow on an actual thread retains its 0.4-second settle policy."""
    async def scenario():
        clock = _Clock()
        backend = _backend()
        worker = _worker(backend, now_fn=clock.now, sleep_fn=clock.sleep)
        worker._config = replace(worker._config, settle_timeout_sec=8)
        assert (await _bounded(worker.execute((1, 2, 0)))).succeeded
        assert clock.value == pytest.approx(0.4)
        assert not worker.busy
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('status', [4, 5, 6])
def test_spin_uses_shared_slot_signed_goal_and_keeps_measured_verdict(status):
    """External Spin cancellation/abort remains advisory when the final pose is correct."""
    async def scenario():
        clock = _Clock()
        yaw = [0.7]

        class SpinClient(_Client):
            def send_goal_async(self, goal, feedback_callback):
                yaw[0] = 0
                return super().send_goal_async(goal, feedback_callback)

        spin = SpinClient(_done(_Handle(status=status)))
        backend = _backend(spin=spin)
        log = _Log()
        worker = _worker(backend, pose_fn=lambda: (1, 2, yaw[0]), logger=log,
                         now_fn=clock.now, sleep_fn=clock.sleep)
        worker._config = replace(worker._config, tolerance=DockTolerance(0.1, 0.44, 1),
                                 settle_timeout_sec=8)
        assert (await _bounded(worker.execute((1, 2, 0)))).succeeded
        assert len(spin.goals) == 1
        assert spin.goals[0].target_yaw == pytest.approx(-0.7)
        assert vars(spin.goals[0].time_allowance) == {'sec': 30, 'nanosec': 0}
        assert clock.value == pytest.approx(0.8)
        expected = 'SUCCEEDED' if status == 4 else f'status={status}'
        assert ('info', '[dock] Spin#1: ' + expected) in log.lines
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('boundary', ['pose', 'approach'])
def test_cancel_keeps_thread_owned_until_blocking_callback_returns(boundary):
    """Cooperative cancellation fences results but cannot pretend to kill a callback."""
    async def scenario():
        loop = asyncio.get_running_loop()
        entered = asyncio.Event()
        release = threading.Event()

        def block(value):
            loop.call_soon_threadsafe(entered.set)
            assert release.wait(3)
            return value

        client = _Client()
        backend = _backend(nav=client)
        log = _Log()
        worker = _worker(backend, logger=log)
        if boundary == 'pose':
            worker._pose = lambda: block((1, 2, 0))
        else:
            worker._check = lambda pose: block(FootprintCheck(True, 'clear'))
        operation = asyncio.create_task(worker.execute((1, 2, 0)))
        try:
            await _bounded(entered.wait())
            thread = worker._active.thread
            cancel = asyncio.create_task(worker.cancel())
            await _settle()
            assert thread.is_alive() and worker.busy and not cancel.done()
            release.set()
            await _bounded(cancel)
            assert (await _bounded(operation)).status == NavStatus.CANCELED
            assert not thread.is_alive()
            assert len(client.goals) == (2 if boundary == 'pose' else 0)
            assert not any('===== 成功' in line for _, line in log.lines)
        finally:
            release.set()
            await worker.close()
            await backend.close()

    asyncio.run(scenario())


def test_canceled_close_waiter_retains_cleanup_and_seals_admission():
    """Dropping a close waiter cannot drop the thread, even when the action is late."""
    async def scenario():
        send = Future()
        client = _Client(send)
        backend = _backend(nav=client)
        worker = _worker(backend)
        operation = asyncio.create_task(worker.execute((1, 2, 0)))
        await _bounded(client.sent.wait())
        close = asyncio.create_task(worker.close())
        await _settle()
        close.cancel()
        with pytest.raises(asyncio.CancelledError):
            await close
        assert worker.busy
        with pytest.raises(ClosedError):
            await worker.execute((1, 2, 0))
        handle = _Handle()
        send.set_result(handle)
        await _bounded(worker.close())
        assert (await _bounded(operation)).status == NavStatus.CANCELED
        assert handle.cancel_calls == 1
        assert not worker.busy
        await backend.close()

    asyncio.run(scenario())


def test_completed_old_cleanup_cannot_cancel_a_new_shared_backend_action():
    """Joining a settled attempt twice is idempotent and never invokes backend cancel again."""
    async def scenario():
        send = Future()
        client = _Client(send)
        backend = _backend(nav=client)
        worker = _worker(backend)
        operation = asyncio.create_task(worker.execute((1, 2, 0)))
        await _bounded(client.sent.wait())
        old = worker._active
        cancel = asyncio.create_task(worker.cancel())
        await _settle()
        send.set_result(_Handle())
        await _bounded(cancel)
        assert (await _bounded(operation)).status == NavStatus.CANCELED
        next_handle = _Handle(cancel_status=None)
        client.sent.clear()
        client.send = _done(next_handle)
        navigation = asyncio.create_task(backend.navigate(_goal(), lambda distance: None))
        await _bounded(client.sent.wait())
        await worker._cleanup_task(old)
        assert next_handle.cancel_calls == 0
        next_handle.result_future.set_result(NS(status=4))
        assert (await _bounded(navigation)).succeeded
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('mode,expected', [
    ('unavailable', '[dock] 阶段1(预备点): action server 不可用'),
    ('rejected', '[dock] 阶段1(预备点): 目标被拒绝'),
    ('aborted', '[dock] 阶段1(预备点): status=6'),
])
def test_admission_and_terminal_failure_logs_remain_frozen(mode, expected):
    """Ordinary action failures retain the precise legacy action log text."""
    async def scenario():
        client = _Client(_done(_Handle(status=6, accepted=mode != 'rejected')),
                         ready=mode != 'unavailable')
        backend = _backend(nav=client)
        log = _Log()
        worker = _worker(backend, logger=log, server_timeout_sec=0)
        assert (await _bounded(worker.execute((1, 2, 0)))).status == NavStatus.FAILED
        assert expected in [line for _, line in log.lines]
        assert not worker.busy
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


def test_send_deadline_logs_before_late_handle_and_cancels_it():
    """A lost admission response retains its late cancellation instead of advancing."""
    async def scenario():
        send = Future()
        client = _Client(send)
        backend = _backend(nav=client)
        log = _Log()
        worker = _worker(backend, logger=log, send_timeout_sec=0)
        operation = asyncio.create_task(worker.execute((1, 2, 0)))
        await _bounded(client.sent.wait())
        await _settle()
        assert ('error', '[dock] 阶段1(预备点): 发目标后 10s 没有应答') in log.lines
        assert not operation.done()
        handle = _Handle()
        send.set_result(handle)
        assert (await _bounded(operation)).status == NavStatus.FAILED
        assert handle.cancel_calls == 1 and len(client.goals) == 1
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


def test_real_task_manager_preempts_dock_only_after_worker_cleanup():
    """The actual manager and worker chain allows POI replacement without stale arrival."""
    from nav_tasks.api import (
        CommandRegistry, EventKind, PreemptPolicy, TaskManager, TaskRequest)

    async def scenario():
        send = Future()
        client = _Client(send)
        backend = _backend(nav=client)
        worker = _worker(backend)
        registry = CommandRegistry()

        async def dock(context, payload):
            return await worker.execute((1, 2, 0))

        async def navigate(context, payload):
            return await backend.navigate(_goal(), context.feedback)

        async def cancel_motion():
            await worker.cancel()
            await backend.cancel()

        registry.register('dock', lambda data: None, dock)
        registry.register('navigate', lambda data: None, navigate)
        manager = TaskManager(registry, cancel_motion)
        events = manager.subscribe_events()
        await manager.submit(TaskRequest('old', 'dock', {}))
        await _bounded(client.sent.wait())
        old_thread = worker._active.thread
        replace_task = asyncio.create_task(manager.submit(TaskRequest(
            'new', 'navigate', {}, preempt=PreemptPolicy.ALLOW)))
        await _settle()
        assert not replace_task.done()
        handle = _Handle(cancel_status=None)
        send.set_result(handle)
        await _bounded(handle.cancel_started.wait())
        assert len(client.goals) == 1
        client.send = _done(_Handle(status=4))
        handle.result_future.set_result(NS(status=5))
        await _bounded(replace_task)
        assert not old_thread.is_alive()
        event = await _bounded(events.get())
        assert event.kind == EventKind.SUCCEEDED and event.task_id == 'new'
        assert events.empty()
        await manager.close()
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


def test_explicit_spin_cancel_does_not_report_success_from_measured_pose():
    """The owner fences the old finish corner where a canceled spin can look successful."""
    async def scenario():
        clock = _Clock()
        handle = _Handle(cancel_status=None)
        spin = _Client(_done(handle))
        backend = _backend(spin=spin)
        yaw = [0.7]
        log = _Log()
        worker = _worker(backend, pose_fn=lambda: (1, 2, yaw[0]), logger=log,
                         now_fn=clock.now, sleep_fn=clock.sleep)
        worker._config = replace(worker._config, tolerance=DockTolerance(0.1, 0.44, 1),
                                 settle_timeout_sec=8)
        operation = asyncio.create_task(worker.execute((1, 2, 0)))
        await _bounded(spin.sent.wait())
        cancel = asyncio.create_task(worker.cancel())
        await _bounded(handle.cancel_started.wait())
        yaw[0] = 0
        handle.result_future.set_result(NS(status=5))
        await _bounded(cancel)
        assert (await _bounded(operation)).status == NavStatus.CANCELED
        assert not any('===== 成功' in line for _, line in log.lines)
        assert len(spin.goals) == 1
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


def test_blocking_port_refuses_calls_from_the_asyncio_owner_thread():
    """An accidental synchronous call from the owner fails instead of deadlocking it."""
    async def scenario():
        send = Future()
        client = _Client(send)
        backend = _backend(nav=client)
        worker = _worker(backend)
        operation = asyncio.create_task(worker.execute((1, 2, 0)))
        await _bounded(client.sent.wait())
        with pytest.raises(RuntimeError, match='blocking Dock'):
            worker._active.motion.navigate((1, 2, 0), 'test', 1, 0)
        send.set_result(_Handle(status=4))
        await _bounded(operation)
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


def test_worker_exception_is_reported_and_cleanup_can_be_retried():
    """A worker port failure never silently releases its owner as success."""
    async def scenario():
        backend = _backend()
        worker = _worker(backend)

        def fail(pose):
            raise OSError('map snapshot failed')

        worker._check = fail
        with pytest.raises(OSError, match='map snapshot'):
            await _bounded(worker.execute((1, 2, 0)))
        assert worker.busy and not worker._active.thread.is_alive()
        await worker.cancel()
        assert not worker.busy
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


def test_cancel_waits_for_late_handle_ack_result_and_worker_exit():
    """Explicit cancel cannot release the worker or admit ordinary POI motion early."""
    async def scenario():
        send = Future()
        client = _Client(send)
        backend = _backend(nav=client)
        log = _Log()
        worker = _worker(backend, logger=log)
        operation = asyncio.create_task(worker.execute((1, 2, 0)))
        await _bounded(client.sent.wait())
        thread = worker._active.thread
        operation.cancel()
        await _settle()
        assert thread.is_alive() and worker.busy
        with pytest.raises(BusyError):
            await worker.execute((2, 3, 0))
        with pytest.raises(BusyError):
            await backend.navigate(_goal(), lambda distance: None)
        handle = _Handle(cancel_status=None)
        handle.cancel_future = Future()
        send.set_result(handle)
        await _bounded(handle.cancel_started.wait())
        handle.result_future.set_result(NS(status=5))
        operation.cancel()
        await _settle()
        assert not operation.done() and worker.busy
        handle.cancel_future.set_result(None)
        with pytest.raises(asyncio.CancelledError):
            await _bounded(operation)
        assert not thread.is_alive() and not worker.busy
        assert len(client.goals) == 1
        assert ('warn', '[dock] 阶段1(预备点): 收到取消请求，取消当前目标') in log.lines
        assert not any('===== 成功' in line for _, line in log.lines)
        client.send = _done(_Handle(status=4))
        assert (await backend.navigate(_goal(), lambda distance: None)).succeeded
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


def test_result_deadline_keeps_frozen_log_and_cleans_before_return():
    """A stage timeout never leaks a second stage or loses action ownership."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        client = _Client(_done(handle))
        backend = _backend(nav=client)
        log = _Log()
        worker = _worker(backend, logger=log)
        worker._config = DockConfig(
            0.5, DockTolerance(), SettleSpec(), 0, 30, 0)
        operation = asyncio.create_task(worker.execute((1, 2, 0)))
        await _bounded(handle.cancel_started.wait())
        assert ('error', '[dock] 阶段1(预备点): 超时 0s，取消') in log.lines
        assert not operation.done() and worker.busy
        handle.result_future.set_result(NS(status=5))
        assert (await _bounded(operation)).status == NavStatus.FAILED
        assert not worker.busy and len(client.goals) == 1
        await worker.close()
        await backend.close()

    asyncio.run(scenario())


def test_cleanup_failure_retains_worker_owner_until_explicit_retry():
    """Cleanup errors propagate instead of becoming a stage failure followed by judgement."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        backend = _backend(nav=_Client(_done(handle)), cancel_timeout_sec=0.01)
        worker = _worker(backend)
        worker._config = DockConfig(
            0.5, DockTolerance(), SettleSpec(), 0, 30, 0)
        with pytest.raises(asyncio.TimeoutError):
            await _bounded(worker.execute((1, 2, 0)))
        assert worker.busy and backend._session.busy
        with pytest.raises(BusyError):
            await worker.execute((2, 3, 0))
        handle.result_future.set_result(NS(status=5))
        await worker.cancel()
        assert not worker.busy
        await worker.close()
        with pytest.raises(ClosedError):
            await worker.execute((1, 2, 0))
        await backend.close()

    asyncio.run(scenario())
