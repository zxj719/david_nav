"""Fake-only task ownership races; no ROS imports, goals, or velocity publications."""

import asyncio

import pytest

from nav_tasks.api import (
    BusyError, ClosedError, CommandRegistry, EventKind, NavOutcome, NavStatus,
    PreemptPolicy, TaskManager, TaskRequest, TaskState,
)


class _Skill:
    def __init__(self):
        self.contexts = []
        self.starts = asyncio.Queue()
        self.results = []
        self.trace = []
        self.cancel_entered = asyncio.Event()
        self.cancel_release = None
        self.cleanup_release = None
        self.cancel_error = None

    async def run(self, context, payload):
        index = len(self.contexts)
        self.contexts.append(context)
        result = asyncio.get_running_loop().create_future()
        self.results.append(result)
        self.trace.append(('start', index, payload))
        self.starts.put_nowait(index)
        try:
            return await result
        finally:
            if self.cleanup_release is not None:
                await self.cleanup_release.wait()
            self.trace.append(('cleanup', index))

    async def cancel(self):
        self.trace.append(('cancel-begin',))
        self.cancel_entered.set()
        if self.cancel_release is not None:
            await self.cancel_release.wait()
        if self.cancel_error is not None:
            raise self.cancel_error
        self.trace.append(('cancel-end',))


def _manager(skill):
    registry = CommandRegistry()
    registry.register('fixture', lambda values: int(values['value']), skill.run)
    return TaskManager(registry, skill.cancel)


def _request(task_id='task', value=1, preempt=PreemptPolicy.REJECT):
    return TaskRequest(task_id, 'fixture', {'value': value}, sub_id=7, preempt=preempt)


async def _settle():
    for _ in range(5):
        await asyncio.sleep(0)


@pytest.mark.parametrize('outcome,kind', [
    (NavOutcome(NavStatus.SUCCEEDED), EventKind.SUCCEEDED),
    (NavOutcome(NavStatus.FAILED, 'failed'), EventKind.FAILED),
])
def test_submit_nonblocking_terminal_once_and_snapshot_immutable(outcome, kind):
    """Admission returns before navigation; terminal ownership is released once."""
    async def scenario():
        skill = _Skill()
        manager = _manager(skill)
        first = manager.subscribe_events()
        second = manager.subscribe_events()
        accepted = await manager.submit(_request())
        assert accepted.state == TaskState.STARTING
        assert accepted.generation == 1
        await skill.starts.get()
        skill.contexts[0].feedback(2.34567)
        assert accepted.remaining_distance == 0.0
        assert manager.status().remaining_distance == 2.34567
        assert manager.status().state == TaskState.NAVIGATING
        assert (await first.get()).kind == EventKind.PROGRESS
        assert (await second.get()).remaining_distance == 2.34567
        manager.unsubscribe_events(second)
        skill.results[0].set_result(outcome)
        event = await first.get()
        assert event.kind == kind
        assert (event.task_id, event.sub_id, event.generation) == ('task', 7, 1)
        assert event.detail == outcome.detail
        assert manager.status().state == TaskState.IDLE
        skill.contexts[0].feedback(99)
        assert first.empty()
        assert second.empty()
        assert await manager.cancel() is False
        await manager.close()

    asyncio.run(scenario())


def test_busy_reject_and_bad_replacement_do_not_cancel_owner():
    """Invalid or busy ingress never cancels an otherwise valid active task."""
    async def scenario():
        skill = _Skill()
        manager = _manager(skill)
        await manager.submit(_request())
        await skill.starts.get()
        with pytest.raises(BusyError):
            await manager.submit(_request('busy'))
        with pytest.raises(ValueError):
            await manager.submit(_request('invalid', 'not-an-int', PreemptPolicy.ALLOW))
        assert manager.status().task_id == 'task'
        assert not skill.cancel_entered.is_set()
        assert await manager.cancel('different') is False
        assert not skill.cancel_entered.is_set()
        await manager.close()

    asyncio.run(scenario())


def test_abort_waits_for_backend_and_worker_cleanup_and_filters_feedback():
    """Cancellation has two cleanup barriers and reports one paused event afterward."""
    async def scenario():
        skill = _Skill()
        skill.cancel_release = asyncio.Event()
        skill.cleanup_release = asyncio.Event()
        manager = _manager(skill)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await skill.starts.get()
        skill.contexts[0].feedback(3.0)
        await events.get()
        cancel = asyncio.create_task(manager.cancel('task'))
        await skill.cancel_entered.wait()
        assert manager.status().state == TaskState.ABORTING
        skill.contexts[0].feedback(99)
        assert events.empty()
        skill.cancel_release.set()
        await _settle()
        assert not cancel.done()
        assert manager.status().task_id == 'task'
        skill.cleanup_release.set()
        assert await cancel is True
        event = await events.get()
        assert (event.kind, event.status, event.remaining_distance) == (
            EventKind.PROGRESS, 'paused', 3.0)
        assert manager.status().state == TaskState.IDLE
        assert events.empty()
        await manager.close()

    asyncio.run(scenario())


def test_preempt_same_request_id_uses_generation_fence_and_suppresses_old_terminal():
    """Reused IDs cannot make old feedback look like feedback for the new owner."""
    async def scenario():
        skill = _Skill()
        skill.cancel_release = asyncio.Event()
        manager = _manager(skill)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await skill.starts.get()
        replacement = asyncio.create_task(manager.submit(
            _request('task', 2, PreemptPolicy.ALLOW)))
        await skill.cancel_entered.wait()
        assert len(skill.contexts) == 1
        skill.contexts[0].feedback(33)
        skill.cancel_release.set()
        accepted = await replacement
        assert accepted.generation == 2
        assert await skill.starts.get() == 1
        assert skill.trace.index(('cancel-end',)) < skill.trace.index(('start', 1, 2))
        assert skill.trace.index(('cleanup', 0)) < skill.trace.index(('start', 1, 2))
        skill.contexts[0].feedback(44)
        assert events.empty()
        skill.contexts[1].feedback(5)
        event = await events.get()
        assert (event.generation, event.remaining_distance) == (2, 5)
        skill.results[1].set_result(NavOutcome(NavStatus.SUCCEEDED))
        assert (await events.get()).generation == 2
        assert events.empty()
        await manager.close()

    asyncio.run(scenario())


def test_external_cancel_preserves_g12_paused_active_semantics():
    """External cancellation retains the characterized ownership until next command."""
    async def scenario():
        skill = _Skill()
        manager = _manager(skill)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await skill.starts.get()
        skill.contexts[0].feedback(8.5)
        await events.get()
        skill.results[0].set_result(NavOutcome(NavStatus.CANCELED))
        event = await events.get()
        assert event.kind == EventKind.PROGRESS
        assert event.status == 'paused'
        assert event.remaining_distance == 0.0
        assert manager.status().remaining_distance == 0.0
        assert manager.status().state == TaskState.NAVIGATING
        assert manager.status().task_id == 'task'
        skill.contexts[0].feedback(99)
        assert events.empty()
        assert await manager.cancel('task') is True
        assert manager.status().state == TaskState.IDLE
        await manager.close()

    asyncio.run(scenario())


def test_api_wait_cancellation_does_not_interrupt_accepted_preemption():
    """Disconnecting an ingress waiter does not orphan an accepted transition."""
    async def scenario():
        skill = _Skill()
        skill.cancel_release = asyncio.Event()
        manager = _manager(skill)
        await manager.submit(_request())
        await skill.starts.get()
        caller = asyncio.create_task(manager.submit(_request('next', 2, PreemptPolicy.ALLOW)))
        await skill.cancel_entered.wait()
        caller.cancel()
        with pytest.raises(asyncio.CancelledError):
            await caller
        assert manager.status().state == TaskState.ABORTING
        skill.cancel_release.set()
        assert await skill.starts.get() == 1
        assert manager.status().task_id == 'next'
        await manager.close()
        await _settle()
        assert manager._supervisor.active_count == 0

    asyncio.run(scenario())


def test_close_seals_queued_preempt_and_survives_canceled_waiter():
    """A queued replacement cannot start after close seals admission."""
    async def scenario():
        skill = _Skill()
        skill.cancel_release = asyncio.Event()
        manager = _manager(skill)
        await manager.submit(_request())
        await skill.starts.get()
        replacement = asyncio.create_task(manager.submit(_request('next', 2, PreemptPolicy.ALLOW)))
        await skill.cancel_entered.wait()
        closer = asyncio.create_task(manager.close())
        await _settle()
        closer.cancel()
        with pytest.raises(asyncio.CancelledError):
            await closer
        with pytest.raises(ClosedError):
            await manager.submit(_request('late'))
        with pytest.raises(ClosedError):
            manager.subscribe_events()
        skill.cancel_release.set()
        with pytest.raises(ClosedError):
            await replacement
        await manager.close()
        assert len(skill.contexts) == 1
        assert manager.status().state == TaskState.CLOSED
        assert await manager.cancel() is False

    asyncio.run(scenario())


def test_failed_cleanup_keeps_ownership_and_can_be_retried():
    """Never claim idle or start a replacement if backend cancellation fails."""
    async def scenario():
        skill = _Skill()
        skill.cancel_error = RuntimeError('cancel unavailable')
        manager = _manager(skill)
        await manager.submit(_request())
        await skill.starts.get()
        with pytest.raises(RuntimeError, match='cancel unavailable'):
            await manager.submit(_request('next', 2, PreemptPolicy.ALLOW))
        assert manager.status().state == TaskState.ERROR
        assert manager.status().task_id == 'task'
        assert len(skill.contexts) == 1
        with pytest.raises(RuntimeError):
            await manager.close()
        with pytest.raises(ClosedError):
            await manager.submit(_request('late'))
        skill.cancel_error = None
        await manager.close()
        assert manager.status().state == TaskState.CLOSED

    asyncio.run(scenario())


def test_crash_reports_failure_and_supervises_motion_cleanup():
    """R2 automatically cleans up, but an error event alone does not prove idle."""
    async def scenario():
        skill = _Skill()
        skill.cancel_release = asyncio.Event()
        manager = _manager(skill)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await skill.starts.get()
        skill.results[0].set_exception(RuntimeError('skill crashed'))
        event = await events.get()
        assert (event.kind, event.detail) == (EventKind.FAILED, '执行器内部错误: RuntimeError')
        assert manager.status().state == TaskState.ERROR
        skill.contexts[0].feedback(100)
        assert manager.status().state == TaskState.ERROR
        assert events.empty()
        await skill.cancel_entered.wait()
        assert skill.cancel_entered.is_set()
        assert manager.status().state == TaskState.ABORTING
        skill.cancel_release.set()
        await _settle()
        assert manager.status().state == TaskState.IDLE
        await manager.close()

    asyncio.run(scenario())


def test_simultaneous_admission_has_one_owner():
    """Loop interleaving cannot admit two non-preempting skills."""
    async def scenario():
        skill = _Skill()
        manager = _manager(skill)
        results = await asyncio.gather(
            manager.submit(_request('one')), manager.submit(_request('two')),
            return_exceptions=True)
        assert sum(isinstance(result, BusyError) for result in results) == 1
        await skill.starts.get()
        assert len(skill.contexts) == 1
        await manager.close()

    asyncio.run(scenario())


def test_skill_returning_success_during_abort_cannot_emit_arrival():
    """A cancellation-suppressing backend cannot report success after abort begins."""
    async def scenario():
        started = asyncio.Event()
        registry = CommandRegistry()

        async def handler(context, payload):
            started.set()
            try:
                await asyncio.Event().wait()
            except asyncio.CancelledError:
                context.feedback(0)
                return NavOutcome(NavStatus.SUCCEEDED)

        async def cancel_backend():
            await asyncio.sleep(0)

        registry.register('fixture', lambda values: None, handler)
        manager = TaskManager(registry, cancel_backend)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await started.wait()
        await manager.cancel()
        event = await events.get()
        assert (event.kind, event.status) == (EventKind.PROGRESS, 'paused')
        assert events.empty()
        await manager.close()

    asyncio.run(scenario())


def test_canceled_cancel_waiter_still_completes_cleanup():
    """The explicit cancel operation outlives an ingress waiter being canceled."""
    async def scenario():
        skill = _Skill()
        skill.cancel_release = asyncio.Event()
        manager = _manager(skill)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await skill.starts.get()
        caller = asyncio.create_task(manager.cancel())
        await skill.cancel_entered.wait()
        caller.cancel()
        with pytest.raises(asyncio.CancelledError):
            await caller
        assert manager.status().state == TaskState.ABORTING
        skill.cancel_release.set()
        assert (await events.get()).status == 'paused'
        assert manager.status().state == TaskState.IDLE
        await manager.close()

    asyncio.run(scenario())


def test_invalid_outcome_is_a_crash_not_success():
    """A handler violating its result contract reports TypeError before cleanup."""
    async def scenario():
        skill = _Skill()
        manager = _manager(skill)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await skill.starts.get()
        skill.results[0].set_result(None)
        event = await events.get()
        assert event.kind == EventKind.FAILED
        assert event.detail == '执行器内部错误: TypeError'
        assert manager.status().state == TaskState.ERROR
        await _settle()
        assert manager.status().state == TaskState.IDLE
        assert skill.cancel_entered.is_set()
        await manager.close()

    asyncio.run(scenario())


def test_manager_refuses_cross_loop_use():
    """State observations and new admission must remain on the owning loop."""
    async def construct():
        manager = _manager(_Skill())
        await manager.close()
        return manager

    manager = asyncio.run(construct())

    async def wrong_loop():
        with pytest.raises(RuntimeError, match='different event loop'):
            manager.status()
        with pytest.raises(RuntimeError, match='different event loop'):
            await manager.submit(_request())

    asyncio.run(wrong_loop())
