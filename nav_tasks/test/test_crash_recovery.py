"""R2 supervised crash recovery with fake motion, including competing lifecycle calls."""

import asyncio

import pytest

from nav_tasks.api import (
    ClosedError, CommandRegistry, EventKind, NavOutcome, NavStatus, PreemptPolicy,
    TaskManager, TaskState,
)

from test_task_manager import _manager, _request, _settle, _Skill


async def _crash(manager, skill, events, receipts):
    await manager.submit(_request(), on_settled=receipts.append)
    await skill.starts.get()
    skill.results[0].set_exception(RuntimeError('private failure detail'))
    event = await asyncio.wait_for(events.get(), 1)
    assert (event.kind, event.detail) == (EventKind.FAILED, '执行器内部错误: RuntimeError')
    await asyncio.wait_for(skill.cancel_entered.wait(), 1)


def test_crash_reports_once_then_releases_only_after_backend_cleanup():
    """An error reply is immediate, while idle and settlement require cleanup proof."""
    async def scenario():
        skill = _Skill()
        skill.cancel_release = asyncio.Event()
        manager = _manager(skill)
        events, receipts = manager.subscribe_events(), []
        try:
            await _crash(manager, skill, events, receipts)
            assert manager.status().task_id == 'task'
            assert manager.status().state == TaskState.ABORTING
            skill.contexts[0].feedback(100)
            assert events.empty() and not receipts
            skill.cancel_release.set()
            await _settle()
            assert manager.status().state == TaskState.IDLE
            assert manager.status().task_id is None
            assert len(receipts) == 1
            assert receipts[0].outcome.status == NavStatus.FAILED
            assert receipts[0].outcome.detail == '执行器内部错误: RuntimeError'
            assert events.empty()
            await manager.submit(_request('next'))
            await skill.starts.get()
            skill.results[1].set_result(NavOutcome(NavStatus.SUCCEEDED))
            assert (await events.get()).kind == EventKind.SUCCEEDED
            assert manager.status().state == TaskState.IDLE
        finally:
            skill.cancel_release.set()
            await manager.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('retry', ['cancel', 'close', 'preempt'])
def test_failed_crash_cleanup_retains_owner_until_successful_retry(retry):
    """Failed cancellation is not disguised as idle or retried in an unbounded loop."""
    async def scenario():
        skill = _Skill()
        skill.cancel_error = RuntimeError('cleanup unavailable')
        manager = _manager(skill)
        events, receipts = manager.subscribe_events(), []
        try:
            await _crash(manager, skill, events, receipts)
            await _settle()
            assert manager.status().state == TaskState.ERROR
            assert manager.status().task_id == 'task'
            assert not receipts and events.empty()
            assert skill.trace.count(('cancel-begin',)) == 1
            skill.cancel_error = None
            if retry == 'cancel':
                assert await manager.cancel('task')
                assert manager.status().state == TaskState.IDLE
            elif retry == 'close':
                await manager.close()
                assert manager.status().state == TaskState.CLOSED
            else:
                await manager.submit(_request('next', preempt=PreemptPolicy.ALLOW))
                await skill.starts.get()
                assert manager.status().task_id == 'next'
            assert len(receipts) == 1 and receipts[0].outcome.status == NavStatus.FAILED
            assert events.empty()
        finally:
            skill.cancel_error = None
            await manager.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('operation', ['cancel', 'preempt', 'close'])
def test_competing_operation_waits_for_crash_cleanup_and_does_not_duplicate_it(operation):
    """Abort, replacement and shutdown share the same serialized ownership cleanup."""
    async def scenario():
        skill = _Skill()
        skill.cancel_release = asyncio.Event()
        manager = _manager(skill)
        events, receipts = manager.subscribe_events(), []
        try:
            await _crash(manager, skill, events, receipts)
            if operation == 'cancel':
                pending = asyncio.create_task(manager.cancel('task'))
            elif operation == 'preempt':
                pending = asyncio.create_task(manager.submit(
                    _request('task', preempt=PreemptPolicy.ALLOW)))
            else:
                pending = asyncio.create_task(manager.close())
            await _settle()
            assert not pending.done() and len(skill.contexts) == 1
            skill.cancel_release.set()
            await pending
            assert len(receipts) == 1 and receipts[0].outcome.status == NavStatus.FAILED
            assert skill.trace.count(('cancel-begin',)) == 1
            if operation == 'preempt':
                await skill.starts.get()
                assert manager.status().generation == 2
                skill.contexts[0].feedback(99)
                assert events.empty()
                skill.results[1].set_result(NavOutcome(NavStatus.SUCCEEDED))
                assert (await events.get()).generation == 2
        finally:
            skill.cancel_release.set()
            await manager.close()

    asyncio.run(scenario())


def test_canceled_close_waiter_cannot_abandon_crash_cleanup():
    """Shutdown seals admission but its caller does not own the cleanup lifetime."""
    async def scenario():
        skill = _Skill()
        skill.cancel_release = asyncio.Event()
        manager = _manager(skill)
        events, receipts = manager.subscribe_events(), []
        try:
            await _crash(manager, skill, events, receipts)
            caller = asyncio.create_task(manager.close())
            await _settle()
            caller.cancel()
            with pytest.raises(asyncio.CancelledError):
                await caller
            with pytest.raises(ClosedError):
                await manager.submit(_request('late'))
            assert manager.status().task_id == 'task'
            skill.cancel_release.set()
            await manager.close()
            assert manager.status().state == TaskState.CLOSED
            assert len(receipts) == 1
        finally:
            skill.cancel_release.set()
            await manager.close()

    asyncio.run(scenario())


def test_queued_recovery_cannot_cancel_a_replacement_with_the_same_id():
    """A preempt already waiting on the lock wins; late recovery is identity fenced."""
    async def scenario():
        skill = _Skill()
        manager = _manager(skill)
        events, receipts = manager.subscribe_events(), []
        try:
            await manager.submit(_request(), on_settled=receipts.append)
            await skill.starts.get()
            async with manager._transition_lock:
                replacing = asyncio.create_task(manager.submit(
                    _request('task', preempt=PreemptPolicy.ALLOW)))
                await _settle()
                error = RuntimeError('old crash')
                skill.results[0].set_exception(error)
                assert (await events.get()).kind == EventKind.FAILED
                await _settle()
                assert not skill.cancel_entered.is_set()
            accepted = await replacing
            await skill.starts.get()
            await _settle()
            manager._on_crash('skill:task:1', error)
            assert manager.status().generation == accepted.generation == 2
            assert skill.trace.count(('cancel-begin',)) == 1
            assert len(receipts) == 1 and receipts[0].outcome.status == NavStatus.FAILED
            assert events.empty()
        finally:
            await manager.close()

    asyncio.run(scenario())


def test_new_worker_exception_during_explicit_cancel_still_requires_retry():
    """A cleanup-time exception is not silently swallowed as an already-reported crash."""
    async def scenario():
        registry, started, receipts = CommandRegistry(), asyncio.Event(), []

        async def run(context, payload):
            started.set()
            try:
                await asyncio.Event().wait()
            finally:
                raise RuntimeError('worker cleanup failed')

        async def cancel():
            pass

        registry.register('fixture', lambda p: p, run)
        manager = TaskManager(registry, cancel)
        events = manager.subscribe_events()
        await manager.submit(_request(), on_settled=receipts.append)
        await started.wait()
        with pytest.raises(RuntimeError, match='worker cleanup failed'):
            await manager.cancel()
        assert manager.status().state == TaskState.ERROR
        assert manager.status().task_id == 'task' and not receipts and events.empty()
        assert await manager.cancel()
        assert manager.status().state == TaskState.IDLE
        assert len(receipts) == 1 and receipts[0].outcome.status == NavStatus.CANCELED
        assert (await events.get()).status == 'paused'
        await manager.close()

    asyncio.run(scenario())
