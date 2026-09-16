"""Task completion receipts do not change legacy events or release failed cleanup."""

import asyncio

import pytest

from nav_tasks.api import (
    CommandRegistry, NavOutcome, NavStatus, PreemptPolicy, TaskManager, TaskRequest, TaskState,
)


def test_completion_is_correlated_and_preemption_remains_wire_silent():
    """A separate receipt follows release while existing WS event semantics stay intact."""
    async def scenario():
        registry, started, receipts = CommandRegistry(), asyncio.Queue(), []
        cancel_started, gate = asyncio.Event(), asyncio.Event()

        async def run(context, payload):
            await started.put(context)
            await asyncio.Event().wait()

        async def cancel():
            cancel_started.set()
            await gate.wait()

        registry.register('dock', lambda p: p, run)
        tasks = TaskManager(registry, cancel)
        events = tasks.subscribe_events()
        first = await tasks.submit(TaskRequest('same', 'dock', {}), on_settled=receipts.append)
        await started.get()
        replacing = asyncio.create_task(tasks.submit(TaskRequest(
            'same', 'dock', {}, preempt=PreemptPolicy.ALLOW)))
        await cancel_started.wait()
        stale_cancel = asyncio.create_task(tasks.cancel('same', generation=first.generation))
        await asyncio.sleep(0)
        gate.set()
        second = await replacing
        await started.get()
        assert len(receipts) == 1
        assert (receipts[0].task_id, receipts[0].generation) == ('same', first.generation)
        assert receipts[0].outcome.status == NavStatus.CANCELED
        assert events.empty()
        assert not await stale_cancel
        assert tasks.status() == second
        await tasks.close()
        assert len(receipts) == 1

    asyncio.run(scenario())


@pytest.mark.parametrize('status', [NavStatus.SUCCEEDED, NavStatus.FAILED])
def test_instant_result_notifies_after_ownership_release(status):
    """A receipt remains available even when a skill finishes before submit returns."""
    async def scenario():
        registry, receipts = CommandRegistry(), []

        async def run(context, payload):
            return NavOutcome(status, 'detail')

        async def cancel():
            pytest.fail('completed motion canceled')

        registry.register('dock', lambda p: p, run)
        tasks = TaskManager(registry, cancel)

        def settled(receipt):
            assert tasks.status().state == TaskState.IDLE
            receipts.append(receipt)

        snapshot = await tasks.submit(TaskRequest('a', 'dock', {}), on_settled=settled)
        await asyncio.sleep(0)
        assert len(receipts) == 1 and receipts[0].generation == snapshot.generation
        assert receipts[0].outcome == NavOutcome(status, 'detail')
        await tasks.close()

    asyncio.run(scenario())


def test_failed_cleanup_retains_receipt_until_successful_retry():
    """An ERROR event is not a completion receipt or proof that motion has stopped."""
    async def scenario():
        registry, receipts, started = CommandRegistry(), [], asyncio.Event()
        fail = True

        async def run(context, payload):
            started.set()
            raise OSError('backend failed')

        async def cancel():
            if fail:
                raise TimeoutError('still owned')

        registry.register('dock', lambda p: p, run)
        tasks = TaskManager(registry, cancel)
        accepted = await tasks.submit(TaskRequest('a', 'dock', {}), on_settled=receipts.append)
        await started.wait()
        assert tasks.status().state == TaskState.ERROR and not receipts
        with pytest.raises(TimeoutError):
            await tasks.cancel('a', generation=accepted.generation)
        assert not receipts and tasks.status().task_id == 'a'
        fail = False
        await tasks.cancel('a', generation=accepted.generation)
        assert len(receipts) == 1 and receipts[0].outcome.status == NavStatus.FAILED
        assert receipts[0].outcome.detail == '执行器内部错误: OSError'
        await tasks.close()

    asyncio.run(scenario())


def test_broken_receipt_observer_cannot_corrupt_released_task():
    """Observer failure is reported without turning completed motion back into ERROR."""
    async def scenario():
        registry = CommandRegistry()

        async def run(context, payload):
            return NavOutcome(NavStatus.SUCCEEDED)

        async def cancel():
            pass

        def broken(receipt):
            raise ValueError('observer')

        registry.register('dock', lambda p: p, run)
        tasks = TaskManager(registry, cancel)
        await tasks.submit(TaskRequest('a', 'dock', {}), on_settled=broken)
        await asyncio.sleep(0)
        assert tasks.status().state == TaskState.IDLE
        await tasks.close()

    asyncio.run(scenario())


def test_cancel_waiter_loss_does_not_drop_completion_receipt():
    """Cancellation cleanup and its receipt stay owned after the public waiter disappears."""
    async def scenario():
        registry, receipts = CommandRegistry(), []
        gate, cancel_started = asyncio.Event(), asyncio.Event()

        async def run(context, payload):
            await asyncio.Event().wait()

        async def cancel():
            cancel_started.set()
            await gate.wait()

        registry.register('dock', lambda p: p, run)
        tasks = TaskManager(registry, cancel)
        snapshot = await tasks.submit(TaskRequest('a', 'dock', {}), on_settled=receipts.append)
        waiter = asyncio.create_task(tasks.cancel('a', generation=snapshot.generation))
        await cancel_started.wait()
        waiter.cancel()
        with pytest.raises(asyncio.CancelledError):
            await waiter
        assert not receipts and tasks.status().state == TaskState.ABORTING
        gate.set()
        await tasks.close()
        assert len(receipts) == 1 and receipts[0].outcome.status == NavStatus.CANCELED

    asyncio.run(scenario())
