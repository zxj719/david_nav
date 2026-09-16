"""Observation-only task lifecycle, separate from frozen WS progress semantics."""

import asyncio

import pytest

from nav_tasks.api import (
    CommandRegistry, EventKind, NavOutcome, NavStatus, PreemptPolicy,
    TaskManager, TaskRequest, TaskState,
)


@pytest.mark.parametrize('status', [NavStatus.SUCCEEDED, NavStatus.FAILED])
def test_terminal_observation_follows_release_without_changing_legacy_events(status):
    """An immediate result has admission then one post-release, correlated terminal."""
    async def scenario():
        registry, observed = CommandRegistry(), []

        async def run(context, payload):
            context.describe('poi', 'navigation')
            context.feedback(1.25)
            return NavOutcome(status, 'detail')

        async def cancel():
            pytest.fail('completed task cannot need cancellation')

        def record(name, data):
            observed.append((name, data, tasks.status().state))

        registry.register('go', lambda payload: payload, run)
        tasks = TaskManager(registry, cancel, on_event=record)
        legacy = tasks.subscribe_events()
        accepted = await tasks.submit(TaskRequest('t', 'go', {}, sub_id=3, source='websocket'))
        await asyncio.sleep(0)
        assert [row[0] for row in observed] == [
            'task_admitted', 'task_phase', 'task_progress', 'task_' + status.value]
        name, data, state = observed[-1]
        assert state == TaskState.IDLE and data['ownership_released']
        assert data['task_id'] == 't' and data['sub_id'] == 3
        assert data['generation'] == accepted.generation
        assert data['ingress_source'] == 'websocket' and data['detail'] == 'detail'
        assert data['target_id'] == 'poi' and data['phase'] == 'navigation'
        assert legacy.get_nowait().kind == EventKind.PROGRESS
        assert legacy.get_nowait().kind.value == status.value
        assert legacy.empty()
        await tasks.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('method', ['cancel', 'preempt', 'close'])
def test_cancellation_is_observed_only_after_cleanup_and_fences_old_generation(method):
    """A reused ID cannot inherit old feedback or premature release observations."""
    async def scenario():
        registry, observed, started = CommandRegistry(), [], asyncio.Queue()
        gate, cancel_started = asyncio.Event(), asyncio.Event()

        async def run(context, payload):
            started.put_nowait(context)
            await asyncio.Event().wait()

        async def cancel():
            cancel_started.set()
            await gate.wait()

        registry.register('go', lambda payload: payload, run)
        tasks = TaskManager(registry, cancel, on_event=lambda n, d: observed.append((n, d)))
        legacy = tasks.subscribe_events()
        first = await tasks.submit(TaskRequest('same', 'go', {}))
        old = await started.get()
        if method == 'preempt':
            pending = asyncio.create_task(tasks.submit(TaskRequest(
                'same', 'go', {}, preempt=PreemptPolicy.ALLOW)))
        else:
            pending = asyncio.create_task(getattr(tasks, method)())
        await cancel_started.wait()
        assert not any(name == 'task_canceled' for name, data in observed)
        assert observed[-1][0] == 'task_cleanup_started'
        assert not observed[-1][1]['ownership_released']
        gate.set()
        await pending
        ended = [data for name, data in observed if name == 'task_canceled']
        assert len(ended) == 1 and ended[0]['generation'] == first.generation
        assert ended[0]['ownership_released']
        before = list(observed)
        old.feedback(999)
        old.describe('stale', 'stale')
        assert observed == before
        if method == 'cancel':
            assert legacy.get_nowait().status == 'paused'
        assert legacy.empty()
        if method == 'preempt':
            second = await started.get()
            assert second.generation > first.generation
        await tasks.close()

    asyncio.run(scenario())


def test_external_cancellation_remains_paused_and_owned_until_explicit_cleanup():
    """G12 is not silently converted into a released task by the new observer."""
    async def scenario():
        registry, observed = CommandRegistry(), []

        async def run(context, payload):
            return NavOutcome(NavStatus.CANCELED)

        async def cancel():
            pass

        registry.register('go', lambda payload: payload, run)
        tasks = TaskManager(registry, cancel, on_event=lambda n, d: observed.append((n, d)))
        await tasks.submit(TaskRequest('a', 'go', {}))
        await asyncio.sleep(0)
        assert [name for name, data in observed] == ['task_admitted', 'task_paused']
        assert not observed[-1][1]['ownership_released']
        assert tasks.status().task_id == 'a'
        await tasks.close()
        assert observed[-1][0] == 'task_canceled'

    asyncio.run(scenario())


def test_failed_crash_cleanup_has_no_terminal_until_successful_retry():
    """Fault and cleanup failure are observations, never proof of released ownership."""
    async def scenario():
        registry, observed, failed = CommandRegistry(), [], asyncio.Event()
        fail = True

        async def run(context, payload):
            raise OSError('private backend detail')

        async def cancel():
            if fail:
                failed.set()
                raise TimeoutError('still owned')

        registry.register('go', lambda payload: payload, run)
        tasks = TaskManager(registry, cancel, on_event=lambda n, d: observed.append((n, d)))
        await tasks.submit(TaskRequest('a', 'go', {}))
        await failed.wait()
        await asyncio.sleep(0)
        assert [name for name, data in observed].count('task_fault') == 1
        assert observed[-1][0] == 'task_cleanup_failed'
        assert not observed[-1][1]['ownership_released']
        assert not any(name == 'task_failed' for name, data in observed)
        fail = False
        await tasks.cancel()
        assert observed[-1][0] == 'task_failed'
        assert observed[-1][1]['ownership_released']
        assert 'private backend detail' not in str(observed)
        await tasks.close()

    asyncio.run(scenario())


def test_broken_observer_cannot_change_task_result_or_existing_receipt():
    """Telemetry errors do not trigger cancellation, alter wire events or lose receipts."""
    async def scenario():
        registry, receipts = CommandRegistry(), []

        async def run(context, payload):
            return NavOutcome(NavStatus.SUCCEEDED)

        async def cancel():
            pytest.fail('observer error must not cancel completed motion')

        def broken(name, data):
            raise ValueError('bad observer')

        registry.register('go', lambda payload: payload, run)
        tasks = TaskManager(registry, cancel, on_event=broken)
        legacy = tasks.subscribe_events()
        await tasks.submit(TaskRequest('a', 'go', {}), on_settled=receipts.append)
        await asyncio.sleep(0)
        assert tasks.status().state == TaskState.IDLE
        assert legacy.get_nowait().kind == EventKind.SUCCEEDED and legacy.empty()
        assert len(receipts) == 1 and receipts[0].outcome.succeeded
        await tasks.close()

    asyncio.run(scenario())
