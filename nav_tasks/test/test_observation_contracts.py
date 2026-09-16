"""Task labels and retry observations remain fenced, immutable and policy-neutral."""

import asyncio
from dataclasses import FrozenInstanceError

import pytest

from nav_tasks.app.logging import RetryAudit
from nav_tasks.core.registry import CommandRegistry, TaskContext
from nav_tasks.core.retry import RetryFailure, RetryPolicy
from nav_tasks.core.state import TaskManager
from nav_tasks.domain.tasks import NavOutcome, NavStatus, PreemptPolicy, TaskRequest


def test_late_description_cannot_relabel_new_owner_or_terminal_task():
    """Reused request IDs cannot bypass the generation/active-object fence."""
    async def scenario():
        contexts = asyncio.Queue()
        release = asyncio.Event()

        async def handler(context, payload):
            context.describe(payload['target'], 'segment')
            contexts.put_nowait(context)
            await release.wait()
            return NavOutcome(NavStatus.SUCCEEDED)

        async def cancel():
            pass

        registry = CommandRegistry()
        registry.register('move', lambda payload: dict(payload), handler)
        manager = TaskManager(registry, cancel)
        events = manager.subscribe_events()
        await manager.submit(TaskRequest('r', 'move', {'target': 'first'}))
        old = await contexts.get()
        frozen = manager.status()
        with pytest.raises(FrozenInstanceError):
            frozen.phase = 'changed'
        await manager.submit(TaskRequest('r', 'move', {'target': 'second'},
                                         preempt=PreemptPolicy.ALLOW))
        current = await contexts.get()
        old.describe('stale', 'bad')
        assert manager.status().target_id == 'second'
        assert frozen.target_id == 'first'
        release.set()
        await events.get()
        current.describe('late', 'bad')
        assert manager.status().target_id is None
        assert manager.status().phase is None
        await manager.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('status', [NavStatus.SUCCEEDED, NavStatus.CANCELED])
def test_nonfailure_never_notifies_retry_observer(status):
    """Canceled and successful attempts do not consume or log retry budget."""
    async def scenario():
        async def attempt():
            return NavOutcome(status)

        def observe(failure):
            raise AssertionError('nonfailure observed')

        assert await RetryPolicy().run(attempt, None, observe) == NavOutcome(status)

    asyncio.run(scenario())


def test_retry_observation_precedes_delay_and_preserves_failure_detail():
    """An immutable one-based notice is published for each FAILED result."""
    async def scenario():
        order = []

        class Clock:
            async def sleep(self, seconds):
                order.append(('delay', seconds))

        async def attempt():
            order.append('attempt')
            return NavOutcome(NavStatus.FAILED, 'reason')

        result = await RetryPolicy(1, 0.5).run(attempt, Clock(), order.append)
        assert order == ['attempt', RetryFailure(1, 2, 0.5, 'reason'), ('delay', 0.5),
                         'attempt', RetryFailure(2, 2, 0.5, 'reason')]
        assert result.detail == '导航失败(共尝试2次): reason'

    asyncio.run(scenario())


def test_default_retry_logger_keeps_legacy_prefix_and_flush(capsys):
    """The runtime's default retry audit still writes the frozen bridge prefix."""
    RetryAudit()(TaskContext('r', 2, 1, lambda distance: None), RetryFailure(1, 1, 1, 'bad'))
    assert capsys.readouterr().out == (
        '[nav2_action_bridge] NAV FAILURE attempt 1/1 request_id=r sub_id=2: bad\n'
        '[nav2_action_bridge] retry budget exhausted (1 attempts) — '
        'reporting on_error to server: bad\n')
