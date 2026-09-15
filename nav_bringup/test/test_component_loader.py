"""Fail-closed component loading without initializing ROS or starting any process."""

import asyncio
from types import SimpleNamespace

import pytest

from nav_bringup.launching.component_loader import CheckedComponentLoader


@pytest.mark.parametrize('case', ['success', 'rejected', 'exception', 'absent', 'stalled', 'stop'])
def test_loader_deadline_failure_and_shutdown(case):
    """Failed loads stop the sequence, pending work cancels, and shutdown never loads more."""
    async def check():
        loop = asyncio.get_running_loop()
        context = SimpleNamespace(asyncio_loop=loop, is_shutdown=False)
        calls, futures = [], []

        def call(request):
            calls.append(request.node_name)
            future = loop.create_future()
            futures.append(future)
            if case == 'exception':
                future.set_exception(RuntimeError('transport failed'))
            elif case not in ('stalled', 'stop'):
                future.set_result(SimpleNamespace(success=case != 'rejected',
                                                  error_message='bad component'))
            elif case == 'stop':
                context.is_shutdown = True
            return future

        client = SimpleNamespace(service_is_ready=lambda: case != 'absent', call_async=call)
        loader = CheckedComponentLoader('/unused', [], timeout_sec=0.02)
        requests = [SimpleNamespace(node_name=name) for name in ('first', 'second')]
        if case in ('success', 'stop'):
            await loader._load(context, client, requests)
        else:
            with pytest.raises(RuntimeError, match='failed|timed out'):
                await loader._load(context, client, requests)
        assert calls == ([] if case == 'absent' else ['first', 'second']
                         if case == 'success' else ['first'])
        if case in ('stalled', 'stop'):
            assert futures[0].cancelled()

    asyncio.run(check())


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_loader_invalid_deadline_rejects_before_ros(value):
    """Every load wait has a finite positive bound."""
    with pytest.raises(ValueError, match='timeout'):
        CheckedComponentLoader('/unused', [], value)


def test_shutdown_before_coroutine_start_never_touches_destroyed_client():
    """A queued task can run after launch has already destroyed its ROS adapter."""
    async def check():
        context = SimpleNamespace(asyncio_loop=asyncio.get_running_loop(), is_shutdown=True)
        client = SimpleNamespace(service_is_ready=lambda: pytest.fail('client already destroyed'))
        await CheckedComponentLoader('/unused', [])._load(
            context, client, [SimpleNamespace(node_name='unused')])

    asyncio.run(check())
