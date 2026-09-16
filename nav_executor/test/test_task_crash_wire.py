"""R2 error delivery and subsequent navigation through the actual WS task adapter."""

import asyncio

from nav_tasks.api import TaskState

from test_task_wire_golden import _ARRIVED, _Harness, _NAVIGATE, _settle, _wire


def test_backend_crash_returns_on_error_cleans_task_and_accepts_next_navigation():
    """A fake navigate exception cannot strand the server behind a zombie task."""
    async def scenario():
        harness = _Harness([4])
        original = harness.backend.navigate

        async def crash(goal, feedback):
            raise RuntimeError('private failure detail')

        try:
            harness.backend.navigate = crash
            assert await harness.adapter.handle_message(dict(_NAVIGATE)) == []
            event = await asyncio.wait_for(harness.adapter.next_event(), 1)
            assert _wire(event) == (
                '{"event_type": "on_error", "request_id": "req_1", "sub_id": 1, '
                '"error_message": "执行器内部错误: RuntimeError"}')
            await _settle()
            assert harness.tasks.status().state == TaskState.IDLE
            assert harness.tasks.status().task_id is None
            harness.backend.navigate = original
            assert await harness.adapter.handle_message(dict(_NAVIGATE)) == []
            assert _wire(await asyncio.wait_for(harness.adapter.next_event(), 1)) == _ARRIVED
            assert len(harness.client.goals) == 1
            assert harness.tasks.status().state == TaskState.IDLE
        finally:
            await harness.close()

    asyncio.run(scenario())
