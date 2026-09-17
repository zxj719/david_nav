"""用内存门闩核验 POI 标记应答时序，不访问 ROS、网络或真实仓库."""

import asyncio
from types import SimpleNamespace

import pytest

from nav_executor.protocol.task_adapter import TaskCommandAdapter


@pytest.mark.parametrize('outcome', ['success', 'error', 'exception'])
def test_mark_ack_is_returned_only_after_mark_finishes(outcome):
    """ACK 先构造不等于先交付；异常退出时适配器不返回已构造的 ACK.

    Args:
        outcome: 标记成功、结构化失败或抛出存储异常。
    """
    async def scenario():
        entered = asyncio.Event()
        release = asyncio.get_running_loop().create_future()
        events = asyncio.Queue()
        detached = []
        tasks = SimpleNamespace(subscribe_events=lambda: events,
                                unsubscribe_events=detached.append)

        async def mark(poi_id, name):
            assert (poi_id, name) == ('poi-1', '大厅')
            entered.set()
            await release
            if outcome == 'exception':
                raise OSError('storage failed')
            return SimpleNamespace(
                error='capture failed' if outcome == 'error' else None,
                poi=SimpleNamespace(poi_id=poi_id, name=name))

        adapter = TaskCommandAdapter(tasks, SimpleNamespace(mark=mark))
        frames = []

        async def dispatch():
            frames.extend(await adapter.handle_message({
                'action': 'mark_current_poi', 'request_id': 'req-1', 'sub_id': 4,
                'poi': {'id': 'poi-1', 'name': '大厅'}}))

        operation = asyncio.create_task(dispatch())
        try:
            await asyncio.wait_for(entered.wait(), timeout=1)
            assert not operation.done()
            assert frames == []
            release.set_result(None)
            if outcome == 'exception':
                with pytest.raises(OSError, match='storage failed'):
                    await asyncio.wait_for(operation, timeout=1)
                assert frames == []
            else:
                await asyncio.wait_for(operation, timeout=1)
                assert frames[0] == {
                    'event_type': 'on_mark_poi_ack', 'request_id': 'req-1',
                    'sub_id': 4, 'status': 'accepted'}
                terminal = {'request_id': 'req-1', 'sub_id': 4}
                if outcome == 'success':
                    terminal.update(event_type='on_mark_poi_success',
                                    poi={'id': 'poi-1', 'name': '大厅'})
                else:
                    terminal.update(event_type='on_mark_poi_error',
                                    error_message='capture failed')
                assert len(frames) == 2
                assert frames[1] == terminal
        finally:
            if not operation.done():
                operation.cancel()
            await asyncio.gather(operation, return_exceptions=True)
            adapter.close()
        assert detached == [events]

    asyncio.run(scenario())
