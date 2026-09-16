"""Real shared runtime assembly publishes observations through fake ROS endpoints only."""

import asyncio
import json

import pytest

from nav_executor.protocol.task_adapter import TaskCommandAdapter
from nav_tasks.api import create_runtime, RuntimeAssemblyError
from test_runtime_assembly import (  # noqa: F401
    _Buffer, _config, _mark, _navigate, _Node, _runtime, ros_io,
)
from test_dock_runtime import _create, _DockNode, _dock_config, _map, dock_ros  # noqa: F401


@pytest.mark.parametrize('publication_fails', [False, True])
@pytest.mark.usefixtures('ros_io')
def test_public_factory_wires_task_events_without_changing_ws_result(
        tmp_path, publication_fails):
    """The actual root opens one publisher and closes it only with its runtime."""
    async def scenario():
        runtime, node, _ = await _runtime(tmp_path)
        assert [publisher.topic for publisher in node.publishers] == ['/nav/events']
        publisher = node.publishers[0]
        publisher.fail = publication_fails
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        assert (await adapter.next_event())['event_type'] == 'on_arrived'
        if not publication_fails:
            events = publisher.messages
            assert events[0].event_type == 'task_admitted'
            assert events[-1].event_type == 'task_succeeded'
            data = json.loads(events[-1].data_json)
            assert data['ingress_source'] == 'websocket' and data['ownership_released']
            assert data['task_id'] == 'r'
        await runtime.close()
        await runtime.close()
        assert node.destroyed_publishers == node.publishers
        before = len(publisher.messages)
        runtime._events.task('late', {})
        assert len(publisher.messages) == before
        adapter.close()

    asyncio.run(scenario())


@pytest.mark.usefixtures('dock_ros')
def test_dock_factory_dual_writes_measured_values_with_no_premature_success(tmp_path):
    """A real worker and factory emit R7 JSON and correlated NavEvent, not a second owner."""
    async def scenario():
        runtime, node, _ = await _create(tmp_path, settle_timeout_sec=0)
        assert [publisher.topic for publisher in node.publishers] == ['/nav/events', '~/events']
        next(sub for sub in node.subscriptions if sub.topic == '/dock_static').callback(_map())
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        assert (await asyncio.wait_for(adapter.next_event(), 3))['event_type'] == 'on_error'
        global_events = node.publishers[0].messages
        measurements = [event for event in global_events if event.event_type == 'dock_measured']
        assert len(measurements) == 1
        value = json.loads(measurements[0].data_json)
        assert value['settled'] is False and value['task_id'] == 'r'
        private = [json.loads(message.data) for message in node.publishers[1].messages]
        measured = next(event for event in private if event['event'] == 'measured')
        assert measured['event_id'] == measurements[0].event_id
        assert measured['stamp'] == measurements[0].stamp
        assert measured['exy'] == value['exy'] and measured['eyaw'] == value['eyaw']
        assert not any(event.event_type == 'task_succeeded' for event in global_events)
        assert global_events[-1].event_type == 'task_failed'
        assert len(node.clients) == 2 and len(node.goals) == 2
        await runtime.close()
        assert len(node.destroyed_publishers) == 2
        adapter.close()

    asyncio.run(scenario())


@pytest.mark.usefixtures('ros_io')
def test_publishers_remain_alive_until_pending_motion_cleanup_finishes(tmp_path):
    """Closing the caller does not destroy observation handles before owned cancellation."""
    async def scenario():
        node = _Node([None])
        node.complete_on_cancel = False
        runtime, _, _ = await _runtime(tmp_path, node)
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        handle = await node.started.get()
        closing = asyncio.create_task(runtime.close())
        await node.canceled.wait()
        assert not node.destroyed_publishers
        handle.result.set_result(type('Result', (), {'status': 5})())
        await closing
        assert node.publishers[0].messages[-1].event_type == 'task_canceled'
        assert node.destroyed_publishers == node.publishers
        adapter.close()

    asyncio.run(scenario())


@pytest.mark.usefixtures('dock_ros')
def test_partial_publisher_open_failure_retains_failed_rollback(tmp_path):
    """The factory retains a publisher even when construction and rollback both fail."""
    async def scenario():
        node = _DockNode()
        node.fail_publisher_at = 2
        node.fail_publisher_destroy = True
        with pytest.raises(RuntimeAssemblyError) as caught:
            await create_runtime(_dock_config(tmp_path), node, _Buffer())
        assert len(node.publishers) == 1 and not node.destroyed_publishers
        assert node.goals == []
        node.fail_publisher_destroy = False
        await caught.value.cleanup()
        await caught.value.cleanup()
        assert node.destroyed_publishers == node.publishers

    asyncio.run(scenario())


@pytest.mark.usefixtures('ros_io')
def test_publisher_teardown_failure_is_retryable_after_other_resources_close(tmp_path):
    """Failure keeps the publisher owner but does not double-destroy prior resources."""
    async def scenario():
        runtime, node, _ = await _runtime(tmp_path)
        assert len(node.publishers) == 1
        node.fail_publisher_destroy = True
        with pytest.raises(RuntimeError, match='publisher'):
            await runtime.close()
        assert len(node.destroyed) == 3 and not node.destroyed_publishers
        node.fail_publisher_destroy = False
        await runtime.close()
        assert len(node.destroyed) == 3
        assert node.clients[0].destroy_count == 1
        assert node.destroyed_publishers == node.publishers

    asyncio.run(scenario())
