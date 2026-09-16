"""Public runtime factory and resource lifetimes with fake ROS I/O only."""

import asyncio
from concurrent.futures import Future
from dataclasses import replace
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from nav_executor.protocol.task_adapter import TaskCommandAdapter
from nav_tasks.api import (
    ClosedError, ConfigError, create_runtime, RuntimeAssemblyError, RuntimeConfig, TaskState,
)


def _done(value):
    result = Future()
    result.set_result(value)
    return result


def _goal():
    return NS(pose=NS(header=NS(frame_id='', stamp=None), pose=NS(
        position=NS(x=0, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1))))


class _Handle:
    def __init__(self, node, status):
        self.accepted = True
        self.node = node
        self.result = Future() if status is None else _done(NS(status=status))

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.node.events.append('cancel')
        self.node.canceled.set()
        if self.node.complete_on_cancel and not self.result.done():
            self.result.set_result(NS(status=5))
        return _done(None)


class _Client:
    def __init__(self, node, action_type, name):
        if node.fail_client:
            raise RuntimeError('client construction failed')
        self.node = node
        self.name = name
        self.destroy_count = 0
        node.clients.append(self)

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback):
        self.node.goals.append(goal)
        handle = _Handle(self.node, self.node.outcomes.pop(0))
        self.node.started.put_nowait(handle)
        return _done(handle)

    def destroy(self):
        if self.node.fail_client_destroy:
            raise RuntimeError('client destroy failed')
        self.destroy_count += 1
        self.node.events.append('client destroyed')


class _Node:
    def __init__(self, outcomes=(4,)):
        self.outcomes = list(outcomes)
        self.subscriptions = []
        self.destroyed = []
        self.clients = []
        self.goals = []
        self.events = []
        self.started = asyncio.Queue()
        self.canceled = asyncio.Event()
        self.complete_on_cancel = True
        self.fail_subscription_at = None
        self.fail_client = False
        self.fail_client_destroy = False
        self.fail_subscription_destroy = False

    def create_subscription(self, message_type, topic, callback, qos):
        if len(self.subscriptions) + 1 == self.fail_subscription_at:
            raise RuntimeError('subscription construction failed')
        subscription = NS(topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        if self.fail_subscription_destroy:
            return False
        self.destroyed.append(subscription)
        self.events.append('subscription destroyed')
        return True

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: 1000))


class _Buffer:
    def __init__(self):
        self.header = NS(frame_id='', stamp=NS(sec=1000, nanosec=0))
        self.position = NS(x=1, y=2, z=0)
        self.rotation = NS(x=0, y=0, z=0, w=1)
        self.seen = asyncio.Event()

    def lookup_transform(self, *args):
        self.seen.set()
        return NS(header=self.header, transform=NS(
            translation=self.position, rotation=self.rotation))

    def odom(self):
        return NS(header=self.header, pose=NS(pose=NS(
            position=self.position, orientation=self.rotation)))


@pytest.fixture
def ros_io(monkeypatch):
    """Install fake ROS constructors without replacing runtime/port implementations."""
    messages = ModuleType('nav_msgs.msg')
    messages.Odometry = object
    messages.OccupancyGrid = object
    nav_messages = ModuleType('nav_msgs')
    nav_messages.msg = messages
    ros_time = ModuleType('rclpy.time')
    ros_time.Time = lambda: 0
    action = ModuleType('rclpy.action')
    action.ActionClient = _Client
    ros = ModuleType('rclpy')
    ros.time, ros.action = ros_time, action
    actions = ModuleType('nav2_msgs.action')
    actions.NavigateToPose = NS(Goal=_goal)
    nav_actions = ModuleType('nav2_msgs')
    nav_actions.action = actions
    for key, module in [('nav_msgs', nav_messages), ('nav_msgs.msg', messages),
                        ('rclpy', ros), ('rclpy.time', ros_time), ('rclpy.action', action),
                        ('nav2_msgs', nav_actions), ('nav2_msgs.action', actions)]:
        monkeypatch.setitem(sys.modules, key, module)


def _config(tmp_path, **overrides):
    config = RuntimeConfig(
        tmp_path / 'store.yaml', 'chassis', '/odom_in', 'world', 'local',
        '/static', '/cost', '/navigate', 'session')
    return replace(config, **overrides)


async def _runtime(tmp_path, node=None, **overrides):
    node = node or _Node()
    buffer = _Buffer()
    runtime = await create_runtime(
        _config(tmp_path, **overrides), node, buffer, now=lambda: 1000.1)
    node.subscriptions[0].callback(buffer.odom())
    return runtime, node, buffer


async def _mark(adapter):
    frames = await adapter.handle_message({
        'action': 'mark_current_poi', 'request_id': 'mark', 'sub_id': 0,
        'poi': {'id': 'p', 'name': '现场'},
    })
    assert frames[-1]['event_type'] == 'on_mark_poi_success'


async def _navigate(adapter):
    await adapter.handle_message({
        'action': 'navigate_to', 'request_id': 'r', 'sub_id': 1, 'target_id': 'p'})


def test_factory_assembles_public_services_without_starting_background_or_motion(ros_io, tmp_path):
    """An actual public factory, not a handcrafted test registry, runs the task chain."""
    async def scenario():
        before = asyncio.all_tasks()
        runtime, node, _ = await _runtime(tmp_path)
        assert asyncio.all_tasks() == before
        assert node.goals == []
        assert len(node.clients) == 1
        assert len(node.subscriptions) == 3
        assert runtime.tasks.status().state == TaskState.IDLE
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        assert await adapter.next_event() == {'event_type': 'on_arrived', 'request_id': 'r',
                                              'sub_id': 1}
        assert node.goals[0].pose.header.frame_id == 'world'
        assert node.goals[0].pose.pose.position.x == 1
        await runtime.close()
        await runtime.close()
        adapter.close()
        assert runtime.tasks.status().state == TaskState.CLOSED
        assert node.clients[0].destroy_count == 1
        assert len(node.destroyed) == 3
        assert node.events == ['client destroyed'] + ['subscription destroyed'] * 3

    asyncio.run(scenario())


def test_factory_imports_manual_source_and_preserves_startup_logs(ros_io, tmp_path):
    """Manual JSON is read-only, loaded before ingress and uses the real repository."""
    source = tmp_path / 'named.json'
    source.write_text('[{"id": "p", "name": "Imported", "position": {"x": 8}}]')
    original = source.read_bytes()

    async def scenario():
        logs = []
        node = _Node()
        runtime = await create_runtime(_config(tmp_path, pose_file=source), node, _Buffer(),
                                       log=logs.append, now=lambda: 1000.1)
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _navigate(adapter)
        assert (await adapter.next_event())['event_type'] == 'on_arrived'
        assert node.goals[0].pose.pose.position.x == 8
        assert logs == [f'loaded 1 POIs from {source}']
        assert source.read_bytes() == original
        await runtime.close()
        adapter.close()

    asyncio.run(scenario())


def test_missing_manual_source_logs_and_does_not_create_it(ros_io, tmp_path):
    """A missing optional import remains a readable startup condition, not a write."""
    async def scenario():
        source = tmp_path / 'missing.json'
        logs = []
        runtime = await create_runtime(_config(tmp_path, pose_file=source), _Node(), _Buffer(),
                                       log=logs.append)
        assert logs == [f'pose file not found: {source}']
        assert not source.exists()
        await runtime.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('failure', ['second_grid', 'client', 'skill'])
def test_partial_assembly_rolls_back_all_completed_resources(ros_io, tmp_path, failure):
    """No task is exposed before successful construction and reverse cleanup completes."""
    async def scenario():
        node = _Node()
        overrides = {}
        if failure == 'second_grid':
            node.fail_subscription_at = 3
        elif failure == 'client':
            node.fail_client = True
        else:
            overrides['progress_min_interval_sec'] = float('nan')
        with pytest.raises((RuntimeError, ConfigError)):
            await create_runtime(_config(tmp_path, **overrides), node, _Buffer())
        assert len(node.destroyed) == len(node.subscriptions)
        assert node.goals == []
        if node.clients:
            assert node.clients[0].destroy_count == 1
            assert node.events[0] == 'client destroyed'

    asyncio.run(scenario())


def test_dock_mode_refused_before_io_instead_of_silently_using_normal_navigation(ros_io, tmp_path):
    """The intermediate assembly cannot bypass required Dock semantics."""
    async def scenario():
        node = _Node()
        with pytest.raises(ConfigError, match='Dock runtime assembly'):
            await create_runtime(_config(tmp_path, dock_poi_ids=('dock',)), node, _Buffer())
        assert node.subscriptions == node.clients == node.goals == []
        assert not (tmp_path / 'store.yaml').exists()

    asyncio.run(scenario())


def test_close_waiter_cancellation_does_not_drop_owned_goal_cleanup(ros_io, tmp_path):
    """Client/subscriptions stay alive until the pending action reports a terminal state."""
    async def scenario():
        node = _Node([None])
        node.complete_on_cancel = False
        runtime, _, _ = await _runtime(tmp_path, node)
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        handle = await node.started.get()
        waiter = asyncio.create_task(runtime.close())
        await node.canceled.wait()
        assert node.destroyed == []
        assert node.clients[0].destroy_count == 0
        waiter.cancel()
        with pytest.raises(asyncio.CancelledError):
            await waiter
        handle.result.set_result(NS(status=5))
        await runtime.close()
        adapter.close()
        assert node.events.index('cancel') < node.events.index('client destroyed')
        assert len(node.destroyed) == 3

    asyncio.run(scenario())


def test_timed_out_motion_close_retains_resources_for_retry(ros_io, tmp_path):
    """A cleanup timeout is not a license to destroy ROS resources or abandon ownership."""
    async def scenario():
        node = _Node([None])
        node.complete_on_cancel = False
        runtime, _, _ = await _runtime(tmp_path, node, cancel_timeout_sec=0.001)
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        handle = await node.started.get()
        with pytest.raises(asyncio.TimeoutError):
            await runtime.close()
        assert node.destroyed == []
        assert node.clients[0].destroy_count == 0
        with pytest.raises(ClosedError):
            runtime.start_loop_closure()
        handle.result.set_result(NS(status=5))
        await runtime.close()
        adapter.close()
        assert len(node.destroyed) == 3

    asyncio.run(scenario())


@pytest.mark.parametrize('failure', ['client', 'subscription'])
def test_resource_close_failure_can_retry_without_double_destroy(ros_io, tmp_path, failure):
    """Completed teardown stages remain completed on a later close attempt."""
    async def scenario():
        runtime, node, _ = await _runtime(tmp_path)
        if failure == 'client':
            node.fail_client_destroy = True
        else:
            node.fail_subscription_destroy = True
        with pytest.raises(RuntimeError, match='destroy'):
            await runtime.close()
        assert node.destroyed == []
        node.fail_client_destroy = node.fail_subscription_destroy = False
        await runtime.close()
        assert node.clients[0].destroy_count == 1
        assert len(node.destroyed) == 3

    asyncio.run(scenario())


def test_observer_can_stop_and_restart_without_owning_motion(ros_io, tmp_path):
    """One watcher per runtime; disconnect policy can stop it without closing tasks."""
    async def scenario():
        runtime, node, buffer = await _runtime(tmp_path, loop_closure_poll_sec=0.001)
        runtime.start_loop_closure()
        first = runtime._watcher
        runtime.start_loop_closure()
        assert runtime._watcher is first
        await asyncio.wait_for(buffer.seen.wait(), 1)
        await runtime.stop_loop_closure()
        assert first.done()
        assert runtime.tasks.status().state == TaskState.IDLE
        runtime.start_loop_closure()
        second = runtime._watcher
        assert second is not first
        await runtime.close()
        assert second.done()
        assert node.goals == []

    asyncio.run(scenario())


def test_factory_configures_retry_audit_and_budget(ros_io, tmp_path, capsys):
    """Real factory wiring invokes the frozen observer and only retries as configured."""
    async def scenario():
        runtime, node, _ = await _runtime(tmp_path, _Node([6, 4]), nav_retry_delay_sec=0)
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        assert (await adapter.next_event())['event_type'] == 'on_arrived'
        assert len(node.goals) == 2
        await runtime.close()
        adapter.close()

    asyncio.run(scenario())
    output = capsys.readouterr().out
    assert '[nav2_action_bridge] NAV FAILURE attempt 1/2 request_id=r sub_id=1:' in output
    assert 're-publishing goal in 0.0s (attempt 2/2)' in output


def test_failed_rollback_retains_cleanup_handle_for_retry(ros_io, tmp_path):
    """A construction error followed by destruction failure must not lose its owner."""
    async def scenario():
        node = _Node()
        node.fail_client_destroy = True
        with pytest.raises(RuntimeAssemblyError) as caught:
            await create_runtime(_config(tmp_path, progress_min_interval_sec=float('nan')),
                                 node, _Buffer())
        assert len(node.destroyed) == 3
        assert node.clients[0].destroy_count == 0
        assert callable(getattr(caught.value, 'cleanup', None))
        assert isinstance(caught.value.__cause__, ConfigError)
        with pytest.raises(RuntimeError, match='client destroy failed'):
            await caught.value.cleanup()
        node.fail_client_destroy = False
        await caught.value.cleanup()
        await caught.value.cleanup()
        assert node.clients[0].destroy_count == 1
        assert node.goals == []

    asyncio.run(scenario())


def test_crashed_observer_does_not_prevent_motion_resource_cleanup(ros_io, tmp_path, monkeypatch):
    """Observer errors are reported by supervision but cannot block final teardown."""
    async def scenario():
        failed = asyncio.Event()

        async def broken(*args, **kwargs):
            failed.set()
            raise OSError('watcher failure')

        monkeypatch.setattr('nav_tasks.app.runtime.watch_loop_closure', broken)
        runtime, node, _ = await _runtime(tmp_path)
        runtime.start_loop_closure()
        await failed.wait()
        await runtime.close()
        assert node.clients[0].destroy_count == 1
        assert len(node.destroyed) == 3

    asyncio.run(scenario())


def test_explicit_loop_closure_thresholds_reach_observer(ros_io, tmp_path, monkeypatch):
    """The public factory passes threshold overrides through to its supervised watcher."""
    async def scenario():
        started = asyncio.Event()
        received = []

        async def observer(*args, **kwargs):
            received.append((args[2], kwargs))
            started.set()
            await asyncio.Event().wait()

        monkeypatch.setattr('nav_tasks.app.runtime.watch_loop_closure', observer)
        runtime, _, _ = await _runtime(
            tmp_path, loop_closure_poll_sec=2.0,
            loop_closure_translation_m=0.08, loop_closure_yaw_rad=0.06)
        runtime.start_loop_closure()
        await asyncio.wait_for(started.wait(), 1)
        await runtime.close()
        assert received == [(2.0, {'translation_threshold_m': 0.08, 'yaw_threshold_rad': 0.06})]

    asyncio.run(scenario())


def test_partial_grid_construction_retains_failed_subscription_cleanup(ros_io, tmp_path):
    """Nested adapter rollback failures retain every resource, including prior adapters."""
    async def scenario():
        node = _Node()
        node.fail_subscription_at = 3
        node.fail_subscription_destroy = True
        with pytest.raises(RuntimeAssemblyError) as caught:
            await create_runtime(_config(tmp_path), node, _Buffer())
        assert len(node.subscriptions) == 2
        assert not node.destroyed
        node.fail_subscription_destroy = False
        await caught.value.cleanup()
        await caught.value.cleanup()
        assert len(node.destroyed) == 2
        assert not node.clients

    asyncio.run(scenario())
