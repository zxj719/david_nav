"""Actual public task factory with Dock and ordinary POIs, using only fake ROS I/O."""

import asyncio
from dataclasses import replace
import math
import sys
import threading
from types import ModuleType, SimpleNamespace as NS

import pytest
import yaml

from nav_executor.protocol.task_adapter import TaskCommandAdapter
from nav_executor.app.config import load_configuration
from nav_tasks.api import (
    ConfigError, create_runtime, DockRuntimeConfig, EventKind, PoseRecord,
    PreemptPolicy, TaskRequest, TaskState,
)
from test_runtime_assembly import (  # noqa: F401
    _Buffer, _Client, _config, _mark, _navigate, _Node, ros_io)
from test_dock_configuration import _parameters, PROFILE


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, line):
        self.lines.append(('info', line))

    def warn(self, line):
        self.lines.append(('warn', line))

    def error(self, line):
        self.lines.append(('error', line))


class _DockNode(_Node):
    def __init__(self, outcomes=(4, 4)):
        super().__init__(outcomes)
        self.logger = _Log()

    def get_logger(self):
        return self.logger


class _DockBuffer(_Buffer):
    def __init__(self):
        super().__init__()
        self.calls = []

    def lookup_transform(self, *args, **kwargs):
        self.calls.append((args, kwargs, threading.get_ident()))
        return NS(header=self.header, transform=NS(
            translation=self.position, rotation=self.rotation))


@pytest.fixture
def dock_ros(request, monkeypatch):
    """Extend fake constructors for Spin, retained QoS and bounded TF durations."""
    request.getfixturevalue('ros_io')
    sys.modules['nav2_msgs.action'].Spin = NS(
        Goal=lambda: NS(target_yaw=0.0, time_allowance=NS(sec=0, nanosec=0)))
    qos = ModuleType('rclpy.qos')
    qos.DurabilityPolicy = NS(TRANSIENT_LOCAL='retained')
    qos.ReliabilityPolicy = NS(RELIABLE='reliable')
    qos.HistoryPolicy = NS(KEEP_LAST='last')
    qos.QoSProfile = lambda **values: NS(**values)
    duration = ModuleType('rclpy.duration')
    duration.Duration = lambda **values: NS(**values)
    monkeypatch.setitem(sys.modules, 'rclpy.qos', qos)
    monkeypatch.setitem(sys.modules, 'rclpy.duration', duration)

    class Client(_Client):
        def send_goal_async(self, goal, feedback_callback):
            self.feedback = feedback_callback
            return super().send_goal_async(goal, feedback_callback)

    monkeypatch.setattr(sys.modules['rclpy.action'], 'ActionClient', Client)


def _dock_config(tmp_path, **overrides):
    dock = DockRuntimeConfig(
        'navigation_center', 'world', '/dock_static', '/spin',
        ((-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)),
        0.5, 0.1, math.radians(25), 0, 120, 30, 0.05, 0.4, 8)
    return _config(tmp_path, dock=replace(dock, **overrides), dock_poi_ids=('p',))


def _map(frame='world', occupied=None):
    data = [0] * 36
    if occupied is not None:
        data[occupied[1] * 6 + occupied[0]] = 100
    return NS(header=NS(frame_id=frame), info=NS(width=6, height=6, resolution=1.0,
              origin=NS(position=NS(x=-1, y=-1), orientation=NS(x=0, y=0, z=0, w=1))),
              data=data)


async def _create(tmp_path, node=None, **overrides):
    node = node or _DockNode()
    buffer = _DockBuffer()
    runtime = await create_runtime(_dock_config(tmp_path, **overrides), node, buffer,
                                   now=lambda: 1000.1)
    node.subscriptions[0].callback(buffer.odom())
    return runtime, node, buffer


async def _bounded(awaitable):
    return await asyncio.wait_for(awaitable, 3)


def test_dock_yaml_reaches_public_factory_without_rewriting_policy(dock_ros, tmp_path):
    """The actual loader/factory/worker chain consumes the existing ROS parameter file."""
    path = tmp_path / 'dock.yaml'
    parameters = _parameters()
    parameters['static_map_topic'] = '/dock_static'
    path.write_text(yaml.safe_dump({'dock_navigator': {'ros__parameters': parameters}}))
    resolved = load_configuration([
        '--dock-config-file', str(path), '--dock-poi-id', 'p', '--poi-store-file',
        str(tmp_path / 'pois.yaml')], profile_path=PROFILE, environ={})

    async def scenario():
        node, buffer = _DockNode(), _DockBuffer()
        runtime = await create_runtime(resolved.task_config(), node, buffer, now=lambda: 1000.1)
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        try:
            node.subscriptions[0].callback(buffer.odom())
            node.subscriptions[-1].callback(_map())
            await _mark(adapter)
            await _navigate(adapter)
            assert await _bounded(adapter.next_event()) == {
                'event_type': 'on_arrived', 'request_id': 'r', 'sub_id': 1}
            assert [goal.pose.pose.position.x for goal in node.goals] == [0.5, 1.0]
            assert all(goal.pose.header.frame_id == 'world' for goal in node.goals)
        finally:
            adapter.close()
            await _bounded(runtime.close())
        assert len(node.destroyed) == 4

    asyncio.run(scenario())


def test_public_factory_routes_dock_poi_through_two_stages_and_full_map(dock_ros, tmp_path):
    """Configured Dock uses the real worker and one shared navigation/Spin backend."""
    async def scenario():
        owner_thread = threading.get_ident()
        runtime, node, buffer = await _create(tmp_path)
        assert runtime.tasks.status().state == TaskState.IDLE
        assert len(node.clients) == 2 and len(node.subscriptions) == 4
        static = node.subscriptions[-1]
        assert static.topic == '/dock_static'
        assert vars(static.qos) == dict(history='last', depth=1,
                                        reliability='reliable', durability='retained')
        static.callback(_map())
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        assert await _bounded(adapter.next_event()) == {
            'event_type': 'on_arrived', 'request_id': 'r', 'sub_id': 1}
        assert [goal.pose.pose.position.x for goal in node.goals] == [0.5, 1]
        assert all(type(goal.pose.pose.position.x) is float for goal in node.goals)
        assert all(goal.pose.header.frame_id == 'world' for goal in node.goals)
        dock_reads = [call for call in buffer.calls if call[0][1] == 'navigation_center']
        assert dock_reads and all(call[2] != owner_thread for call in dock_reads)
        assert all(call[1]['timeout'].seconds == 2 for call in dock_reads)
        policy = runtime._dock_worker._config
        assert policy.settle_spec.max_rate_rad_s == 0.05
        assert policy.settle_spec.window_sec == 0.4
        assert policy.tolerance.yaw_ok == math.radians(25)
        assert policy.tolerance.max_yaw_retries == 0
        assert any(call[0][1] == 'chassis' for call in buffer.calls)
        await runtime.close()
        adapter.close()
        assert all(client.destroy_count == 1 for client in node.clients)
        assert len(node.destroyed) == 4

    asyncio.run(scenario())


@pytest.mark.parametrize('case', ['missing', 'wrong_frame', 'occupied', 'invalid_grid'])
def test_dock_guard_rejects_before_any_action_or_retry(dock_ros, tmp_path, case):
    """The actual configured factory does not turn a failed footprint guard into snapping."""
    async def scenario():
        runtime, node, _ = await _create(tmp_path)
        if case != 'missing':
            message = _map('other' if case == 'wrong_frame' else 'world',
                           (1, 3) if case == 'occupied' else None)
            if case == 'invalid_grid':
                message.data = []
            node.subscriptions[-1].callback(message)
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        event = await _bounded(adapter.next_event())
        assert event['event_type'] == 'on_error'
        assert event['error_message'].startswith(
            '导航失败(共尝试1次): Nav2 终态 status=6(ABORTED), target=p(现场) ')
        assert node.goals == []
        await runtime.close()
        adapter.close()

    asyncio.run(scenario())


def test_ordinary_poi_keeps_retry_policy_when_dock_is_enabled(dock_ros, tmp_path):
    """Only configured Dock IDs bypass snap/retry; ordinary navigation is unchanged."""
    async def scenario():
        node = _DockNode((6, 4))
        buffer = _DockBuffer()
        config = replace(_dock_config(tmp_path), nav_retry_delay_sec=0)
        runtime = await create_runtime(config, node, buffer, now=lambda: 1000.1)
        node.subscriptions[0].callback(buffer.odom())
        # No Dock map has arrived: ordinary POI missing-map behavior is still permissive.
        await runtime.pois.mark('normal', '普通')
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await adapter.handle_message({'action': 'navigate_to', 'request_id': 'n',
                                      'sub_id': 0, 'target_id': 'normal'})
        assert (await _bounded(adapter.next_event()))['event_type'] == 'on_arrived'
        assert [goal.pose.pose.position.x for goal in node.goals] == [1, 1]
        assert not node.logger.lines
        await runtime.close()
        adapter.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('outcomes,event,count', [
    ((6,), 'on_error', 1), ((4, 6), 'on_arrived', 2)])
def test_stage_failures_keep_no_retry_and_measured_stage_two_policy(
    dock_ros, tmp_path, outcomes, event, count,
):
    """Stage one fails once; stage two ABORTED can still pass the existing measured check."""
    async def scenario():
        runtime, node, _ = await _create(tmp_path, _DockNode(outcomes))
        node.subscriptions[-1].callback(_map())
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        assert (await _bounded(adapter.next_event()))['event_type'] == event
        assert len(node.goals) == count
        await runtime.close()
        adapter.close()

    asyncio.run(scenario())


def test_direct_dock_and_poi_share_one_manager_and_preemption(dock_ros, tmp_path):
    """Direct pose tasks can preempt a POI only after its existing action is canceled."""
    async def scenario():
        runtime, node, _ = await _create(tmp_path, _DockNode((None, 4, 4)))
        node.subscriptions[-1].callback(_map())
        await runtime.pois.mark('normal', '普通')
        events = runtime.tasks.subscribe_events()
        await runtime.tasks.submit(TaskRequest('old', 'navigate_to', {'target_id': 'normal'}))
        await _bounded(node.started.get())
        await runtime.tasks.submit(TaskRequest(
            'direct', 'dock', {'pose': PoseRecord('ignored_header', 1, 2, 99, 0, 0)},
            preempt=PreemptPolicy.ALLOW))
        event = await _bounded(events.get())
        assert event.kind == EventKind.SUCCEEDED and event.task_id == 'direct'
        assert len(node.goals) == 3
        assert node.events[0] == 'cancel'
        assert [goal.pose.header.frame_id for goal in node.goals] == ['world'] * 3
        assert [goal.pose.pose.position.z for goal in node.goals] == [0, 0, 0]
        assert events.empty()
        await runtime.close()

    asyncio.run(scenario())


def test_full_feedback_and_phase_offset_survive_factory_routing(dock_ros, tmp_path):
    """Public raw feedback retains every field while task progress gets adjusted distance."""
    async def scenario():
        node, buffer = _DockNode((None, 4)), _DockBuffer()
        raw_events = asyncio.Queue()
        runtime = await create_runtime(
            _dock_config(tmp_path), node, buffer, now=lambda: 1000.1,
            on_dock_feedback=lambda goal, raw, extra: raw_events.put_nowait(
                (goal, raw, extra, threading.get_ident())))
        node.subscriptions[0].callback(buffer.odom())
        node.subscriptions[-1].callback(_map())
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        await _mark(adapter)
        await _navigate(adapter)
        handle = await _bounded(node.started.get())
        raw = NS(feedback=NS(
            distance_remaining=2.25, current_pose=object(), navigation_time=object(),
            estimated_time_remaining=object(), number_of_recoveries=3))
        node.clients[0].feedback(raw)
        goal, received, extra, thread = await _bounded(raw_events.get())
        assert received is raw and extra == 0.5
        assert (goal.request_id, goal.sub_id, goal.generation) == ('r', 1, 1)
        assert thread == threading.get_ident()
        event = await _bounded(adapter.next_event())
        assert event['event_type'] == 'on_progress' and event['remaining_distance'] == 2.75
        handle.result.set_result(NS(status=4))
        assert (await _bounded(adapter.next_event()))['event_type'] == 'on_arrived'
        await runtime.close()
        adapter.close()

    asyncio.run(scenario())


def test_failed_dock_close_keeps_clients_maps_and_worker_for_retry(dock_ros, tmp_path):
    """Timeout is not successful runtime shutdown; live resources stay until terminal proof."""
    async def scenario():
        node, buffer = _DockNode((None,)), _DockBuffer()
        node.complete_on_cancel = False
        runtime = await create_runtime(
            replace(_dock_config(tmp_path), cancel_timeout_sec=0.01), node, buffer,
            now=lambda: 1000.1)
        node.subscriptions[0].callback(buffer.odom())
        node.subscriptions[-1].callback(_map())
        await runtime.tasks.submit(TaskRequest(
            'direct', 'dock', {'pose': PoseRecord('world', 1, 2, 0, 0, 0)}))
        handle = await _bounded(node.started.get())
        with pytest.raises(asyncio.TimeoutError):
            await _bounded(runtime.close())
        assert not node.destroyed and all(client.destroy_count == 0 for client in node.clients)
        assert runtime.tasks.status().state == TaskState.ERROR
        handle.result.set_result(NS(status=5))
        await _bounded(runtime.close())
        assert runtime.tasks.status().state == TaskState.CLOSED
        assert len(node.destroyed) == 4
        assert all(client.destroy_count == 1 for client in node.clients)

    asyncio.run(scenario())


def test_failed_dock_map_construction_rolls_back_both_clients(dock_ros, tmp_path):
    """Partial Dock assembly never exposes tasks or leaks previously created waitables."""
    async def scenario():
        node = _DockNode()
        node.fail_subscription_at = 4
        with pytest.raises(RuntimeError, match='subscription construction'):
            await create_runtime(_dock_config(tmp_path), node, _DockBuffer())
        assert len(node.clients) == 2
        assert all(client.destroy_count == 1 for client in node.clients)
        assert len(node.destroyed) == 3 and not node.goals

    asyncio.run(scenario())


@pytest.mark.parametrize('field,value', [
    ('robot_base_frame', ''), ('global_frame', ''), ('static_map_topic', False),
    ('spin_action_name', ''), ('footprint', ((0, 0), (1, 1), (2, 2))),
    ('footprint', [[0, 0], [1, 0], [0, 1]]), ('approach_dist', -1),
    ('xy_ok', float('nan')), ('yaw_ok', float('inf')), ('max_yaw_retries', True),
    ('nav_timeout_sec', -1), ('spin_timeout_sec', -1), ('settle_max_rate_rad_s', -1),
    ('settle_window_sec', -1), ('settle_timeout_sec', -1),
    ('static_occupied_threshold', 101), ('pose_timeout_sec', -1),
])
def test_invalid_dock_config_fails_before_storage_or_ros(dock_ros, tmp_path, field, value):
    """Bad explicit policy cannot create clients/subscriptions or admit ordinary fallback."""
    async def scenario():
        node = _DockNode()
        with pytest.raises(ConfigError, match='dock.'):
            await create_runtime(_dock_config(tmp_path, **{field: value}), node, _DockBuffer())
        assert not node.clients and not node.subscriptions
        assert not (tmp_path / 'store.yaml').exists()

    asyncio.run(scenario())
