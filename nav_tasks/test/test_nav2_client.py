"""Single-attempt backend with fake node, messages and action futures only."""

import asyncio
from concurrent.futures import Future
import sys
from types import ModuleType, SimpleNamespace

import pytest

from nav_tasks.domain.tasks import NavGoal, NavStatus
from nav_tasks.domain.types import PoseRecord, StoredPoi
from nav_tasks.nav2_client import Nav2Client


def _done(value):
    future = Future()
    future.set_result(value)
    return future


def _message():
    return SimpleNamespace(pose=SimpleNamespace(
        header=SimpleNamespace(frame_id='', stamp=None),
        pose=SimpleNamespace(position=SimpleNamespace(x=0, y=0, z=0),
                             orientation=SimpleNamespace(x=0, y=0, z=0, w=1))))


def _goal():
    pose = PoseRecord('world', 1, 2, 0.3, 0.5, 100)
    return NavGoal(StoredPoi('POI_1', '前台', pose, pose, 'session'), 'req', 2)


class _Clock:
    def __init__(self):
        self.value = 10.0
        self.on_sleep = lambda: None

    def now(self):
        return self.value

    async def sleep(self, seconds):
        self.value += seconds
        self.on_sleep()
        await asyncio.sleep(0)


class _Client:
    def __init__(self, status=4, accepted=True):
        self.result = _done(SimpleNamespace(status=status))
        self.handle = SimpleNamespace(
            accepted=accepted, get_result_async=lambda: self.result,
            cancel_goal_async=lambda: _done(None))
        self.ready = True
        self.goals = []
        self.feedback = None
        self.sent = asyncio.Event()

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback):
        self.goals.append(goal)
        self.feedback = feedback_callback
        self.sent.set()
        return _done(self.handle)


def _backend(client, clock=None):
    clock = clock or _Clock()
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: clock.value)))
    return Nav2Client(node, 'fixture_navigate', 0.2, client=client,
                      goal_factory=_message, clock=clock)


@pytest.mark.parametrize('status,expected', [(4, NavStatus.SUCCEEDED), (5, NavStatus.CANCELED)])
def test_single_attempt_terminal_and_exact_goal_fields(status, expected):
    """No backend retry occurs and goal fields retain the legacy conversion."""
    async def scenario():
        client = _Client(status=status)
        backend = _backend(client)
        result = await backend.navigate(_goal(), lambda distance: None)
        assert result.status == expected
        assert len(client.goals) == 1
        message = client.goals[0]
        assert message.pose.header.frame_id == 'world'
        assert message.pose.header.stamp == 10
        assert vars(message.pose.pose.position) == {'x': 1, 'y': 2, 'z': 0.3}
        assert vars(message.pose.pose.orientation) == _goal().poi.map_pose.orientation()
        await backend.close()

    asyncio.run(scenario())


def test_failure_text_matches_legacy_without_core_retry_prefix():
    """Single-attempt failure detail preserves punctuation, Chinese and precision."""
    async def scenario():
        client = _Client(status=6)
        backend = _backend(client)
        outcome = await backend.navigate(_goal(), lambda distance: None)
        assert outcome.status == NavStatus.FAILED
        assert outcome.detail == (
            'Nav2 终态 status=6(ABORTED), target=POI_1(前台) '
            'goal=world(x=1.00, y=2.00, yaw=0.50rad), '
            '剩余距离=n/a(无 feedback,可能 planner 直接失败), 本次耗时=0.0s')
        assert len(client.goals) == 1
        await backend.close()

    asyncio.run(scenario())


def test_rejected_and_unavailable_text():
    """No implicit retries or changed error text when admission fails."""
    async def scenario():
        client = _Client(accepted=False)
        backend = _backend(client)
        rejected = await backend.navigate(_goal(), lambda distance: None)
        assert rejected.detail == (
            'Nav2 拒绝 goal(bt_navigator rejected;常见于 TF 未就绪/上一个 '
            'goal 未清), target=POI_1(前台) goal=world(x=1.00, y=2.00, yaw=0.50rad)')
        client.ready = False
        unavailable = await backend.navigate(_goal(), lambda distance: None)
        assert unavailable.detail == (
            "Nav2 action server 'fixture_navigate' 在 "
            '0.2s 内不可用(未起来或 lifecycle 未 Activate), '
            'target=POI_1(前台) goal=world(x=1.00, y=2.00, yaw=0.50rad)')
        assert len(client.goals) == 1
        await backend.close()

    asyncio.run(scenario())


def test_feedback_detail_uses_unthrottled_latest_distance():
    """The skill can throttle reporting without losing backend failure context."""
    async def scenario():
        client = _Client()
        client.result = Future()
        clock = _Clock()
        backend = _backend(client, clock)
        received = asyncio.Queue()
        task = asyncio.create_task(backend.navigate(_goal(), received.put_nowait))
        await client.sent.wait()
        client.feedback(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=2.345)))
        assert await received.get() == 2.345
        clock.value += 1.2
        client.result.set_result(SimpleNamespace(status=6))
        outcome = await task
        assert '剩余距离=2.35m, 本次耗时=1.2s' in outcome.detail
        await backend.close()

    asyncio.run(scenario())


def test_goal_stamp_is_created_after_server_readiness_wait_and_each_attempt():
    """Waiting for a server must not produce a stale stamped goal."""
    async def scenario():
        client = _Client()
        client.ready = False
        clock = _Clock()
        clock.on_sleep = lambda: setattr(client, 'ready', True)
        backend = _backend(client, clock)
        await backend.navigate(_goal(), lambda distance: None)
        assert client.goals[0].pose.header.stamp == clock.value
        clock.value += 1
        await backend.navigate(_goal(), lambda distance: None)
        assert client.goals[1].pose.header.stamp == clock.value
        assert client.goals[0] is not client.goals[1]
        await backend.close()

    asyncio.run(scenario())


def test_real_core_skill_backend_chain_cancels_then_accepts_next_task():
    """Exercise the full new motion chain using only fake ROS action machinery."""
    from nav_tasks.api import CommandRegistry, EventKind, TaskManager, TaskRequest, TaskState
    from nav_tasks.core.goal_resolution import DirectGoalResolver
    from nav_tasks.core.handlers.navigate_to import NavigateToPoi

    async def scenario():
        client = _Client()
        client.result = Future()
        canceled = []

        def cancel_goal():
            canceled.append(True)
            client.result.set_result(SimpleNamespace(status=5))
            return _done(None)

        client.handle.cancel_goal_async = cancel_goal
        clock = _Clock()
        backend = _backend(client, clock)
        skill = NavigateToPoi(
            backend, SimpleNamespace(get=lambda key: _goal().poi), DirectGoalResolver(),
            clock=clock)
        registry = CommandRegistry()
        registry.register('navigate_to', skill.decode, skill.run)
        manager = TaskManager(registry, backend.cancel)
        events = manager.subscribe_events()
        await manager.submit(TaskRequest('one', 'navigate_to', {'target_id': 'POI_1'}))
        await client.sent.wait()
        client.feedback(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=2.5)))
        assert (await events.get()).remaining_distance == 2.5
        await manager.cancel('one')
        event = await events.get()
        assert (event.status, event.remaining_distance) == ('paused', 2.5)
        assert events.empty()
        assert canceled == [True]
        assert manager.status().state == TaskState.IDLE
        client.result = _done(SimpleNamespace(status=4))
        await manager.submit(TaskRequest('two', 'navigate_to', {'target_id': 'POI_1'}))
        event = await events.get()
        assert (event.task_id, event.kind) == ('two', EventKind.SUCCEEDED)
        assert len(client.goals) == 2
        await manager.close()
        await backend.close()

    asyncio.run(scenario())


def test_owned_client_is_not_destroyed_until_cleanup_finishes(monkeypatch):
    """Exercise the constructor-owned client branch with fake imported ROS factories."""
    async def scenario():
        client = _Client()
        client.result = Future()
        destroyed = []
        admitted = asyncio.Event()

        def get_result():
            admitted.set()
            return client.result

        client.handle.get_result_async = get_result
        client.destroy = lambda: destroyed.append(True)
        ros = ModuleType('rclpy')
        actions = ModuleType('rclpy.action')
        actions.ActionClient = lambda node, action_type, name: client
        ros.action = actions
        messages = ModuleType('nav2_msgs')
        nav_actions = ModuleType('nav2_msgs.action')
        nav_actions.NavigateToPose = SimpleNamespace(Goal=_message)
        messages.action = nav_actions
        for name, module in (
                ('rclpy', ros), ('rclpy.action', actions),
                ('nav2_msgs', messages), ('nav2_msgs.action', nav_actions)):
            monkeypatch.setitem(sys.modules, name, module)
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: 10)))
        backend = Nav2Client(node, 'fixture_navigate', 0, cancel_timeout_sec=0.01)
        navigation = asyncio.create_task(backend.navigate(_goal(), lambda distance: None))
        await client.sent.wait()
        await admitted.wait()
        with pytest.raises(asyncio.TimeoutError):
            await backend.close()
        assert destroyed == []
        assert backend._session.busy
        client.result.set_result(SimpleNamespace(status=5))
        assert (await navigation).status == NavStatus.CANCELED
        await backend.close()
        assert destroyed == [True]
        await backend.close()
        assert destroyed == [True]

    asyncio.run(scenario())
