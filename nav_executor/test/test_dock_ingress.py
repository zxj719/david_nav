"""Real manager/Dock worker and ROS ingress callbacks with in-memory ROS I/O only."""

import asyncio
from dataclasses import replace
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from nav_tasks.api import (
    ConfigError, create_dock_ingress, create_runtime, DockGoal, DockNodeConfig, PoseRecord,
    PreemptPolicy,
    TaskRequest, TaskState,
)
from nav_executor.protocol.task_adapter import TaskCommandAdapter
from test_dock_runtime import (  # noqa: F401
    _DockBuffer, _dock_config, _DockNode, _map, _mark, _navigate, dock_ros, ros_io,
)


class _Handle:
    def __init__(self, pose=None):
        self.request = NS(pose=pose or _pose())
        self.is_cancel_requested = False
        self.is_active = True
        self.terminal = []
        self.feedback = []
        self.fail_feedback = False

    def succeed(self):
        self.terminal.append('succeeded')
        self.is_active = False

    def abort(self):
        self.terminal.append('aborted')
        self.is_active = False

    def canceled(self):
        self.terminal.append('canceled')
        self.is_active = False

    def publish_feedback(self, feedback):
        if self.fail_feedback:
            raise RuntimeError('feedback failed')
        self.feedback.append(feedback)


class _Server:
    def __init__(self, node, action, name, execute, **callbacks):
        if node.fail_server:
            raise RuntimeError('server construction failed')
        self.name, self.execute = name, execute
        self.callbacks = callbacks
        self.destroy_count = 0
        self.fail_destroy = False
        node.servers.append(self)

    def destroy(self):
        if self.fail_destroy:
            raise RuntimeError('server destruction failed')
        self.destroy_count += 1


class _Node(_DockNode):
    def __init__(self, outcomes=(4, 4)):
        super().__init__(outcomes)
        self.servers = []
        self.fail_server = False

    def create_subscription(self, *args, **kwargs):
        subscription = super().create_subscription(*args)
        subscription.group = kwargs.get('callback_group')
        return subscription


def _pose():
    return NS(header=NS(frame_id='ignored_header'), pose=NS(
        position=NS(x=1.0, y=2.0, z=9.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))


@pytest.fixture
def ingress_ros(request, monkeypatch):
    """Replace every ROS constructor with an in-memory fake; no context can be initialized."""
    request.getfixturevalue('dock_ros')
    action = sys.modules['rclpy.action']
    action.ActionServer = _Server
    action.GoalResponse = NS(ACCEPT='accept', REJECT='reject')
    action.CancelResponse = NS(ACCEPT='cancel', REJECT='reject')
    nav = sys.modules['nav2_msgs.action'].NavigateToPose
    nav.Result = lambda: NS()
    nav.Feedback = lambda: NS()
    geometry = ModuleType('geometry_msgs.msg')
    geometry.PoseStamped = object
    groups = ModuleType('rclpy.callback_groups')
    groups.ReentrantCallbackGroup = object
    monkeypatch.setitem(sys.modules, 'geometry_msgs.msg', geometry)
    monkeypatch.setitem(sys.modules, 'rclpy.callback_groups', groups)


async def _create(tmp_path, node=None, with_map=True):
    node, buffer = node or _Node(), _DockBuffer()
    config = _dock_config(tmp_path)
    ingress = None

    def feedback(*values):
        ingress.on_feedback(*values)

    runtime = await create_runtime(config, node, buffer, now=lambda: 1000.1,
                                   on_dock_feedback=feedback)
    ingress = create_dock_ingress(
        runtime.tasks, DockNodeConfig('/custom_dock', '/custom_action', config.dock),
        cleanup_retry_sec=0.001)
    ingress.open(node)
    if with_map:
        node.subscriptions[-2].callback(_map())
    return runtime, ingress, node


async def _finish(runtime, ingress):
    await asyncio.wait_for(ingress.close(), 3)
    await asyncio.wait_for(runtime.close(), 3)
    # All fake callback threads have been awaited by each test before destruction.
    ingress.destroy()


@pytest.mark.parametrize('with_map,terminal', [(True, 'succeeded'), (False, 'aborted')])
def test_action_runs_shared_dock_and_guard_before_any_goal(
    ingress_ros, tmp_path, with_map, terminal,
):
    """Actual callback thread waits for managed Dock; missing-map rejection sends nothing."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path, with_map=with_map)
        events = runtime.tasks.subscribe_events()
        server, handle = node.servers[0], _Handle()
        response = await asyncio.to_thread(server.callbacks['goal_callback'], handle.request)
        assert response == 'accept'
        await asyncio.wait_for(asyncio.to_thread(server.execute, handle), 3)
        assert handle.terminal == [terminal]
        assert len(node.clients) == 2  # Ingress never makes a third client.
        assert len(node.goals) == (2 if with_map else 0)
        assert runtime.tasks.status().state == TaskState.IDLE
        event = events.get_nowait()
        assert event.source == 'ros_dock' and event.task_id.startswith('dock:')
        runtime.tasks.unsubscribe_events(events)
        assert node.subscriptions[-1].topic == '/custom_dock'
        assert node.subscriptions[-1].qos == 1
        assert node.subscriptions[-1].group is server.callbacks['callback_group']
        if with_map:
            assert all(goal.pose.header.frame_id == 'world' for goal in node.goals)
            assert all(goal.pose.pose.position.z == 0 for goal in node.goals)
        await _finish(runtime, ingress)
        assert server.destroy_count == 1 and len(node.destroyed) == 5

    asyncio.run(scenario())


def test_busy_topic_and_action_requests_do_not_preempt_motion(ingress_ros, tmp_path):
    """ROS rejects busy requests while the same manager supports other ingress policies."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path, _Node([None]))
        handle, server = _Handle(), node.servers[0]
        executing = asyncio.create_task(asyncio.to_thread(server.execute, handle))
        await asyncio.wait_for(node.started.get(), 1)
        assert await asyncio.to_thread(server.callbacks['goal_callback'], _Handle().request) == (
            'reject')
        await asyncio.to_thread(node.subscriptions[-1].callback, _pose())
        second = _Handle()
        await asyncio.wait_for(asyncio.to_thread(server.execute, second), 1)
        assert second.terminal == ['aborted']
        assert not node.canceled.is_set() and len(node.goals) == 1
        await ingress.close()
        await executing
        assert handle.terminal == ['aborted']
        lines = [line for level, line in node.logger.lines]
        assert '[dock] 上一个目标还在执行，拒绝动作目标' in lines
        assert '[dock] 上一个目标还在执行，动作目标放弃' in lines
        assert '[dock] 上一个目标还在执行，忽略本次' in lines
        await _finish(runtime, ingress)

    asyncio.run(scenario())


def test_topic_detaches_pose_and_does_not_wait_for_navigation(ingress_ros, tmp_path):
    """Topic callback returns immediately and later caller mutation cannot change the target."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path, _Node([None]))
        pose = _pose()
        node.subscriptions[-1].callback(pose)
        pose.pose.position.x = 999
        await asyncio.wait_for(node.started.get(), 1)
        assert node.goals[0].pose.pose.position.x == 0.5
        await _finish(runtime, ingress)

    asyncio.run(scenario())


@pytest.mark.parametrize('early', [False, True])
def test_cancel_waits_for_ack_and_cleanup_without_late_success(ingress_ros, tmp_path, early):
    """Cancellation before admission or during motion never produces a succeeded result."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path, _Node([None]))
        handle, server = _Handle(), node.servers[0]
        if early:
            assert await asyncio.to_thread(server.callbacks['cancel_callback'], handle) == 'cancel'
        executing = asyncio.create_task(asyncio.to_thread(server.execute, handle))
        if not early:
            await asyncio.wait_for(node.started.get(), 1)
            assert await asyncio.to_thread(server.callbacks['cancel_callback'], handle) == 'cancel'
            await asyncio.wait_for(node.canceled.wait(), 1)
        await asyncio.sleep(0.02)
        assert not executing.done() and not handle.terminal
        handle.is_cancel_requested = True  # Fake rclpy cancel-response state transition.
        await asyncio.wait_for(executing, 1)
        assert handle.terminal == ['canceled']
        assert len(node.goals) == (0 if early else 1)
        await _finish(runtime, ingress)

    asyncio.run(scenario())


def test_preemption_settles_ros_goal_without_canceling_new_owner(ingress_ros, tmp_path):
    """The new completion receipt covers wire-silent preemption; no polling or extra goals."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path, _Node([None, None]))
        handle, server = _Handle(), node.servers[0]
        executing = asyncio.create_task(asyncio.to_thread(server.execute, handle))
        await asyncio.wait_for(node.started.get(), 1)
        accepted = await runtime.tasks.submit(TaskRequest(
            'replacement', 'dock', {'pose': PoseRecord('world', 1, 2, 0, 0, 0)},
            preempt=PreemptPolicy.ALLOW))
        await asyncio.wait_for(node.started.get(), 1)
        await asyncio.wait_for(executing, 1)
        assert handle.terminal == ['aborted']
        await ingress.close()
        assert runtime.tasks.status().task_id == accepted.task_id
        assert node.events.count('cancel') == 1
        await _finish(runtime, ingress)

    asyncio.run(scenario())


@pytest.mark.parametrize('fail_feedback', [False, True])
def test_raw_feedback_is_complete_correlated_and_fenced(ingress_ros, tmp_path, fail_feedback):
    """Full fields and distance offset survive; stale or failing feedback never leaks success."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path, _Node([None]))
        handle, server = _Handle(), node.servers[0]
        handle.fail_feedback = fail_feedback
        executing = asyncio.create_task(asyncio.to_thread(server.execute, handle))
        await asyncio.wait_for(node.started.get(), 1)
        snapshot = runtime.tasks.status()
        goal = DockGoal(PoseRecord('world', 1, 2, 0, 0, 0), snapshot.task_id,
                        snapshot.sub_id, snapshot.generation)
        raw = NS(feedback=NS(current_pose=object(), navigation_time=object(),
                             estimated_time_remaining=object(), number_of_recoveries=7,
                             distance_remaining=2.25))
        ingress.on_feedback(replace(goal, generation=goal.generation + 1), raw, 0.5)
        assert not handle.feedback
        ingress.on_feedback(goal, raw, 0.5)
        if fail_feedback:
            await asyncio.wait_for(executing, 1)
            assert handle.terminal == ['aborted'] and node.canceled.is_set()
        else:
            feedback = handle.feedback[0]
            assert feedback.current_pose is raw.feedback.current_pose
            assert feedback.navigation_time is raw.feedback.navigation_time
            assert feedback.estimated_time_remaining is raw.feedback.estimated_time_remaining
            assert feedback.number_of_recoveries == 7 and feedback.distance_remaining == 2.75
            assert raw.feedback.distance_remaining == 2.25
        ingress.stop()
        ingress.on_feedback(goal, raw, 0.5)
        assert len(handle.feedback) == (0 if fail_feedback else 1)
        await ingress.close()
        await executing
        await _finish(runtime, ingress)

    asyncio.run(scenario())


def test_cleanup_timeout_retains_action_and_close_waiter_can_be_canceled(ingress_ros, tmp_path):
    """Neither a cleanup deadline nor losing close's waiter destroys pending resources."""
    async def scenario():
        node = _Node([None])
        node.complete_on_cancel = False
        runtime, ingress, node = await _create(tmp_path, node)
        runtime._backend._session._cancel_timeout = 0.001
        handle, server = _Handle(), node.servers[0]
        executing = asyncio.create_task(asyncio.to_thread(server.execute, handle))
        source = await asyncio.wait_for(node.started.get(), 1)
        closing = asyncio.create_task(ingress.close())
        await asyncio.wait_for(node.canceled.wait(), 1)
        closing.cancel()
        with pytest.raises(asyncio.CancelledError):
            await closing
        await asyncio.sleep(0.03)
        assert not executing.done() and not handle.terminal
        assert server.destroy_count == 0 and not node.destroyed
        with pytest.raises(RuntimeError):
            ingress.destroy()
        source.result.set_result(NS(status=5))
        await asyncio.wait_for(ingress.close(), 1)
        await executing
        assert handle.terminal == ['aborted']
        await _finish(runtime, ingress)

    asyncio.run(scenario())


def test_partial_open_and_failed_destroy_retain_owned_resources(ingress_ros, tmp_path):
    """Constructor failure preserves the completed subscription; teardown retries only failures."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path)
        server = node.servers[0]
        await ingress.close()
        server.fail_destroy = True
        with pytest.raises(RuntimeError, match='server destruction'):
            ingress.destroy()
        assert not node.destroyed
        server.fail_destroy = False
        node.fail_subscription_destroy = True
        with pytest.raises(RuntimeError, match='subscription destruction'):
            ingress.destroy()
        assert server.destroy_count == 1
        node.fail_subscription_destroy = False
        ingress.destroy()
        assert server.destroy_count == 1
        second = create_dock_ingress(runtime.tasks,
                                     DockNodeConfig('/other_goal', '/other_action',
                                                    _dock_config(tmp_path).dock))
        node.fail_server = True
        with pytest.raises(RuntimeError, match='server construction'):
            second.open(node)
        node.subscriptions[-1].callback(_pose())
        await asyncio.sleep(0.02)
        assert not node.goals  # Failed assembly must never expose admission.
        await second.close()
        second.destroy()
        await runtime.close()
        assert len(node.destroyed) == 6

    asyncio.run(scenario())


def test_late_callbacks_after_stop_cannot_admit_motion(ingress_ros, tmp_path):
    """An already accepted ROS goal arriving after sealing is aborted without task admission."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path)
        server = node.servers[0]
        ingress.stop()
        node.subscriptions[-1].callback(_pose())
        handle = _Handle()
        await asyncio.wait_for(asyncio.to_thread(server.execute, handle), 1)
        assert handle.terminal == ['aborted'] and not node.goals
        response = await asyncio.to_thread(server.callbacks['goal_callback'], handle.request)
        assert response == 'reject'
        with pytest.raises(RuntimeError, match='multithreaded'):
            server.execute(_Handle())
        await _finish(runtime, ingress)

    asyncio.run(scenario())


def test_ros_tasks_do_not_emit_unsolicited_ws_arrival_frames(ingress_ros, tmp_path):
    """Shared manager observability must not turn a ROS goal into a server-request arrival."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path, _Node([4, 4, 4, 4]))
        node.subscriptions[0].callback(_DockBuffer().odom())
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        next_ws = asyncio.create_task(adapter.next_event())
        try:
            await asyncio.wait_for(asyncio.to_thread(node.servers[0].execute, _Handle()), 3)
            await asyncio.sleep(0)
            assert not next_ws.done()
            await _mark(adapter)
            await _navigate(adapter)
            assert await asyncio.wait_for(next_ws, 3) == {
                'event_type': 'on_arrived', 'request_id': 'r', 'sub_id': 1}
        finally:
            next_ws.cancel()
            await asyncio.gather(next_ws, return_exceptions=True)
            adapter.close()
            await _finish(runtime, ingress)

    asyncio.run(scenario())


def test_worker_failure_waits_for_owned_cleanup_before_action_abort(ingress_ros, tmp_path):
    """A failure event alone cannot terminate the ROS action while cleanup is unresolved."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path)
        gate, cancel_started = asyncio.Event(), asyncio.Event()
        original = runtime.tasks._cancel_backend

        async def delayed_cleanup():
            cancel_started.set()
            await gate.wait()
            await original()

        def broken_pose():
            raise OSError('pose port failed')

        runtime.tasks._cancel_backend = delayed_cleanup
        runtime._dock_worker._pose = broken_pose
        handle = _Handle()
        executing = asyncio.create_task(asyncio.to_thread(node.servers[0].execute, handle))
        try:
            await asyncio.wait_for(cancel_started.wait(), 2)
            assert not executing.done() and not handle.terminal
            assert runtime.tasks.status().state == TaskState.ABORTING
        finally:
            gate.set()
            await asyncio.wait_for(executing, 2)
            await _finish(runtime, ingress)
        assert handle.terminal == ['aborted']

    asyncio.run(scenario())


def test_invalid_pose_cannot_preempt_existing_motion(ingress_ros, tmp_path):
    """Malformed direct ingress is rejected before the manager can cancel a current owner."""
    async def scenario():
        runtime, ingress, node = await _create(tmp_path, _Node([None]))
        first = await runtime.tasks.submit(TaskRequest(
            'owner', 'dock', {'pose': PoseRecord('world', 1, 2, 0, 0, 0)}))
        await asyncio.wait_for(node.started.get(), 1)
        pose = _pose()
        pose.pose.position.x = float('nan')
        handle = _Handle(pose)
        await asyncio.wait_for(asyncio.to_thread(node.servers[0].execute, handle), 1)
        assert handle.terminal == ['aborted']
        assert runtime.tasks.status().task_id == first.task_id
        assert not node.canceled.is_set() and len(node.goals) == 1
        await _finish(runtime, ingress)

    asyncio.run(scenario())


@pytest.mark.parametrize('retry', [True, 0, float('nan'), '1'])
def test_bad_cleanup_interval_is_rejected_without_io(tmp_path, retry):
    """Invalid retry timing cannot open an ingress owner or create a busy cleanup loop."""
    async def scenario():
        config = DockNodeConfig('/goal', '/action', _dock_config(tmp_path).dock)
        with pytest.raises(ConfigError):
            create_dock_ingress(object(), config, cleanup_retry_sec=retry)

    asyncio.run(scenario())
