"""WS v1 byte contracts through new core/skill/backend with fake ROS action I/O."""

import asyncio
from concurrent.futures import Future
from dataclasses import replace
import json
from types import SimpleNamespace

import pytest

from nav_executor.adapters.transport.outbox import ReliableOutbox
from nav_executor.protocol.task_adapter import TaskCommandAdapter
from nav_tasks.api import ClosedError, CommandRegistry, TaskManager, TaskState
from nav_tasks.core.goal_resolution import DirectGoalResolver
from nav_tasks.core.handlers.navigate_to import NavigateToPoi
from nav_tasks.core.handlers.poi_commands import PoiCommands
from nav_tasks.domain.tasks import ResolvedGoal
from nav_tasks.domain.types import CapturedPose, PoseRecord, StoredPoi
from nav_tasks.nav2_client import Nav2Client

_NAVIGATE = {
    'action': 'navigate_to', 'request_id': 'req_1', 'sub_id': 1, 'target_id': 'POI_001',
}
_ARRIVED = '{"event_type": "on_arrived", "request_id": "req_1", "sub_id": 1}'


def _wire(frame):
    return json.dumps(frame, ensure_ascii=False)


def _done(value):
    future = Future()
    future.set_result(value)
    return future


def _message():
    return SimpleNamespace(pose=SimpleNamespace(
        header=SimpleNamespace(frame_id='', stamp=None),
        pose=SimpleNamespace(position=SimpleNamespace(x=0, y=0, z=0),
                             orientation=SimpleNamespace(x=0, y=0, z=0, w=1))))


class _Clock:
    def __init__(self):
        self.value = 10.0
        self.delays = []

    def now(self):
        return self.value

    async def sleep(self, seconds):
        self.delays.append(seconds)
        self.value += seconds
        await asyncio.sleep(0)


class _Handle:
    def __init__(self, status, feedback):
        self.accepted = status != 'rejected'
        self.result = Future() if status is None else _done(SimpleNamespace(status=status))
        self.feedback = feedback
        self.cancel_count = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        if not self.result.done():
            self.finish(5)
        return _done(None)

    def finish(self, status):
        self.result.set_result(SimpleNamespace(status=status))

    def progress(self, remaining):
        self.feedback(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=remaining)))


class _Client:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.handles = []
        self.goals = []
        self.started = asyncio.Queue()

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback):
        handle = _Handle(self.outcomes.pop(0), feedback_callback)
        self.handles.append(handle)
        self.goals.append(goal)
        self.started.put_nowait(handle)
        return _done(handle)


class _Repository:
    def __init__(self):
        pose = PoseRecord('map', 1, 2, 0, 0, 1000)
        poi = StoredPoi('POI_001', '前台', pose, replace(pose, frame_id='odom'), 'session')
        self.pois = {poi.poi_id: poi, 'POI_002': replace(poi, poi_id='POI_002', name='大厅')}
        self.writes = 0

    def get(self, poi_id):
        return self.pois.get(poi_id)

    def upsert(self, poi):
        self.pois[poi.poi_id] = poi
        self.writes += 1
        return poi

    def rename_many(self, pairs):
        for poi_id, name in pairs:
            if poi_id in self.pois:
                self.pois[poi_id] = replace(self.pois[poi_id], name=name)
        self.writes += 1


class _Poses:
    def __init__(self):
        self.stamp = 1000
        self.error = None

    async def capture_for_poi(self):
        if self.error is not None:
            raise self.error
        pose = PoseRecord('map', 3, 4, 0, 1, self.stamp)
        return CapturedPose(pose, replace(pose, frame_id='odom'), 'live_session')


class _Resolver:
    def __init__(self, admissible):
        self.admissible = admissible

    def resolve(self, poi):
        return ResolvedGoal(replace(poi, map_pose=replace(poi.map_pose, x=0.5)), True)

    def is_admissible(self, poi):
        return self.admissible


class _Harness:
    def __init__(self, outcomes, resolver=None, on_failure=None):
        self.clock = _Clock()
        self.client = _Client(outcomes)
        self.repository = _Repository()
        self.poses = _Poses()
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: self.clock.value)))
        self.backend = Nav2Client(node, '/navigate_to_pose', 0.1, client=self.client,
                                  goal_factory=_message, clock=self.clock)
        skill = NavigateToPoi(
            self.backend, self.repository, resolver or DirectGoalResolver(), clock=self.clock,
            on_failure=on_failure)
        registry = CommandRegistry()
        registry.register('navigate_to', skill.decode, skill.run)
        self.tasks = TaskManager(registry, self.backend.cancel)
        commands = PoiCommands(self.poses, self.repository, lambda: 1000.1)
        self.adapter = TaskCommandAdapter(self.tasks, commands)

    async def close(self):
        await self.tasks.close()
        await self.backend.close()
        self.adapter.close()


async def _settle():
    for _ in range(8):
        await asyncio.sleep(0)


def test_g1_running_then_arrived_exact_bytes_and_nonblocking_admission():
    """Actual new navigation code returns admission before a fake terminal result."""
    async def scenario():
        harness = _Harness([None])
        assert await harness.adapter.handle_message(dict(_NAVIGATE)) == []
        handle = await harness.client.started.get()
        handle.progress(2.3456)
        assert _wire(await harness.adapter.next_event()) == (
            '{"event_type": "on_progress", "request_id": "req_1", "sub_id": 1, '
            '"remaining_distance": 2.346, "status": "running"}')
        handle.finish(4)
        assert _wire(await harness.adapter.next_event()) == _ARRIVED
        assert harness.tasks.status().state == TaskState.IDLE
        assert harness.tasks.status().task_id is None
        await harness.close()

    asyncio.run(scenario())


def test_g2_unknown_poi_exact_error_without_goal():
    """Admission failures retain the legacy missing-target text and key order."""
    async def scenario():
        harness = _Harness([])
        frames = await harness.adapter.handle_message(dict(_NAVIGATE, target_id='POI_404'))
        assert list(map(_wire, frames)) == [
            '{"event_type": "on_error", "request_id": "req_1", "sub_id": 1, '
            '"error_message": "目标点不存在: POI_404"}']
        assert harness.client.goals == []
        await harness.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('target', [None, 42, {}, '   '])
def test_g3_bad_target_exact_error_without_goal(target):
    """Malformed target IDs are handled before any task admission."""
    async def scenario():
        harness = _Harness([])
        frames = await harness.adapter.handle_message(dict(_NAVIGATE, target_id=target))
        assert list(map(_wire, frames)) == [
            '{"event_type": "on_error", "request_id": "req_1", "sub_id": 1, '
            '"error_message": "navigate_to 缺少有效 target_id(v1 只认 POI id,target_pose 已废弃)"}']
        assert harness.client.goals == []
        await harness.close()

    asyncio.run(scenario())


def test_g4_snap_success_when_anchor_stays_blocked():
    """Only one arrival is emitted and no second navigation goal is submitted."""
    async def scenario():
        harness = _Harness([4], _Resolver(False))
        assert await harness.adapter.handle_message(dict(_NAVIGATE)) == []
        assert _wire(await harness.adapter.next_event()) == _ARRIVED
        assert len(harness.client.goals) == 1
        assert harness.client.goals[0].pose.pose.position.x == 0.5
        assert harness.adapter._events.empty()
        await harness.close()

    asyncio.run(scenario())


def test_g5_anchor_second_leg_must_finish_before_arrival():
    """Snap completion alone never emits arrival when the anchor is now usable."""
    async def scenario():
        harness = _Harness([4, None], _Resolver(True))
        await harness.adapter.handle_message(dict(_NAVIGATE))
        await harness.client.started.get()
        anchor_handle = await harness.client.started.get()
        assert harness.adapter._events.empty()
        assert harness.tasks.status().state == TaskState.STARTING
        assert [goal.pose.pose.position.x for goal in harness.client.goals] == [0.5, 1]
        anchor_handle.finish(4)
        assert _wire(await harness.adapter.next_event()) == _ARRIVED
        assert harness.adapter._events.empty()
        await harness.close()

    asyncio.run(scenario())


def test_g6_unreachable_exact_error_without_goal():
    """Unreachable resolution returns the original unwrapped failure reason."""
    async def scenario():
        harness = _Harness([], SimpleNamespace(resolve=lambda poi: None))
        await harness.adapter.handle_message(dict(_NAVIGATE))
        assert _wire(await harness.adapter.next_event()) == (
            '{"event_type": "on_error", "request_id": "req_1", "sub_id": 1, '
            '"error_message": "目标点不可达，且未找到可行扩圈点"}')
        assert harness.client.goals == []
        await harness.close()

    asyncio.run(scenario())


def test_g7_retry_success_suppresses_intermediate_failure():
    """The core retries the actual one-attempt backend without a premature error."""
    async def scenario():
        harness = _Harness([6, 4])
        await harness.adapter.handle_message(dict(_NAVIGATE))
        assert _wire(await harness.adapter.next_event()) == _ARRIVED
        assert len(harness.client.goals) == 2
        assert harness.clock.delays == [1.0]
        assert harness.adapter._events.empty()
        await harness.close()

    asyncio.run(scenario())


def test_g8_retry_exhausted_exact_backend_detail():
    """Real new backend and core compose the frozen retry error, not a canned fake."""
    async def scenario():
        harness = _Harness([6, 6])
        await harness.adapter.handle_message(dict(_NAVIGATE))
        assert _wire(await harness.adapter.next_event()) == (
            '{"event_type": "on_error", "request_id": "req_1", "sub_id": 1, '
            '"error_message": "导航失败(共尝试2次): Nav2 终态 status=6(ABORTED), '
            'target=POI_001(前台) goal=map(x=1.00, y=2.00, yaw=0.00rad), '
            '剩余距离=n/a(无 feedback,可能 planner 直接失败), 本次耗时=0.0s"}')
        assert len(harness.client.goals) == 2
        assert harness.tasks.status().state == TaskState.IDLE
        await harness.close()

    asyncio.run(scenario())


def test_g9_abort_paused_rounding_and_late_feedback_filter():
    """Abort emits one rounded paused frame after cleanup; late events are ignored."""
    async def scenario():
        harness = _Harness([None])
        await harness.adapter.handle_message(dict(_NAVIGATE))
        handle = await harness.client.started.get()
        handle.progress(2.3456)
        await harness.adapter.next_event()
        assert await harness.adapter.handle_message(
            {'action': 'abort_navigation', 'request_id': 'req_1'}) == []
        assert _wire(await harness.adapter.next_event()) == (
            '{"event_type": "on_progress", "request_id": "req_1", "sub_id": 1, '
            '"remaining_distance": 2.346, "status": "paused"}')
        handle.progress(99)
        await _settle()
        assert harness.adapter._events.empty()
        assert handle.cancel_count == 1
        assert harness.tasks.status().state == TaskState.IDLE
        assert await harness.adapter.handle_message(
            {'action': 'abort_navigation', 'request_id': 'req_1'}) == []
        assert harness.adapter._events.empty()
        await harness.close()

    asyncio.run(scenario())


def test_g10_idle_and_mismatched_abort_do_nothing():
    """A stale request cannot cancel the current navigation task."""
    async def scenario():
        harness = _Harness([None])
        stale = {'action': 'abort_navigation', 'request_id': 'stale'}
        assert await harness.adapter.handle_message(stale) == []
        await harness.adapter.handle_message(dict(_NAVIGATE))
        handle = await harness.client.started.get()
        assert await harness.adapter.handle_message(stale) == []
        assert harness.tasks.status().task_id == 'req_1'
        assert handle.cancel_count == 0
        assert harness.adapter._events.empty()
        await harness.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('replacement_id', ['req_2', 'req_1'])
def test_g11_preempt_filters_old_goal_even_when_request_id_is_reused(replacement_id):
    """Ingress allows preemption and the core generation fence excludes old results."""
    async def scenario():
        harness = _Harness([None, None])
        await harness.adapter.handle_message(dict(_NAVIGATE))
        old = await harness.client.started.get()
        await harness.adapter.handle_message(dict(
            _NAVIGATE, request_id=replacement_id, sub_id=2, target_id='POI_002'))
        new = await harness.client.started.get()
        assert old.cancel_count == 1
        assert harness.adapter._events.empty()
        old.progress(99)
        await _settle()
        assert harness.adapter._events.empty()
        new.finish(4)
        assert _wire(await harness.adapter.next_event()) == (
            '{"event_type": "on_arrived", "request_id": "' + replacement_id + '", "sub_id": 2}')
        assert harness.adapter._events.empty()
        await harness.close()

    asyncio.run(scenario())


def test_g12_external_cancel_resets_distance_but_retains_active_task():
    """External CANCELED is paused with zero distance and the characterized active state."""
    async def scenario():
        harness = _Harness([None])
        await harness.adapter.handle_message(dict(_NAVIGATE))
        handle = await harness.client.started.get()
        handle.progress(8.5)
        await harness.adapter.next_event()
        handle.finish(5)
        assert _wire(await harness.adapter.next_event()) == (
            '{"event_type": "on_progress", "request_id": "req_1", "sub_id": 1, '
            '"remaining_distance": 0.0, "status": "paused"}')
        assert harness.tasks.status().state == TaskState.NAVIGATING
        assert harness.tasks.status().task_id == 'req_1'
        assert handle.cancel_count == 0
        await harness.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('failure', [None, 'stale', 'capture'])
def test_g13_mark_exact_ack_and_completion_without_motion(failure):
    """Marking preserves wire order, Chinese text and legacy capture failures."""
    async def scenario():
        harness = _Harness([])
        if failure == 'stale':
            harness.poses.stamp = 0
        if failure == 'capture':
            harness.poses.error = RuntimeError('TF unavailable')
        frames = await harness.adapter.handle_message({
            'action': 'mark_current_poi', 'request_id': 'req_9', 'sub_id': 2,
            'poi': {'id': 'POI_NEW', 'name': '大厅'},
        })
        expected = [
            '{"event_type": "on_mark_poi_ack", "request_id": "req_9", "sub_id": 2, '
            '"status": "accepted"}']
        if failure is None:
            expected.append(
                '{"event_type": "on_mark_poi_success", "request_id": "req_9", "sub_id": 2, '
                '"poi": {"id": "POI_NEW", "name": "大厅"}}')
        else:
            detail = '当前位置位姿数据过期' if failure == 'stale' else 'TF unavailable'
            expected.append(
                '{"event_type": "on_mark_poi_error", "request_id": "req_9", "sub_id": 2, '
                '"error_message": "' + detail + '"}')
        assert list(map(_wire, frames)) == expected
        assert harness.repository.writes == int(failure is None)
        assert harness.client.goals == []
        await harness.close()

    asyncio.run(scenario())


def test_g14_directory_update_and_mark_do_not_preempt_navigation():
    """Administrative commands change names/POIs without taking motion ownership."""
    async def scenario():
        harness = _Harness([None])
        await harness.adapter.handle_message(dict(_NAVIGATE))
        handle = await harness.client.started.get()
        before = harness.tasks.status()
        original_pose = harness.repository.get('POI_001').map_pose
        assert await harness.adapter.handle_message({
            'action': 'update_poi_list', 'version': 123,
            'poi_list': [{'id': 'POI_001', 'name': '新前台'}, {'id': 'missing', 'name': '无坐标'}],
        }) == []
        assert harness.repository.get('POI_001').name == '新前台'
        assert harness.repository.get('POI_001').map_pose is original_pose
        assert harness.repository.get('missing') is None
        await harness.adapter.handle_message({
            'action': 'mark_current_poi', 'request_id': 'mark', 'sub_id': 1,
            'poi': {'id': 'POI_NEW', 'name': '大厅'},
        })
        assert harness.repository.writes == 2
        assert harness.tasks.status() == before
        assert handle.cancel_count == 0
        assert harness.adapter._events.empty()
        await harness.close()

    asyncio.run(scenario())


def test_g15_arrival_retained_across_failed_send_before_later_admin_frames():
    """New core-to-outbox chain keeps arrival ahead of frames queued while disconnected."""
    from nav_executor.app.logging import OutboundAudit

    async def scenario():
        harness = _Harness([4])
        audited = []
        lines = []
        audit = OutboundAudit(lines.append)

        def sent(frame, replay):
            audited.append((frame, replay))
            audit.on_sent(frame, replay)

        outbox = ReliableOutbox(on_sending=audit.on_sending, on_sent=sent)
        pump = asyncio.create_task(harness.adapter.pump_events(outbox.put))

        async def fail_send(encoded):
            assert encoded == _ARRIVED
            raise ConnectionError('connection lost')

        sender = asyncio.create_task(outbox.drain(fail_send))
        await harness.adapter.handle_message(dict(_NAVIGATE))
        with pytest.raises(ConnectionError):
            await sender
        assert outbox.pending_count == 1
        for frame in await harness.adapter.handle_message({
                'action': 'mark_current_poi', 'request_id': 'mark', 'sub_id': 2,
                'poi': {'id': 'POI_NEW', 'name': '大厅'}}):
            outbox.put(frame)
        assert outbox.pending_count == 3
        received = []
        completed = asyncio.Event()

        async def reconnected_send(encoded):
            received.append(encoded)
            if len(received) == 3:
                completed.set()

        sender = asyncio.create_task(outbox.drain(reconnected_send))
        await completed.wait()
        assert received[0] == _ARRIVED
        assert [json.loads(frame)['event_type'] for frame in received] == [
            'on_arrived', 'on_mark_poi_ack', 'on_mark_poi_success']
        assert [replay for _, replay in audited] == [True, False, False]
        assert lines == ['OUT RESENT(断联重发): on_arrived request_id=req_1 sub_id=1']
        assert outbox.pending_count == 0
        for task in (sender, pump):
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
        await harness.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('payload,error', [
    ({'action': 'mark_current_poi', 'request_id': 'r'}, KeyError),
    ({'action': 'abort_navigation'}, KeyError),
    ({'action': 'update_poi_list', 'version': 'invalid'}, ValueError),
    ({'action': 'navigate_to', 'target_id': 'POI_001'}, ValueError),
    ({'action': 'unsupported'}, ValueError),
])
def test_malformed_frame_behavior_is_not_silently_repaired_in_adapter(payload, error):
    """R1 remains a separate repair; this adapter does not conceal decoder changes."""
    async def scenario():
        harness = _Harness([])
        with pytest.raises(error):
            await harness.adapter.handle_message(payload)
        assert harness.client.goals == []
        assert harness.tasks.status().state == TaskState.IDLE
        await harness.close()

    asyncio.run(scenario())


def test_unknown_poi_during_navigation_does_not_discard_existing_owner():
    """The documented ownership fix rejects unknown IDs before preemption."""
    async def scenario():
        harness = _Harness([None])
        await harness.adapter.handle_message(dict(_NAVIGATE))
        handle = await harness.client.started.get()
        frames = await harness.adapter.handle_message(dict(
            _NAVIGATE, request_id='missing', target_id='POI_404'))
        assert frames[0]['event_type'] == 'on_error'
        assert frames[0]['request_id'] == 'missing'
        assert harness.tasks.status().task_id == 'req_1'
        assert handle.cancel_count == 0
        handle.finish(4)
        assert _wire(await harness.adapter.next_event()) == _ARRIVED
        await harness.close()

    asyncio.run(scenario())


def test_detaching_adapter_does_not_cancel_task_ownership():
    """Shutdown detachment has no implicit motion-cancel side effect."""
    async def scenario():
        harness = _Harness([None])
        await harness.adapter.handle_message(dict(_NAVIGATE))
        handle = await harness.client.started.get()
        harness.adapter.close()
        assert harness.tasks.status().task_id == 'req_1'
        assert handle.cancel_count == 0
        with pytest.raises(ClosedError):
            await harness.adapter.next_event()
        with pytest.raises(ClosedError):
            await harness.adapter.handle_message(dict(_NAVIGATE))
        handle.finish(4)
        await _settle()
        assert harness.tasks.status().state == TaskState.IDLE
        await harness.close()

    asyncio.run(scenario())


def test_g16_status_through_new_chain_tracks_snap_anchor_and_idle():
    """Read-only public snapshots supply the exact legacy status fields."""
    from nav_executor.app.status import format_status_line

    async def scenario():
        harness = _Harness([None, None], _Resolver(True))
        poses = SimpleNamespace(current_map_pose=lambda: PoseRecord('map', 1, 2, 0, 0.5, 0))
        await harness.adapter.handle_message(dict(_NAVIGATE))
        first = await harness.client.started.get()
        assert harness.tasks.status().target_id == 'POI_001'
        assert harness.tasks.status().phase == 'to_snap'
        first.progress(1.234)
        await harness.adapter.next_event()
        assert format_status_line(harness.tasks.status(), poses) == (
            'STATUS state=navigating poi=POI_001 phase=to_snap d2g=1.23m '
            'x=1.00 y=2.00 yaw=0.50rad')
        frozen = harness.tasks.status()
        first.finish(4)
        second = await harness.client.started.get()
        assert harness.tasks.status().phase == 'to_anchor'
        assert harness.tasks.status().state == TaskState.STARTING
        assert frozen.phase == 'to_snap'
        assert format_status_line(harness.tasks.status(), poses) == (
            'STATUS state=starting poi=POI_001 phase=to_anchor d2g=1.23m '
            'x=1.00 y=2.00 yaw=0.50rad')
        second.progress(1.234)
        await harness.adapter.next_event()
        assert format_status_line(harness.tasks.status(), poses) == (
            'STATUS state=navigating poi=POI_001 phase=to_anchor d2g=1.23m '
            'x=1.00 y=2.00 yaw=0.50rad')
        second.finish(4)
        assert _wire(await harness.adapter.next_event()) == _ARRIVED
        assert format_status_line(harness.tasks.status(), poses) == (
            'STATUS state=idle poi=- phase=- d2g=- x=1.00 y=2.00 yaw=0.50rad')
        await harness.close()

    asyncio.run(scenario())


def test_g8_new_chain_retry_audit_preserves_frozen_lines():
    """The new retry observer reports each failure before delay or final error."""
    from nav_tasks.app.logging import RetryAudit

    async def scenario():
        lines = []
        harness = _Harness([6, 6], on_failure=RetryAudit(lines.append))
        await harness.adapter.handle_message(dict(_NAVIGATE))
        frame = await harness.adapter.next_event()
        detail = frame['error_message'].split(': ', 1)[1]
        assert lines == [
            '[nav2_action_bridge] NAV FAILURE attempt 1/2 request_id=req_1 sub_id=1: ' + detail,
            '[nav2_action_bridge] suppressing failure report, re-publishing goal in '
            '1.0s (attempt 2/2)',
            '[nav2_action_bridge] NAV FAILURE attempt 2/2 request_id=req_1 sub_id=1: ' + detail,
            '[nav2_action_bridge] retry budget exhausted (2 attempts) — '
            'reporting on_error to server: ' + detail,
        ]
        await harness.close()

    asyncio.run(scenario())


def test_retry_audit_starts_a_fresh_budget_for_each_navigation_segment():
    """Snap and anchor retain independent failure logs without premature wire errors."""
    from nav_tasks.app.logging import RetryAudit

    async def scenario():
        lines = []
        harness = _Harness([6, 4, 6, 4], _Resolver(True), RetryAudit(lines.append))
        await harness.adapter.handle_message(dict(_NAVIGATE))
        assert _wire(await harness.adapter.next_event()) == _ARRIVED
        assert len(lines) == 4
        assert all('NAV FAILURE attempt 1/2 request_id=req_1 sub_id=1: ' in lines[i]
                   for i in (0, 2))
        assert 'goal=map(x=0.50,' in lines[0]
        assert 'goal=map(x=1.00,' in lines[2]
        assert lines[1] == lines[3]
        assert harness.adapter._events.empty()
        await harness.close()

    asyncio.run(scenario())
