"""Characterization (golden) tests for the navigation executor wire contract.

These tests pin down the *current* observable behaviour of the executor before
the refactor described in ``2026-09-14-david-nav-executor-refactor-spec.md``
(SPEC §6 P0, scenarios G1-G16) and ``2026-09-14-david-nav-system-refactor-plan.md``.
They complement the existing unit tests by asserting exact wire bytes (key
order, ``ensure_ascii=False`` Chinese text, rounding) and frozen log formats.

A failing test here means the refactor changed externally visible behaviour.
Do not update an expectation without recording the decision in an ADR.
"""

import asyncio
import json
from pathlib import Path
import re
from types import SimpleNamespace

from david_nav import nav2_action_bridge
from david_nav import navigation_executor
from david_nav.nav2_action_bridge import Nav2ActionBridge
from david_nav.navigation_executor_core import ExecutorState
from david_nav.navigation_executor_core import NavigationExecutorCore
from david_nav.navigation_types import CapturedPose
from david_nav.navigation_types import PoseRecord
from david_nav.navigation_types import StoredPoi
from david_nav.poi_store import LocalPoiStore


# --------------------------------------------------------------------------
# Harness
# --------------------------------------------------------------------------

class _FakeRclpyFuture:
    """Minimal stand-in for an already-completed rclpy future."""

    def __init__(self, value):
        self._value = value

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._value


class _FakeGoalHandle:
    """Goal handle whose result resolves immediately with ``status``."""

    def __init__(self, status, accepted=True):
        self.accepted = accepted
        self._status = status

    def get_result_async(self):
        return _FakeRclpyFuture(SimpleNamespace(status=self._status))

    def cancel_goal_async(self):
        return _FakeRclpyFuture(None)


class _FakeGoal:
    """Shape-compatible replacement for ``NavigateToPose.Goal``."""

    def __init__(self):
        self.pose = SimpleNamespace(
            header=SimpleNamespace(frame_id='', stamp=None),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )


class _FakeActionClient:
    """Consumes scripted outcomes: GoalStatus ints, 'rejected' or 'no_server'."""

    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.sent_goals = []

    def wait_for_server(self, timeout_sec):
        if self.outcomes and self.outcomes[0] == 'no_server':
            self.outcomes.pop(0)
            return False
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent_goals.append(goal)
        outcome = self.outcomes.pop(0)
        if outcome == 'rejected':
            return _FakeRclpyFuture(_FakeGoalHandle(status=None, accepted=False))
        return _FakeRclpyFuture(_FakeGoalHandle(status=outcome))


class _HarnessBridge(Nav2ActionBridge):
    """Real bridge logic without rclpy / nav2_msgs."""

    def __init__(self, outcomes, event_callback, goal_retry_count=1):
        # Deliberately skip super().__init__(): it imports ROS message types.
        self.node = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 0))
        )
        self.action_name = '/navigate_to_pose'
        self.server_timeout = 0.1
        self.event_callback = event_callback
        self.progress_min_interval_sec = 1.0
        self.goal_retry_count = max(0, int(goal_retry_count))
        self.goal_retry_delay_sec = 0.0
        self._navigate_to_pose = SimpleNamespace(Goal=_FakeGoal)
        self.client = _FakeActionClient(outcomes)
        self._current_goal_handle = None


class _FakePoseProvider:
    def __init__(self, captured_pose=None):
        self.captured_pose = captured_pose

    async def capture_for_poi(self):
        return self.captured_pose


class _NoCandidateResolver:
    """Resolver whose snap search never finds a reachable point."""

    def resolve_initial_goal(self, poi):
        return None

    def is_anchor_admissible(self, poi):
        return False

    def is_anchor_static_blocked(self, poi):
        return False


def _store(tmp_path: Path) -> LocalPoiStore:
    store = LocalPoiStore(tmp_path / 'poi_store.yaml')
    store.upsert_marked_poi(
        'POI_001',
        '前台',
        PoseRecord('map', 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord('odom', 1.0, 2.0, 0.0, 0.0, 100.0),
        'session_a',
    )
    return store


def _wire(payload: dict) -> str:
    """Serialize exactly like navigation_executor._drain_outbound_queue."""
    return json.dumps(payload, ensure_ascii=False)


def _build_core_with_bridge(tmp_path, outcomes, goal_retry_count=1, goal_resolver=None):
    """Wire core + bridge the same way main_async does and collect frames."""
    frames = []
    holder = {}

    async def on_bridge_event(event):
        for payload in await holder['core'].handle_bridge_event(event):
            frames.append(payload)

    bridge = _HarnessBridge(outcomes, on_bridge_event, goal_retry_count)
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=_FakePoseProvider(),
        poi_store=_store(tmp_path),
        goal_resolver=goal_resolver,
    )
    holder['core'] = core
    return core, bridge, frames


_NAVIGATE = {'action': 'navigate_to', 'request_id': 'req_1', 'sub_id': 1, 'target_id': 'POI_001'}


# --------------------------------------------------------------------------
# G1 / G8 / G12: navigate_to end-to-end through core + real bridge logic
# --------------------------------------------------------------------------

def test_g1_arrival_emits_exact_on_arrived_frame(tmp_path):
    core, bridge, frames = _build_core_with_bridge(tmp_path, outcomes=[4])

    async def scenario():
        immediate = await core.handle_message(dict(_NAVIGATE))
        await core._nav_task
        return immediate

    immediate = asyncio.run(scenario())

    assert immediate == []
    assert [_wire(f) for f in frames] == [
        '{"event_type": "on_arrived", "request_id": "req_1", "sub_id": 1}'
    ]
    assert core.state == ExecutorState.IDLE
    assert core.active_task is None
    assert len(bridge.client.sent_goals) == 1


def test_g8_retry_exhausted_emits_exact_error_text(tmp_path, capsys):
    core, bridge, frames = _build_core_with_bridge(tmp_path, outcomes=[6, 6], goal_retry_count=1)

    async def scenario():
        await core.handle_message(dict(_NAVIGATE))
        await core._nav_task

    asyncio.run(scenario())

    assert len(bridge.client.sent_goals) == 2
    assert len(frames) == 1
    wire = _wire(frames[0])
    assert re.fullmatch(
        r'\{"event_type": "on_error", "request_id": "req_1", "sub_id": 1, '
        r'"error_message": "导航失败\(共尝试2次\): Nav2 终态 status=6\(ABORTED\), '
        r'target=POI_001\(前台\) goal=map\(x=1\.00, y=2\.00, yaw=0\.00rad\), '
        r'剩余距离=n/a\(无 feedback,可能 planner 直接失败\), 本次耗时=\d+\.\ds"\}',
        wire,
    ), wire
    assert core.state == ExecutorState.IDLE
    assert core.active_task is None

    out = capsys.readouterr().out
    assert '[nav2_action_bridge] NAV FAILURE attempt 1/2 request_id=req_1 sub_id=1: ' in out
    assert '[nav2_action_bridge] NAV FAILURE attempt 2/2 request_id=req_1 sub_id=1: ' in out


def test_g12_external_cancel_forwards_paused_with_current_request_id(tmp_path):
    # CANCELED (5) that the executor did not request: the bridge reports a
    # paused progress and core forwards it under the active request id.
    core, bridge, frames = _build_core_with_bridge(tmp_path, outcomes=[5])

    async def scenario():
        await core.handle_message(dict(_NAVIGATE))
        await core._nav_task

    asyncio.run(scenario())

    assert [_wire(f) for f in frames] == [
        '{"event_type": "on_progress", "request_id": "req_1", "sub_id": 1, '
        '"remaining_distance": 0.0, "status": "paused"}'
    ]
    assert len(bridge.client.sent_goals) == 1
    # Characterized current behaviour: the task is not cleared on an external
    # cancel; the executor stays NAVIGATING until the next command.
    assert core.state == ExecutorState.NAVIGATING
    assert core.active_task is not None
    assert core.active_task.request_id == 'req_1'


# --------------------------------------------------------------------------
# G2 / G6: rejected before any motion
# --------------------------------------------------------------------------

def test_g2_unknown_poi_exact_error_frame(tmp_path):
    core, bridge, frames = _build_core_with_bridge(tmp_path, outcomes=[])

    outbound = asyncio.run(core.handle_message(
        {'action': 'navigate_to', 'request_id': 'req_1', 'sub_id': 1, 'target_id': 'POI_404'}
    ))

    assert [_wire(f) for f in outbound] == [
        '{"event_type": "on_error", "request_id": "req_1", "sub_id": 1, '
        '"error_message": "目标点不存在: POI_404"}'
    ]
    assert bridge.client.sent_goals == []
    assert core.state == ExecutorState.IDLE


def test_g6_no_snap_candidate_exact_error_frame(tmp_path):
    core, bridge, frames = _build_core_with_bridge(
        tmp_path, outcomes=[], goal_resolver=_NoCandidateResolver()
    )

    outbound = asyncio.run(core.handle_message(dict(_NAVIGATE)))

    assert [_wire(f) for f in outbound] == [
        '{"event_type": "on_error", "request_id": "req_1", "sub_id": 1, '
        '"error_message": "目标点不可达，且未找到可行扩圈点"}'
    ]
    assert bridge.client.sent_goals == []
    assert core.state == ExecutorState.IDLE
    assert core.active_task is None


# --------------------------------------------------------------------------
# G9: abort frame rounding
# --------------------------------------------------------------------------

def test_g9_abort_paused_frame_rounds_last_remaining_distance(tmp_path):
    frames = []

    class _HangingBridge:
        async def start_navigation(self, request_id, sub_id, poi):
            await asyncio.Event().wait()

        async def cancel_current(self):
            return None

    core = NavigationExecutorCore(
        bridge=_HangingBridge(), pose_provider=_FakePoseProvider(), poi_store=_store(tmp_path)
    )

    async def scenario():
        await core.handle_message(dict(_NAVIGATE))
        frames.extend(await core.handle_bridge_event({
            'kind': 'progress', 'request_id': 'req_1', 'sub_id': 1,
            'remaining_distance': 2.3456, 'status': 'running',
        }))
        frames.extend(await core.handle_message(
            {'action': 'abort_navigation', 'request_id': 'req_1'}
        ))

    asyncio.run(scenario())

    assert [_wire(f) for f in frames] == [
        '{"event_type": "on_progress", "request_id": "req_1", "sub_id": 1, '
        '"remaining_distance": 2.346, "status": "running"}',
        '{"event_type": "on_progress", "request_id": "req_1", "sub_id": 1, '
        '"remaining_distance": 2.346, "status": "paused"}',
    ]
    assert core.state == ExecutorState.IDLE
    assert core.active_task is None


# --------------------------------------------------------------------------
# G13: mark_current_poi success frames
# --------------------------------------------------------------------------

def test_g13_mark_current_poi_success_exact_frames(tmp_path):
    captured = CapturedPose(
        map_pose=PoseRecord('map', 3.0, 4.0, 0.0, 1.0, 1000.0),
        odom_pose=PoseRecord('odom', 3.5, 4.5, 0.0, 1.0, 1000.0),
        slam_session_id='live_session',
    )
    core = NavigationExecutorCore(
        bridge=SimpleNamespace(),
        pose_provider=_FakePoseProvider(captured),
        poi_store=_store(tmp_path),
        now_fn=lambda: 1000.1,
    )

    outbound = asyncio.run(core.handle_message({
        'action': 'mark_current_poi', 'request_id': 'req_9', 'sub_id': 2,
        'poi': {'id': 'POI_NEW', 'name': '大厅'},
    }))

    assert [_wire(f) for f in outbound] == [
        '{"event_type": "on_mark_poi_ack", "request_id": "req_9", "sub_id": 2, '
        '"status": "accepted"}',
        '{"event_type": "on_mark_poi_success", "request_id": "req_9", "sub_id": 2, '
        '"poi": {"id": "POI_NEW", "name": "大厅"}}',
    ]


# --------------------------------------------------------------------------
# G16: CLI surface and frozen log lines
# --------------------------------------------------------------------------

_EXPECTED_HELP = '\n'.join([
    'usage: navigation_executor.py [--server-uri URI] [--poi-store-file PATH] '
    '[--pose-file PATH] [ROS args...]',
    '',
    'Robot-side WebSocket navigation executor for david_nav.',
    '',
    'options:',
    '  --server-uri URI        Agent WebSocket server (nav WS v1). Default:',
    '                          $NAV_WS_URL or ws://127.0.0.1:8101',
    '  --poi-store-file PATH   Local YAML file used to persist executor-owned POIs',
    '  --pose-file PATH        named_poses.json to import into the POI store at startup',
    '  --action-name NAME      NavigateToPose action server to drive',
    '                          (default: /navigate_to_pose; waypoint stack uses',
    '                          /waypoint_follower/navigate)',
    '  --reconnect-delay SEC   Delay between WebSocket reconnect attempts (default: 3.0)',
    '  --nav-retry-count N     Silent re-publish attempts after a nav failure before',
    '                          reporting on_error to the server (default: 1)',
    '  --nav-retry-delay SEC   Wait between a nav failure and the re-publish (default: 1.0)',
    # 2026-09-15 alias 0 精准入位（用户要求的行为变更，golden 同步更新）
    '  --dock-poi-id ID        Navigate this POI with two-stage precise docking through',
    '                          the docking action (repeatable; never snapped or retried)',
    '  --dock-action-name NAME Docking action server (default: /dock_to_pose)',
    '  -h, --help              Show this help message and exit',
])


def test_g16_help_text_is_frozen():
    assert navigation_executor._build_help_text() == _EXPECTED_HELP


def test_g16_default_cli_config_is_frozen():
    config, ros_args = navigation_executor._parse_cli_args([])

    assert ros_args == []
    assert config == {
        'server_uri': navigation_executor.DEFAULT_SERVER_URI,
        'poi_store_file': str(Path('~/.ros/david_nav/poi_store.yaml').expanduser()),
        'pose_file': None,
        'heartbeat_interval': 30.0,
        'loop_closure_poll_sec': 1.0,
        'action_name': '/navigate_to_pose',
        'server_timeout': 10.0,
        'robot_base_frame': 'base_link',
        'odom_topic': '/lightning/odometry',
        'slam_session_id': 'live_session',
        'grid_map_topic': '/lightning/grid_map',
        'global_costmap_topic': '/global_costmap/costmap',
        'nav_goal_snap_radius': 0.5,
        'nav_goal_fallback_snap_radius': 1.0,
        'nav_goal_clearance_radius': 0.1,
        'nav_goal_max_snap_radius': 3.0,
        'nav_goal_snap_step': 0.25,
        'reconnect_delay': 3.0,
        'nav_retry_count': 1,
        'nav_retry_delay_sec': 1.0,
        'dock_action_name': '/dock_to_pose',
        'dock_poi_ids': [],
    }


def test_frozen_log_line_nav_error(capsys):
    navigation_executor._log_outbound_frame({
        'event_type': 'on_error', 'request_id': 'req_1', 'sub_id': 1, 'error_message': '目标点不存在: X',
    })

    assert capsys.readouterr().out == (
        '[navigation_executor] NAV ERROR reported to server: request_id=req_1 sub_id=1 '
        'error_message=目标点不存在: X\n'
    )


def test_frozen_log_line_status():
    core = SimpleNamespace(
        state=ExecutorState.NAVIGATING,
        active_task=SimpleNamespace(target_id='POI_001', phase='to_anchor', remaining_distance=1.234),
    )
    pose_provider = SimpleNamespace(
        current_map_pose=lambda: PoseRecord('map', 1.0, 2.0, 0.0, 0.5, 0.0)
    )

    line = navigation_executor._format_status_line(core, pose_provider)

    assert line == (
        'STATUS state=navigating poi=POI_001 phase=to_anchor d2g=1.23m '
        'x=1.00 y=2.00 yaw=0.50rad'
    )


def test_frozen_goal_status_names():
    assert nav2_action_bridge.GOAL_STATUS_NAMES == {
        0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
        4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
    }
