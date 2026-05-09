import asyncio
from dataclasses import dataclass
from pathlib import Path

from g1_nav.navigation_executor_core import ExecutorState, NavigationExecutorCore
from g1_nav.navigation_types import CapturedPose, PoseRecord, StoredPoi
from g1_nav.poi_store import LocalPoiStore


class FakeBridge:
    def __init__(self):
        self.started = []
        self.canceled = 0

    async def start_navigation(self, request_id, sub_id, poi):
        self.started.append((request_id, sub_id, poi.poi_id))

    async def cancel_current(self):
        self.canceled += 1


class FakePoseProvider:
    def __init__(self, captured_pose=None):
        self.captured_pose = captured_pose
        self.calls = 0

    async def capture_for_poi(self):
        self.calls += 1
        if self.captured_pose is None:
            raise AssertionError("mark_current_poi is not part of this test")
        return self.captured_pose


@dataclass
class FakeResolvedNavigationGoal:
    poi: StoredPoi
    used_snap: bool


class FakeGoalResolver:
    def __init__(self, resolved_goal=None, anchor_admissible=True):
        self.resolved_goal = resolved_goal
        self.anchor_admissible = anchor_admissible
        self.resolve_calls = []
        self.anchor_checks = []

    def resolve_initial_goal(self, poi):
        self.resolve_calls.append(poi.poi_id)
        return self.resolved_goal

    def is_anchor_admissible(self, poi):
        self.anchor_checks.append(poi.poi_id)
        return self.anchor_admissible


def test_navigate_to_starts_bridge_and_progress_event_is_forwarded(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
        "session_a",
    )
    bridge = FakeBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        outbound = await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_1",
                "sub_id": 1,
                "target_id": "POI_001",
            }
        )
        state_after_start = core.state
        forwarded = await core.handle_bridge_event(
            {
                "kind": "progress",
                "request_id": "req_1",
                "sub_id": 1,
                "remaining_distance": 2.25,
                "status": "running",
            }
        )
        state_after_progress = core.state
        return outbound, forwarded, state_after_start, state_after_progress

    outbound, forwarded, state_after_start, state_after_progress = asyncio.run(
        run_scenario()
    )

    assert outbound == []
    assert bridge.started == [("req_1", 1, "POI_001")]
    assert state_after_start == ExecutorState.STARTING
    assert state_after_progress == ExecutorState.NAVIGATING
    assert forwarded == [
        {
            "event_type": "on_progress",
            "request_id": "req_1",
            "sub_id": 1,
            "remaining_distance": 2.25,
            "status": "running",
        }
    ]


def test_abort_navigation_emits_paused_progress(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
        "session_a",
    )
    bridge = FakeBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_1",
                "sub_id": 1,
                "target_id": "POI_001",
            }
        )
        return await core.handle_message(
            {
                "action": "abort_navigation",
                "request_id": "req_1",
            }
        )

    outbound = asyncio.run(run_scenario())

    assert bridge.canceled == 1
    assert outbound == [
        {
            "event_type": "on_progress",
            "request_id": "req_1",
            "sub_id": 1,
            "remaining_distance": 0.0,
            "status": "paused",
        }
    ]


def test_navigate_to_unknown_poi_returns_error_without_starting_bridge(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    bridge = FakeBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        return await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_missing",
                "sub_id": 9,
                "target_id": "POI_014",
            }
        )

    outbound = asyncio.run(run_scenario())

    assert outbound == [
        {
            "event_type": "on_error",
            "request_id": "req_missing",
            "sub_id": 9,
            "error_message": "目标点不存在: POI_014",
        }
    ]
    assert bridge.started == []
    assert core.state == ExecutorState.IDLE
    assert core.active_task is None


def test_navigate_to_uses_snap_goal_then_retries_anchor(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    anchor_poi = store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 0.0, 0.0, 0.0, 1.2, 100.0),
        PoseRecord("odom", 0.0, 0.0, 0.0, 1.2, 100.0),
        "session_a",
    )
    snap_poi = StoredPoi(
        poi_id="POI_001",
        name="前台",
        map_pose=PoseRecord("map", 0.5, 0.0, 0.0, 1.2, 100.0),
        odom_pose=anchor_poi.odom_pose,
        slam_session_id=anchor_poi.slam_session_id,
    )
    bridge = FakeBridge()
    resolver = FakeGoalResolver(
        resolved_goal=FakeResolvedNavigationGoal(snap_poi, used_snap=True),
        anchor_admissible=True,
    )
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
        goal_resolver=resolver,
    )

    async def run_scenario():
        await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_1",
                "sub_id": 1,
                "target_id": "POI_001",
            }
        )
        outbound = await core.handle_bridge_event(
            {
                "kind": "arrived",
                "request_id": "req_1",
                "sub_id": 1,
            }
        )
        return outbound, core.state, core.active_task

    outbound, state_after_snap_arrival, active_task = asyncio.run(run_scenario())

    assert outbound == []
    assert bridge.started == [
        ("req_1", 1, "POI_001"),
        ("req_1", 1, "POI_001"),
    ]
    assert resolver.resolve_calls == ["POI_001"]
    assert resolver.anchor_checks == ["POI_001"]
    assert state_after_snap_arrival == ExecutorState.STARTING
    assert active_task is not None
    assert active_task.phase == "to_anchor"


def test_snap_arrival_aborts_when_anchor_remains_unreachable(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    anchor_poi = store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 0.0, 0.0, 0.0, 1.2, 100.0),
        PoseRecord("odom", 0.0, 0.0, 0.0, 1.2, 100.0),
        "session_a",
    )
    snap_poi = StoredPoi(
        poi_id="POI_001",
        name="前台",
        map_pose=PoseRecord("map", 0.5, 0.0, 0.0, 1.2, 100.0),
        odom_pose=anchor_poi.odom_pose,
        slam_session_id=anchor_poi.slam_session_id,
    )
    bridge = FakeBridge()
    resolver = FakeGoalResolver(
        resolved_goal=FakeResolvedNavigationGoal(snap_poi, used_snap=True),
        anchor_admissible=False,
    )
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
        goal_resolver=resolver,
    )

    async def run_scenario():
        await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_1",
                "sub_id": 1,
                "target_id": "POI_001",
            }
        )
        outbound = await core.handle_bridge_event(
            {
                "kind": "arrived",
                "request_id": "req_1",
                "sub_id": 1,
            }
        )
        return outbound, core.state, core.active_task

    outbound, state_after_error, active_task = asyncio.run(run_scenario())

    assert outbound == [
        {
            "event_type": "on_error",
            "request_id": "req_1",
            "sub_id": 1,
            "error_message": "到达扩圈可行点后，目标点仍不可达",
        }
    ]
    assert bridge.started == [("req_1", 1, "POI_001")]
    assert resolver.anchor_checks == ["POI_001"]
    assert state_after_error == ExecutorState.IDLE
    assert active_task is None


def test_mark_current_poi_while_navigating_does_not_cancel_navigation(
    tmp_path: Path,
):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
        "session_a",
    )
    bridge = FakeBridge()
    pose_provider = FakePoseProvider(
        CapturedPose(
            map_pose=PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 101.0),
            odom_pose=PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 101.0),
            slam_session_id="session_a",
        )
    )
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=pose_provider,
        poi_store=store,
        now_fn=lambda: 101.2,
    )

    async def run_scenario():
        await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_nav",
                "sub_id": 1,
                "target_id": "POI_001",
            }
        )
        return await core.handle_message(
            {
                "action": "mark_current_poi",
                "request_id": "req_mark",
                "sub_id": 2,
                "poi": {"id": "POI_006", "name": "会议室门口"},
            }
        )

    outbound = asyncio.run(run_scenario())

    assert bridge.canceled == 0
    assert pose_provider.calls == 1
    assert [item["event_type"] for item in outbound] == [
        "on_mark_poi_ack",
        "on_mark_poi_success",
    ]
    assert store.get_required("POI_006").name == "会议室门口"


def test_mark_current_poi_returns_error_for_stale_pose(tmp_path: Path):
    stale_capture = CapturedPose(
        map_pose=PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 10.0),
        odom_pose=PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 10.0),
        slam_session_id="session_a",
    )
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    bridge = FakeBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(stale_capture),
        poi_store=store,
        pose_stale_after_sec=0.5,
        now_fn=lambda: 11.0,
    )

    async def run_scenario():
        return await core.handle_message(
            {
                "action": "mark_current_poi",
                "request_id": "req_mark",
                "sub_id": 2,
                "poi": {"id": "POI_006", "name": "会议室门口"},
            }
        )

    outbound = asyncio.run(run_scenario())

    assert outbound[-1]["event_type"] == "on_mark_poi_error"
    assert outbound[-1]["error_message"] == "当前位置位姿数据过期"


def test_mark_current_poi_returns_error_when_pose_capture_raises(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    bridge = FakeBridge()

    class RaisingPoseProvider:
        async def capture_for_poi(self):
            raise RuntimeError(
                '"map" passed to lookupTransform argument target_frame does not exist.'
            )

    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=RaisingPoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        return await core.handle_message(
            {
                "action": "mark_current_poi",
                "request_id": "req_mark",
                "sub_id": 3,
                "poi": {"id": "POI_006", "name": "会议室门口"},
            }
        )

    outbound = asyncio.run(run_scenario())

    assert [item["event_type"] for item in outbound] == [
        "on_mark_poi_ack",
        "on_mark_poi_error",
    ]
    assert outbound[-1]["request_id"] == "req_mark"
    assert outbound[-1]["sub_id"] == 3
    assert '"map" passed to lookupTransform' in outbound[-1]["error_message"]


def test_update_poi_list_renames_existing_poi_without_touching_geometry(
    tmp_path: Path,
):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_006",
        "旧名字",
        PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 100.0),
        PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 100.0),
        "session_a",
    )
    core = NavigationExecutorCore(
        bridge=FakeBridge(),
        pose_provider=FakePoseProvider(
            CapturedPose(
                map_pose=PoseRecord("map", 0.0, 0.0, 0.0, 0.0, 100.0),
                odom_pose=PoseRecord("odom", 0.0, 0.0, 0.0, 0.0, 100.0),
                slam_session_id="session_a",
            )
        ),
        poi_store=store,
    )

    async def run_scenario():
        return await core.handle_message(
            {
                "action": "update_poi_list",
                "version": 3,
                "poi_list": [{"id": "POI_006", "name": "会议室门口"}],
            }
        )

    outbound = asyncio.run(run_scenario())
    stored = store.get_required("POI_006")

    assert outbound == []
    assert stored.name == "会议室门口"
    assert stored.map_pose.x == 12.3


def test_error_event_returns_executor_to_idle_and_clears_active_task(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
        "session_a",
    )
    bridge = FakeBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_1",
                "sub_id": 1,
                "target_id": "POI_001",
            }
        )
        outbound = await core.handle_bridge_event(
            {
                "kind": "error",
                "request_id": "req_1",
                "sub_id": 1,
                "error_message": "Nav2 finished with status 6",
            }
        )
        return outbound, core.state, core.active_task

    outbound, state_after_error, active_task_after_error = asyncio.run(run_scenario())

    assert outbound == [
        {
            "event_type": "on_error",
            "request_id": "req_1",
            "sub_id": 1,
            "error_message": "Nav2 finished with status 6",
        }
    ]
    assert state_after_error == ExecutorState.IDLE
    assert active_task_after_error is None
