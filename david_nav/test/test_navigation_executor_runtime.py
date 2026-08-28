import asyncio
from dataclasses import dataclass
from pathlib import Path

from david_nav.navigation_executor_core import ExecutorState, NavigationExecutorCore
from david_nav.navigation_types import CapturedPose, PoseRecord, StoredPoi
from david_nav.poi_store import LocalPoiStore


class FakeBridge:
    def __init__(self):
        self.started = []
        self.canceled = 0

    async def start_navigation(self, request_id, sub_id, poi):
        self.started.append((request_id, sub_id, poi.poi_id))

    async def cancel_current(self):
        self.canceled += 1


class HangingBridge(FakeBridge):
    """模拟真实 bridge:start_navigation 挂起直到 nav2 goal 终态。

    2026-07-27(nav WS v1.1):真实 Nav2ActionBridge.start_navigation 会
    await 到导航终态才返回。core 若 inline await 它,WS 收包循环整个被卡,
    abort_navigation 根本读不进来 —— 必须后台 task 化。这个 fake 用来验证
    "导航进行中仍能处理 abort"。
    """

    def __init__(self):
        super().__init__()
        self.cancelled_tasks = 0

    async def start_navigation(self, request_id, sub_id, poi):
        self.started.append((request_id, sub_id, poi.poi_id))
        try:
            await asyncio.Event().wait()  # 永不完成,等着被 cancel
        except asyncio.CancelledError:
            self.cancelled_tasks += 1
            raise


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
    def __init__(self, resolved_goal=None, anchor_admissible=True, static_blocked=False):
        self.resolved_goal = resolved_goal
        self.anchor_admissible = anchor_admissible
        self.static_blocked = static_blocked
        self.resolve_calls = []
        self.anchor_checks = []
        self.static_checks = []

    def resolve_initial_goal(self, poi):
        self.resolve_calls.append(poi.poi_id)
        return self.resolved_goal

    def is_anchor_admissible(self, poi):
        self.anchor_checks.append(poi.poi_id)
        return self.anchor_admissible

    def is_anchor_static_blocked(self, poi):
        self.static_checks.append(poi.poi_id)
        return self.static_blocked


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


def test_navigate_to_v2_target_pose_only_returns_error_without_starting(tmp_path: Path):
    # v1(2026-07-22 回退定稿):只带 target_pose 的旧 v2 消息 → on_error,
    # 不启车,状态不动。
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
                "request_id": "nav_ab12cd34ef56",
                "sub_id": 0,
                "target_pose": {"x": 9.52, "y": -8.82, "theta": 0.5},
                "label": "前台",
            }
        )

    outbound = asyncio.run(run_scenario())

    assert bridge.started == []
    assert core.state == ExecutorState.IDLE
    assert core.active_task is None
    assert len(outbound) == 1
    assert outbound[0]["event_type"] == "on_error"
    assert outbound[0]["request_id"] == "nav_ab12cd34ef56"
    assert "target_id" in outbound[0]["error_message"]


def test_navigate_to_with_stray_target_pose_uses_target_id(tmp_path: Path):
    # v1:target_id 与残留的 target_pose 并存时,只认 target_id 查 POI 表。
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_012",
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
        return await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "nav_mix",
                "sub_id": 0,
                "target_id": "POI_012",
                "target_pose": {"x": 99.0, "y": 99.0, "theta": 0.0},
            }
        )

    outbound = asyncio.run(run_scenario())

    assert outbound == []
    assert bridge.started == [("nav_mix", 0, "POI_012")]
    assert core.active_task is not None
    assert core.active_task.anchor_poi.map_pose.x == 1.0


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


def _store_with_poi(tmp_path: Path, *poi_ids: str) -> LocalPoiStore:
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    for poi_id in poi_ids:
        store.upsert_marked_poi(
            poi_id,
            poi_id,
            PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
            PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
            "session_a",
        )
    return store


def test_navigate_to_returns_while_bridge_navigation_still_running(tmp_path: Path):
    # nav WS v1.1:navigate_to 不能阻塞消息循环 —— bridge 还挂着(真实导航
    # 进行中)时 handle_message 必须已经返回,否则 abort 永远排不上队。
    store = _store_with_poi(tmp_path, "POI_001")
    bridge = HangingBridge()
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
                "sub_id": 0,
                "target_id": "POI_001",
            }
        )
        started_snapshot = list(bridge.started)
        state = core.state
        # 收尾:别让挂起任务泄漏出事件循环
        await core.handle_message(
            {"action": "abort_navigation", "request_id": "req_1"}
        )
        return outbound, started_snapshot, state

    outbound, started, state = asyncio.run(run_scenario())

    assert outbound == []
    assert started == [("req_1", 0, "POI_001")]
    assert state == ExecutorState.STARTING


def test_abort_interrupts_inflight_navigation_and_returns_idle(tmp_path: Path):
    # 导航进行中(bridge 挂在 nav2 终态上)收到 abort:后台导航 task 被
    # cancel、nav2 goal 被 cancel、回 paused 确认帧、状态回 IDLE。
    store = _store_with_poi(tmp_path, "POI_001")
    bridge = HangingBridge()
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
                "sub_id": 0,
                "target_id": "POI_001",
            }
        )
        return await core.handle_message(
            {"action": "abort_navigation", "request_id": "req_1"}
        )

    outbound = asyncio.run(run_scenario())

    assert bridge.cancelled_tasks == 1
    assert bridge.canceled == 1
    assert outbound == [
        {
            "event_type": "on_progress",
            "request_id": "req_1",
            "sub_id": 0,
            "remaining_distance": 0.0,
            "status": "paused",
        }
    ]
    assert core.state == ExecutorState.IDLE
    assert core.active_task is None


def test_abort_when_idle_is_idempotent_and_forces_idle(tmp_path: Path):
    # v1.1 幂等要求:空闲/重复/已终态的 abort 不崩、不回帧;任意状态下
    # 中断都收敛回 IDLE。
    store = _store_with_poi(tmp_path)
    bridge = FakeBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        first = await core.handle_message(
            {"action": "abort_navigation", "request_id": "req_ghost"}
        )
        core.state = ExecutorState.ERROR  # 人为制造异常状态
        second = await core.handle_message(
            {"action": "abort_navigation", "request_id": "req_ghost"}
        )
        return first, second

    first, second = asyncio.run(run_scenario())

    assert first == []
    assert second == []
    assert core.state == ExecutorState.IDLE
    assert bridge.canceled == 0


def test_repeated_abort_after_paused_is_noop(tmp_path: Path):
    store = _store_with_poi(tmp_path, "POI_001")
    bridge = HangingBridge()
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
                "sub_id": 0,
                "target_id": "POI_001",
            }
        )
        first = await core.handle_message(
            {"action": "abort_navigation", "request_id": "req_1"}
        )
        second = await core.handle_message(
            {"action": "abort_navigation", "request_id": "req_1"}
        )
        return first, second

    first, second = asyncio.run(run_scenario())

    assert [f["status"] for f in first] == ["paused"]
    assert second == []
    assert bridge.canceled == 1
    assert core.state == ExecutorState.IDLE


def test_abort_with_stale_request_id_keeps_active_navigation(tmp_path: Path):
    # 迟到的旧 abort 不能杀掉新导航(agent 同一时刻只有一个 active request,
    # request_id 不匹配 = 一定是过期帧)。
    store = _store_with_poi(tmp_path, "POI_001")
    bridge = HangingBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        await core.handle_message(
            {
                "action": "navigate_to",
                "request_id": "req_new",
                "sub_id": 0,
                "target_id": "POI_001",
            }
        )
        outbound = await core.handle_message(
            {"action": "abort_navigation", "request_id": "req_old"}
        )
        still_active = core.active_task is not None
        state = core.state
        await core.handle_message(
            {"action": "abort_navigation", "request_id": "req_new"}
        )
        return outbound, still_active, state

    outbound, still_active, state = asyncio.run(run_scenario())

    assert outbound == []
    assert still_active is True
    assert state == ExecutorState.STARTING
    assert bridge.canceled == 1  # 只有正主的 abort 才 cancel


def test_preempt_cancels_previous_inflight_navigation_task(tmp_path: Path):
    # navigate_to 顶掉旧任务:旧的后台导航 task 必须被 cancel(否则它还
    # 挂在旧 goal 的重试/结果上,可能把旧目标重新发出去)。
    store = _store_with_poi(tmp_path, "POI_001", "POI_002")
    bridge = HangingBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(),
        poi_store=store,
    )

    async def run_scenario():
        await core.handle_message(
            {"action": "navigate_to", "request_id": "req_old", "sub_id": 0, "target_id": "POI_001"}
        )
        await core.handle_message(
            {"action": "navigate_to", "request_id": "req_new", "sub_id": 0, "target_id": "POI_002"}
        )
        cancelled_after_preempt = bridge.cancelled_tasks
        await core.handle_message(
            {"action": "abort_navigation", "request_id": "req_new"}
        )
        return cancelled_after_preempt

    cancelled_after_preempt = asyncio.run(run_scenario())

    assert cancelled_after_preempt == 1
    assert bridge.started == [
        ("req_old", 0, "POI_001"),
        ("req_new", 0, "POI_002"),
    ]


def test_stale_bridge_event_from_preempted_goal_is_dropped(tmp_path: Path):
    # navigate_to 顶掉旧任务后,旧 goal 的 canceled 结果(paused progress)
    # 会晚到;不过滤会以新 request_id 发 paused,被 agent 当成新导航被取消
    # (v1 §五:paused 只能跟在 abort 之后)。
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    for poi_id in ("POI_001", "POI_002"):
        store.upsert_marked_poi(
            poi_id,
            poi_id,
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
            {"action": "navigate_to", "request_id": "req_old", "sub_id": 0, "target_id": "POI_001"}
        )
        await core.handle_message(
            {"action": "navigate_to", "request_id": "req_new", "sub_id": 0, "target_id": "POI_002"}
        )
        return await core.handle_bridge_event(
            {
                "kind": "progress",
                "request_id": "req_old",
                "sub_id": 0,
                "remaining_distance": 2.0,
                "status": "paused",
            }
        )

    outbound = asyncio.run(run_scenario())

    assert bridge.canceled == 1
    assert outbound == []
    assert core.active_task is not None
    assert core.active_task.request_id == "req_new"


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


def test_snap_arrival_succeeds_when_anchor_remains_unreachable(tmp_path: Path):
    """2026-07-27：到达扩圈可行点即判导航成功，无论 anchor 是静态占用还是
    costmap 动态占用（旧语义的"仍不可达"报错只会让服务端原地重试同样失败）。"""
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
            "event_type": "on_arrived",
            "request_id": "req_1",
            "sub_id": 1,
        }
    ]
    assert bridge.started == [("req_1", 1, "POI_001")]
    assert resolver.anchor_checks == ["POI_001"]
    assert state_after_error == ExecutorState.IDLE
    assert active_task is None


def test_snap_arrival_succeeds_when_anchor_blocked_by_static_map(tmp_path: Path):
    """2026-07-24：POI 压在静态地图障碍里（永久不可达）时，
    到达扩圈可行点应判导航成功(on_arrived)，而不是"仍不可达"报错。"""
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
        static_blocked=True,
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

    outbound, state_after_arrival, active_task = asyncio.run(run_scenario())

    assert outbound == [
        {
            "event_type": "on_arrived",
            "request_id": "req_1",
            "sub_id": 1,
        }
    ]
    assert bridge.started == [("req_1", 1, "POI_001")]
    assert resolver.anchor_checks == ["POI_001"]
    # 2026-07-27：到点即成功不再区分静态/动态占用，static 检查不再被调用
    assert resolver.static_checks == []
    assert state_after_arrival == ExecutorState.IDLE
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
