"""精准入位 POI 的路由（alias 0，2026-09-15）。

不起 ROS：用记录型假 bridge / 假 resolver 验证 DockRoutingBridge 与
DockAwareGoalResolver，并按文件路径加载 navigation_executor.py 验证新 CLI 参数。
"""
import asyncio
from legacy_entry_reference import load_entry

from david_nav.dock_routing import DockAwareGoalResolver, DockRoutingBridge
from david_nav.navigation_executor_core import ResolvedNavigationGoal
from david_nav.navigation_types import PoseRecord, StoredPoi


def _poi(poi_id):
    pose = PoseRecord("map", 14.24, -4.25, 0.0, -0.09, 100.0)
    return StoredPoi(
        poi_id=poi_id,
        name=poi_id,
        map_pose=pose,
        odom_pose=pose,
        slam_session_id="named_poses",
    )


class RecordingBridge:
    def __init__(self):
        self.started = []
        self.cancels = 0

    async def start_navigation(self, request_id, sub_id, poi):
        self.started.append((request_id, sub_id, poi.poi_id))

    async def cancel_current(self):
        self.cancels += 1


def test_dock_pois_go_to_the_dock_bridge_and_others_to_nav2():
    default, dock = RecordingBridge(), RecordingBridge()
    routing = DockRoutingBridge(default, dock, ["POI_017"])

    async def scenario():
        await routing.start_navigation("r1", 1, _poi("POI_017"))
        await routing.cancel_current()
        await routing.start_navigation("r2", 2, _poi("POI_001"))
        await routing.cancel_current()

    asyncio.run(scenario())

    assert dock.started == [("r1", 1, "POI_017")]
    assert default.started == [("r2", 2, "POI_001")]
    assert (dock.cancels, default.cancels) == (1, 1)


def test_cancel_before_any_navigation_is_a_noop():
    default, dock = RecordingBridge(), RecordingBridge()

    asyncio.run(DockRoutingBridge(default, dock, ["POI_017"]).cancel_current())

    assert (dock.cancels, default.cancels) == (0, 0)


class SnappingResolver:
    def __init__(self):
        self.resolved = []

    def resolve_initial_goal(self, poi):
        self.resolved.append(poi.poi_id)
        return ResolvedNavigationGoal(poi=_poi("SNAPPED"), used_snap=True)

    def is_anchor_admissible(self, poi):
        return False

    def is_anchor_static_blocked(self, poi):
        return True


def test_dock_pois_are_never_snapped():
    inner = SnappingResolver()
    resolver = DockAwareGoalResolver(inner, ["POI_017"])

    dock_goal = resolver.resolve_initial_goal(_poi("POI_017"))
    other_goal = resolver.resolve_initial_goal(_poi("POI_001"))

    assert (dock_goal.poi.poi_id, dock_goal.used_snap) == ("POI_017", False)
    assert (other_goal.poi.poi_id, other_goal.used_snap) == ("SNAPPED", True)
    assert inner.resolved == ["POI_001"]
    assert resolver.is_anchor_admissible(_poi("POI_001")) is False
    assert resolver.is_anchor_static_blocked(_poi("POI_001")) is True


def _load_executor_script():
    return load_entry('navigation_executor')


def test_executor_cli_parses_dock_options():
    module = _load_executor_script()

    config, ros_args = module._parse_cli_args([
        "--dock-poi-id", "POI_017",
        "--dock-poi-id", "POI_020",
        "--dock-poi-id", "POI_017",
        "--dock-action-name", "/custom_dock",
        "--ros-args",
    ])

    assert config["dock_poi_ids"] == ["POI_017", "POI_020"]
    assert config["dock_action_name"] == "/custom_dock"
    assert ros_args == ["--ros-args"]


def test_executor_cli_has_no_dock_pois_by_default():
    config, _ = _load_executor_script()._parse_cli_args([])

    assert config["dock_poi_ids"] == []
    assert config["dock_action_name"] == "/dock_to_pose"
