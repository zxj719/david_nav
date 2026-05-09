from pathlib import Path

from g1_nav.navigation_types import PoseRecord, Transform2D
from g1_nav.poi_store import LocalPoiStore


def test_apply_semantic_directory_keeps_existing_geometry(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        poi_id="POI_006",
        name="旧名字",
        map_pose=PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 100.0),
        odom_pose=PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 100.0),
        slam_session_id="session_a",
    )

    store.apply_semantic_directory([{"id": "POI_006", "name": "会议室门口"}])
    stored = store.get_required("POI_006")

    assert stored.name == "会议室门口"
    assert stored.map_pose.x == 12.3
    assert stored.odom_pose.x == 5.0


def test_reproject_session_updates_map_pose_from_odom_anchor(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        poi_id="POI_006",
        name="会议室门口",
        map_pose=PoseRecord("map", 0.0, 0.0, 0.0, 0.0, 100.0),
        odom_pose=PoseRecord("odom", 1.0, 0.0, 0.0, 0.2, 100.0),
        slam_session_id="session_a",
    )

    changed = store.reproject_session(
        slam_session_id="session_a",
        map_from_odom=Transform2D(x=10.0, y=2.0, yaw=1.0),
        translation_threshold=0.0,
        yaw_threshold=0.0,
    )

    assert [poi.poi_id for poi in changed] == ["POI_006"]
    updated = store.get_required("POI_006")
    assert round(updated.map_pose.x, 3) == 10.54
    assert round(updated.map_pose.y, 3) == 2.841
    assert round(updated.map_pose.yaw, 3) == 1.2
