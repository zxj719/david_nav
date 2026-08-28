from pathlib import Path

from david_nav.navigation_types import PoseRecord, Transform2D
from david_nav.poi_store import LocalPoiStore


def test_load_imported_named_pose_store(tmp_path: Path):
    store_path = tmp_path / "poi_store.yaml"
    store_path.write_text(
        """
pois:
- id: POI_001
  name: "1"
  map_pose:
    frame_id: map
    x: -3.4
    y: 0.7
    z: 0.0
    yaw: -1.57
    stamp_sec: 0.0
  odom_pose:
    frame_id: map
    x: -3.4
    y: 0.7
    z: 0.0
    yaw: -1.57
    stamp_sec: 0.0
  slam_session_id: semantic_map
""",
        encoding="utf-8",
    )

    store = LocalPoiStore(store_path)
    store.load()

    stored = store.get_required("POI_001")
    assert stored.name == "1"
    assert stored.map_pose.frame_id == "map"
    assert stored.map_pose.x == -3.4
    assert stored.slam_session_id == "semantic_map"


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


def test_prune_session_removes_stale_ids_but_keeps_other_sessions(tmp_path: Path):
    # named_poses.json 是映射唯一真源:重新导入时清掉该会话里 json 已
    # 不存在的旧 id;其他会话(如 mark_current_poi 的实测点)不受影响。
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    for poi_id in ("POI_001", "POI_002"):
        store.upsert_marked_poi(
            poi_id,
            poi_id,
            PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
            PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
            "named_poses",
        )
    store.upsert_marked_poi(
        "POI_MARKED",
        "实测点",
        PoseRecord("map", 5.0, 5.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 5.0, 5.0, 0.0, 0.0, 100.0),
        "session_live",
    )

    removed = store.prune_session("named_poses", {"POI_001"})

    assert removed == ["POI_002"]
    assert store.get_required("POI_001").poi_id == "POI_001"
    assert store.get_required("POI_MARKED").slam_session_id == "session_live"
    try:
        store.get_required("POI_002")
    except KeyError:
        pass
    else:
        raise AssertionError("POI_002 should be pruned")

    # 删除要落盘:重新 load 后旧 id 不能复活。
    reloaded = LocalPoiStore(tmp_path / "poi_store.yaml")
    reloaded.load()
    try:
        reloaded.get_required("POI_002")
    except KeyError:
        pass
    else:
        raise AssertionError("POI_002 revived after reload")
