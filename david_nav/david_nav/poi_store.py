from __future__ import annotations

from dataclasses import replace
import math
import os
from pathlib import Path

import yaml

from david_nav.navigation_types import PoseRecord, StoredPoi, Transform2D


def _transform_pose(transform: Transform2D, pose: PoseRecord) -> PoseRecord:
    cos_yaw = math.cos(transform.yaw)
    sin_yaw = math.sin(transform.yaw)
    x = transform.x + cos_yaw * pose.x - sin_yaw * pose.y
    y = transform.y + sin_yaw * pose.x + cos_yaw * pose.y
    return PoseRecord("map", x, y, pose.z, transform.yaw + pose.yaw, pose.stamp_sec)


class LocalPoiStore:
    def __init__(self, path: Path, source_path: Path | None = None):
        self.path = Path(path)
        self.source_path = Path(source_path) if source_path is not None else None
        self._pois: dict[str, StoredPoi] = {}

    def load(self) -> None:
        self._pois = {}
        if self.source_path is not None:
            self._load_from(self.source_path)
        self._load_from(self.path)

    def _load_from(self, path: Path) -> None:
        if not path.exists():
            return

        with path.open("r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}

        items = payload.get("pois", []) if isinstance(payload, dict) else payload
        if not isinstance(items, list):
            return

        for item in items:
            if not isinstance(item, dict):
                continue
            stored = self._stored_poi_from_mapping(item)
            if stored is not None:
                self._pois[stored.poi_id] = stored

    def get_required(self, poi_id: str) -> StoredPoi:
        return self._pois[poi_id]

    def upsert_marked_poi(
        self,
        poi_id: str,
        name: str,
        map_pose: PoseRecord,
        odom_pose: PoseRecord,
        slam_session_id: str,
    ) -> StoredPoi:
        stored = StoredPoi(poi_id, name, map_pose, odom_pose, slam_session_id)
        self._pois[poi_id] = stored
        self._write()
        return stored

    def prune_session(self, slam_session_id: str, keep_ids: set[str]) -> list[str]:
        """删掉某会话中不在 keep_ids 里的点位,返回被删的 id 列表。

        named_poses.json 是点位映射的唯一真源(nav WS v1 §四.1):启动导入
        时用它清理上次导入残留的旧 id,否则 json 里已删/已改的点位会一直
        躺在持久化 store 里,agent 发旧 id 时会导航到过期坐标而不是报错。
        """
        removed = [
            poi_id
            for poi_id, poi in self._pois.items()
            if poi.slam_session_id == slam_session_id and poi_id not in keep_ids
        ]
        for poi_id in removed:
            del self._pois[poi_id]
        if removed:
            self._write()
        return removed

    def apply_semantic_directory(self, semantic_pois: list[dict]) -> None:
        for item in semantic_pois:
            poi_id = str(item["id"])
            if poi_id in self._pois:
                self._pois[poi_id] = replace(
                    self._pois[poi_id],
                    name=str(item["name"]),
                )
        self._write()

    def reproject_session(
        self,
        slam_session_id: str,
        map_from_odom: Transform2D,
        translation_threshold: float,
        yaw_threshold: float,
    ) -> list[StoredPoi]:
        changed: list[StoredPoi] = []
        for poi_id, poi in list(self._pois.items()):
            if poi.slam_session_id != slam_session_id:
                continue
            next_map_pose = _transform_pose(map_from_odom, poi.odom_pose)
            dx = next_map_pose.x - poi.map_pose.x
            dy = next_map_pose.y - poi.map_pose.y
            dyaw = next_map_pose.yaw - poi.map_pose.yaw
            if math.hypot(dx, dy) <= translation_threshold and abs(dyaw) <= yaw_threshold:
                continue
            updated = replace(poi, map_pose=next_map_pose)
            self._pois[poi_id] = updated
            changed.append(updated)
        if changed:
            self._write()
        return changed

    def _write(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.path.with_suffix(self.path.suffix + ".tmp")
        payload = {
            "pois": [
                {
                    "id": poi.poi_id,
                    "name": poi.name,
                    "map_pose": {
                        "frame_id": poi.map_pose.frame_id,
                        "x": poi.map_pose.x,
                        "y": poi.map_pose.y,
                        "z": poi.map_pose.z,
                        "yaw": poi.map_pose.yaw,
                        "stamp_sec": poi.map_pose.stamp_sec,
                    },
                    "odom_pose": {
                        "frame_id": poi.odom_pose.frame_id,
                        "x": poi.odom_pose.x,
                        "y": poi.odom_pose.y,
                        "z": poi.odom_pose.z,
                        "yaw": poi.odom_pose.yaw,
                        "stamp_sec": poi.odom_pose.stamp_sec,
                    },
                    "slam_session_id": poi.slam_session_id,
                }
                for poi in self._pois.values()
            ]
        }
        with tmp_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, allow_unicode=True, sort_keys=False)
        os.replace(tmp_path, self.path)

    def _stored_poi_from_mapping(self, item: dict) -> StoredPoi | None:
        poi_id = item.get("id")
        if poi_id is None:
            return None
        name = str(item.get("name") or poi_id)
        map_pose = self._pose_from_mapping(item.get("map_pose") or item)
        odom_pose = self._pose_from_mapping(item.get("odom_pose") or item)
        return StoredPoi(
            poi_id=str(poi_id),
            name=name,
            map_pose=map_pose,
            odom_pose=odom_pose,
            slam_session_id=str(item.get("slam_session_id") or "semantic_map"),
        )

    @staticmethod
    def _pose_from_mapping(item: dict) -> PoseRecord:
        position = item.get("position") or {}
        return PoseRecord(
            frame_id=str(item.get("frame_id") or "map"),
            x=float(item.get("x", position.get("x", 0.0))),
            y=float(item.get("y", position.get("y", 0.0))),
            z=float(item.get("z", position.get("z", 0.0))),
            yaw=float(item.get("yaw", 0.0)),
            stamp_sec=float(item.get("stamp_sec", 0.0)),
        )
