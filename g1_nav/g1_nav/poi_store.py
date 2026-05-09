from __future__ import annotations

from dataclasses import replace
import math
import os
from pathlib import Path

import yaml

from g1_nav.navigation_types import PoseRecord, StoredPoi, Transform2D


def _transform_pose(transform: Transform2D, pose: PoseRecord) -> PoseRecord:
    cos_yaw = math.cos(transform.yaw)
    sin_yaw = math.sin(transform.yaw)
    x = transform.x + cos_yaw * pose.x - sin_yaw * pose.y
    y = transform.y + sin_yaw * pose.x + cos_yaw * pose.y
    return PoseRecord("map", x, y, pose.z, transform.yaw + pose.yaw, pose.stamp_sec)


class LocalPoiStore:
    def __init__(self, path: Path):
        self.path = Path(path)
        self._pois: dict[str, StoredPoi] = {}

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
