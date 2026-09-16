"""Legacy-compatible YAML POI persistence without ROS or transport dependencies."""

from dataclasses import asdict, replace
import math
import os
from pathlib import Path
from typing import Iterable

import yaml

from nav_tasks.domain.types import PoseRecord, StoredPoi, Transform2D


class YamlPoiRepository:
    """Single-writer repository preserving legacy file shape and merge order.

    The optional source is read-only; the writable file overrides matching IDs.
    Writes replace the destination atomically after closing a sibling temporary
    file. As in the old store, mutations precede persistence: write failure can
    leave memory ahead of disk. This is not a multi-process transaction manager.

    Attributes:
        path: POI 缓存 YAML 文件路径。
        source_path: 可选的只读初始 POI YAML 文件路径。
    """

    def __init__(
        self, path: Path, source_path: Path | None = None, *, map_frame: str,
    ) -> None:
        """Select paths and the configured output/fallback global frame.

        Args:
            path: Executor-owned writable YAML file, never the manual source file.
            source_path: Optional read-only initial YAML data.
            map_frame: Explicit reprojection target and legacy missing-frame fallback.
        """
        self.path = Path(path)
        self.source_path = Path(source_path) if source_path is not None else None
        self._map_frame = map_frame
        self._pois: dict[str, StoredPoi] = {}

    def load(self) -> None:
        """Reload source then local records; missing files are ignored.

        Invalid record structure retains the old parser's exceptions/skips; this
        migration does not silently repair or rewrite user data.
        """
        self._pois = {}
        if self.source_path is not None:
            self._load_from(self.source_path)
        self._load_from(self.path)

    def _load_from(self, path: Path) -> None:
        if not path.exists():
            return
        with path.open('r', encoding='utf-8') as handle:
            payload = yaml.safe_load(handle) or {}
        items = payload.get('pois', []) if isinstance(payload, dict) else payload
        if not isinstance(items, list):
            return
        for item in items:
            if not isinstance(item, dict):
                continue
            stored = self._stored_poi_from_mapping(item)
            if stored is not None:
                self._pois[stored.poi_id] = stored

    def get(self, poi_id: str) -> StoredPoi | None:
        """Read a stored immutable value without creating missing records.

        Args:
            poi_id: POI identifier.

        Returns:
            Stored POI or None.
        """
        return self._pois.get(poi_id)

    def upsert(self, poi: StoredPoi) -> StoredPoi:
        """Insert or replace a POI and atomically replace the YAML file.

        Args:
            poi: Immutable value to persist.

        Returns:
            The supplied stored value.
        """
        self._pois[poi.poi_id] = poi
        self._write()
        return poi

    def rename_many(self, id_to_name: Iterable[tuple[str, str]]) -> None:
        """Rename known IDs without changing geometry, order or creating records.

        Args:
            id_to_name: Identifier/name pairs in input order.
        """
        for poi_id, name in id_to_name:
            if poi_id in self._pois:
                self._pois[poi_id] = replace(self._pois[poi_id], name=name)
        self._write()

    def prune_session(self, session_id: str, keep_ids: set[str]) -> list[str]:
        """Remove stale records only in the specified authoritative session.

        Args:
            session_id: Session to prune.
            keep_ids: IDs still present in its source.

        Returns:
            Removed IDs in existing repository order.
        """
        removed = [key for key, poi in self._pois.items()
                   if poi.slam_session_id == session_id and key not in keep_ids]
        for key in removed:
            del self._pois[key]
        if removed:
            self._write()
        return removed

    def reproject_session(
        self, session_id: str, map_from_odom: Transform2D,
        translation_threshold_m: float, yaw_threshold_rad: float,
    ) -> list[StoredPoi]:
        """Reproject matching odometry anchors, preserving legacy unwrapped yaw.

        Args:
            session_id: Session whose measured anchors may move.
            map_from_odom: Transform to the configured global frame.
            translation_threshold_m: Inclusive translation threshold for no change.
            yaw_threshold_rad: Inclusive raw yaw-difference threshold for no change.

        Returns:
            Updated records in repository order.
        """
        changed = []
        for key, poi in list(self._pois.items()):
            if poi.slam_session_id != session_id:
                continue
            pose = self._reproject_pose(map_from_odom, poi.odom_pose)
            dx, dy = pose.x - poi.map_pose.x, pose.y - poi.map_pose.y
            dyaw = pose.yaw - poi.map_pose.yaw
            if math.hypot(dx, dy) <= translation_threshold_m and abs(dyaw) <= yaw_threshold_rad:
                continue
            updated = replace(poi, map_pose=pose)
            self._pois[key] = updated
            changed.append(updated)
        if changed:
            self._write()
        return changed

    def _reproject_pose(self, transform: Transform2D, pose: PoseRecord) -> PoseRecord:
        # Do not use the normalized geometry helper: existing files and threshold
        # decisions retain yaw outside [-pi, pi). See site-data.md.
        cosine, sine = math.cos(transform.yaw), math.sin(transform.yaw)
        return PoseRecord(
            self._map_frame, transform.x + cosine * pose.x - sine * pose.y,
            transform.y + sine * pose.x + cosine * pose.y, pose.z,
            transform.yaw + pose.yaw, pose.stamp_sec)

    def _write(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        payload = {'pois': [
            {'id': poi.poi_id, 'name': poi.name, 'map_pose': asdict(poi.map_pose),
             'odom_pose': asdict(poi.odom_pose), 'slam_session_id': poi.slam_session_id}
            for poi in self._pois.values()
        ]}
        with temporary.open('w', encoding='utf-8') as handle:
            yaml.safe_dump(payload, handle, allow_unicode=True, sort_keys=False)
        os.replace(temporary, self.path)

    def _stored_poi_from_mapping(self, item: dict) -> StoredPoi | None:
        poi_id = item.get('id')
        if poi_id is None:
            return None
        return StoredPoi(
            str(poi_id), str(item.get('name') or poi_id),
            self._pose_from_mapping(item.get('map_pose') or item),
            self._pose_from_mapping(item.get('odom_pose') or item),
            str(item.get('slam_session_id') or 'semantic_map'))

    def _pose_from_mapping(self, item: dict) -> PoseRecord:
        position = item.get('position') or {}
        return PoseRecord(
            str(item.get('frame_id') or self._map_frame),
            float(item.get('x', position.get('x', 0.0))),
            float(item.get('y', position.get('y', 0.0))),
            float(item.get('z', position.get('z', 0.0))),
            float(item.get('yaw', 0.0)), float(item.get('stamp_sec', 0.0)))
