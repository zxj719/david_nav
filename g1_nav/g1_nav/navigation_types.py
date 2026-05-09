from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseRecord:
    frame_id: str
    x: float
    y: float
    z: float
    yaw: float
    stamp_sec: float

    def orientation(self) -> dict[str, float]:
        half = self.yaw * 0.5
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half),
            "w": math.cos(half),
        }


@dataclass(frozen=True)
class StoredPoi:
    poi_id: str
    name: str
    map_pose: PoseRecord
    odom_pose: PoseRecord
    slam_session_id: str


@dataclass(frozen=True)
class Transform2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CapturedPose:
    map_pose: PoseRecord
    odom_pose: PoseRecord
    slam_session_id: str
