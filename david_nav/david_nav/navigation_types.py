"""Immutable navigation value objects independent of ROS and storage."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseRecord:
    """Timestamped planar orientation and three-dimensional position in a named frame."""

    frame_id: str
    x: float
    y: float
    z: float
    yaw: float
    stamp_sec: float

    def orientation(self) -> dict[str, float]:
        """Convert the stored yaw to an x/y/z/w quaternion mapping.

        Returns:
            Quaternion fields in the legacy serialization order.
        """
        half = self.yaw * 0.5
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half),
            "w": math.cos(half),
        }


@dataclass(frozen=True)
class StoredPoi:
    """Named POI retaining both coordinate representations and its SLAM session."""

    poi_id: str
    name: str
    map_pose: PoseRecord
    odom_pose: PoseRecord
    slam_session_id: str


@dataclass(frozen=True)
class Transform2D:
    """Planar translation and yaw defining a coordinate transformation."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CapturedPose:
    """Map/odometry capture and session identity used when marking a POI."""

    map_pose: PoseRecord
    odom_pose: PoseRecord
    slam_session_id: str
