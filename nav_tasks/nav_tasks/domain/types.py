"""Immutable navigation value objects independent of ROS and storage."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseRecord:
    """Timestamped planar orientation and three-dimensional position in a named frame.

    Attributes:
        frame_id: 位姿所属坐标系。
        stamp_sec: 来源位姿的时间戳（秒），不是单调计时值。
        x: 该坐标系的 x 位置（米）。
        y: 该坐标系的 y 位置（米）。
        yaw: 平面偏航角（弧度）。
        z: 该坐标系的 z 位置（米）。
    """

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
    """Named POI retaining both coordinate representations and its SLAM session.

    Attributes:
        map_pose: 保留的地图坐标表示。
        name: 点位显示名称。
        odom_pose: 保留的里程计坐标表示。
        poi_id: 点位标识。
        slam_session_id: 该点位关联的定位会话标识。
    """

    poi_id: str
    name: str
    map_pose: PoseRecord
    odom_pose: PoseRecord
    slam_session_id: str


@dataclass(frozen=True)
class Transform2D:
    """Planar translation and yaw defining a coordinate transformation.

    Attributes:
        x: 平面变换的 x 平移（米）。
        y: 平面变换的 y 平移（米）。
        yaw: 平面变换的偏航角（弧度）。
    """

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CapturedPose:
    """Map/odometry capture and session identity used when marking a POI.

    Attributes:
        map_pose: 捕获时的地图坐标位姿。
        odom_pose: 捕获时的里程计坐标位姿。
        slam_session_id: 捕获时关联的定位会话标识。
    """

    map_pose: PoseRecord
    odom_pose: PoseRecord
    slam_session_id: str
