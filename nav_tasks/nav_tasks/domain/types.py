"""定义不依赖 ROS 或存储的冻结导航值对象.

构造器保存字段而不验证数值、坐标系或会话一致性；这些约束由各入口负责。
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseRecord:
    """保存命名坐标系中的三维位置、平面朝向和来源时间戳.

    构造时不验证有限性、帧名或时间新鲜度，也不执行坐标变换。

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
        """直接把保存的 yaw 转换为旧 x/y/z/w 顺序的四元数字典.

        不折返角度、不增加有限性检查；与 geometry.quaternion_from_yaw 的前置
        校验不同。调用方应提供有效数值，NaN 会直接传播到 z/w。

        Returns:
            x/y 为零、z/w 按半角正弦余弦计算的字段映射。

        Raises:
            ValueError: 保存的 yaw 为无穷值，导致三角函数定义域错误。
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
    """保存点位名称、地图/里程计两种位姿表示及定位会话标识.

    数据类本身不证明两个位姿在同一时刻一致，也不验证标识或会话归属。

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
    """保存平面平移和偏航，不携带源/目标坐标系名称.

    调用方负责约定变换方向和单位；构造器不查 TF 或验证几何有限性。

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
    """保存标记 POI 时取得的地图/里程计位姿及会话标识.

    构造器不验证两种表示的时间或坐标一致性，捕获源负责建立相应契约。

    Attributes:
        map_pose: 捕获时的地图坐标位姿。
        odom_pose: 捕获时的里程计坐标位姿。
        slam_session_id: 捕获时关联的定位会话标识。
    """

    map_pose: PoseRecord
    odom_pose: PoseRecord
    slam_session_id: str
