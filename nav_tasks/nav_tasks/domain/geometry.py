"""提供不依赖 ROS 的平面几何运算，坐标系方向由调用方显式约定."""

import math

from nav_tasks.domain.types import PoseRecord, Transform2D


def wrap_to_pi(angle: float) -> float:
    """把有限弧度角按既有浮点取模公式折返到 [-pi, pi).

    Args:
        angle: 有限弧度角；不在此额外执行严格数值类型检查。

    Returns:
        折返后的弧度角；正负 pi 边界都映射为负 pi。

    Raises:
        ValueError: 输入角度非有限。
    """
    if not math.isfinite(angle):
        raise ValueError('angle must be finite')
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """先缩放并归一化非零四元数，再提取平面偏航角.

    最大绝对分量缩放避免直接平方原始大数；该函数与兼容迁移用的
    legacy_yaw_from_quaternion 不可无条件互换。

    Args:
        x: 四元数 x 分量。
        y: 四元数 y 分量。
        z: 四元数 z 分量。
        w: 四元数 w 分量。

    Returns:
        [-pi, pi) 内的偏航角，单位弧度。

    Raises:
        ValueError: 分量非有限或四个分量全部为零。
    """
    components = (x, y, z, w)
    if not all(math.isfinite(value) for value in components):
        raise ValueError('quaternion must be finite')
    scale = max(abs(value) for value in components)
    if scale == 0.0:
        raise ValueError('quaternion must be nonzero')
    scaled = tuple(value / scale for value in components)
    norm = math.sqrt(sum(value * value for value in scaled))
    x, y, z, w = (value / norm for value in scaled)
    return wrap_to_pi(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def quaternion_from_yaw(yaw: float) -> dict[str, float]:
    """按旧 x/y/z/w 字段顺序生成平面四元数字典.

    Args:
        yaw: 有限偏航角，单位弧度；转换前不折返角度。

    Returns:
        对有限输入与 PoseRecord.orientation 计算一致的字段映射。
        后者不执行本函数的有限性前置检查。

    Raises:
        ValueError: 偏航角非有限。
    """
    if not math.isfinite(yaw):
        raise ValueError('yaw must be finite')
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw * 0.5), 'w': math.cos(yaw * 0.5)}


def legacy_yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """保留旧位姿来源的四元数算式，维持线协议和存储数值兼容.

    与 yaw_from_quaternion 不同，不做归一化、零值或有限性检查，也不折返正 pi。
    新几何逻辑通常应使用归一化版本；迁移中的 POI 即使只有微小舍入变化，
    也会改变未舍入的 JSON 字段，替换需另行决定。

    Args:
        x: 旧四元数 x 分量。
        y: 旧四元数 y 分量。
        z: 旧四元数 z 分量。
        w: 旧四元数 w 分量。

    Returns:
        按旧算式直接求得的 atan2 结果，不保证输入有效或结果有限。
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_pose(tf: Transform2D, pose: PoseRecord, *, target_frame: str) -> PoseRecord:
    """按调用方给定的源到目标平面变换生成新位姿.

    Transform2D 不携带帧名，本函数不查询 TF，也不核对变换与 pose.frame_id
    是否匹配。只检查目标名非空和输入几何有限；时间戳原样保留，输出 x/y
    没有再次做有限性检查，有限大数参与运算仍可能溢出。

    Args:
        tf: 将源系坐标映射到目标系的平移和旋转。
        pose: 源系位姿；高度和时间戳保持不变。
        target_frame: 显式指定的新位姿帧名，不在此完整验证 TF 命名规则。

    Returns:
        x/y 经平面变换、yaw 折返后的新位姿。

    Raises:
        ValueError: 目标名为空、输入几何非有限，或相加后的 yaw 非有限。
    """
    if not target_frame:
        raise ValueError('target_frame must be nonempty')
    if not all(math.isfinite(value) for value in (
            tf.x, tf.y, tf.yaw, pose.x, pose.y, pose.z, pose.yaw)):
        raise ValueError('transform and pose geometry must be finite')
    cosine, sine = math.cos(tf.yaw), math.sin(tf.yaw)
    return PoseRecord(
        target_frame, tf.x + cosine * pose.x - sine * pose.y,
        tf.y + sine * pose.x + cosine * pose.y, pose.z,
        wrap_to_pi(tf.yaw + pose.yaw), pose.stamp_sec)
