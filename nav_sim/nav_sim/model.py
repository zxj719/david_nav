"""提供不依赖 ROS 的平面模型，保留旧假底盘的显式欧拉积分规则."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Tuple


@dataclass(frozen=True)
class PlanarPose:
    """保存配置里程计帧下的平面位姿；构造时不自行检查数值.

    Attributes:
        x: x 位置，单位米。
        y: y 位置，单位米。
        yaw: 航向，单位弧度。
    """

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def finite_number(value: float, name: str) -> float:
    """检查数值有限并转换为浮点数，拒绝把布尔值当作数值.

    Args:
        value: int 或 float 数值，包括其非布尔子类。
        name: 错误诊断中使用的参数名。

    Returns:
        有限浮点数。

    Raises:
        ValueError: 输入类型不符合要求或数值非有限。
        OverflowError: 超大整数无法供浮点有限性检查或转换使用。
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def integrate(pose: PlanarPose, linear: float, angular: float, dt: float) -> PlanarPose:
    """按旧假底盘规则执行一步显式欧拉积分，以旧航向计算平移.

    输入有限性不保证算术中间值或输出不会溢出；不额外检查输出有限性。

    Args:
        pose: 前一位姿，位置为米，航向为弧度。
        linear: 前向速度，单位米/秒；不积分侧向运动。
        angular: 航向角速度，单位弧度/秒。
        dt: 非负固定步长，单位秒，不是自动测得的实际经过时间。

    Returns:
        新的冻结位姿，航向通过 atan2(sin, cos) 归一化。

    Raises:
        ValueError: 输入类型或有限性检查失败、步长为负，或溢出的航向无法求三角函数。
        OverflowError: 超大整数在输入数值检查中无法转换。
    """
    x, y, yaw, linear, angular, dt = (
        finite_number(value, name) for name, value in (
            ('x', pose.x), ('y', pose.y), ('yaw', pose.yaw),
            ('linear', linear), ('angular', angular), ('dt', dt)))
    if dt < 0:
        raise ValueError('dt must be nonnegative')
    heading = yaw + angular * dt
    return PlanarPose(x + linear * math.cos(yaw) * dt,
                      y + linear * math.sin(yaw) * dt,
                      math.atan2(math.sin(heading), math.cos(heading)))


def quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """由平面航向生成单位四元数，按 x、y、z、w 顺序返回.

    Args:
        yaw: 有限航向角，单位弧度。

    Returns:
        x、y 为零的平面旋转四元数四元组。

    Raises:
        ValueError: yaw 不是接受的有限数值。
        OverflowError: 超大整数无法转换为浮点数。
    """
    yaw = finite_number(yaw, 'yaw')
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def require_sandbox(environment: Mapping[str, str]) -> None:
    """拒绝未显式选择保留域 99 的仿真入口，不授予任何运动权限.

    Args:
        environment: 进程环境变量映射。

    Raises:
        RuntimeError: ROS_DOMAIN_ID 的字符串值不严格等于 '99'。
    """
    if environment.get('ROS_DOMAIN_ID') != '99':
        raise RuntimeError('nav_sim requires explicit ROS_DOMAIN_ID=99')


def frame_name(value: str, parameter: str) -> str:
    """检查必填帧名的基本形式，不补默认值或执行完整 TF 语法验证.

    Args:
        value: 机器人配置提供的帧名。
        parameter: 错误诊断中使用的参数名。

    Returns:
        原样返回通过检查的帧名。

    Raises:
        ValueError: 不是字符串、为空、以斜杠开头或包含空白字符。
    """
    if (not isinstance(value, str) or not value or value.startswith('/')
            or any(char.isspace() for char in value)):
        raise ValueError(f'{parameter} must be a nonempty relative TF frame')
    return value


def localization_frames(map_frame: str, odom_frame: str, body_frame: str) -> Tuple[str, ...]:
    """校验假定位自身两条动态边的帧名及互异性，不证明全局 TF 图无环.

    Args:
        map_frame: 全局固定帧。
        odom_frame: 积分里程计帧。
        body_frame: 机器人机身帧，不是导航旋转中心帧。

    Returns:
        已检查的 map、odom、机身帧名三元组。

    Raises:
        ValueError: 帧名形式不合法或三者不互异，导致本地自边或环。
    """
    frames = tuple(frame_name(value, name) for name, value in (
        ('map_frame', map_frame), ('odom_frame', odom_frame), ('body_frame', body_frame)))
    if len(set(frames)) != 3:
        raise ValueError('map_frame, odom_frame and body_frame must be distinct')
    return frames
