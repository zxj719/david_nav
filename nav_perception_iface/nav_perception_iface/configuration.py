"""点云转扫描的档案读取与参数映射；不依赖 ROS."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import yaml

SCAN_NUMBERS = (
    'transform_tolerance', 'min_height', 'max_height', 'angle_min', 'angle_max',
    'angle_increment', 'scan_time', 'range_min', 'range_max')


@dataclass(frozen=True)
class ScanConfiguration:
    """一个点云转扫描节点的全部接线及参数.

    Attributes:
        enabled: 是否启动；默认关闭，不增加常驻进程。
        cloud_topic: PointCloud2 输入话题。
        scan_topic: LaserScan 输出话题。
        parameters: 原样传给 pointcloud_to_laserscan 的参数。
    """

    enabled: bool
    cloud_topic: str
    scan_topic: str
    parameters: Mapping[str, Any]


def profile_value(profile: Mapping[str, Any], key: str) -> Any:
    """读取一个点分隔档案键，不使用机器人相关默认值.

    Args:
        profile: 完整或模式合并后的机器人档案。
        key: 点分隔键名。

    Returns:
        档案中的值。

    Raises:
        ValueError: 键缺失或父节点不是映射。
    """
    value = profile
    for part in key.split('.'):
        if not isinstance(value, Mapping) or part not in value:
            raise ValueError(f'missing profile key: {key}')
        value = value[part]
    return value


def string_value(profile: Mapping[str, Any], key: str) -> str:
    """读取非空的话题或坐标系名称.

    Args:
        profile: 机器人档案。
        key: 点分隔键名。

    Returns:
        非空字符串，保持原文不重写命名空间。

    Raises:
        ValueError: 缺失或不是非空字符串。
    """
    value = profile_value(profile, key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{key}: expected a non-empty string')
    return value


def scan_configuration(profile: Mapping[str, Any]) -> ScanConfiguration:
    """从有效档案构造点云节点参数，保留原节点所有数值与类型.

    Args:
        profile: 模式合并后的机器人档案。

    Returns:
        可供 launch 使用的接线与参数。

    Raises:
        ValueError: 必要键、类型或取值区间无效。
    """
    section = profile_value(profile, 'sensors.scan_from_cloud')
    if not isinstance(section, Mapping):
        raise ValueError('sensors.scan_from_cloud: expected a mapping')
    enabled = section.get('enabled', False)
    if not isinstance(enabled, bool) or not isinstance(section.get('use_inf'), bool):
        raise ValueError('scan enabled and use_inf must be booleans')
    parameters: Dict[str, Any] = {
        'target_frame': string_value(profile, 'frames.scan_target'),
        'use_inf': section['use_inf'],
    }
    for key in SCAN_NUMBERS:
        value = section.get(key)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            raise ValueError(f'sensors.scan_from_cloud.{key}: expected a finite number')
        parameters[key] = float(value)
    for lower, upper in (
        ('min_height', 'max_height'), ('angle_min', 'angle_max'), ('range_min', 'range_max'),
    ):
        if parameters[lower] >= parameters[upper]:
            raise ValueError(f'scan {lower} must be less than {upper}')
    if (parameters['transform_tolerance'] < 0 or parameters['range_min'] < 0
            or parameters['angle_increment'] <= 0 or parameters['scan_time'] <= 0):
        raise ValueError('scan time/increment must be positive; tolerance/range non-negative')
    return ScanConfiguration(
        enabled,
        string_value(profile, 'sensors.scan_from_cloud.cloud_topic'),
        string_value(profile, 'sensors.scan_from_cloud.scan_topic'), parameters)


def load_scan_configuration(
    path: Path, enabled_override: Optional[bool] = None,
) -> ScanConfiguration:
    """读取有效档案，兼容旧入口显式打开或关闭扫描桥的参数.

    Args:
        path: 机器人档案或 navrun 写入运行目录的有效档案。
        enabled_override: None 使用档案值，否则覆盖启用状态。

    Returns:
        不修改输入文件的扫描配置。

    Raises:
        ValueError: YAML、档案结构或覆盖值无效。
    """
    try:
        profile = yaml.safe_load(path.read_text(encoding='utf-8'))
    except yaml.YAMLError as error:
        raise ValueError(f'invalid profile YAML: {error}') from error
    config = scan_configuration(profile)
    if enabled_override is not None and not isinstance(enabled_override, bool):
        raise ValueError('enabled_override must be bool or None')
    return ScanConfiguration(
        config.enabled if enabled_override is None else enabled_override,
        config.cloud_topic, config.scan_topic, config.parameters)
