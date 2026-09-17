"""在打开资源或 POI 存储前校验显式 Dock API 值并生成工作线程策略."""

import math

from nav_tasks.domain.errors import ConfigError
from nav_tasks.domain.footprint import parse_footprint
from nav_tasks.domain.runtime_config import DockRuntimeConfig
from nav_tasks.skills.docking.finish import DockTolerance
from nav_tasks.skills.docking.settle import SettleSpec
from nav_tasks.skills.docking.workflow import DockConfig


def workflow_config(config: DockRuntimeConfig) -> DockConfig:
    """校验不可变配置及几何并转换显式策略，不猜测机器人调参.

    帧和端点要求非空字符串；数值要求有限且非负，零仍允许。
    轮廓要求坐标元组组成的元组，再交既有多边形解析器检查。
    构造本身不打开资源；采样周期沿用 SettleSpec 的 0.05 秒默认值。
    解析器或数值转换的非 ValueError 异常不统一包装，例如超大整数的 OverflowError。

    Args:
        config: 由外层配置根提供的完整 DockRuntimeConfig。

    Returns:
        带显式机器人容差、停稳限制与阶段预算的阻塞工作流策略。

    Raises:
        ConfigError: 配置类型、接线、轮廓、容差、次数或预算不符合已有校验。
    """
    if not isinstance(config, DockRuntimeConfig):
        raise ConfigError('dock must be a DockRuntimeConfig')
    for name in ('robot_base_frame', 'global_frame', 'static_map_topic', 'spin_action_name'):
        value = getattr(config, name)
        if not isinstance(value, str) or not value.strip():
            raise ConfigError(f'dock.{name} must be a nonempty string')
    for name in ('approach_dist', 'xy_ok', 'yaw_ok', 'nav_timeout_sec', 'spin_timeout_sec',
                 'settle_max_rate_rad_s', 'settle_window_sec', 'settle_timeout_sec',
                 'pose_timeout_sec'):
        value = getattr(config, name)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ConfigError(f'dock.{name} must be finite and nonnegative')
    if type(config.max_yaw_retries) is not int or config.max_yaw_retries < 0:
        raise ConfigError('dock.max_yaw_retries must be a nonnegative integer')
    if (type(config.static_occupied_threshold) is not int
            or not 0 <= config.static_occupied_threshold <= 100):
        raise ConfigError('dock.static_occupied_threshold must be an integer in [0, 100]')
    if not isinstance(config.footprint, tuple) or not all(
            isinstance(point, tuple) for point in config.footprint):
        raise ConfigError('dock.footprint must be an immutable tuple of coordinate tuples')
    try:
        parse_footprint(repr(config.footprint))
    except ValueError as error:
        raise ConfigError(f'dock.footprint: {error}') from error
    return DockConfig(
        config.approach_dist,
        DockTolerance(config.xy_ok, config.yaw_ok, config.max_yaw_retries),
        SettleSpec(max_rate_rad_s=config.settle_max_rate_rad_s,
                   window_sec=config.settle_window_sec),
        config.nav_timeout_sec, config.spin_timeout_sec, config.settle_timeout_sec)
