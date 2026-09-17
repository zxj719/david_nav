"""把完整旧 Dock 参数映射转换为不可变任务配置，保留参数名与声明顺序."""

from typing import Mapping

from nav_tasks.domain.errors import ConfigError
from nav_tasks.domain.footprint import parse_footprint
from nav_tasks.domain.runtime_config import DockNodeConfig, DockRuntimeConfig
from nav_tasks.skills.docking.config import workflow_config

# Keep the old declaration order and names; no robot-specific defaults live here.
PARAMETER_NAMES = (
    'approach_dist', 'xy_ok', 'yaw_ok', 'max_yaw_retries', 'goal_topic', 'action_name',
    'robot_base_frame', 'global_frame', 'nav_timeout_sec', 'spin_timeout_sec',
    'static_map_topic', 'static_occupied_threshold', 'footprint', 'settle_max_rate_rad_s',
    'settle_window_sec', 'settle_timeout_sec',
)


def from_parameters(
    parameters: Mapping[str, object], *, spin_action_name: str = 'spin',
    pose_timeout_sec: float = 2.0,
) -> DockNodeConfig:
    """校验全部旧参数并转换轮廓，不补造缺失的机器人或模式策略.

    映射键必须恰好覆盖十六个参数；footprint 保留 ROS 字符串输入格式。
    转换后调用 workflow_config 校验，不打开 ROS、存储或运动资源。
    解析及转换中非 ValueError 异常仍可传播，不保证所有异常都为 ConfigError。

    Args:
        parameters: 完整十六项旧 Dock ROS 参数映射，footprint 为字符串。
        spin_action_name: 共享旋转后端端点，沿用旧相对名称 spin 的默认值。
        pose_timeout_sec: 显式 TF 查询预算（秒），保留旧两秒默认值。

    Returns:
        脱离输入映射的不可变入口及运行配置，轮廓转换为坐标元组。

    Raises:
        ConfigError: 参数缺失、未知，或不满足已有格式与范围检查。
    """
    if not isinstance(parameters, Mapping):
        raise ConfigError('Dock parameters must be a mapping')
    missing = set(PARAMETER_NAMES) - parameters.keys()
    unknown = parameters.keys() - set(PARAMETER_NAMES)
    if missing or unknown:
        raise ConfigError(f'Dock parameters: missing={sorted(missing)}, '
                          f'unknown={sorted(unknown, key=str)}')
    for name in ('goal_topic', 'action_name'):
        value = parameters[name]
        if not isinstance(value, str) or not value.strip():
            raise ConfigError(f'dock.{name} must be a nonempty string')
    if not isinstance(parameters['footprint'], str):
        raise ConfigError('dock.footprint must retain the ROS string parameter format')
    try:
        footprint = parse_footprint(parameters['footprint'])
    except ValueError as error:
        raise ConfigError(f'dock.footprint: {error}') from error
    values = {name: value for name, value in parameters.items()
              if name not in ('goal_topic', 'action_name', 'footprint')}
    runtime = DockRuntimeConfig(**values, footprint=footprint,
                                spin_action_name=spin_action_name,
                                pose_timeout_sec=pose_timeout_sec)
    workflow_config(runtime)
    return DockNodeConfig(parameters['goal_topic'], parameters['action_name'], runtime)
