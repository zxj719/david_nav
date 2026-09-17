"""保留 Dock 的 ROS 参数声明表，并校验启动文件与命令行覆盖后的值."""

from typing import Mapping

from nav_tasks.domain.runtime_config import DockNodeConfig
from nav_tasks.skills.docking.parameters import from_parameters, PARAMETER_NAMES


def declare_parameters(
    node: object, defaults: Mapping[str, object], *, spin_action_name: str = 'spin',
    pose_timeout_sec: float = 2.0,
) -> DockNodeConfig:
    """声明既有十六个参数，校验默认值和 ROS 覆盖后的实际值.

    默认值必须完整且先通过纯配置校验，随后按冻结顺序逐项声明、读取并
    再次校验。失败不回滚已声明参数；外部节点所有者负责残留状态的清理。
    不初始化 ROS、不建订阅/客户端或启动工作线程。

    Args:
        node: 外部节点，名称与重映射由装配根决定。
        defaults: 完整旧参数默认值映射，footprint 仍为字符串。
        spin_action_name: 独立后端端点参数，不新增 ROS 声明项。
        pose_timeout_sec: 独立 TF 查询预算，不新增 ROS 声明项。

    Returns:
        经 ROS 启动或命令行覆盖后校验的不可变配置。

    Raises:
        ConfigError: 默认值或覆盖后的已检查配置无效。
        Exception: 转换、声明或读取失败，调用方仍需持有节点清理。
    """
    from_parameters(defaults, spin_action_name=spin_action_name,
                    pose_timeout_sec=pose_timeout_sec)
    defaults = dict(defaults)
    for name in PARAMETER_NAMES:
        node.declare_parameter(name, defaults[name])
    effective = {name: node.get_parameter(name).value for name in PARAMETER_NAMES}
    return from_parameters(effective, spin_action_name=spin_action_name,
                           pose_timeout_sec=pose_timeout_sec)
