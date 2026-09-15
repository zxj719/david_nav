"""定位、底盘及可选点云接口的只读契约判定，ROS 图采样由外层注入."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, FrozenSet, Mapping, Optional, Tuple

from nav_perception_iface.configuration import profile_value, string_value


@dataclass(frozen=True)
class InterfaceSnapshot:
    """诊断采样窗口中的接口事实，不持有 ROS 节点.

    Attributes:
        transforms: 已确认可查询的 (target, source) 坐标变换。
        publishers: 话题发布者数量。
        subscribers: 符合底盘命令 Twist 类型契约的话题订阅者数量。
        samples: 采样窗口内实际收到消息的话题。
        types: 话题实际公布的消息类型集合。
        cloud_frame: 最新输入点云的 header.frame_id，未收到则为 None。
    """

    transforms: FrozenSet[Tuple[str, str]]
    publishers: Mapping[str, int]
    subscribers: Mapping[str, int]
    samples: FrozenSet[str]
    types: Mapping[str, FrozenSet[str]]
    cloud_frame: Optional[str] = None


@dataclass(frozen=True)
class ContractFinding:
    """一个接口诊断结论.

    Attributes:
        severity: info 或 error；等待人工定位仅是 info。
        code: 稳定机器可读标识。
        message: 人可读说明。
    """

    severity: str
    code: str
    message: str


def check_contracts(
    profile: Mapping[str, Any], snapshot: InterfaceSnapshot, chassis_required: bool = True,
) -> Tuple[ContractFinding, ...]:
    """核对接口，并把等待人工定位与定位后的接线故障区分开.

    Args:
        profile: 有效机器人档案；不更改 Nav2 odom 接线。
        snapshot: 外层采集的 TF、端点和消息证据。
        chassis_required: chassis=off 的仿真/台架可不检查实体驱动契约。

    Returns:
        全部诊断结论；空元组表示本采样窗口契约满足。

    Raises:
        ValueError: 档案缺少必要的坐标系或话题键。
    """
    findings = []
    map_frame = string_value(profile, 'frames.map')
    odom_frame = string_value(profile, 'frames.odom')
    body_frame = string_value(profile, 'frames.body')
    nav_frame = string_value(profile, 'frames.nav_base')

    def has_transform(target, source):
        return target == source or (target, source) in snapshot.transforms

    def topic_data(topic, expected_type, code):
        if snapshot.publishers.get(topic, 0) < 1:
            findings.append(ContractFinding('error', code, f'{topic}: no publisher'))
        elif expected_type not in snapshot.types.get(topic, frozenset()):
            findings.append(ContractFinding('error', code, f'{topic}: expected {expected_type}'))
        elif topic not in snapshot.samples:
            findings.append(ContractFinding('error', code, f'{topic}: no samples in check window'))

    if not has_transform(map_frame, odom_frame):
        findings.append(ContractFinding(
            'info', 'localization_waiting', '等待在 RViz 手动给初始位姿'))
    else:
        for target, source in ((odom_frame, body_frame), (body_frame, nav_frame)):
            if not has_transform(target, source):
                findings.append(ContractFinding(
                    'error', 'localization_tf_missing', f'cannot transform {target} <- {source}'))
        topic_data(string_value(profile, 'localization.odom_topic'),
                   'nav_msgs/msg/Odometry', 'localization_odom_missing')

    if chassis_required:
        command = string_value(profile, 'chassis.cmd_topic')
        if snapshot.subscribers.get(command, 0) < 1:
            findings.append(ContractFinding(
                'error', 'chassis_cmd_missing', f'{command}: no subscriber'))
        topic_data(string_value(profile, 'chassis.odom_topic'),
                   'nav_msgs/msg/Odometry', 'chassis_odom_missing')

    scan = profile_value(profile, 'sensors.scan_from_cloud')
    if not isinstance(scan, Mapping) or not isinstance(scan.get('enabled', False), bool):
        raise ValueError('sensors.scan_from_cloud.enabled must be boolean')
    if scan.get('enabled', False):
        cloud = string_value(profile, 'sensors.scan_from_cloud.cloud_topic')
        topic_data(cloud, 'sensor_msgs/msg/PointCloud2', 'scan_cloud_missing')
        target = string_value(profile, 'frames.scan_target')
        if not snapshot.cloud_frame or not has_transform(target, snapshot.cloud_frame):
            findings.append(ContractFinding(
                'error', 'scan_cloud_tf_missing',
                f'{cloud}: cannot transform cloud into {target}'))
    return tuple(findings)
