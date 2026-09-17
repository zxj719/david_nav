"""从仿真里程计发布 map→odom 和 odom→机身 TF，不运行 Lightning 定位."""

import copy
import math
import os
from typing import Optional, Sequence

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from nav_sim import _ros
from nav_sim.model import localization_frames, require_sandbox


class FakeLocalization(Node):
    """把仿真里程计映射到 TF；机身到导航旋转中心的变换仍由 URDF 提供."""

    def __init__(self) -> None:
        """核对域与三个不同的帧，订阅仿真里程计并按配置创建转发输出.

        配置错误会向外传播；构造中途失败不在此显式回滚已创建的 ROS 节点。
        """
        require_sandbox(os.environ)
        super().__init__('fake_localization')
        _ros.require_node_sandbox(self)
        self._map, self._odom, self._body = localization_frames(*(
            self.declare_parameter(name, '').value
            for name in ('map_frame', 'odom_frame', 'body_frame')))
        self._broadcaster = TransformBroadcaster(self)
        self._publisher = None
        output = self.declare_parameter('output_odom_topic', '').value
        source = self.declare_parameter('odom_topic', '/odom').value
        if output and self.resolve_topic_name(output) != self.resolve_topic_name(source):
            self._publisher = self.create_publisher(Odometry, output, 10)
        self._subscription = self.create_subscription(Odometry, source, self._odometry, 10)

    def _odometry(self, message: Odometry) -> None:
        """从仿真里程计发布两段 TF，拒绝帧不符或非有限/非单位四元数样本.

        Args:
            message: 配置的里程计帧到机身帧消息，位置使用米；沿用原时间戳。
                地图到里程计变换为单位变换，机身到导航基准仍由 URDF 持有。

        验证失败仅警告并忽略；可选输出话题发布深拷贝，不修改输入消息。
        这是假定位组件，不启动或控制 Lightning/uslam。

        Raises:
            Exception: TF 广播或可选里程计发布失败，交给节点外层处理。
        """
        if message.header.frame_id != self._odom or message.child_frame_id != self._body:
            self.get_logger().warning('simulated odometry frame mismatch; ignoring sample')
            return
        pose = message.pose.pose
        quaternion = pose.orientation
        components = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if (not all(math.isfinite(value) for value in (
                pose.position.x, pose.position.y, pose.position.z, *components))
                or abs(sum(value * value for value in components) - 1.0) > 1e-6):
            self.get_logger().warning('invalid simulated odometry pose; ignoring sample')
            return
        fixed, moving = TransformStamped(), TransformStamped()
        fixed.header.stamp = message.header.stamp
        fixed.header.frame_id = self._map
        fixed.child_frame_id = self._odom
        fixed.transform.rotation.w = 1.0
        moving.header.stamp = message.header.stamp
        moving.header.frame_id = self._odom
        moving.child_frame_id = self._body
        moving.transform.translation.x = pose.position.x
        moving.transform.translation.y = pose.position.y
        moving.transform.translation.z = pose.position.z
        moving.transform.rotation = copy.deepcopy(pose.orientation)
        self._broadcaster.sendTransform([fixed, moving])
        if self._publisher is not None:
            self._publisher.publish(copy.deepcopy(message))


def main(args: Optional[Sequence[str]] = None) -> None:
    """运行导航自测假定位，不启动或修改 SLAM 栈.

    Args:
        args: 可选 ROS 命令行参数。
    """
    _ros.run(FakeLocalization, args)
