#!/usr/bin/env python3
"""odom_to_base_tf —— 把 wheeltec 的 /odom 话题转成 odom->base_link TF。

为什么需要：
  wheeltec 驱动只发 /odom 话题、不发任何 TF(2026-06-09 实查，源码无
  TransformBroadcaster)。原先 odom->base 这条边是 lightning 发的。
  现在 slam_toolbox 替换 lightning，只发 map->odom，odom->base_link 就断了。
  本节点用轮速计补上这条边(用户选定的 "复用轮速计 odom")。

干净约定(绕过 david.urdf 的 90° 坑)：
  david.urdf 的 base_link 是 IMU 体系(+X=机身左)，所以历史上 odom->base->base_link
  要塞一个 Rz(-pi/2)。本包不沿用那套：直接把 base_link 定义成 REP-105 前向系
  (+X=机身正前，落在地面)，wheeltec /odom 本来就是这个朝向，所以原样转发即可，
  无需任何旋转/偏置。配套的 base_link->laser_link 静态也是纯平移(0.352,0,0.3775)，
  见 launch。

  如机械上想把 base_link 抬到车体高度，给 z_offset 即可(对 2D 导航无影响)。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToBaseTf(Node):
    def __init__(self):
        super().__init__('odom_to_base_tf')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('z_offset', 0.0)   # base_link 默认落地;抬车体高度时设此

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.z_offset = float(self.get_parameter('z_offset').value)
        odom_topic = self.get_parameter('odom_topic').value

        self.br = TransformBroadcaster(self)
        qos = QoSProfile(depth=20,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, qos)
        self.get_logger().info(
            f'odom_to_base_tf: {odom_topic} -> TF {self.parent_frame}->{self.child_frame} '
            f'(原样转发, z_offset={self.z_offset}m)')

    def cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z + self.z_offset
        t.transform.rotation = o          # 原样转发，无旋转修正
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
