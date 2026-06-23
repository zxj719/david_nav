#!/usr/bin/env python3
"""groundtruth_pose_publisher —— 把 slam_toolbox 的 map->base_footprint 位姿
以固定频率发成 /groundtruth/pose (PoseWithCovarianceStamped)。

用途：给 controller 调参测试做近似真值。位姿来自 slam_toolbox 的 2D scan-match
(map 系)，独立于 cmd_vel 链、独立于轮速计漂移，适合衡量路径跟踪/过冲/震荡。
注意：localization 模式下 map 锁在 mapping 阶段存的那张图上 -> 跨轮帧一致。

诚实边界：这不是外部独立真值(slam_toolbox 同时也是 nav 的定位源)，
抓不了 "定位本身导致的误差"；详见 docs/reports 那篇报告 §2。

输出 frame_id=map，pose=base_footprint(REP-105，+X 物理前)。
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


class GroundtruthPosePublisher(Node):
    def __init__(self):
        super().__init__('groundtruth_pose_publisher')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('pose_topic', '/groundtruth/pose')

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate = float(self.get_parameter('rate_hz').value)
        topic = self.get_parameter('pose_topic').value

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self._warned = False
        self.get_logger().info(
            f'groundtruth_pose_publisher: TF {self.map_frame}->{self.base_frame} '
            f'-> {topic} @ {rate:.0f}Hz')

    def tick(self):
        try:
            tf = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self._warned:
                self.get_logger().warn(
                    f'等 {self.map_frame}->{self.base_frame} TF '
                    f'(slam_toolbox + odom_to_base_tf 都起了吗?)...')
                self._warned = True
            return
        self._warned = False

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        # 近似真值的标称协方差(x,y ~3cm, yaw ~1deg)；非严格估计，仅供下游加权
        cov = [0.0] * 36
        cov[0] = 0.001    # x
        cov[7] = 0.001    # y
        cov[35] = 0.0003  # yaw
        msg.pose.covariance = cov
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundtruthPosePublisher()
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
