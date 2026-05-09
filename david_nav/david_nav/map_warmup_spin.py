#!/usr/bin/env python3
"""Spin in place until the first live map grows beyond a usable size."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class MapWarmupSpin(Node):
    def __init__(self):
        super().__init__('map_warmup_spin')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('live_map_ready_topic', '/map_live_ready')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('resume_topic', 'explore/resume')
        self.declare_parameter('spin_angular_speed', 0.45)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('min_live_map_area_m2', 12.0)
        self.declare_parameter('finish_hold_sec', 1.0)
        self.declare_parameter('status_log_period', 3.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.live_map_ready_topic = self.get_parameter('live_map_ready_topic').value
        self.cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        self.resume_topic = self.get_parameter('resume_topic').value
        self.spin_angular_speed = float(
            self.get_parameter('spin_angular_speed').value
        )
        self.publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value)
        )
        self.min_live_map_area_m2 = float(
            self.get_parameter('min_live_map_area_m2').value
        )
        self.finish_hold_sec = max(
            0.0, float(self.get_parameter('finish_hold_sec').value)
        )
        self.status_log_period = max(
            0.5, float(self.get_parameter('status_log_period').value)
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_nav_topic, 10)
        self.resume_pub = self.create_publisher(Bool, self.resume_topic, 10)
        self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, latched_qos
        )
        self.create_subscription(
            Bool, self.live_map_ready_topic, self.live_map_ready_callback, latched_qos
        )

        self.latest_map = None
        self.live_map_ready = False
        self.warmup_complete = False
        self.finish_pub_cycles = 0
        self.last_status_log_time = 0.0

        self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            'map_warmup_spin ready: '
            f'spin={self.spin_angular_speed:.2f} rad/s, '
            f'min_live_map_area={self.min_live_map_area_m2:.2f} m^2'
        )

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        if not self.live_map_ready:
            # When consuming Lightning's live transient-local map directly,
            # any received map is already a live SLAM update.
            self.live_map_ready = True
            self.get_logger().info(
                'Received live map data directly from the map topic; '
                'marking live map as ready.'
            )

    def live_map_ready_callback(self, msg: Bool):
        self.live_map_ready = bool(msg.data)

    def current_map_area_m2(self) -> float:
        if self.latest_map is None:
            return 0.0
        return (
            float(self.latest_map.info.width)
            * float(self.latest_map.info.height)
            * float(self.latest_map.info.resolution) ** 2
        )

    def publish_resume(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.resume_pub.publish(msg)

    def publish_spin_cmd(self, angular_z: float):
        msg = Twist()
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def maybe_log_status(self, message: str):
        now = time.monotonic()
        if now - self.last_status_log_time < self.status_log_period:
            return
        self.last_status_log_time = now
        self.get_logger().info(message)

    def finish_warmup(self):
        self.warmup_complete = True
        self.finish_pub_cycles = max(1, int(self.finish_hold_sec * self.publish_rate))
        area = self.current_map_area_m2()
        width = self.latest_map.info.width if self.latest_map else 0
        height = self.latest_map.info.height if self.latest_map else 0
        self.get_logger().info(
            'Map warmup complete: '
            f'live map area={area:.2f} m^2 ({width}x{height}). '
            'Resuming exploration.'
        )
        self.publish_spin_cmd(0.0)
        self.publish_resume(True)

    def timer_callback(self):
        if self.warmup_complete:
            if self.finish_pub_cycles > 0:
                self.publish_spin_cmd(0.0)
                self.publish_resume(True)
                self.finish_pub_cycles -= 1
            return

        area = self.current_map_area_m2()
        if self.live_map_ready and area >= self.min_live_map_area_m2:
            self.finish_warmup()
            return

        self.publish_resume(False)
        self.publish_spin_cmd(self.spin_angular_speed)

        if not self.live_map_ready:
            self.maybe_log_status(
                'Waiting for the first live map update; spinning in place to warm up SLAM.'
            )
            return

        width = self.latest_map.info.width if self.latest_map else 0
        height = self.latest_map.info.height if self.latest_map else 0
        self.maybe_log_status(
            'Live map is still too small '
            f'({width}x{height}, area={area:.2f} m^2 < '
            f'{self.min_live_map_area_m2:.2f} m^2); keep spinning.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapWarmupSpin()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
