#!/usr/bin/env python3
"""Ideal differential-drive chassis sim: integrate /cmd_vel_position -> /odom."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class FakeChassis(Node):
    def __init__(self):
        super().__init__("fake_chassis")
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.sub = self.create_subscription(Twist, "/cmd_vel_position", self._cmd, 10)
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self._tick)

    def _cmd(self, m):
        self.v = m.linear.x
        self.w = m.angular.z

    def _tick(self):
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * self.dt),
                              math.cos(self.yaw + self.w * self.dt))
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "odom"
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2)
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.w
        self.pub.publish(o)


def main():
    rclpy.init()
    n = FakeChassis()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
