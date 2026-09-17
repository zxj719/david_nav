"""Configurable replacement for the legacy differential-drive fake chassis."""

import os
from typing import Optional, Sequence

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from nav_sim import _ros
from nav_sim.model import finite_number, frame_name, integrate, PlanarPose, quaternion
from nav_sim.model import require_sandbox


class FakeChassis(Node):
    """Integrate Twist into Odometry; never open hardware or publish commands."""

    def __init__(self) -> None:
        """Require explicit robot frames and retain the legacy 0.02 second step."""
        require_sandbox(os.environ)
        super().__init__('fake_chassis')
        _ros.require_node_sandbox(self)
        self._odom_frame = frame_name(
            self.declare_parameter('odom_frame', '').value, 'odom_frame')
        self._body_frame = frame_name(
            self.declare_parameter('body_frame', '').value, 'body_frame')
        self._dt = finite_number(self.declare_parameter('dt', 0.02).value, 'dt')
        if self._dt <= 0:
            raise ValueError('dt must be positive')
        self._pose = PlanarPose()
        self._linear = self._angular = 0.0
        self._publisher = self.create_publisher(
            Odometry, self.declare_parameter('odom_topic', '/odom').value, 10)
        self._subscription = self.create_subscription(
            Twist, self.declare_parameter('cmd_topic', '/cmd_vel').value, self._command, 10)
        self._timer = self.create_timer(self._dt, self._tick)

    def _command(self, message: Twist) -> None:
        """接收仿真底盘命令数值，任一所用轴无效时清零两个积分输入.

        只保存线速度 x 与角速度 z，不处理其它轴；不发布真实底盘命令。
        这里不设置额外限幅，有限性由 finite_number 检查。

        Args:
            message: 仿真订阅收到的 Twist 消息。

        Raises:
            Exception: 消息字段缺失或失败日志接收器异常等未转换错误。
        """
        try:
            linear = finite_number(message.linear.x, 'linear.x')
            angular = finite_number(message.angular.z, 'angular.z')
        except ValueError as error:
            self._linear = self._angular = 0.0
            self.get_logger().warning(str(error))
        else:
            self._linear, self._angular = linear, angular

    def _tick(self) -> None:
        """以配置的固定秒制步长积分缓存速度，只发布仿真里程计.

        线速度为米/秒、角速度为弧度/秒；消息时间使用节点当前时钟，而积分
        不按回调实际间隔补偿。本回调不打开硬件，也不发布速度命令。

        Raises:
            Exception: 积分、读取时钟、构造或发布消息失败，交给节点外层处理。
        """
        self._pose = integrate(self._pose, self._linear, self._angular, self._dt)
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._body_frame
        message.pose.pose.position.x = self._pose.x
        message.pose.pose.position.y = self._pose.y
        orientation = message.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion(self._pose.yaw)
        message.twist.twist.linear.x = self._linear
        message.twist.twist.angular.z = self._angular
        self._publisher.publish(message)


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run the isolated fake chassis, with no hardware dependencies.

    Args:
        args: Optional ROS command-line arguments.
    """
    _ros.run(FakeChassis, args)
