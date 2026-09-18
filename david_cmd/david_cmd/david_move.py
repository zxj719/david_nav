# Copyright 2026 xingjian zhang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Wheeltec base velocity bridge for the david_move executable."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


DEFAULT_INPUT_TOPIC = "cmd_vel_safe"
DEFAULT_OUTPUT_TOPIC = "/cmd_vel"
DEFAULT_TIMEOUT = 2.0
DEFAULT_MIN_LINEAR_X = 0.20
DEFAULT_MAX_LINEAR_X = 0.20
# 2026-05-18: 弧形运动（linear.x != 0）下的角速度地板。从 0.20 → 0.0。
# 原值会把 DWB 跟踪路径时的小角速度修正（如 +0.026 rad/s）放大到 ±0.20，
# 导致直行段被强制 "5 倍放大" 的左右摆动 ⇒ DWB 后续反向纠正 ⇒ 极化采样
# (-0.5 / 0 / +0.5)，机器人偏左偏右往返。带线速度时差速本就无需克服静摩擦。
# 详见 /tmp/nav_drift/drift_run1_*.csv 和当日记录。
DEFAULT_MIN_ANGULAR_Z = 0.0
# 原地转向地板保留 0.35（静止时确实需要克服静摩擦才能起步）。
DEFAULT_MIN_STATIONARY_ANGULAR_Z = 0.35
DEFAULT_STATIONARY_ANGULAR_THRESHOLD = 0.025
DEFAULT_MAX_ANGULAR_Z = 0.50
DEFAULT_PUBLISH_HZ = 50.0


def clamp(value, limit):
    """Clamp a numeric value to +/- limit."""
    limit = abs(float(limit))
    return max(min(float(value), limit), -limit)


def lift_deadzone(value, minimum):
    """Lift a non-zero command to a minimum actionable magnitude."""
    value = float(value)
    minimum = abs(float(minimum))
    if value == 0.0 or minimum <= 0.0 or abs(value) >= minimum:
        return value
    return minimum if value > 0.0 else -minimum


def zero_twist():
    """Create a zero velocity command."""
    return Twist()


class DavidMoveNode(Node):
    """Forward safe velocity commands to the Wheeltec base."""

    def __init__(self, **kwargs):
        super().__init__("david_move", **kwargs)

        self.declare_parameter("input_cmd_vel_topic", DEFAULT_INPUT_TOPIC)
        self.declare_parameter("output_cmd_vel_topic", DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("cmd_vel_timeout", DEFAULT_TIMEOUT)
        self.declare_parameter("min_linear_x", DEFAULT_MIN_LINEAR_X)
        self.declare_parameter("max_linear_x", DEFAULT_MAX_LINEAR_X)
        self.declare_parameter("min_angular_z", DEFAULT_MIN_ANGULAR_Z)
        self.declare_parameter(
            "min_stationary_angular_z", DEFAULT_MIN_STATIONARY_ANGULAR_Z
        )
        self.declare_parameter(
            "stationary_angular_threshold", DEFAULT_STATIONARY_ANGULAR_THRESHOLD
        )
        self.declare_parameter("max_angular_z", DEFAULT_MAX_ANGULAR_Z)
        self.declare_parameter("publish_hz", DEFAULT_PUBLISH_HZ)

        self.input_topic = self.get_parameter(
            "input_cmd_vel_topic"
        ).get_parameter_value().string_value
        self.output_topic = self.get_parameter(
            "output_cmd_vel_topic"
        ).get_parameter_value().string_value
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.min_linear_x = float(self.get_parameter("min_linear_x").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.min_angular_z = float(self.get_parameter("min_angular_z").value)
        self.min_stationary_angular_z = float(
            self.get_parameter("min_stationary_angular_z").value
        )
        self.stationary_angular_threshold = float(
            self.get_parameter("stationary_angular_threshold").value
        )
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)

        self.latest_twist = zero_twist()
        self.last_cmd_time = self.get_clock().now() - Duration(seconds=3600.0)
        # Track whether we've already announced the stale-stop, so we publish
        # zero exactly once on entering stale and then go silent — that frees
        # /cmd_vel_raw (and downstream /cmd_vel) for manual teleop when nav2
        # is idle. Starts True so a freshly-launched bridge is silent until
        # the first real cmd arrives.
        self._stale_stop_sent = True

        self.cmd_publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(
            Twist,
            self.input_topic,
            self.velocity_callback,
            10,
        )
        self.create_timer(1.0 / self.publish_hz, self.publish_current_command)

        self.get_logger().info(
            "Wheeltec velocity bridge ready: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"timeout={self.cmd_vel_timeout:.2f}s, "
            f"min_linear_x={self.min_linear_x:.2f}m/s, "
            f"max_linear_x={self.max_linear_x:.2f}m/s, "
            f"min_angular_z={self.min_angular_z:.2f}rad/s, "
            f"min_stationary_angular_z={self.min_stationary_angular_z:.2f}rad/s, "
            f"stationary_angular_threshold="
            f"{self.stationary_angular_threshold:.3f}rad/s, "
            f"max_angular_z={self.max_angular_z:.2f}rad/s, "
            f"publish_hz={self.publish_hz:.1f}"
        )

    def velocity_callback(self, msg):
        """Store the latest sanitized safe velocity command."""
        self.latest_twist = self.sanitize_twist(msg)
        self.last_cmd_time = self.get_clock().now()
        self._stale_stop_sent = False

    def sanitize_twist(self, msg):
        """Limit commands for the differential Wheeltec base."""
        twist = Twist()
        twist.linear.x = lift_deadzone(
            clamp(msg.linear.x, self.max_linear_x),
            min(self.min_linear_x, self.max_linear_x),
        )
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        angular_minimum = self.angular_deadzone_minimum(msg)
        twist.angular.z = lift_deadzone(
            clamp(msg.angular.z, self.max_angular_z),
            min(angular_minimum, self.max_angular_z),
        )
        return twist

    def angular_deadzone_minimum(self, msg):
        """Use a stronger yaw floor for deliberate in-place turns."""
        if msg.angular.z == 0.0:
            return 0.0

        linear_motion = abs(msg.linear.x) > 0.0 or abs(msg.linear.y) > 0.0
        if linear_motion:
            return self.min_angular_z

        if abs(msg.angular.z) >= abs(self.stationary_angular_threshold):
            return self.min_stationary_angular_z

        return self.min_angular_z

    def is_stale(self):
        """Return True if no fresh input has arrived within the timeout."""
        if self.cmd_vel_timeout <= 0.0:
            return False
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        return age > self.cmd_vel_timeout

    def publish_current_command(self):
        """Forward fresh nav input, otherwise emit one stop then stay silent.

        Going silent on stale releases /cmd_vel_raw so manual teleop (which
        publishes directly to /cmd_vel) is not overwritten by a 50 Hz zero
        stream while nav2 has nothing to say.
        """
        if self.is_stale():
            if not self._stale_stop_sent:
                self.cmd_publisher.publish(zero_twist())
                self._stale_stop_sent = True
            return
        self.cmd_publisher.publish(self.latest_twist)

    def publish_zero(self):
        """Publish one explicit stop command."""
        try:
            self.cmd_publisher.publish(zero_twist())
        except Exception as exc:
            self.get_logger().warn(f"Unable to publish stop command: {exc}")

    def publish_stop_burst(self, repetitions=5, period_sec=0.02):
        """Publish repeated stop commands for shutdown and failure paths."""
        for _ in range(max(int(repetitions), 1)):
            self.publish_zero()
            time.sleep(max(float(period_sec), 0.0))


def main(args=None):
    """Run the Wheeltec velocity bridge."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DavidMoveNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_stop_burst()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
