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
DEFAULT_TIMEOUT = 0.25
DEFAULT_MAX_LINEAR_X = 0.20
DEFAULT_MAX_ANGULAR_Z = 0.50
DEFAULT_PUBLISH_HZ = 50.0


def clamp(value, limit):
    """Clamp a numeric value to +/- limit."""
    limit = abs(float(limit))
    return max(min(float(value), limit), -limit)


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
        self.declare_parameter("max_linear_x", DEFAULT_MAX_LINEAR_X)
        self.declare_parameter("max_angular_z", DEFAULT_MAX_ANGULAR_Z)
        self.declare_parameter("publish_hz", DEFAULT_PUBLISH_HZ)

        self.input_topic = self.get_parameter(
            "input_cmd_vel_topic"
        ).get_parameter_value().string_value
        self.output_topic = self.get_parameter(
            "output_cmd_vel_topic"
        ).get_parameter_value().string_value
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)

        self.latest_twist = zero_twist()
        self.last_cmd_time = self.get_clock().now() - Duration(seconds=3600.0)

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
            f"max_linear_x={self.max_linear_x:.2f}m/s, "
            f"max_angular_z={self.max_angular_z:.2f}rad/s, "
            f"publish_hz={self.publish_hz:.1f}"
        )

    def velocity_callback(self, msg):
        """Store the latest sanitized safe velocity command."""
        self.latest_twist = self.sanitize_twist(msg)
        self.last_cmd_time = self.get_clock().now()

    def sanitize_twist(self, msg):
        """Limit commands for the differential Wheeltec base."""
        twist = Twist()
        twist.linear.x = clamp(msg.linear.x, self.max_linear_x)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = clamp(msg.angular.z, self.max_angular_z)
        return twist

    def current_command(self):
        """Return the command that should be published now."""
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if self.cmd_vel_timeout > 0.0 and age > self.cmd_vel_timeout:
            return zero_twist()
        return self.latest_twist

    def publish_current_command(self):
        """Publish either the latest fresh command or a zero timeout command."""
        self.cmd_publisher.publish(self.current_command())

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
