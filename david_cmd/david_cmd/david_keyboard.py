"""Keyboard teleoperation publisher for the david_move velocity bridge."""

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


DEFAULT_CMD_TOPIC = "cmd_vel_safe"
DEFAULT_LINEAR_SPEED = 0.20
DEFAULT_ANGULAR_SPEED = 0.35
DEFAULT_LINEAR_DEADZONE = 0.20
DEFAULT_ANGULAR_DEADZONE = 0.35
DEFAULT_MAX_LINEAR_X = 0.20
DEFAULT_MAX_ANGULAR_Z = 0.50
DEFAULT_PUBLISH_HZ = 20.0
DEFAULT_KEY_TIMEOUT = 0.40

MOTION_KEYS = {
    "w": (1.0, 0.0),
    "s": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
    "\x1b[A": (1.0, 0.0),
    "\x1b[B": (-1.0, 0.0),
    "\x1b[D": (0.0, 1.0),
    "\x1b[C": (0.0, -1.0),
}
STOP_KEYS = {" ", "k", "K"}
QUIT_KEYS = {"q", "Q", "\x03"}

HELP_TEXT = """
Keyboard teleop for david_move

  w / Up       forward
  s / Down     backward
  a / Left     turn left
  d / Right    turn right
  Space / k    stop
  q / Ctrl-C   quit
"""


def clamp(value, limit):
    """Clamp a numeric value to +/- limit."""
    limit = abs(float(limit))
    return max(min(float(value), limit), -limit)


def apply_min_effective_speed(value, minimum, maximum):
    """Lift non-zero commands out of the measured base deadzone."""
    limited = clamp(value, maximum)
    if limited == 0.0:
        return 0.0

    minimum = min(abs(float(minimum)), abs(float(maximum)))
    if abs(limited) < minimum:
        return math.copysign(minimum, limited)
    return limited


def zero_twist():
    """Create a zero velocity command."""
    return Twist()


def motion_for_key(key):
    """Return linear and angular direction multipliers for a key."""
    lookup = key.lower() if len(key) == 1 else key
    return MOTION_KEYS.get(lookup)


class TerminalKeyboard:
    """Read single keys from a terminal without waiting for Enter."""

    def __init__(self, stream=None):
        self.stream = stream if stream is not None else sys.stdin
        self.fd = self.stream.fileno()
        self.settings = None

    def __enter__(self):
        if not self.stream.isatty():
            raise RuntimeError("Keyboard teleop requires an interactive TTY.")

        self.settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, traceback):
        if self.settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)

    def read_key(self, timeout_sec):
        """Read one key or escape sequence."""
        readable, _, _ = select.select([self.fd], [], [], timeout_sec)
        if not readable:
            return ""

        key = os.read(self.fd, 1).decode(errors="ignore")
        if key != "\x1b":
            return key

        deadline = time.monotonic() + 0.01
        while time.monotonic() < deadline and len(key) < 3:
            wait = max(deadline - time.monotonic(), 0.0)
            readable, _, _ = select.select([self.fd], [], [], wait)
            if not readable:
                break
            key += os.read(self.fd, 1).decode(errors="ignore")
        return key


class DavidKeyboardNode(Node):
    """Publish deadzone-aware manual velocity commands."""

    def __init__(self, **kwargs):
        super().__init__("david_keyboard", **kwargs)

        self.declare_parameter("cmd_vel_topic", DEFAULT_CMD_TOPIC)
        self.declare_parameter("linear_speed", DEFAULT_LINEAR_SPEED)
        self.declare_parameter("angular_speed", DEFAULT_ANGULAR_SPEED)
        self.declare_parameter("linear_deadzone", DEFAULT_LINEAR_DEADZONE)
        self.declare_parameter("angular_deadzone", DEFAULT_ANGULAR_DEADZONE)
        self.declare_parameter("max_linear_x", DEFAULT_MAX_LINEAR_X)
        self.declare_parameter("max_angular_z", DEFAULT_MAX_ANGULAR_Z)
        self.declare_parameter("publish_hz", DEFAULT_PUBLISH_HZ)
        self.declare_parameter("key_timeout", DEFAULT_KEY_TIMEOUT)

        self.cmd_vel_topic = self.get_parameter(
            "cmd_vel_topic"
        ).get_parameter_value().string_value
        self.linear_speed = abs(
            float(self.get_parameter("linear_speed").value)
        )
        self.angular_speed = abs(
            float(self.get_parameter("angular_speed").value)
        )
        self.linear_deadzone = abs(
            float(self.get_parameter("linear_deadzone").value)
        )
        self.angular_deadzone = abs(
            float(self.get_parameter("angular_deadzone").value)
        )
        self.max_linear_x = abs(
            float(self.get_parameter("max_linear_x").value)
        )
        self.max_angular_z = abs(
            float(self.get_parameter("max_angular_z").value)
        )
        self.publish_hz = max(
            float(self.get_parameter("publish_hz").value),
            1.0,
        )
        self.key_timeout = max(
            float(self.get_parameter("key_timeout").value),
            0.0,
        )
        self.period_sec = 1.0 / self.publish_hz

        self.cmd_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.active_twist = zero_twist()
        self.last_motion_time = 0.0

        self.get_logger().info(
            "Keyboard teleop ready: "
            f"topic={self.cmd_vel_topic}, "
            f"linear={self.linear_speed:.2f}m/s "
            f"(deadzone>={self.linear_deadzone:.2f}), "
            f"angular={self.angular_speed:.2f}rad/s "
            f"(deadzone>={self.angular_deadzone:.2f}), "
            f"timeout={self.key_timeout:.2f}s"
        )

    def build_twist(self, linear_direction, angular_direction):
        """Build a limited Twist from direction multipliers."""
        twist = Twist()
        twist.linear.x = apply_min_effective_speed(
            linear_direction * self.linear_speed,
            self.linear_deadzone,
            self.max_linear_x,
        )
        twist.angular.z = apply_min_effective_speed(
            angular_direction * self.angular_speed,
            self.angular_deadzone,
            self.max_angular_z,
        )
        return twist

    def set_motion(self, linear_direction, angular_direction):
        """Set the active command from keyboard directions."""
        self.active_twist = self.build_twist(
            linear_direction,
            angular_direction,
        )
        self.last_motion_time = time.monotonic()

    def stop(self):
        """Stop immediately."""
        self.active_twist = zero_twist()
        self.last_motion_time = 0.0
        self.publish_zero()

    def handle_key(self, key):
        """Apply a key event. Return False when the loop should quit."""
        if key in QUIT_KEYS:
            return False

        if key in STOP_KEYS:
            self.stop()
            return True

        motion = motion_for_key(key)
        if motion is not None:
            self.set_motion(*motion)
        return True

    def current_command(self):
        """Return the active command or zero after keyboard inactivity."""
        if self.last_motion_time == 0.0:
            return zero_twist()

        age = time.monotonic() - self.last_motion_time
        if self.key_timeout > 0.0 and age > self.key_timeout:
            return zero_twist()
        return self.active_twist

    def publish_current_command(self):
        """Publish the current deadman-protected command."""
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


def spin_keyboard(node, keyboard):
    """Run the keyboard polling and publish loop."""
    print(HELP_TEXT)
    while rclpy.ok():
        key = keyboard.read_key(node.period_sec)
        if key and not node.handle_key(key):
            break
        node.publish_current_command()
        rclpy.spin_once(node, timeout_sec=0.0)


def main(args=None):
    """Run keyboard teleoperation for david_move."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DavidKeyboardNode()

    try:
        with TerminalKeyboard() as keyboard:
            spin_keyboard(node, keyboard)
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_stop_burst()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
