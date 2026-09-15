"""Standalone guarded keyboard runtime; combined supervisor integration remains pending."""

from dataclasses import asdict
import math
import signal
import sys
import threading
import time
from typing import Any, Optional, Sequence

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from chassis_teleop.control import KeyboardState, TeleopConfig
from chassis_teleop.interlock import CommandConflict, wait_for_clear_topics
from chassis_teleop.terminal import TerminalKeyboard, TerminalUnavailable
from nav_bringup.navrun.locking import LockBusy, RunLock


class KeyboardNode(Node):
    """Declare parameters first; create no command publisher before explicit arming."""

    def __init__(self, **kwargs: Any) -> None:
        """Load startup parameters without publishing or reading a terminal.

        Args:
            **kwargs: Standard ROS node keyword arguments.
        """
        super().__init__('chassis_teleop', **kwargs)
        defaults = asdict(TeleopConfig())
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = TeleopConfig(**{name: self.get_parameter(name).value for name in defaults})
        self.state = KeyboardState(self.config)
        self.declare_parameter('guard_topics', ['/cmd_vel', '/cmd_vel_nav'])
        self.declare_parameter('discovery_sec', 1.0)
        topics = self.get_parameter('guard_topics').value
        if (not isinstance(topics, (list, tuple)) or not topics
                or not all(isinstance(topic, str) and topic for topic in topics)):
            raise ValueError('guard_topics must be a nonempty command-topic list')
        self.guard_topics = tuple(dict.fromkeys(
            self.resolve_topic_name(topic) for topic in [*topics, self.config.cmd_vel_topic]))
        self.discovery_sec = self.get_parameter('discovery_sec').value
        if (isinstance(self.discovery_sec, bool)
                or not isinstance(self.discovery_sec, (int, float))
                or not math.isfinite(self.discovery_sec) or self.discovery_sec <= 0.0):
            raise ValueError('discovery_sec must be finite and positive')
        self.cmd_publisher = None

    def arm(self) -> None:
        """Create output only after the caller's lock, TTY and graph checks pass.

        Raises:
            RuntimeError: The output has already been armed.
        """
        if self.cmd_publisher is not None:
            raise RuntimeError('keyboard output already armed')
        self.cmd_publisher = self.create_publisher(Twist, self.config.cmd_vel_topic, 10)

    def handle_key(self, key: str) -> str:
        """Handle one complete event and immediately publish zero for a stop key.

        Args:
            key: Complete terminal key or escape sequence.

        Returns:
            motion, stop, quit or ignored, for the foreground loop.
        """
        event = self.state.handle_key(key, time.monotonic())
        if event == 'stop':
            self.publish_zero()
        return event

    def publish_current_command(self) -> None:
        """Publish a deadman-protected command after arming, never before it.

        Raises:
            RuntimeError: The guard has not armed output.
        """
        if self.cmd_publisher is None:
            raise RuntimeError('keyboard output is not armed')
        message = Twist()
        message.linear.x, message.angular.z = self.state.command(time.monotonic())
        self.cmd_publisher.publish(message)

    def publish_zero(self) -> None:
        """Attempt a stop only when this runtime has already armed its own publisher."""
        if self.cmd_publisher is not None:
            try:
                self.cmd_publisher.publish(Twist())
            except Exception as error:
                self.get_logger().warning(f'Unable to publish keyboard stop: {error}')

    def publish_stop_burst(self) -> None:
        """Preserve the five-zero shutdown burst without touching a rejected command graph."""
        if self.cmd_publisher is not None:
            for _ in range(5):
                self.publish_zero()
                time.sleep(0.02)


def run_keyboard(node: KeyboardNode, keyboard: TerminalKeyboard, stopped: threading.Event) -> None:
    """Guard and operate one keyboard while the caller holds its navigation lease.

    Args:
        node: Unarmed keyboard node owned by the caller.
        keyboard: Open terminal owned by the caller.
        stopped: INT/TERM flag; no signal handler publishes or destroys ROS objects.
    """
    def spin(duration: float) -> None:
        if stopped.is_set() or not rclpy.ok():
            raise KeyboardInterrupt
        rclpy.spin_once(node, timeout_sec=duration)

    wait_for_clear_topics(node.guard_topics, node.get_publishers_info_by_topic,
                          spin, time.monotonic, node.discovery_sec)
    if stopped.is_set() or not rclpy.ok():
        return
    node.arm()
    print('Keyboard ready: WASD/arrows (or configured uiojkl); space/k stop; q/Ctrl-C quit.',
          flush=True)
    while rclpy.ok() and not stopped.is_set():
        key = keyboard.read_key(node.config.period_sec)
        if stopped.is_set():
            break
        if key and node.handle_key(key) == 'quit':
            break
        node.publish_current_command()
        rclpy.spin_once(node, timeout_sec=0.0)


def main(args: Optional[Sequence[str]] = None) -> int:
    """Run a standalone keyboard with exclusive lock and startup publisher checks.

    Args:
        args: Optional ROS arguments; None uses the command line.

    Returns:
        Zero after normal quit/signals, two for lock, TTY, graph or configuration
        rejection. A rejected startup never creates a command publisher.
    """
    try:
        with RunLock():
            return _run(args)
    except (LockBusy, CommandConflict, TerminalUnavailable, ValueError, OSError) as error:
        print(f'chassis_teleop: {error}', file=sys.stderr)
        return 2


def _run(args: Optional[Sequence[str]]) -> int:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous = {}
    stopped = threading.Event()
    node = None

    def stop(_signum: int, _frame: Any) -> None:
        stopped.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, stop)
        node = KeyboardNode()
        with TerminalKeyboard(node.config.tty_device) as keyboard:
            try:
                run_keyboard(node, keyboard, stopped)
            finally:
                node.publish_stop_burst()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                for signum, handler in previous.items():
                    signal.signal(signum, handler)
    return 0
