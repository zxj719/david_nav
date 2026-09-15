"""ROS interface for the pure adapter, with diagnostics and orderly exit-zero."""

from dataclasses import asdict
import signal
import threading
import time
from typing import Any, Optional, Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from chassis_adapter.core import AdapterConfig, AdapterState


class AdapterNode(Node):
    """Adapt configured command topics without owning hardware or localization."""

    def __init__(self, **kwargs: Any) -> None:
        """Declare legacy-compatible parameters and construct the configured endpoints.

        Args:
            **kwargs: Standard rclpy Node options, including parameter overrides.
        """
        super().__init__('chassis_adapter', **kwargs)
        defaults = asdict(AdapterConfig())
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = AdapterConfig(**{
            name: self.get_parameter(name).value for name in defaults})
        self.config.validate()
        self.state = AdapterState(self.config, self._now())
        self.cmd_publisher = self.create_publisher(Twist, self.config.output_cmd_vel_topic, 10)
        self.create_subscription(
            Twist, self.config.input_cmd_vel_topic, self.velocity_callback, 10)
        self.create_timer(self.config.period_sec, self.publish_current_command)
        self.diagnostic_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._diagnostic_time = time.monotonic()
        self._diagnostic_count = 0
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f'Command adapter ready: {self.config.input_cmd_vel_topic} -> '
            f'{self.config.output_cmd_vel_topic}; timeout={self.config.cmd_vel_timeout:.2f}s')

    def _now(self) -> int:
        return self.get_clock().now().nanoseconds

    def velocity_callback(self, message: Twist) -> None:
        """Store a command without publishing from the subscription callback.

        Args:
            message: Input Twist, with only forward and yaw retained in output.
        """
        self.state.receive(message.linear.x, message.linear.y, message.angular.z, self._now())

    def publish_current_command(self) -> None:
        """Publish the latest command or the single timeout stop, then remain silent."""
        output = self.state.output(self._now())
        if output is not None:
            message = Twist()
            message.linear.x, message.angular.z = output
            self.cmd_publisher.publish(message)

    def publish_diagnostics(self) -> None:
        """Publish one-hertz input rate, timeout state and current clamped values."""
        now = time.monotonic()
        elapsed = now - self._diagnostic_time
        values = self.state.diagnostics(self._now())
        values['input_hz'] = ((self.state.input_count - self._diagnostic_count) / elapsed
                              if elapsed > 0.0 else 0.0)
        self._diagnostic_time = now
        self._diagnostic_count = self.state.input_count
        status = DiagnosticStatus()
        status.name = self.get_fully_qualified_name()
        status.level = DiagnosticStatus.WARN if values['timed_out'] else DiagnosticStatus.OK
        status.message = 'input stale: output released after stop' if values['timed_out'] else 'OK'
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostic_publisher.publish(message)

    def publish_zero(self) -> None:
        """Attempt one explicit stop while the ROS context is still usable."""
        try:
            self.cmd_publisher.publish(Twist())
        except Exception as error:
            # Report transport failure; publishing is not evidence of a hardware stop.
            self.get_logger().warning(f'Unable to publish stop command: {error}')

    def publish_stop_burst(self, repetitions: int = 5, period_sec: float = 0.02) -> None:
        """Attempt the legacy five-message stop burst before destroying the publisher.

        Args:
            repetitions: At least one attempt, default five.
            period_sec: Nonnegative gap between attempts.
        """
        for _ in range(max(int(repetitions), 1)):
            self.publish_zero()
            time.sleep(max(float(period_sec), 0.0))


def main(args: Optional[Sequence[str]] = None) -> int:
    """Run until INT/TERM, publishing zero before node and context destruction.

    Args:
        args: Optional ROS command-line arguments.

    Returns:
        Zero for an orderly stop. Unexpected errors propagate after cleanup.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopped = threading.Event()
    previous = {}
    node = None

    def stop(_signum: int, _frame: Any) -> None:
        stopped.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, stop)
        node = AdapterNode()
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.publish_stop_burst()
                finally:
                    node.destroy_node()
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                for signum, handler in previous.items():
                    signal.signal(signum, handler)
    return 0
