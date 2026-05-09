"""Closed-loop Wheeltec base velocity evaluation using wheel odometry."""

from dataclasses import dataclass
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


DEFAULT_CMD_TOPIC = "cmd_vel_safe"
DEFAULT_ODOM_TOPIC = "/odom"
DEFAULT_PUBLISH_HZ = 20.0
DEFAULT_LINEAR_SPEED = 0.10
DEFAULT_ANGULAR_SPEED = 0.30
DEFAULT_TARGET_DISTANCE = 0.25
DEFAULT_TARGET_YAW_DEG = 60.0
DEFAULT_SEGMENT_TIMEOUT = 8.0
DEFAULT_ODOM_TIMEOUT = 0.5
DEFAULT_ODOM_WAIT_TIMEOUT = 5.0
DEFAULT_LINEAR_TWIST_THRESHOLD = 0.02
DEFAULT_ANGULAR_TWIST_THRESHOLD = 0.03
DEFAULT_DISTANCE_TOLERANCE = 0.05
DEFAULT_YAW_TOLERANCE_DEG = 5.0
DEFAULT_RESET_DISTANCE_TOLERANCE = 0.06
DEFAULT_RESET_YAW_TOLERANCE_DEG = 8.0
DEFAULT_REVERSE_DRIFT_LIMIT = 0.03


@dataclass
class Pose2D:
    """Planar odometry pose."""

    x: float
    y: float
    yaw: float


@dataclass
class SegmentResult:
    """Result for one evaluation segment."""

    name: str
    target: float
    actual: float
    error: float
    start_latency: float | None
    stop_latency: float | None
    passed: bool
    message: str = ""


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    """Extract yaw from a geometry_msgs Quaternion."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_from_odom(odom):
    """Convert nav_msgs/Odometry to Pose2D."""
    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation
    return Pose2D(position.x, position.y, yaw_from_quaternion(orientation))


def forward_progress(start_pose, current_pose):
    """Project odom displacement onto the robot heading at segment start."""
    dx = current_pose.x - start_pose.x
    dy = current_pose.y - start_pose.y
    return dx * math.cos(start_pose.yaw) + dy * math.sin(start_pose.yaw)


def yaw_progress(start_pose, current_pose):
    """Return signed yaw progress from the segment start."""
    return normalize_angle(current_pose.yaw - start_pose.yaw)


def pose_error(reference_pose, current_pose):
    """Return planar distance and yaw error between two poses."""
    distance = math.hypot(
        current_pose.x - reference_pose.x,
        current_pose.y - reference_pose.y,
    )
    yaw_error = abs(normalize_angle(current_pose.yaw - reference_pose.yaw))
    return distance, yaw_error


def linear_motion_started(odom, threshold):
    """Check whether wheel odometry reports linear motion."""
    twist = odom.twist.twist
    return abs(twist.linear.x) >= threshold


def angular_motion_started(odom, threshold):
    """Check whether wheel odometry reports angular motion."""
    twist = odom.twist.twist
    return abs(twist.angular.z) >= threshold


class WheeltecBaseEval(Node):
    """Run a fixed odom-closed-loop base motion evaluation."""

    def __init__(self, **kwargs):
        super().__init__("wheeltec_base_eval", **kwargs)

        self.declare_parameter("cmd_vel_topic", DEFAULT_CMD_TOPIC)
        self.declare_parameter("odom_topic", DEFAULT_ODOM_TOPIC)
        self.declare_parameter("publish_hz", DEFAULT_PUBLISH_HZ)
        self.declare_parameter("linear_speed", DEFAULT_LINEAR_SPEED)
        self.declare_parameter("angular_speed", DEFAULT_ANGULAR_SPEED)
        self.declare_parameter("target_distance", DEFAULT_TARGET_DISTANCE)
        self.declare_parameter("target_yaw_deg", DEFAULT_TARGET_YAW_DEG)
        self.declare_parameter("segment_timeout", DEFAULT_SEGMENT_TIMEOUT)
        self.declare_parameter("odom_timeout", DEFAULT_ODOM_TIMEOUT)
        self.declare_parameter("odom_wait_timeout", DEFAULT_ODOM_WAIT_TIMEOUT)
        self.declare_parameter(
            "linear_twist_threshold",
            DEFAULT_LINEAR_TWIST_THRESHOLD,
        )
        self.declare_parameter(
            "angular_twist_threshold",
            DEFAULT_ANGULAR_TWIST_THRESHOLD,
        )
        self.declare_parameter("distance_tolerance", DEFAULT_DISTANCE_TOLERANCE)
        self.declare_parameter("yaw_tolerance_deg", DEFAULT_YAW_TOLERANCE_DEG)
        self.declare_parameter(
            "reset_distance_tolerance",
            DEFAULT_RESET_DISTANCE_TOLERANCE,
        )
        self.declare_parameter(
            "reset_yaw_tolerance_deg",
            DEFAULT_RESET_YAW_TOLERANCE_DEG,
        )
        self.declare_parameter("reverse_drift_limit", DEFAULT_REVERSE_DRIFT_LIMIT)

        self.cmd_vel_topic = self.get_parameter(
            "cmd_vel_topic"
        ).get_parameter_value().string_value
        self.odom_topic = self.get_parameter(
            "odom_topic"
        ).get_parameter_value().string_value
        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.linear_speed = abs(float(self.get_parameter("linear_speed").value))
        self.angular_speed = abs(float(self.get_parameter("angular_speed").value))
        self.target_distance = abs(
            float(self.get_parameter("target_distance").value)
        )
        self.target_yaw = math.radians(
            abs(float(self.get_parameter("target_yaw_deg").value))
        )
        self.segment_timeout = float(self.get_parameter("segment_timeout").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.odom_wait_timeout = float(
            self.get_parameter("odom_wait_timeout").value
        )
        self.linear_twist_threshold = float(
            self.get_parameter("linear_twist_threshold").value
        )
        self.angular_twist_threshold = float(
            self.get_parameter("angular_twist_threshold").value
        )
        self.distance_tolerance = float(
            self.get_parameter("distance_tolerance").value
        )
        self.yaw_tolerance = math.radians(
            float(self.get_parameter("yaw_tolerance_deg").value)
        )
        self.reset_distance_tolerance = float(
            self.get_parameter("reset_distance_tolerance").value
        )
        self.reset_yaw_tolerance = math.radians(
            float(self.get_parameter("reset_yaw_tolerance_deg").value)
        )
        self.reverse_drift_limit = float(
            self.get_parameter("reverse_drift_limit").value
        )

        self.latest_odom = None
        self.latest_odom_time = None
        self.initial_pose = None
        self.results = []

        self.cmd_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)

    def odom_callback(self, msg):
        """Store latest odometry feedback."""
        self.latest_odom = msg
        self.latest_odom_time = time.monotonic()

    @property
    def period(self):
        """Command publication period."""
        return 1.0 / self.publish_hz

    def odom_is_fresh(self):
        """Return True if odom has arrived recently enough."""
        if self.latest_odom is None or self.latest_odom_time is None:
            return False
        return time.monotonic() - self.latest_odom_time <= self.odom_timeout

    def wait_for_fresh_odom(self, timeout=None):
        """Wait for odom before starting any closed-loop movement."""
        timeout = self.odom_wait_timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=self.period)
            if self.odom_is_fresh():
                return True
        return False

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        """Publish one test command."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular_z)
        self.cmd_publisher.publish(twist)

    def publish_zero_burst(self, duration_sec=0.5):
        """Continuously publish zero velocity for a bounded duration."""
        end_time = time.monotonic() + max(float(duration_sec), self.period)
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=self.period)

    def stop_and_measure(self, threshold_fn, threshold):
        """Publish zero velocity and measure stop latency from odom twist."""
        start = time.monotonic()
        deadline = start + self.odom_wait_timeout
        stop_latency = None
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=self.period)
            if self.odom_is_fresh() and not threshold_fn(self.latest_odom, threshold):
                stop_latency = time.monotonic() - start
                break
        return stop_latency

    def run_distance_segment(self, name, target_distance, speed):
        """Run one forward or backward odom-closed-loop segment."""
        sign = 1.0 if target_distance >= 0.0 else -1.0
        start_pose = pose_from_odom(self.latest_odom)
        start_time = time.monotonic()
        deadline = start_time + self.segment_timeout
        start_latency = None
        actual = 0.0
        message = ""
        passed = False

        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_cmd(linear_x=sign * abs(speed))
            rclpy.spin_once(self, timeout_sec=self.period)

            if not self.odom_is_fresh():
                message = "odom timeout"
                break

            current_pose = pose_from_odom(self.latest_odom)
            signed_progress = forward_progress(start_pose, current_pose)
            directed_progress = signed_progress * sign
            actual = directed_progress

            if (
                start_latency is None
                and linear_motion_started(
                    self.latest_odom,
                    self.linear_twist_threshold,
                )
            ):
                start_latency = time.monotonic() - start_time

            if directed_progress < -self.reverse_drift_limit:
                message = "reverse odom drift"
                break

            if directed_progress >= abs(target_distance):
                passed = True
                break

        if not passed and not message:
            message = "segment timeout"

        stop_latency = self.stop_and_measure(
            linear_motion_started,
            self.linear_twist_threshold,
        )
        error = abs(actual - abs(target_distance))
        passed = passed and error <= self.distance_tolerance
        if stop_latency is None:
            passed = False
            message = message or "stop timeout"

        return SegmentResult(
            name=name,
            target=target_distance,
            actual=actual * sign,
            error=error,
            start_latency=start_latency,
            stop_latency=stop_latency,
            passed=passed,
            message=message,
        )

    def run_turn_segment(self, name, target_yaw, speed):
        """Run one left or right odom-closed-loop turn segment."""
        sign = 1.0 if target_yaw >= 0.0 else -1.0
        start_pose = pose_from_odom(self.latest_odom)
        start_time = time.monotonic()
        deadline = start_time + self.segment_timeout
        start_latency = None
        actual = 0.0
        message = ""
        passed = False

        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_cmd(angular_z=sign * abs(speed))
            rclpy.spin_once(self, timeout_sec=self.period)

            if not self.odom_is_fresh():
                message = "odom timeout"
                break

            current_pose = pose_from_odom(self.latest_odom)
            signed_progress = yaw_progress(start_pose, current_pose)
            directed_progress = signed_progress * sign
            actual = directed_progress

            if (
                start_latency is None
                and angular_motion_started(
                    self.latest_odom,
                    self.angular_twist_threshold,
                )
            ):
                start_latency = time.monotonic() - start_time

            if directed_progress < -math.radians(3.0):
                message = "reverse yaw drift"
                break

            if directed_progress >= abs(target_yaw):
                passed = True
                break

        if not passed and not message:
            message = "segment timeout"

        stop_latency = self.stop_and_measure(
            angular_motion_started,
            self.angular_twist_threshold,
        )
        error = abs(actual - abs(target_yaw))
        passed = passed and error <= self.yaw_tolerance
        if stop_latency is None:
            passed = False
            message = message or "stop timeout"

        return SegmentResult(
            name=name,
            target=target_yaw,
            actual=actual * sign,
            error=error,
            start_latency=start_latency,
            stop_latency=stop_latency,
            passed=passed,
            message=message,
        )

    def log_segment_result(self, result, angular=False):
        """Print a segment result in human-readable units."""
        if angular:
            target = math.degrees(result.target)
            actual = math.degrees(result.actual)
            error = math.degrees(result.error)
            unit = "deg"
        else:
            target = result.target
            actual = result.actual
            error = result.error
            unit = "m"

        start_latency = (
            "n/a"
            if result.start_latency is None
            else f"{result.start_latency:.3f}s"
        )
        stop_latency = (
            "n/a"
            if result.stop_latency is None
            else f"{result.stop_latency:.3f}s"
        )
        status = "PASS" if result.passed else "FAIL"
        message = f", reason={result.message}" if result.message else ""
        self.get_logger().info(
            f"{result.name}: target={target:.3f}{unit}, "
            f"actual={actual:.3f}{unit}, error={error:.3f}{unit}, "
            f"start_latency={start_latency}, stop_latency={stop_latency}, "
            f"{status}{message}"
        )

    def run(self):
        """Execute the full evaluation sequence."""
        self.get_logger().info(
            f"Starting Wheeltec base eval: cmd={self.cmd_vel_topic}, "
            f"odom={self.odom_topic}, no IMU"
        )

        self.publish_zero_burst(duration_sec=1.0)
        if not self.wait_for_fresh_odom():
            self.get_logger().error("FAIL: no fresh /odom feedback")
            self.publish_zero_burst(duration_sec=2.0)
            return False

        self.initial_pose = pose_from_odom(self.latest_odom)
        sequence = [
            (
                self.run_distance_segment,
                "forward",
                self.target_distance,
                self.linear_speed,
                False,
            ),
            (
                self.run_distance_segment,
                "backward",
                -self.target_distance,
                self.linear_speed,
                False,
            ),
            (
                self.run_turn_segment,
                "turn_left",
                self.target_yaw,
                self.angular_speed,
                True,
            ),
            (
                self.run_turn_segment,
                "turn_right",
                -self.target_yaw,
                self.angular_speed,
                True,
            ),
        ]

        passed = True
        for runner, name, target, speed, angular in sequence:
            result = runner(name, target, speed)
            self.results.append(result)
            self.log_segment_result(result, angular=angular)
            passed = passed and result.passed
            self.publish_zero_burst(duration_sec=1.0)
            if not result.passed:
                break

        self.publish_zero_burst(duration_sec=2.0)

        if self.odom_is_fresh():
            final_pose = pose_from_odom(self.latest_odom)
            distance_error, yaw_error = pose_error(self.initial_pose, final_pose)
        else:
            distance_error = float("inf")
            yaw_error = float("inf")

        reset_passed = (
            distance_error <= self.reset_distance_tolerance
            and yaw_error <= self.reset_yaw_tolerance
        )
        passed = passed and reset_passed

        self.get_logger().info(
            "reset: "
            f"distance_error={distance_error:.3f}m, "
            f"yaw_error={math.degrees(yaw_error):.2f}deg, "
            f"{'PASS' if reset_passed else 'FAIL'}"
        )
        self.get_logger().info(f"FINAL {'PASS' if passed else 'FAIL'}")
        return passed


def main(args=None):
    """Run Wheeltec base evaluation."""
    rclpy.init(args=args)
    node = WheeltecBaseEval()

    try:
        passed = node.run()
    except KeyboardInterrupt:
        passed = False
        node.get_logger().warn("Interrupted, publishing zero velocity")
        node.publish_zero_burst(duration_sec=2.0)
    except Exception as exc:
        passed = False
        node.get_logger().error(f"Evaluation failed: {exc}")
        node.publish_zero_burst(duration_sec=2.0)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
