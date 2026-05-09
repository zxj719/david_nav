"""Low-travel Wheeltec base diagnostics with odom and IMU feedback."""

from dataclasses import dataclass
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


GRAVITY = 9.80665


@dataclass
class Pose2D:
    """Planar pose extracted from odometry."""

    x: float
    y: float
    yaw: float


@dataclass
class ProbeResult:
    """One command probe result."""

    name: str
    command: float
    duration: float
    displacement: float
    max_odom_vx: float
    max_odom_wz: float
    max_imu_accel_delta: float
    max_imu_gyro_delta: float
    bridge_latency: float | None
    odom_latency: float | None
    imu_latency: float | None
    stop_latency: float | None
    moved: bool
    message: str = ""


class SafetyStop(Exception):
    """Raised when the robot reaches the hard travel limit."""


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    """Extract yaw from a quaternion."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_from_odom(odom):
    """Convert odom pose to a compact planar pose."""
    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation
    return Pose2D(position.x, position.y, yaw_from_quaternion(orientation))


def forward_progress(start_pose, current_pose):
    """Project displacement onto the start heading."""
    dx = current_pose.x - start_pose.x
    dy = current_pose.y - start_pose.y
    return dx * math.cos(start_pose.yaw) + dy * math.sin(start_pose.yaw)


def pose_distance(start_pose, current_pose):
    """Planar distance between two poses."""
    return math.hypot(current_pose.x - start_pose.x, current_pose.y - start_pose.y)


def mean(values):
    """Return mean or zero for an empty sequence."""
    return sum(values) / len(values) if values else 0.0


def stddev(values):
    """Return population standard deviation or zero for too few samples."""
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return math.sqrt(sum((value - avg) ** 2 for value in values) / len(values))


def sample_rate(samples):
    """Estimate sample rate from monotonic timestamps."""
    if len(samples) < 2:
        return 0.0
    dt = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / dt if dt > 0.0 else 0.0


class WheeltecMicroEval(Node):
    """Run a micro-motion diagnostic with a hard 10 cm travel guard."""

    def __init__(self, **kwargs):
        super().__init__("wheeltec_micro_eval", **kwargs)

        self.declare_parameter("cmd_vel_topic", "cmd_vel_safe")
        self.declare_parameter("forwarded_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("baseline_duration", 2.0)
        self.declare_parameter("probe_duration", 0.30)
        self.declare_parameter("settle_duration", 0.65)
        self.declare_parameter("hard_distance_limit", 0.10)
        self.declare_parameter("soft_distance_limit", 0.06)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("imu_timeout", 0.5)
        self.declare_parameter("wait_timeout", 8.0)
        self.declare_parameter(
            "linear_levels",
            [0.02, 0.035, 0.05, 0.065, 0.08, 0.10],
        )
        self.declare_parameter(
            "angular_levels",
            [0.05, 0.08, 0.12, 0.16, 0.20, 0.25],
        )
        self.declare_parameter("max_linear_command", 0.10)
        self.declare_parameter("max_angular_command", 0.25)
        self.declare_parameter("odom_vx_threshold", 0.015)
        self.declare_parameter("odom_wz_threshold", 0.025)
        self.declare_parameter("move_distance_threshold", 0.003)
        self.declare_parameter("imu_accel_delta_threshold", 0.08)
        self.declare_parameter("imu_gyro_delta_threshold", 0.025)

        self.cmd_vel_topic = self.string_param("cmd_vel_topic")
        self.forwarded_cmd_vel_topic = self.string_param("forwarded_cmd_vel_topic")
        self.odom_topic = self.string_param("odom_topic")
        self.imu_topic = self.string_param("imu_topic")
        self.publish_hz = max(self.float_param("publish_hz"), 1.0)
        self.baseline_duration = self.float_param("baseline_duration")
        self.probe_duration = self.float_param("probe_duration")
        self.settle_duration = self.float_param("settle_duration")
        self.hard_distance_limit = self.float_param("hard_distance_limit")
        self.soft_distance_limit = self.float_param("soft_distance_limit")
        self.odom_timeout = self.float_param("odom_timeout")
        self.imu_timeout = self.float_param("imu_timeout")
        self.wait_timeout = self.float_param("wait_timeout")
        self.linear_levels = self.float_list_param("linear_levels")
        self.angular_levels = self.float_list_param("angular_levels")
        self.max_linear_command = self.float_param("max_linear_command")
        self.max_angular_command = self.float_param("max_angular_command")
        self.odom_vx_threshold = self.float_param("odom_vx_threshold")
        self.odom_wz_threshold = self.float_param("odom_wz_threshold")
        self.move_distance_threshold = self.float_param("move_distance_threshold")
        self.imu_accel_delta_threshold = self.float_param(
            "imu_accel_delta_threshold"
        )
        self.imu_gyro_delta_threshold = self.float_param("imu_gyro_delta_threshold")

        self.latest_odom = None
        self.latest_odom_time = None
        self.latest_imu = None
        self.latest_imu_time = None
        self.initial_pose = None
        self.baseline = {}

        self.cmd_samples = []
        self.odom_samples = []
        self.imu_samples = []
        self.results = []

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(
            Twist,
            self.forwarded_cmd_vel_topic,
            self.cmd_callback,
            100,
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 100)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 200)

    @property
    def period(self):
        """Command publish period."""
        return 1.0 / self.publish_hz

    def string_param(self, name):
        """Read a string parameter."""
        return self.get_parameter(name).get_parameter_value().string_value

    def float_param(self, name):
        """Read a float parameter."""
        return float(self.get_parameter(name).value)

    def float_list_param(self, name):
        """Read a float list parameter."""
        values = self.get_parameter(name).value
        return [float(value) for value in values]

    def cmd_callback(self, msg):
        """Record forwarded cmd_vel samples for bridge latency."""
        self.cmd_samples.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def odom_callback(self, msg):
        """Record odometry samples."""
        now = time.monotonic()
        self.latest_odom = msg
        self.latest_odom_time = now
        self.odom_samples.append(
            (
                now,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z,
                yaw_from_quaternion(msg.pose.pose.orientation),
            )
        )

    def imu_callback(self, msg):
        """Record IMU samples."""
        now = time.monotonic()
        self.latest_imu = msg
        self.latest_imu_time = now
        self.imu_samples.append(
            (
                now,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                yaw_from_quaternion(msg.orientation),
            )
        )

    def odom_is_fresh(self):
        """Return True when odom has arrived recently."""
        return (
            self.latest_odom_time is not None
            and time.monotonic() - self.latest_odom_time <= self.odom_timeout
        )

    def imu_is_fresh(self):
        """Return True when IMU has arrived recently."""
        return (
            self.latest_imu_time is not None
            and time.monotonic() - self.latest_imu_time <= self.imu_timeout
        )

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        """Publish one safe command sample."""
        twist = Twist()
        twist.linear.x = max(
            min(float(linear_x), self.max_linear_command),
            -self.max_linear_command,
        )
        twist.angular.z = max(
            min(float(angular_z), self.max_angular_command),
            -self.max_angular_command,
        )
        self.cmd_pub.publish(twist)

    def publish_zero_for(self, duration):
        """Publish zero velocity for a bounded duration."""
        end_time = time.monotonic() + max(float(duration), self.period)
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=self.period)

    def wait_for_feedback(self):
        """Wait for odom and IMU feedback before moving."""
        deadline = time.monotonic() + self.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=self.period)
            if self.odom_is_fresh() and self.imu_is_fresh():
                return True
        return False

    def guard_travel(self):
        """Stop immediately if odom reports travel near the hard limit."""
        if self.initial_pose is None or self.latest_odom is None:
            return
        current_pose = pose_from_odom(self.latest_odom)
        distance = pose_distance(self.initial_pose, current_pose)
        if distance >= self.hard_distance_limit:
            self.publish_zero_for(2.0)
            raise SafetyStop(
                f"hard travel limit reached: {distance:.3f}m "
                f">= {self.hard_distance_limit:.3f}m"
            )

    def collect_baseline(self):
        """Collect stationary odom and IMU statistics."""
        start_cmd = len(self.cmd_samples)
        start_odom = len(self.odom_samples)
        start_imu = len(self.imu_samples)
        self.publish_zero_for(self.baseline_duration)

        odom = self.odom_samples[start_odom:]
        imu = self.imu_samples[start_imu:]
        cmd = self.cmd_samples[start_cmd:]
        accel_norms = [
            math.sqrt(sample[1] ** 2 + sample[2] ** 2 + sample[3] ** 2)
            for sample in imu
        ]
        yaw_values = [sample[7] for sample in imu]

        self.baseline = {
            "odom_rate": sample_rate(odom),
            "imu_rate": sample_rate(imu),
            "cmd_rate": sample_rate(cmd),
            "accel_x": mean([sample[1] for sample in imu]),
            "accel_y": mean([sample[2] for sample in imu]),
            "accel_z": mean([sample[3] for sample in imu]),
            "accel_norm": mean(accel_norms),
            "accel_norm_std": stddev(accel_norms),
            "gyro_x": mean([sample[4] for sample in imu]),
            "gyro_y": mean([sample[5] for sample in imu]),
            "gyro_z": mean([sample[6] for sample in imu]),
            "gyro_z_std": stddev([sample[6] for sample in imu]),
            "imu_yaw_std": stddev(yaw_values),
            "odom_vx_std": stddev([sample[3] for sample in odom]),
            "odom_wz_std": stddev([sample[4] for sample in odom]),
        }

    def first_bridge_latency(self, start_time, linear_x=0.0, angular_z=0.0):
        """Find first forwarded cmd_vel sample matching the active command."""
        for stamp, vx, wz in self.cmd_samples:
            if stamp < start_time:
                continue
            linear_match = linear_x and abs(vx) >= min(abs(linear_x) * 0.5, 0.01)
            angular_match = angular_z and abs(wz) >= min(abs(angular_z) * 0.5, 0.01)
            if linear_match or angular_match:
                return stamp - start_time
        return None

    def measure_stop_latency(self, stop_time, angular=False):
        """Measure odom stop latency after zero command publication."""
        deadline = stop_time + self.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=self.period)
            if not self.odom_is_fresh():
                continue
            twist = self.latest_odom.twist.twist
            if angular:
                stopped = abs(twist.angular.z) < self.odom_wz_threshold
            else:
                stopped = abs(twist.linear.x) < self.odom_vx_threshold
            if stopped:
                return time.monotonic() - stop_time
        return None

    def run_linear_probe(self, level):
        """Run one short forward velocity probe."""
        start_pose = pose_from_odom(self.latest_odom)
        start_time = time.monotonic()
        end_time = start_time + self.probe_duration
        max_vx = 0.0
        max_accel_delta = 0.0
        odom_latency = None
        imu_latency = None
        displacement = 0.0

        while rclpy.ok() and time.monotonic() < end_time:
            self.guard_travel()
            self.publish_cmd(linear_x=level)
            rclpy.spin_once(self, timeout_sec=self.period)
            if not self.odom_is_fresh() or not self.imu_is_fresh():
                continue

            current_pose = pose_from_odom(self.latest_odom)
            displacement = forward_progress(start_pose, current_pose)
            total_distance = pose_distance(self.initial_pose, current_pose)
            twist = self.latest_odom.twist.twist
            accel_delta = abs(
                self.latest_imu.linear_acceleration.x - self.baseline["accel_x"]
            )
            max_vx = max(max_vx, abs(twist.linear.x))
            max_accel_delta = max(max_accel_delta, accel_delta)

            if (
                odom_latency is None
                and (
                    abs(twist.linear.x) >= self.odom_vx_threshold
                    or abs(displacement) >= self.move_distance_threshold
                )
            ):
                odom_latency = time.monotonic() - start_time
            if imu_latency is None and accel_delta >= self.imu_accel_delta_threshold:
                imu_latency = time.monotonic() - start_time
            if total_distance >= self.soft_distance_limit:
                break

        stop_time = time.monotonic()
        self.publish_cmd()
        stop_latency = self.measure_stop_latency(stop_time, angular=False)
        moved = (
            max_vx >= self.odom_vx_threshold
            or abs(displacement) >= self.move_distance_threshold
        )
        return ProbeResult(
            name="linear",
            command=level,
            duration=time.monotonic() - start_time,
            displacement=displacement,
            max_odom_vx=max_vx,
            max_odom_wz=0.0,
            max_imu_accel_delta=max_accel_delta,
            max_imu_gyro_delta=0.0,
            bridge_latency=self.first_bridge_latency(start_time, linear_x=level),
            odom_latency=odom_latency,
            imu_latency=imu_latency,
            stop_latency=stop_latency,
            moved=moved,
        )

    def run_angular_probe(self, level):
        """Run one short yaw velocity probe."""
        start_pose = pose_from_odom(self.latest_odom)
        start_time = time.monotonic()
        end_time = start_time + self.probe_duration
        max_wz = 0.0
        max_gyro_delta = 0.0
        odom_latency = None
        imu_latency = None

        while rclpy.ok() and time.monotonic() < end_time:
            self.guard_travel()
            self.publish_cmd(angular_z=level)
            rclpy.spin_once(self, timeout_sec=self.period)
            if not self.odom_is_fresh() or not self.imu_is_fresh():
                continue

            current_pose = pose_from_odom(self.latest_odom)
            yaw_delta = abs(normalize_angle(current_pose.yaw - start_pose.yaw))
            twist = self.latest_odom.twist.twist
            gyro_delta = abs(
                self.latest_imu.angular_velocity.z - self.baseline["gyro_z"]
            )
            max_wz = max(max_wz, abs(twist.angular.z))
            max_gyro_delta = max(max_gyro_delta, gyro_delta)

            if (
                odom_latency is None
                and (abs(twist.angular.z) >= self.odom_wz_threshold or yaw_delta > 0.01)
            ):
                odom_latency = time.monotonic() - start_time
            if imu_latency is None and gyro_delta >= self.imu_gyro_delta_threshold:
                imu_latency = time.monotonic() - start_time

        stop_time = time.monotonic()
        self.publish_cmd()
        stop_latency = self.measure_stop_latency(stop_time, angular=True)
        moved = max_wz >= self.odom_wz_threshold or max_gyro_delta >= self.imu_gyro_delta_threshold
        return ProbeResult(
            name="angular",
            command=level,
            duration=time.monotonic() - start_time,
            displacement=0.0,
            max_odom_vx=0.0,
            max_odom_wz=max_wz,
            max_imu_accel_delta=0.0,
            max_imu_gyro_delta=max_gyro_delta,
            bridge_latency=self.first_bridge_latency(start_time, angular_z=level),
            odom_latency=odom_latency,
            imu_latency=imu_latency,
            stop_latency=stop_latency,
            moved=moved,
        )

    def reset_translation(self):
        """Return close to the initial odom x/y pose when possible."""
        if self.initial_pose is None or self.latest_odom is None:
            return
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.guard_travel()
            current_pose = pose_from_odom(self.latest_odom)
            error = forward_progress(self.initial_pose, current_pose)
            if abs(error) < 0.008:
                break
            command = -0.06 if error > 0.0 else 0.06
            self.publish_cmd(linear_x=command)
            rclpy.spin_once(self, timeout_sec=self.period)
        self.publish_zero_for(self.settle_duration)

    def log_baseline(self):
        """Log stationary baseline metrics."""
        accel_error = self.baseline["accel_norm"] - GRAVITY
        self.get_logger().info(
            "baseline: "
            f"odom_rate={self.baseline['odom_rate']:.1f}Hz, "
            f"imu_rate={self.baseline['imu_rate']:.1f}Hz, "
            f"cmd_vel_rate={self.baseline['cmd_rate']:.1f}Hz"
        )
        self.get_logger().info(
            "imu_static: "
            f"accel_mean=({self.baseline['accel_x']:.3f}, "
            f"{self.baseline['accel_y']:.3f}, "
            f"{self.baseline['accel_z']:.3f})m/s^2, "
            f"|a|-g={accel_error:.3f}m/s^2, "
            f"|a|_std={self.baseline['accel_norm_std']:.4f}, "
            f"gyro_bias=({self.baseline['gyro_x']:.4f}, "
            f"{self.baseline['gyro_y']:.4f}, "
            f"{self.baseline['gyro_z']:.4f})rad/s, "
            f"gyro_z_std={self.baseline['gyro_z_std']:.4f}, "
            f"yaw_std={math.degrees(self.baseline['imu_yaw_std']):.3f}deg"
        )

    def log_result(self, result):
        """Log one probe result."""
        def fmt_latency(value):
            return "n/a" if value is None else f"{value:.3f}s"

        self.get_logger().info(
            f"{result.name}_probe cmd={result.command:.3f}: "
            f"bridge_latency={fmt_latency(result.bridge_latency)}, "
            f"odom_latency={fmt_latency(result.odom_latency)}, "
            f"imu_latency={fmt_latency(result.imu_latency)}, "
            f"stop_latency={fmt_latency(result.stop_latency)}, "
            f"disp={result.displacement:.3f}m, "
            f"max_vx={result.max_odom_vx:.3f}m/s, "
            f"max_wz={result.max_odom_wz:.3f}rad/s, "
            f"max_accel_delta={result.max_imu_accel_delta:.3f}m/s^2, "
            f"max_gyro_delta={result.max_imu_gyro_delta:.3f}rad/s, "
            f"moved={result.moved}"
        )

    def graph_summary(self):
        """Log current endpoint counts for the key topics."""
        cmd_publishers = self.get_publishers_info_by_topic(
            self.forwarded_cmd_vel_topic
        )
        cmd_subscribers = self.get_subscriptions_info_by_topic(
            self.forwarded_cmd_vel_topic
        )
        self.get_logger().info(
            "graph: "
            f"{self.forwarded_cmd_vel_topic} publishers={len(cmd_publishers)}, "
            f"subscribers={len(cmd_subscribers)}; "
            f"{self.cmd_vel_topic} subscribers={self.cmd_pub.get_subscription_count()}"
        )

    def run(self):
        """Execute the micro diagnostic."""
        self.get_logger().info(
            "Starting micro eval: "
            f"cmd={self.cmd_vel_topic}, forwarded={self.forwarded_cmd_vel_topic}, "
            f"odom={self.odom_topic}, imu={self.imu_topic}, "
            f"hard_limit={self.hard_distance_limit:.3f}m"
        )
        self.graph_summary()
        self.publish_zero_for(1.0)
        if not self.wait_for_feedback():
            self.get_logger().error("FAIL: missing fresh odom or IMU feedback")
            self.publish_zero_for(2.0)
            return False

        self.initial_pose = pose_from_odom(self.latest_odom)
        self.collect_baseline()
        self.log_baseline()

        linear_deadzone = None
        accel_deadzone = None
        for level in self.linear_levels:
            result = self.run_linear_probe(level)
            self.results.append(result)
            self.log_result(result)
            self.publish_zero_for(self.settle_duration)
            if result.moved and linear_deadzone is None:
                linear_deadzone = level
            if result.imu_latency is not None and accel_deadzone is None:
                accel_deadzone = level
            if linear_deadzone is not None and accel_deadzone is not None:
                break

        self.reset_translation()

        angular_deadzone = None
        gyro_deadzone = None
        for level in self.angular_levels:
            result = self.run_angular_probe(level)
            self.results.append(result)
            self.log_result(result)
            self.publish_zero_for(self.settle_duration)
            if result.odom_latency is not None and angular_deadzone is None:
                angular_deadzone = level
            if result.imu_latency is not None and gyro_deadzone is None:
                gyro_deadzone = level
            if angular_deadzone is not None and gyro_deadzone is not None:
                break

        self.publish_zero_for(2.0)
        final_pose = pose_from_odom(self.latest_odom)
        final_distance = pose_distance(self.initial_pose, final_pose)
        final_yaw = abs(normalize_angle(final_pose.yaw - self.initial_pose.yaw))
        passed = final_distance < self.hard_distance_limit

        self.get_logger().info(
            "deadzone_summary: "
            f"linear_odom={linear_deadzone}, "
            f"linear_imu_accel={accel_deadzone}, "
            f"angular_odom={angular_deadzone}, "
            f"angular_imu_gyro={gyro_deadzone}"
        )
        self.get_logger().info(
            "final: "
            f"distance_from_start={final_distance:.3f}m, "
            f"yaw_from_start={math.degrees(final_yaw):.2f}deg, "
            f"{'PASS' if passed else 'FAIL'}"
        )
        return passed


def main(args=None):
    """Run the low-travel base and IMU diagnostic."""
    rclpy.init(args=args)
    node = WheeltecMicroEval()

    try:
        passed = node.run()
    except KeyboardInterrupt:
        passed = False
        node.get_logger().warn("Interrupted, publishing zero velocity")
        node.publish_zero_for(2.0)
    except SafetyStop as exc:
        passed = False
        node.get_logger().error(f"SAFETY STOP: {exc}")
        node.publish_zero_for(2.0)
    except Exception as exc:
        passed = False
        node.get_logger().error(f"Evaluation failed: {exc}")
        node.publish_zero_for(2.0)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
