"""Direct cmd_vel pulse probe using IMU only, without odometry feedback."""

from dataclasses import dataclass
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu


GRAVITY = 9.80665


@dataclass
class ImuStats:
    """Summary of IMU samples."""

    count: int
    rate_hz: float
    accel_mean: tuple[float, float, float]
    accel_norm_mean: float
    accel_norm_std: float
    gyro_mean: tuple[float, float, float]
    gyro_norm_mean: float
    gyro_norm_std: float


@dataclass
class PulseResult:
    """One command pulse result."""

    name: str
    linear_x: float
    angular_z: float
    duration: float
    expected_max_linear_travel: float
    expected_max_yaw: float
    cmd_latency: float | None
    max_accel_delta: float
    max_gyro_delta: float
    moved_by_imu: bool


def mean(values):
    """Return mean or zero for an empty sequence."""
    return sum(values) / len(values) if values else 0.0


def stddev(values):
    """Return population stddev or zero for empty input."""
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return math.sqrt(sum((value - avg) ** 2 for value in values) / len(values))


def sample_rate(samples):
    """Estimate sample rate from monotonic timestamps."""
    if len(samples) < 2:
        return 0.0
    duration = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / duration if duration > 0.0 else 0.0


def imu_stats(samples):
    """Compute static IMU statistics."""
    accel_norms = [
        math.sqrt(sample[1] ** 2 + sample[2] ** 2 + sample[3] ** 2)
        for sample in samples
    ]
    gyro_norms = [
        math.sqrt(sample[4] ** 2 + sample[5] ** 2 + sample[6] ** 2)
        for sample in samples
    ]
    return ImuStats(
        count=len(samples),
        rate_hz=sample_rate(samples),
        accel_mean=(
            mean([sample[1] for sample in samples]),
            mean([sample[2] for sample in samples]),
            mean([sample[3] for sample in samples]),
        ),
        accel_norm_mean=mean(accel_norms),
        accel_norm_std=stddev(accel_norms),
        gyro_mean=(
            mean([sample[4] for sample in samples]),
            mean([sample[5] for sample in samples]),
            mean([sample[6] for sample in samples]),
        ),
        gyro_norm_mean=mean(gyro_norms),
        gyro_norm_std=stddev(gyro_norms),
    )


class WheeltecImuCmdProbe(Node):
    """Send very short cmd_vel pulses and judge response from IMU only."""

    def __init__(self, **kwargs):
        super().__init__("wheeltec_imu_cmd_probe", **kwargs)

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("listen_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("baseline_duration", 2.0)
        self.declare_parameter("pulse_duration", 0.25)
        self.declare_parameter("settle_duration", 1.0)
        self.declare_parameter("linear_levels", [0.12, 0.16, 0.20])
        self.declare_parameter("angular_levels", [0.25, 0.35, 0.45])
        self.declare_parameter("max_expected_travel", 0.10)
        self.declare_parameter("accel_delta_threshold", 0.08)
        self.declare_parameter("gyro_delta_threshold", 0.025)
        self.declare_parameter("wait_timeout", 6.0)

        self.cmd_vel_topic = self.string_param("cmd_vel_topic")
        self.imu_topic = self.string_param("imu_topic")
        self.listen_cmd_vel_topic = self.string_param("listen_cmd_vel_topic")
        self.publish_hz = max(self.float_param("publish_hz"), 1.0)
        self.baseline_duration = self.float_param("baseline_duration")
        self.pulse_duration = self.float_param("pulse_duration")
        self.settle_duration = self.float_param("settle_duration")
        self.linear_levels = self.float_list_param("linear_levels")
        self.angular_levels = self.float_list_param("angular_levels")
        self.max_expected_travel = self.float_param("max_expected_travel")
        self.accel_delta_threshold = self.float_param("accel_delta_threshold")
        self.gyro_delta_threshold = self.float_param("gyro_delta_threshold")
        self.wait_timeout = self.float_param("wait_timeout")

        self.imu_samples = []
        self.cmd_samples = []
        self.baseline = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 200)
        self.create_subscription(
            Twist,
            self.listen_cmd_vel_topic,
            self.cmd_callback,
            100,
        )

    @property
    def period(self):
        """Publish period."""
        return 1.0 / self.publish_hz

    def string_param(self, name):
        """Read string parameter."""
        return self.get_parameter(name).get_parameter_value().string_value

    def float_param(self, name):
        """Read float parameter."""
        return float(self.get_parameter(name).value)

    def float_list_param(self, name):
        """Read float list parameter."""
        return [float(value) for value in self.get_parameter(name).value]

    def imu_callback(self, msg):
        """Record IMU sample."""
        self.imu_samples.append(
            (
                time.monotonic(),
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            )
        )

    def cmd_callback(self, msg):
        """Record observed cmd_vel sample."""
        self.cmd_samples.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        """Publish one cmd_vel sample."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def publish_zero_for(self, duration):
        """Publish zero velocity for a bounded duration."""
        end = time.monotonic() + max(float(duration), self.period)
        while rclpy.ok() and time.monotonic() < end:
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=self.period)

    def wait_for_imu(self):
        """Wait for IMU samples before sending commands."""
        deadline = time.monotonic() + self.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=self.period)
            if self.imu_samples:
                return True
        return False

    def collect_baseline(self):
        """Collect stationary IMU baseline."""
        start = len(self.imu_samples)
        self.publish_zero_for(self.baseline_duration)
        self.baseline = imu_stats(self.imu_samples[start:])

    def cmd_latency(self, start_time, linear_x, angular_z):
        """Find latency until the command is visible on cmd_vel."""
        for stamp, vx, wz in self.cmd_samples:
            if stamp < start_time:
                continue
            linear_match = linear_x and abs(vx) >= abs(linear_x) * 0.5
            angular_match = angular_z and abs(wz) >= abs(angular_z) * 0.5
            if linear_match or angular_match:
                return stamp - start_time
        return None

    def run_pulse(self, name, linear_x=0.0, angular_z=0.0):
        """Run one short command pulse and inspect IMU deltas."""
        expected_travel = abs(linear_x) * self.pulse_duration
        if expected_travel > self.max_expected_travel:
            raise RuntimeError(
                f"refusing {name}: expected travel {expected_travel:.3f}m "
                f"> {self.max_expected_travel:.3f}m"
            )

        start_imu = len(self.imu_samples)
        start_time = time.monotonic()
        end_time = start_time + self.pulse_duration
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_cmd(linear_x=linear_x, angular_z=angular_z)
            rclpy.spin_once(self, timeout_sec=self.period)

        self.publish_zero_for(self.settle_duration)
        samples = self.imu_samples[start_imu:]
        accel_baseline = self.baseline.accel_mean
        gyro_baseline = self.baseline.gyro_mean

        max_accel_delta = 0.0
        max_gyro_delta = 0.0
        for sample in samples:
            accel_delta = math.sqrt(
                (sample[1] - accel_baseline[0]) ** 2
                + (sample[2] - accel_baseline[1]) ** 2
                + (sample[3] - accel_baseline[2]) ** 2
            )
            gyro_delta = math.sqrt(
                (sample[4] - gyro_baseline[0]) ** 2
                + (sample[5] - gyro_baseline[1]) ** 2
                + (sample[6] - gyro_baseline[2]) ** 2
            )
            max_accel_delta = max(max_accel_delta, accel_delta)
            max_gyro_delta = max(max_gyro_delta, gyro_delta)

        moved_by_imu = (
            max_accel_delta >= self.accel_delta_threshold
            or max_gyro_delta >= self.gyro_delta_threshold
        )
        return PulseResult(
            name=name,
            linear_x=linear_x,
            angular_z=angular_z,
            duration=self.pulse_duration,
            expected_max_linear_travel=expected_travel,
            expected_max_yaw=abs(angular_z) * self.pulse_duration,
            cmd_latency=self.cmd_latency(start_time, linear_x, angular_z),
            max_accel_delta=max_accel_delta,
            max_gyro_delta=max_gyro_delta,
            moved_by_imu=moved_by_imu,
        )

    def log_baseline(self):
        """Log stationary IMU baseline."""
        accel_error = self.baseline.accel_norm_mean - GRAVITY
        self.get_logger().info(
            "imu_baseline: "
            f"count={self.baseline.count}, "
            f"rate={self.baseline.rate_hz:.1f}Hz, "
            f"accel_mean=({self.baseline.accel_mean[0]:.3f}, "
            f"{self.baseline.accel_mean[1]:.3f}, "
            f"{self.baseline.accel_mean[2]:.3f})m/s^2, "
            f"|a|-g={accel_error:.3f}m/s^2, "
            f"|a|_std={self.baseline.accel_norm_std:.4f}, "
            f"gyro_mean=({self.baseline.gyro_mean[0]:.4f}, "
            f"{self.baseline.gyro_mean[1]:.4f}, "
            f"{self.baseline.gyro_mean[2]:.4f})rad/s, "
            f"gyro_norm_std={self.baseline.gyro_norm_std:.4f}"
        )

    def log_result(self, result):
        """Log one pulse result."""
        latency = "n/a" if result.cmd_latency is None else f"{result.cmd_latency:.3f}s"
        self.get_logger().info(
            f"{result.name}: vx={result.linear_x:.3f}, wz={result.angular_z:.3f}, "
            f"duration={result.duration:.3f}s, "
            f"expected_travel<={result.expected_max_linear_travel:.3f}m, "
            f"expected_yaw<={math.degrees(result.expected_max_yaw):.2f}deg, "
            f"cmd_latency={latency}, "
            f"max_accel_delta={result.max_accel_delta:.3f}m/s^2, "
            f"max_gyro_delta={result.max_gyro_delta:.3f}rad/s, "
            f"moved_by_imu={result.moved_by_imu}"
        )

    def run(self):
        """Run the IMU-only pulse probe."""
        self.get_logger().info(
            "Starting IMU-only cmd probe: "
            f"cmd={self.cmd_vel_topic}, imu={self.imu_topic}, "
            f"no odom, max_expected_travel={self.max_expected_travel:.3f}m"
        )
        self.publish_zero_for(1.0)
        if not self.wait_for_imu():
            self.get_logger().error("FAIL: no IMU feedback")
            self.publish_zero_for(2.0)
            return False

        self.collect_baseline()
        self.log_baseline()

        moved = False
        for level in self.linear_levels:
            result = self.run_pulse(f"linear_{level:.3f}", linear_x=level)
            self.log_result(result)
            moved = moved or result.moved_by_imu
            if result.moved_by_imu:
                break

        for level in self.angular_levels:
            result = self.run_pulse(f"angular_{level:.3f}", angular_z=level)
            self.log_result(result)
            moved = moved or result.moved_by_imu
            if result.moved_by_imu:
                break

        self.publish_zero_for(2.0)
        self.get_logger().info(f"FINAL moved_by_imu={moved}")
        return True


def main(args=None):
    """Run the probe."""
    rclpy.init(args=args)
    node = WheeltecImuCmdProbe()
    try:
        passed = node.run()
    except KeyboardInterrupt:
        passed = False
        node.get_logger().warn("Interrupted, publishing zero velocity")
        node.publish_zero_for(2.0)
    except Exception as exc:
        passed = False
        node.get_logger().error(f"Probe failed: {exc}")
        node.publish_zero_for(2.0)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
