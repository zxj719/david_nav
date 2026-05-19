import inspect
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter

import david_cmd.david_move as david_move
from david_cmd.david_move import DavidMoveNode


@pytest.fixture
def rclpy_initialized():
    already_initialized = rclpy.ok()
    if not already_initialized:
        rclpy.init()

    try:
        yield
    finally:
        if not already_initialized and rclpy.ok():
            rclpy.shutdown()


def make_bridge(**overrides):
    parameters = {
        "input_cmd_vel_topic": "/test_cmd_vel_safe",
        "output_cmd_vel_topic": "/test_cmd_vel_out",
        "cmd_vel_timeout": 2.0,
        "min_linear_x": 0.20,
        "max_linear_x": 0.20,
        "min_angular_z": 0.20,
        "min_stationary_angular_z": 0.35,
        "stationary_angular_threshold": 0.025,
        "max_angular_z": 0.50,
        "publish_hz": 100.0,
    }
    parameters.update(overrides)
    return DavidMoveNode(
        parameter_overrides=[
            Parameter(name, Parameter.Type.from_parameter_value(value), value)
            for name, value in parameters.items()
        ]
    )


def spin_until(predicate, nodes, timeout_sec=1.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.01)
        if predicate():
            return True
    return False


def test_david_move_has_no_unitree_or_multiprocessing_runtime_path():
    source = inspect.getsource(david_move)

    assert "unitree" not in source.lower()
    assert "multiprocessing" not in source
    assert "shared_arr" not in source


def test_velocity_input_is_limited_and_forwarded(rclpy_initialized):
    bridge = make_bridge()
    observer = Node("test_david_move_observer")
    outputs = []
    observer.create_subscription(Twist, "/test_cmd_vel_out", outputs.append, 10)
    publisher = observer.create_publisher(Twist, "/test_cmd_vel_safe", 10)

    try:
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 0.4
        msg.linear.z = 0.2
        msg.angular.x = 0.1
        msg.angular.y = -0.1
        msg.angular.z = -2.0

        assert spin_until(
            lambda: publisher.get_subscription_count() > 0,
            [observer, bridge],
        )
        publisher.publish(msg)

        assert spin_until(
            lambda: any(abs(output.linear.x) > 0.0 for output in outputs),
            [observer, bridge],
        )
        output = next(
            output for output in reversed(outputs) if abs(output.linear.x) > 0.0
        )

        assert output.linear.x == pytest.approx(0.20)
        assert output.linear.y == 0.0
        assert output.linear.z == 0.0
        assert output.angular.x == 0.0
        assert output.angular.y == 0.0
        assert output.angular.z == pytest.approx(-0.50)
    finally:
        observer.destroy_node()
        bridge.destroy_node()


def test_small_nonzero_velocity_is_lifted_above_deadzone(rclpy_initialized):
    bridge = make_bridge(max_linear_x=0.40)

    try:
        msg = Twist()
        msg.linear.x = 0.05
        msg.angular.z = -0.05

        output = bridge.sanitize_twist(msg)

        assert output.linear.x == pytest.approx(0.20)
        assert output.angular.z == pytest.approx(-0.20)
    finally:
        bridge.destroy_node()


def test_stationary_nav2_yaw_is_lifted_to_turning_deadzone(rclpy_initialized):
    bridge = make_bridge()

    try:
        msg = Twist()
        msg.angular.z = 0.029

        output = bridge.sanitize_twist(msg)

        assert output.linear.x == 0.0
        assert output.angular.z == pytest.approx(0.35)
    finally:
        bridge.destroy_node()


def test_moving_yaw_uses_lower_arc_deadzone(rclpy_initialized):
    """Legacy behaviour: when min_angular_z is explicitly set, arc motion
    lifts small angular commands. Documents that the deadzone-lift code
    path still works when the parameter is non-zero."""
    bridge = make_bridge(max_linear_x=0.40, min_angular_z=0.20)

    try:
        msg = Twist()
        msg.linear.x = 0.10
        msg.angular.z = -0.029

        output = bridge.sanitize_twist(msg)

        assert output.linear.x == pytest.approx(0.20)
        assert output.angular.z == pytest.approx(-0.20)
    finally:
        bridge.destroy_node()


def test_arc_motion_passes_small_angular_through_by_default(rclpy_initialized):
    """Default-config regression: with DEFAULT_MIN_ANGULAR_Z=0.0, DWB's small
    path-tracking corrections (e.g. +0.026 rad/s during forward motion) must
    pass through unchanged. The old default 0.20 amplified them 5x and
    caused the controller to oscillate (see drift_run1 recording)."""
    bridge = make_bridge(max_linear_x=0.40, min_angular_z=0.0)

    try:
        msg = Twist()
        msg.linear.x = 0.20
        msg.angular.z = 0.026

        output = bridge.sanitize_twist(msg)

        assert output.linear.x == pytest.approx(0.20)
        assert output.angular.z == pytest.approx(0.026, abs=1e-6)
    finally:
        bridge.destroy_node()


def test_stationary_yaw_still_lifts_after_arc_default_change(rclpy_initialized):
    """Regression: even with the new MIN_ANGULAR_Z=0.0 arc default, a
    stationary in-place rotation must still get lifted to
    min_stationary_angular_z so the robot actually moves."""
    bridge = make_bridge(min_angular_z=0.0, min_stationary_angular_z=0.35)

    try:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.05  # small, above stationary threshold 0.025

        output = bridge.sanitize_twist(msg)

        assert output.linear.x == 0.0
        assert output.angular.z == pytest.approx(0.35)
    finally:
        bridge.destroy_node()


def test_tiny_stationary_yaw_uses_normal_deadzone(rclpy_initialized):
    bridge = make_bridge()

    try:
        msg = Twist()
        msg.angular.z = 0.01

        output = bridge.sanitize_twist(msg)

        assert output.angular.z == pytest.approx(0.20)
    finally:
        bridge.destroy_node()


def test_zero_velocity_stays_zero(rclpy_initialized):
    bridge = make_bridge()

    try:
        output = bridge.sanitize_twist(Twist())

        assert output.linear.x == 0.0
        assert output.angular.z == 0.0
    finally:
        bridge.destroy_node()


def test_stale_command_times_out_to_zero(rclpy_initialized):
    bridge = make_bridge()
    observer = Node("test_david_move_timeout_observer")
    outputs = []
    observer.create_subscription(Twist, "/test_cmd_vel_out", outputs.append, 10)

    try:
        msg = Twist()
        msg.linear.x = 0.10
        msg.angular.z = 0.20
        bridge.velocity_callback(msg)
        bridge.last_cmd_time = bridge.get_clock().now() - Duration(seconds=3.0)
        bridge.publish_current_command()

        assert spin_until(lambda: bool(outputs), [observer, bridge])
        output = outputs[-1]

        assert output.linear.x == 0.0
        assert output.linear.y == 0.0
        assert output.angular.z == 0.0
    finally:
        observer.destroy_node()
        bridge.destroy_node()


def test_publish_stop_burst_sends_zero_command(rclpy_initialized):
    bridge = make_bridge()
    observer = Node("test_david_move_stop_observer")
    outputs = []
    observer.create_subscription(Twist, "/test_cmd_vel_out", outputs.append, 10)

    try:
        bridge.publish_stop_burst(repetitions=1, period_sec=0.0)

        assert spin_until(lambda: bool(outputs), [observer, bridge])
        output = outputs[-1]

        assert output.linear.x == 0.0
        assert output.linear.y == 0.0
        assert output.angular.z == 0.0
    finally:
        observer.destroy_node()
        bridge.destroy_node()
