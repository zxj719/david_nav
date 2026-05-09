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
        "cmd_vel_timeout": 0.25,
        "max_linear_x": 0.20,
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
        bridge.last_cmd_time = bridge.get_clock().now() - Duration(seconds=1.0)
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
