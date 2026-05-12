import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from david_cmd.david_keyboard import (
    DavidKeyboardNode,
    apply_min_effective_speed,
    motion_for_key,
)


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


def make_keyboard(**overrides):
    parameters = {
        "cmd_vel_topic": "/test_cmd_vel_safe_keyboard",
        "linear_speed": 0.10,
        "angular_speed": 0.20,
        "linear_deadzone": 0.20,
        "angular_deadzone": 0.35,
        "max_linear_x": 0.30,
        "max_angular_z": 0.50,
        "publish_hz": 20.0,
        "key_timeout": 0.20,
    }
    parameters.update(overrides)
    return DavidKeyboardNode(
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


def test_apply_min_effective_speed_lifts_non_zero_commands_out_of_deadzone():
    assert apply_min_effective_speed(0.0, 0.20, 0.30) == 0.0
    assert apply_min_effective_speed(0.10, 0.20, 0.30) == pytest.approx(0.20)
    assert apply_min_effective_speed(-0.10, 0.20, 0.30) == pytest.approx(-0.20)
    assert apply_min_effective_speed(0.40, 0.20, 0.30) == pytest.approx(0.30)


def test_arrow_and_wasd_keys_map_to_base_directions():
    assert motion_for_key("w") == (1.0, 0.0)
    assert motion_for_key("S") == (-1.0, 0.0)
    assert motion_for_key("a") == (0.0, 1.0)
    assert motion_for_key("d") == (0.0, -1.0)
    assert motion_for_key("\x1b[A") == (1.0, 0.0)
    assert motion_for_key("\x1b[B") == (-1.0, 0.0)
    assert motion_for_key("\x1b[D") == (0.0, 1.0)
    assert motion_for_key("\x1b[C") == (0.0, -1.0)


def test_keyboard_node_builds_deadzone_aware_twists(rclpy_initialized):
    node = make_keyboard()

    try:
        forward = node.build_twist(1.0, 0.0)
        left = node.build_twist(0.0, 1.0)

        assert forward.linear.x == pytest.approx(0.20)
        assert forward.angular.z == 0.0
        assert left.linear.x == 0.0
        assert left.angular.z == pytest.approx(0.35)
    finally:
        node.destroy_node()


def test_keyboard_timeout_publishes_zero(rclpy_initialized):
    keyboard = make_keyboard(key_timeout=0.05)
    observer = Node("test_david_keyboard_observer")
    outputs = []
    observer.create_subscription(
        type(keyboard.active_twist),
        "/test_cmd_vel_safe_keyboard",
        outputs.append,
        10,
    )

    try:
        keyboard.set_motion(1.0, 0.0)
        keyboard.last_motion_time = time.monotonic() - 1.0
        keyboard.publish_current_command()

        assert spin_until(lambda: bool(outputs), [observer, keyboard])
        assert outputs[-1].linear.x == 0.0
        assert outputs[-1].angular.z == 0.0
    finally:
        observer.destroy_node()
        keyboard.destroy_node()
