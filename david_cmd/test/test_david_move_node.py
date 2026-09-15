import inspect
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.time import Time

import david_cmd.david_move as david_move
from david_cmd.david_move import DavidMoveNode


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
    node = object.__new__(DavidMoveNode)
    for name, value in parameters.items():
        setattr(node, name, value)
    node.outputs = []
    node.cmd_publisher = SimpleNamespace(publish=node.outputs.append)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=100.0))
    node.get_logger = lambda: SimpleNamespace(warn=lambda _: None)
    node.latest_twist = Twist()
    node.last_cmd_time = Time(seconds=0.0)
    node._stale_stop_sent = True
    node.destroy_node = lambda: None
    return node


def test_david_move_has_no_unitree_or_multiprocessing_runtime_path():
    source = inspect.getsource(david_move)

    assert "unitree" not in source.lower()
    assert "multiprocessing" not in source
    assert "shared_arr" not in source


def test_velocity_input_is_limited_and_forwarded():
    bridge = make_bridge()
    outputs = bridge.outputs

    try:
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 0.4
        msg.linear.z = 0.2
        msg.angular.x = 0.1
        msg.angular.y = -0.1
        msg.angular.z = -2.0

        bridge.velocity_callback(msg)
        bridge.publish_current_command()

        assert any(abs(output.linear.x) > 0.0 for output in outputs)
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
        bridge.destroy_node()


def test_small_nonzero_velocity_is_lifted_above_deadzone():
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


def test_stationary_nav2_yaw_is_lifted_to_turning_deadzone():
    bridge = make_bridge()

    try:
        msg = Twist()
        msg.angular.z = 0.029

        output = bridge.sanitize_twist(msg)

        assert output.linear.x == 0.0
        assert output.angular.z == pytest.approx(0.35)
    finally:
        bridge.destroy_node()


def test_moving_yaw_uses_lower_arc_deadzone():
    """Arc motion lifts small angular commands when min_angular_z is set.

    Legacy behaviour: documents that the deadzone-lift code path still works
    when the parameter is non-zero.
    """
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


def test_arc_motion_passes_small_angular_through_by_default():
    """Small angular corrections pass through unchanged by default.

    Default-config regression: with DEFAULT_MIN_ANGULAR_Z=0.0, DWB's small
    path-tracking corrections (e.g. +0.026 rad/s during forward motion) must
    pass through unchanged. The old default 0.20 amplified them 5x and
    caused the controller to oscillate (see drift_run1 recording).
    """
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


def test_stationary_yaw_still_lifts_after_arc_default_change():
    """Stationary in-place rotation is still lifted with the new arc default.

    Regression: even with MIN_ANGULAR_Z=0.0 for arcs, a stationary rotation
    must still get lifted to min_stationary_angular_z so the robot moves.
    """
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


def test_tiny_stationary_yaw_uses_normal_deadzone():
    bridge = make_bridge()

    try:
        msg = Twist()
        msg.angular.z = 0.01

        output = bridge.sanitize_twist(msg)

        assert output.angular.z == pytest.approx(0.20)
    finally:
        bridge.destroy_node()


def test_zero_velocity_stays_zero():
    bridge = make_bridge()

    try:
        output = bridge.sanitize_twist(Twist())

        assert output.linear.x == 0.0
        assert output.angular.z == 0.0
    finally:
        bridge.destroy_node()


def test_stale_command_times_out_to_zero():
    bridge = make_bridge()
    outputs = bridge.outputs

    try:
        msg = Twist()
        msg.linear.x = 0.10
        msg.angular.z = 0.20
        bridge.velocity_callback(msg)
        bridge.last_cmd_time = bridge.get_clock().now() - Duration(seconds=3.0)
        bridge.publish_current_command()

        assert outputs
        output = outputs[-1]

        assert output.linear.x == 0.0
        assert output.linear.y == 0.0
        assert output.angular.z == 0.0
    finally:
        bridge.destroy_node()


def test_publish_stop_burst_sends_zero_command():
    bridge = make_bridge()
    outputs = bridge.outputs

    try:
        bridge.publish_stop_burst(repetitions=1, period_sec=0.0)

        assert outputs
        output = outputs[-1]
        assert output.linear.x == 0.0
        assert output.linear.y == 0.0
        assert output.angular.z == 0.0
    finally:
        bridge.destroy_node()


def test_stale_transition_emits_only_one_stop_and_then_releases_output():
    """Initial silence and one-shot stale stop are checked without a DDS publisher."""
    bridge = make_bridge()
    bridge.publish_current_command()
    assert bridge.outputs == []
    bridge.velocity_callback(Twist())
    bridge.last_cmd_time = Time(seconds=90.0)
    bridge.publish_current_command()
    bridge.publish_current_command()
    assert len(bridge.outputs) == 1
    assert bridge.outputs[0].linear.x == 0.0


def test_ros_initialization_is_guarded():
    """The ordinary suite cannot silently regain a graph-facing test path."""
    import rclpy

    with pytest.raises(pytest.fail.Exception, match="ordinary tests"):
        rclpy.init()
