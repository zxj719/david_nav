import math

import pytest
from nav_msgs.msg import Odometry

import g1_cmd.wheeltec_base_eval as wheeltec_base_eval
from g1_cmd.wheeltec_base_eval import (
    Pose2D,
    angular_motion_started,
    forward_progress,
    linear_motion_started,
    normalize_angle,
    pose_error,
    pose_from_odom,
    yaw_progress,
)


def make_odom(x=0.0, y=0.0, yaw=0.0, linear_x=0.0, angular_z=0.0):
    odom = Odometry()
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
    odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
    odom.twist.twist.linear.x = linear_x
    odom.twist.twist.angular.z = angular_z
    return odom


def test_pose_from_odom_extracts_planar_pose():
    odom = make_odom(x=1.2, y=-0.3, yaw=math.radians(45.0))

    pose = pose_from_odom(odom)

    assert pose.x == pytest.approx(1.2)
    assert pose.y == pytest.approx(-0.3)
    assert pose.yaw == pytest.approx(math.radians(45.0))


def test_forward_progress_uses_segment_start_heading():
    start = Pose2D(x=1.0, y=2.0, yaw=math.radians(90.0))
    current = Pose2D(x=1.1, y=2.25, yaw=math.radians(90.0))

    assert forward_progress(start, current) == pytest.approx(0.25)


def test_yaw_progress_wraps_shortest_direction():
    start = Pose2D(x=0.0, y=0.0, yaw=math.radians(170.0))
    current = Pose2D(x=0.0, y=0.0, yaw=math.radians(-130.0))

    assert yaw_progress(start, current) == pytest.approx(math.radians(60.0))


def test_pose_error_reports_reset_distance_and_yaw_error():
    start = Pose2D(x=0.0, y=0.0, yaw=math.radians(5.0))
    current = Pose2D(x=0.03, y=0.04, yaw=math.radians(-2.0))

    distance_error, yaw_error = pose_error(start, current)

    assert distance_error == pytest.approx(0.05)
    assert yaw_error == pytest.approx(math.radians(7.0))


def test_motion_threshold_helpers_use_odom_twist_only():
    odom = make_odom(linear_x=0.03, angular_z=0.04)

    assert linear_motion_started(odom, threshold=0.02)
    assert angular_motion_started(odom, threshold=0.03)
    assert not linear_motion_started(odom, threshold=0.05)
    assert not angular_motion_started(odom, threshold=0.05)


def test_normalize_angle_keeps_value_in_pi_range():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)


def test_eval_module_does_not_subscribe_to_imu():
    source = open(wheeltec_base_eval.__file__, encoding="utf-8").read()

    assert "sensor_msgs" not in source
    assert "Imu" not in source
    assert "/imu" not in source
