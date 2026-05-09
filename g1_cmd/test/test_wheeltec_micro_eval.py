import math

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from g1_cmd.wheeltec_micro_eval import (
    GRAVITY,
    Pose2D,
    forward_progress,
    normalize_angle,
    pose_distance,
    pose_from_odom,
    sample_rate,
    stddev,
    yaw_from_quaternion,
)


def make_odom(x=0.0, y=0.0, yaw=0.0):
    odom = Odometry()
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
    odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return odom


def make_imu(accel_z=GRAVITY, gyro_z=0.0):
    imu = Imu()
    imu.linear_acceleration.z = accel_z
    imu.angular_velocity.z = gyro_z
    return imu


def test_pose_helpers_extract_yaw_and_distance():
    odom = make_odom(x=0.03, y=0.04, yaw=math.radians(30.0))

    pose = pose_from_odom(odom)

    assert pose.x == pytest.approx(0.03)
    assert pose.y == pytest.approx(0.04)
    assert pose.yaw == pytest.approx(math.radians(30.0))
    assert pose_distance(Pose2D(0.0, 0.0, 0.0), pose) == pytest.approx(0.05)


def test_forward_progress_respects_start_heading():
    start = Pose2D(1.0, 2.0, math.radians(90.0))
    current = Pose2D(1.05, 2.04, math.radians(90.0))

    assert forward_progress(start, current) == pytest.approx(0.04)


def test_angle_and_quaternion_helpers_wrap():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert yaw_from_quaternion(make_odom(yaw=-math.pi / 2.0).pose.pose.orientation) == (
        pytest.approx(-math.pi / 2.0)
    )


def test_sample_stats_are_stable_for_small_sets():
    samples = [(0.0, 1.0), (0.5, 2.0), (1.0, 3.0)]

    assert sample_rate(samples) == pytest.approx(2.0)
    assert stddev([1.0, 2.0, 3.0]) == pytest.approx(math.sqrt(2.0 / 3.0))
    assert stddev([]) == 0.0


def test_imu_message_fixture_represents_stationary_gravity():
    imu = make_imu()

    assert imu.linear_acceleration.z == pytest.approx(GRAVITY)
    assert imu.angular_velocity.z == 0.0
