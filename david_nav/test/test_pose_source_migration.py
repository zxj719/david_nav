"""Bit-level geometry and captured POI equivalence across old/new ROS adapters."""

import asyncio
import json
import math
import random
import struct
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from david_nav.pose_provider import TfPoseProvider, _yaw_from_quaternion
from nav_executor.protocol.codec import build_mark_poi_success_event
from nav_tasks.adapters.ros.tf_pose_source import TfPoseSource
from nav_tasks.domain.geometry import legacy_yaw_from_quaternion
from nav_tasks.domain.types import StoredPoi


@pytest.mark.parametrize('seed', range(8))
def test_quaternion_conversion_retains_exact_legacy_float_bits(seed):
    """Unit/nonunit/boundary quaternions cannot silently change unrounded wire values."""
    rng = random.Random(seed)
    values = [tuple(rng.uniform(-2, 2) for _ in range(4)) for _ in range(100)]
    values.extend([(0, 0, 0, 0), (0, 0, 1, 0), (0, 0, -0.0, 1), (0, 0, 0, -1)])
    for quaternion in values:
        assert struct.pack('d', legacy_yaw_from_quaternion(*quaternion)) == (
            struct.pack('d', _yaw_from_quaternion(*quaternion)))


def test_actual_old_and_new_pose_capture_produce_identical_mark_frames(monkeypatch):
    """Configured legacy frame names and raw sample arithmetic preserve wire bytes."""
    messages, package = ModuleType('nav_msgs.msg'), ModuleType('nav_msgs')
    messages.Odometry = object
    package.msg = messages
    ros = ModuleType('rclpy')
    ros.time = NS(Time=lambda: 0)
    for key, value in [('nav_msgs', package), ('nav_msgs.msg', messages), ('rclpy', ros)]:
        monkeypatch.setitem(sys.modules, key, value)
    callbacks = []
    node = NS(create_subscription=lambda message, topic, cb, qos: callbacks.append(cb),
              destroy_subscription=lambda sub: True)
    rotation = NS(x=0.2, y=0.1, z=math.sin(0.7), w=math.cos(0.7))
    position = NS(x=1.125, y=-2.25, z=0.4075)
    header = NS(frame_id='', stamp=NS(sec=1000, nanosec=123456789))
    tf = NS(header=header, transform=NS(translation=position, rotation=rotation))
    buffer = NS(lookup_transform=lambda *args: tf)
    old = TfPoseProvider(node, buffer, 'robot_frame', '/odom', lambda: 's')
    new = TfPoseSource(node, buffer, robot_base_frame='robot_frame', odom_topic='/odom',
                       map_frame='map', odom_frame='odom', session_id=lambda: 's',
                       odometry_type=object, latest_time=lambda: 0)
    odometry = NS(header=header, pose=NS(pose=NS(position=position, orientation=rotation)))
    for callback in callbacks:
        callback(odometry)
    old_capture = asyncio.run(old.capture_for_poi())
    new_capture = asyncio.run(new.capture_for_poi())
    assert old_capture == new_capture
    assert old.current_map_from_odom() == new.current_map_from_odom()

    def frame(capture):
        poi = StoredPoi('p', '点位', capture.map_pose, capture.odom_pose, capture.slam_session_id)
        return json.dumps(build_mark_poi_success_event('r', 1, poi), ensure_ascii=False)

    assert frame(old_capture) == frame(new_capture)
    new.close()
