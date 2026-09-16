"""Retained map snapshots and Dock TF contracts with no ROS nodes or executors."""

import math
from types import SimpleNamespace as NS

import pytest

from nav_tasks.adapters.ros.dock_observations import DockObservations
from nav_tasks.domain.errors import ClosedError


def _message():
    return NS(header=NS(frame_id='world'), info=NS(width=8, height=8, resolution=1.0,
              origin=NS(position=NS(x=10, y=10),
                        orientation=NS(x=0, y=0, z=math.sin(math.pi / 4),
                                       w=math.cos(math.pi / 4)))), data=[0] * 64)


def _source():
    state = NS(callback=None, subscription=object(), destroys=0, refuse=False, logs=[])

    def subscribe(message_type, topic, callback, qos):
        assert message_type is object and topic == '/fixture_static' and qos == 'retained'
        state.callback = callback
        return state.subscription

    def destroy(subscription):
        assert subscription is state.subscription
        state.destroys += 1
        return not state.refuse

    node = NS(create_subscription=subscribe, destroy_subscription=destroy)
    state.lookups = []

    def lookup(*args, **kwargs):
        state.lookups.append((args, kwargs))
        return NS(transform=NS(translation=NS(x=1, y=2, z=3),
                               rotation=NS(x=0.2, y=0.3, z=0.4, w=0.8)))

    buffer = NS(lookup_transform=lookup)
    source = DockObservations(
        node, buffer, global_frame='world', robot_base_frame='rotation_center',
        static_map_topic='/fixture_static', footprint=((-0.4, -0.4), (0.4, -0.4), (0, 0.4)),
        occupied_threshold=50, logger=NS(warn=state.logs.append), occupancy_type=object,
        qos='retained', latest_time=lambda: 'latest', duration=lambda **values: values)
    return source, state, buffer


def test_static_map_copy_retains_rotation_and_is_detached_from_ros_message():
    """The map callback preserves rotated origin and does not alias mutable message data."""
    source, state, _ = _source()
    pose = (5.5, 14.5, math.pi / 2)
    assert source.check_approach(pose).reason == 'static_map_not_received'
    message = _message()
    state.callback(message)
    assert source.check_approach(pose).clear
    message.data[4 * 8 + 4] = 100
    assert source.check_approach(pose).clear  # Existing snapshot is immutable.
    state.callback(message)
    assert source.check_approach(pose).occupied_cell == (4, 4, 100)
    source.close()


@pytest.mark.parametrize('case,reason', [
    ('frame', 'static_map_frame_mismatch:other!=world'),
    ('outside', 'footprint_outside_static_map'),
    ('invalid', 'invalid_static_map'),
])
def test_real_snapshot_guard_rejects_unusable_maps(case, reason):
    """Frame and full-area checks happen after actual ROS-like snapshot conversion."""
    source, state, _ = _source()
    message = _message()
    if case == 'frame':
        message.header.frame_id = 'other'
    elif case == 'invalid':
        message.info.width = 0
    state.callback(message)
    pose = (0, 0, 0) if case == 'outside' else (5.5, 14.5, math.pi / 2)
    assert source.check_approach(pose).reason == reason
    source.close()


def test_tf_lookup_keeps_navigation_frame_budget_and_legacy_quaternion_arithmetic():
    """Dock reads its rotation center with the original bounded latest-time lookup."""
    source, state, _ = _source()
    expected = math.atan2(2 * (0.8 * 0.4 + 0.2 * 0.3), 1 - 2 * (0.3**2 + 0.4**2))
    assert source.pose() == (1, 2, expected)
    assert state.lookups == [(('world', 'rotation_center', 'latest'), {'timeout': {'seconds': 2}})]
    source.close()


def test_tf_failure_retains_none_and_frozen_warning():
    """Missing localization is reported, not changed or worked around by navigation."""
    source, state, buffer = _source()

    def fail(*args, **kwargs):
        raise LookupError('transform unavailable')

    buffer.lookup_transform = fail
    assert source.pose() is None
    assert state.logs == ['[dock] 取位姿失败: transform unavailable']
    source.close()


def test_failed_close_retries_owned_subscription_and_seals_callbacks():
    """The adapter never destroys external nodes/TF or reopens after a cleanup error."""
    source, state, _ = _source()
    state.refuse = True
    with pytest.raises(RuntimeError):
        source.close()
    state.callback(object())  # Sealed callback does not even decode the late message.
    with pytest.raises(ClosedError):
        source.check_approach((0, 0, 0))
    with pytest.raises(ClosedError):
        source.pose()
    state.refuse = False
    source.close()
    source.close()
    assert state.destroys == 2
