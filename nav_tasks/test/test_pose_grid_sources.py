"""ROS observation adapters exercised with injected nodes/messages, never ROS init."""

import asyncio
from dataclasses import replace
import math
import sys
import threading
from types import ModuleType, SimpleNamespace as NS

import pytest

from nav_tasks.adapters.ros.grid_source import RosGridSource
from nav_tasks.adapters.ros.tf_pose_source import TfPoseSource
from nav_tasks.core.goal_resolution import GridGoalResolver, SnapSearch
from nav_tasks.domain.errors import ClosedError, PoseUnavailableError
from nav_tasks.domain.types import PoseRecord, StoredPoi, Transform2D


class _Node:
    def __init__(self):
        self.subscriptions = []
        self.destroyed = []
        self.destroy_ok = True
        self.fail_second = False

    def create_subscription(self, message_type, topic, callback, qos):
        if self.fail_second and self.subscriptions:
            raise ValueError('second subscription failed')
        sub = NS(message_type=message_type, topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(sub)
        return sub

    def destroy_subscription(self, subscription):
        if not self.destroy_ok:
            return False
        self.destroyed.append(subscription)
        return True


def _transform(frame='world'):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=42, nanosec=500000000)),
              transform=NS(translation=NS(x=1, y=2, z=3),
                           rotation=NS(x=0, y=0, z=math.sin(0.5), w=math.cos(0.5))))


def _odom(frame='local'):
    tf = _transform(frame)
    return NS(header=tf.header,
              pose=NS(pose=NS(position=tf.transform.translation,
                              orientation=tf.transform.rotation)))


class _Buffer:
    def __init__(self):
        self.calls = []
        self.transform = _transform()
        self.error = None

    def lookup_transform(self, target, source, stamp):
        self.calls.append((target, source, stamp))
        if self.error is not None:
            raise self.error
        return self.transform


def _poses(node=None, buffer=None):
    node, buffer = node or _Node(), buffer or _Buffer()
    source = TfPoseSource(
        node, buffer, robot_base_frame='chassis', odom_topic='/configured_odom',
        map_frame='world', odom_frame='local', session_id=lambda: 'session',
        odometry_type=object, latest_time=lambda: 'latest')
    return source, node, buffer


def _grid(blocked=None):
    cells = [0] * 100
    if blocked is not None:
        cells[blocked] = 100
    info = NS(width=10, height=10, resolution=1.0, origin=NS(position=NS(x=0, y=0)))
    return NS(data=cells, info=info)


def test_pose_source_preserves_configured_lookups_message_fields_and_qos():
    """TF and odometry samples retain frame/stamp/height and latest-time semantics."""
    source, node, buffer = _poses()
    sub = node.subscriptions[0]
    assert (sub.message_type, sub.topic, sub.qos) == (object, '/configured_odom', 10)
    thread = threading.Thread(target=sub.callback, args=(_odom(''),))
    thread.start()
    thread.join()
    captured = asyncio.run(source.capture_for_poi())
    assert captured.map_pose == PoseRecord('world', 1, 2, 3, 1.0, 42.5)
    assert captured.odom_pose == replace(captured.map_pose, frame_id='local')
    assert captured.slam_session_id == 'session'
    assert source.current_map_from_odom() == Transform2D(1, 2, 1.0)
    assert buffer.calls == [('world', 'chassis', 'latest'), ('world', 'local', 'latest')]
    buffer.transform.header.frame_id = 'reported_frame'
    assert source.current_map_pose().frame_id == 'reported_frame'
    source.close()


def test_missing_odom_and_lookup_errors_have_typed_readable_failures():
    """New port errors retain old messages and the original TF exception cause."""
    source, node, buffer = _poses()
    with pytest.raises(PoseUnavailableError, match='no odometry available on /configured_odom'):
        asyncio.run(source.capture_for_poi())
    original = LookupError('TF unavailable')
    buffer.error = original
    for read in (source.current_map_pose, source.current_map_from_odom):
        with pytest.raises(PoseUnavailableError, match='TF unavailable') as error:
            read()
        assert error.value.__cause__ is original
    source.close()


def test_pose_close_is_owned_idempotent_and_ignores_late_callbacks():
    """Closing removes only the owned subscription and blocks further observations."""
    source, node, _ = _poses()
    node.subscriptions[0].callback(_odom())
    source.close()
    source.close()
    node.subscriptions[0].callback(object())
    assert node.destroyed == node.subscriptions
    for read in (source.current_map_pose, source.current_map_from_odom,
                 source.current_slam_session_id):
        with pytest.raises(ClosedError):
            read()


@pytest.mark.parametrize('kind', ['pose', 'grid'])
def test_failed_subscription_cleanup_is_retryable(kind):
    """Failed destruction retains its handle while the adapter stays sealed."""
    node = _Node()
    source = _poses(node)[0] if kind == 'pose' else RosGridSource(
        node, map_topic='/static', costmap_topic='/cost', occupancy_type=object)
    node.destroy_ok = False
    with pytest.raises(RuntimeError, match='destroy'):
        source.close()
    assert node.destroyed == []
    node.destroy_ok = True
    source.close()
    source.close()
    assert node.destroyed == node.subscriptions


def test_grid_source_snapshot_copies_cells_and_feeds_existing_resolver():
    """The ROS boundary preserves conversion and pure missing-map/snap policy."""
    node = _Node()
    source = RosGridSource(node, map_topic='/static', costmap_topic='/cost', occupancy_type=object)
    assert [(sub.topic, sub.qos) for sub in node.subscriptions] == [('/static', 10), ('/cost', 10)]
    assert source.snapshot() == (None, None)
    pose = PoseRecord('world', 5.5, 5.5, 0, 0, 0)
    poi = StoredPoi('p', 'target', pose, pose, 's')
    resolver = GridGoalResolver(source.snapshot, NS(current_map_pose=lambda: pose),
                                SnapSearch(goal_clearance_radius=0, max_snap_radius=1))
    assert resolver.resolve(poi).used_snap is False
    node.subscriptions[0].callback(_grid())
    cost = _grid(55)
    node.subscriptions[1].callback(cost)
    snapshot = source.snapshot()
    assert snapshot[1].value(5, 5) == 100
    result = resolver.resolve(poi)
    assert result.used_snap
    assert result.poi.poi_id == 'p'
    assert not resolver.is_admissible(poi)
    cost.data[55] = 0
    assert snapshot[1].value(5, 5) == 100
    assert resolver.is_admissible(poi)
    source.close()
    node.subscriptions[0].callback(object())
    with pytest.raises(ClosedError):
        source.snapshot()
    assert node.destroyed == node.subscriptions


def test_second_grid_subscription_failure_releases_first():
    """Partial construction cannot leave its first subscription running unowned."""
    node = _Node()
    node.fail_second = True
    with pytest.raises(ValueError, match='second subscription'):
        RosGridSource(node, map_topic='/static', costmap_topic='/cost', occupancy_type=object)
    assert node.destroyed == node.subscriptions


def test_default_constructor_paths_import_ros_types_lazily(monkeypatch):
    """Exercise production import branches with stand-in modules, without ROS init."""
    messages = ModuleType('nav_msgs.msg')
    messages.Odometry = type('FakeOdometry', (), {})
    messages.OccupancyGrid = type('FakeOccupancyGrid', (), {})
    package = ModuleType('nav_msgs')
    package.msg = messages
    ros = ModuleType('rclpy')
    time_module = ModuleType('rclpy.time')
    time_module.Time = lambda: 'latest'
    ros.time = time_module
    for name, module in [('nav_msgs', package), ('nav_msgs.msg', messages),
                         ('rclpy', ros), ('rclpy.time', time_module)]:
        monkeypatch.setitem(sys.modules, name, module)
    node, buffer = _Node(), _Buffer()
    poses = TfPoseSource(node, buffer, robot_base_frame='chassis', odom_topic='/odom_in',
                         map_frame='world', odom_frame='local', session_id=lambda: 's')
    grids = RosGridSource(node, map_topic='/static', costmap_topic='/cost')
    assert node.subscriptions[0].message_type is messages.Odometry
    assert node.subscriptions[1].message_type is messages.OccupancyGrid
    assert poses.current_map_pose().x == 1
    assert buffer.calls == [('world', 'chassis', 'latest')]
    poses.close()
    grids.close()
