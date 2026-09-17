"""按名保护ROS适配器契约，并记录观察与销毁的纯内存边界."""

import ast
import asyncio
from pathlib import Path
import re
from types import SimpleNamespace as NS

import pytest

from nav_tasks.adapters.ros.grid_source import RosGridSource
from test_dock_observations import _source
from test_pose_grid_sources import _Node, _odom, _poses


@pytest.mark.parametrize('path, qualified_name', [
    ('adapters/ros/dock_motion.py', '<module>'),
    ('adapters/ros/dock_motion.py', '_DockMotion'),
    ('adapters/ros/dock_motion.py', '_DockMotion.navigate'),
    ('adapters/ros/dock_motion.py', '_DockMotion.spin'),
    ('adapters/ros/dock_motion.py', '_DockMotion.request_cancel'),
    ('adapters/ros/dock_worker.py', '<module>'),
    ('adapters/ros/dock_worker.py', '_WorkerStopped'),
    ('adapters/ros/dock_worker.py', 'DockWorker'),
    ('adapters/ros/dock_worker.py', 'DockWorker.__init__'),
    ('adapters/ros/dock_worker.py', 'DockWorker.busy'),
    ('adapters/ros/dock_worker.py', 'DockWorker.dock'),
    ('adapters/ros/dock_worker.py', 'DockWorker.execute'),
    ('adapters/ros/dock_worker.py', 'DockWorker.cancel'),
    ('adapters/ros/dock_worker.py', 'DockWorker.close'),
    ('adapters/ros/dock_ingress.py', '<module>'),
    ('adapters/ros/dock_ingress.py', 'DockIngress'),
    ('adapters/ros/dock_ingress.py', 'DockIngress.__init__'),
    ('adapters/ros/dock_ingress.py', 'DockIngress.open'),
    ('adapters/ros/dock_ingress.py', 'DockIngress.stop'),
    ('adapters/ros/dock_ingress.py', 'DockIngress.destroy'),
    ('adapters/ros/dock_ingress.py', 'DockIngress.on_feedback'),
    ('adapters/ros/dock_observations.py', '<module>'),
    ('adapters/ros/dock_observations.py', 'DockObservations'),
    ('adapters/ros/dock_observations.py', 'DockObservations.__init__'),
    ('adapters/ros/dock_observations.py', 'DockObservations.pose'),
    ('adapters/ros/dock_observations.py', 'DockObservations.check_approach'),
    ('adapters/ros/dock_observations.py', 'DockObservations.close'),
    ('adapters/ros/dock_parameters.py', '<module>'),
    ('adapters/ros/dock_parameters.py', 'declare_parameters'),
    ('adapters/ros/event_publisher.py', '<module>'),
    ('adapters/ros/event_publisher.py', 'NavigationEvents'),
    ('adapters/ros/event_publisher.py', 'NavigationEvents.__init__'),
    ('adapters/ros/event_publisher.py', 'NavigationEvents.open'),
    ('adapters/ros/event_publisher.py', 'NavigationEvents.task'),
    ('adapters/ros/event_publisher.py', 'NavigationEvents.dock'),
    ('adapters/ros/event_publisher.py', 'NavigationEvents.close'),
    ('adapters/ros/grid_source.py', '<module>'),
    ('adapters/ros/grid_source.py', 'RosGridSource'),
    ('adapters/ros/grid_source.py', 'RosGridSource.__init__'),
    ('adapters/ros/grid_source.py', 'RosGridSource.snapshot'),
    ('adapters/ros/grid_source.py', 'RosGridSource.close'),
    ('adapters/ros/tf_pose_source.py', '<module>'),
    ('adapters/ros/tf_pose_source.py', 'TfPoseSource'),
    ('adapters/ros/tf_pose_source.py', 'TfPoseSource.__init__'),
    ('adapters/ros/tf_pose_source.py', 'TfPoseSource.current_map_pose'),
    ('adapters/ros/tf_pose_source.py', 'TfPoseSource.capture_for_poi'),
    ('adapters/ros/tf_pose_source.py', 'TfPoseSource.current_slam_session_id'),
    ('adapters/ros/tf_pose_source.py', 'TfPoseSource.current_map_from_odom'),
    ('adapters/ros/tf_pose_source.py', 'TfPoseSource.close'),
    ('adapters/ros/dock_motion.py', '_DockMotion.__init__'),
    ('adapters/ros/dock_worker.py', '_WorkerAttempt'),
    ('adapters/ros/dock_ingress.py', '_Entry'),
])
def test_reviewed_adapter_contract_summary(path, qualified_name):
    """检查已审说明有中文摘要，不以语言检查替代语义或行为证明.

    Args:
        path: 包内模块路径。
        qualified_name: 模块或类方法的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_tasks'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    if qualified_name != '<module>':
        for name in qualified_name.split('.'):
            matches = [child for child in node.body
                       if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                       and child.name == name]
            assert len(matches) == 1
            node = matches[0]
    doc = ast.get_docstring(node) or ''
    assert doc and re.search('[\\u4e00-\\u9fff]', doc.splitlines()[0]), qualified_name


@pytest.mark.parametrize('kind', ['dock', 'poi'])
def test_malformed_successful_tf_response_is_not_a_lookup_failure(kind):
    """查询成功后的字段解码错误直接传播，不伪装成TF缺失.

    Args:
        kind: Dock和普通POI的独立观察源。
    """
    if kind == 'dock':
        source, state, buffer = _source()
        read = source.pose
    else:
        source, _, buffer = _poses()
        read = source.current_map_pose
    buffer.lookup_transform = lambda *args, **kwargs: NS()
    try:
        with pytest.raises(AttributeError):
            read()
        if kind == 'dock':
            assert state.logs == []
    finally:
        source.close()


def test_capture_keeps_independent_sample_times_and_detaches_odometry():
    """采集不对齐时间戳或执行新鲜度策略，已缓存里程计不别名原消息."""
    source, node, buffer = _poses()
    message = _odom()
    message.header.stamp.sec = 3
    message.header.stamp.nanosec = 0
    node.subscriptions[0].callback(message)
    message.pose.pose.position.x = 999
    buffer.transform.header.stamp.sec = 100
    try:
        captured = asyncio.run(source.capture_for_poi())
        assert captured.map_pose.stamp_sec == 100.5
        assert captured.odom_pose.stamp_sec == 3
        assert captured.odom_pose.x == 1
        assert captured.slam_session_id == 'session'
    finally:
        source.close()


def test_grid_close_keeps_only_failed_tail_and_seals_both_callbacks():
    """顺序销毁成功项不重做，失败项保留且两个旧回调均已封闭."""
    node = _Node()
    source = RosGridSource(node, map_topic='/static', costmap_topic='/cost', occupancy_type=object)
    attempts = []
    fail = True

    def destroy(subscription):
        attempts.append(subscription)
        if fail and subscription is node.subscriptions[1]:
            return False
        node.destroyed.append(subscription)
        return True

    node.destroy_subscription = destroy
    with pytest.raises(RuntimeError, match='destroy'):
        source.close()
    assert node.destroyed == node.subscriptions[:1]
    for subscription in node.subscriptions:
        subscription.callback(object())
    fail = False
    source.close()
    source.close()
    assert attempts == [node.subscriptions[0], node.subscriptions[1], node.subscriptions[1]]
    assert node.destroyed == node.subscriptions
