"""按名保护ROS适配器契约，并记录观察与销毁的纯内存边界."""

import asyncio
from types import SimpleNamespace as NS

import pytest

from nav_tasks.adapters.ros.grid_source import RosGridSource
from test_dock_observations import _source
from test_pose_grid_sources import _Node, _odom, _poses


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
