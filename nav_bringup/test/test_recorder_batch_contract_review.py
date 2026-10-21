"""纯文件边界：不初始化 ROS、不发布、不清理实际运行记录."""

import pytest

from nav_bringup.recorder import event_flush, summary
from nav_bringup.recorder.bt_log_watcher import status_changes_from_msg


class _Stamp:
    """仅有秒与纳秒两个字段的假时间戳."""

    sec, nanosec = 0, 0


class _Change:
    """仅有行为树日志所需字段的假状态变化，不依赖 ROS 消息类型."""

    def __init__(self, name, previous, current):
        self.node_name, self.previous_status, self.current_status = name, previous, current
        self.timestamp = _Stamp()


class _Message:
    """仅有 event_log 字段的假消息."""

    def __init__(self, changes):
        self.event_log = changes


def test_status_changes_preserve_message_order_without_filtering():
    """只搬运字段：顺序保持，重复项不合并，也不要求来自真实 ROS."""
    changes = status_changes_from_msg(_Message([
        _Change('Spin', 'IDLE', 'RUNNING'),
        _Change('Spin', 'IDLE', 'RUNNING'),
        _Change('BackUp', 'IDLE', 'RUNNING')]))
    assert [item.node_name for item in changes] == ['Spin', 'Spin', 'BackUp']


@pytest.mark.parametrize('timeout,discovery', [
    (0.0, 0.0), (-1.0, 0.5), (float('inf'), 1.0), (2.0, -0.5), (2.0, 3.0)])
def test_invalid_budgets_are_refused_before_ros_or_file_ownership(tmp_path, timeout, discovery):
    """时间预算非法在取得 ROS 或文件归属之前就拒绝，运行目录保持不动."""
    path = tmp_path / 'events.jsonl'
    path.write_bytes(b'')
    with pytest.raises(ValueError):
        event_flush.publish_tail(path, timeout_sec=timeout, discovery_sec=discovery,
                                 warn=lambda text: None)
    assert path.read_bytes() == b'' and [item.name for item in tmp_path.iterdir()] \
        == ['events.jsonl']


def test_summary_writer_overwrites_without_backup(tmp_path):
    """写摘要直接覆盖目标路径，不留备份文件."""
    path = tmp_path / 'summary.md'
    summary.write_summary(path, '# 旧\n')
    summary.write_summary(path, '# 新\n')
    assert path.read_text(encoding='utf-8') == '# 新\n'
    assert [item.name for item in tmp_path.iterdir()] == ['summary.md']
