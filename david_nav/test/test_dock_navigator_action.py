"""dock_navigator 动作接口的纯逻辑部分（2026-09-15）。

不 spin ROS：验证可取消等待、feedback 转换，以及节点方法不遮蔽 rclpy Node 的属性
（2026-08-31 的 handle 事故：子类方法覆盖 Node.handle 导致节点建不出来）。
"""
import threading
import time

from nav2_msgs.action import NavigateToPose
from rclpy.node import Node

from legacy_entry_reference import load_entry

dock_navigator = load_entry('dock_navigator')


class FakeFuture:
    def __init__(self):
        self._callbacks = []
        self._done = False

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def done(self):
        return self._done

    def finish(self):
        self._done = True
        for callback in self._callbacks:
            callback(self)


def test_wait_done_returns_when_the_future_completes():
    fut = FakeFuture()
    threading.Timer(0.05, fut.finish).start()

    assert dock_navigator.wait_done(fut, 2.0, should_cancel=lambda: False)


def test_wait_done_stops_waiting_once_cancel_is_requested():
    fut = FakeFuture()
    requested = threading.Event()
    threading.Timer(0.1, requested.set).start()
    started = time.monotonic()

    assert not dock_navigator.wait_done(
        fut, 5.0, should_cancel=requested.is_set, poll_sec=0.02)
    assert time.monotonic() - started < 1.0


def test_wait_done_times_out_and_accepts_finished_futures():
    finished = FakeFuture()
    finished.finish()

    assert not dock_navigator.wait_done(FakeFuture(), 0.05)
    assert dock_navigator.wait_done(finished, 0.0)


def test_stage_feedback_adds_the_remaining_stage_distance():
    inner = NavigateToPose.Feedback()
    inner.distance_remaining = 1.25
    inner.number_of_recoveries = 2
    inner.current_pose.pose.position.x = 3.0

    feedback = dock_navigator.stage_feedback(inner, 0.5)

    assert abs(feedback.distance_remaining - 1.75) < 1e-6
    assert feedback.number_of_recoveries == 2
    assert feedback.current_pose.pose.position.x == 3.0


def test_dock_navigator_methods_do_not_shadow_node_attributes():
    shadowed = [
        name for name in vars(dock_navigator.DockNavigator)
        if not name.startswith('__') and hasattr(Node, name)
    ]

    assert shadowed == []
