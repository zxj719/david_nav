"""nav2_action_bridge 失败消抖重试策略单测。

2026-07-24 需求:nav2 单次失败(ABORTED/rejected/server 不可用)不上报
agent,记详细日志后等 goal_retry_delay_sec 重发同一导航点;连败耗尽
重试预算才发 on_error。CANCELED 是主动取消,不重试。

不起 ROS:HarnessBridge 绕过父类 __init__,用 FakeActionClient 按脚本
给出每次 send_goal 的终态,走真实 start_navigation/_attempt_navigation。
"""
import asyncio
from types import SimpleNamespace

from david_nav.nav2_action_bridge import Nav2ActionBridge
from david_nav.navigation_types import PoseRecord, StoredPoi


def _poi() -> StoredPoi:
    return StoredPoi(
        poi_id="POI_001",
        name="前台",
        map_pose=PoseRecord("map", 1.0, 2.0, 0.0, 0.5, 100.0),
        odom_pose=PoseRecord("odom", 1.0, 2.0, 0.0, 0.5, 100.0),
        slam_session_id="session_a",
    )


class FakeRclpyFuture:
    def __init__(self, value):
        self._value = value

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._value


class FakeGoalHandle:
    def __init__(self, status, accepted=True):
        self.accepted = accepted
        self._status = status

    def get_result_async(self):
        return FakeRclpyFuture(SimpleNamespace(status=self._status))


class FakeGoal:
    def __init__(self):
        self.pose = SimpleNamespace(
            header=SimpleNamespace(frame_id="", stamp=None),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )


class FakeActionClient:
    """outcomes 按次序消费:int=GoalStatus 终态(4/5/6...),
    "rejected"=goal 被拒,"no_server"=wait_for_server 超时。"""

    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.sent_goals = []

    def wait_for_server(self, timeout_sec):
        if self.outcomes and self.outcomes[0] == "no_server":
            self.outcomes.pop(0)
            return False
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent_goals.append(goal)
        outcome = self.outcomes.pop(0)
        if outcome == "rejected":
            return FakeRclpyFuture(FakeGoalHandle(status=None, accepted=False))
        return FakeRclpyFuture(FakeGoalHandle(status=outcome))


class HarnessBridge(Nav2ActionBridge):
    def __init__(self, outcomes, event_callback, goal_retry_count=1, goal_retry_delay_sec=0.0):
        # 故意不调 super().__init__():不需要 rclpy/nav2_msgs。
        self.node = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: 0)
            )
        )
        self.action_name = "/navigate_to_pose"
        self.server_timeout = 0.1
        self.event_callback = event_callback
        self.progress_min_interval_sec = 1.0
        self.goal_retry_count = max(0, int(goal_retry_count))
        self.goal_retry_delay_sec = float(goal_retry_delay_sec)
        self._navigate_to_pose = SimpleNamespace(Goal=FakeGoal)
        self.client = FakeActionClient(outcomes)
        self._current_goal_handle = None


def _collect_events():
    events = []

    async def callback(event):
        events.append(event)

    return events, callback


def test_single_failure_is_debounced_and_goal_republished():
    # 第 1 次 ABORTED(6) → 不上报,重发;第 2 次 SUCCEEDED(4) → arrived。
    events, callback = _collect_events()
    bridge = HarnessBridge([6, 4], callback)

    asyncio.run(bridge.start_navigation("req_1", 1, _poi()))

    assert [e["kind"] for e in events] == ["arrived"]
    assert len(bridge.client.sent_goals) == 2


def test_retry_exhausted_reports_detailed_error():
    events, callback = _collect_events()
    bridge = HarnessBridge([6, 6], callback, goal_retry_count=1)

    asyncio.run(bridge.start_navigation("req_1", 1, _poi()))

    assert len(bridge.client.sent_goals) == 2
    assert len(events) == 1
    error = events[0]
    assert error["kind"] == "error"
    assert error["request_id"] == "req_1"
    # 详细原因:尝试次数 + 终态码/名 + 目标点标识与坐标
    assert "共尝试2次" in error["error_message"]
    assert "status=6(ABORTED)" in error["error_message"]
    assert "POI_001" in error["error_message"]
    assert "x=1.00" in error["error_message"]


def test_canceled_goal_is_not_retried():
    # CANCELED(5) 是 abort/preempt 的主动取消:发 paused,不重发。
    events, callback = _collect_events()
    bridge = HarnessBridge([5], callback, goal_retry_count=1)

    asyncio.run(bridge.start_navigation("req_1", 1, _poi()))

    assert len(bridge.client.sent_goals) == 1
    assert [e["kind"] for e in events] == ["progress"]
    assert events[0]["status"] == "paused"


def test_server_unavailable_is_retried_then_recovers():
    events, callback = _collect_events()
    bridge = HarnessBridge(["no_server", 4], callback, goal_retry_count=1)

    asyncio.run(bridge.start_navigation("req_1", 1, _poi()))

    # 第 1 次 wait_for_server 失败(没发 goal),重试后成功。
    assert len(bridge.client.sent_goals) == 1
    assert [e["kind"] for e in events] == ["arrived"]


def test_rejected_goal_exhausts_retry_with_reason():
    events, callback = _collect_events()
    bridge = HarnessBridge(["rejected", "rejected"], callback, goal_retry_count=1)

    asyncio.run(bridge.start_navigation("req_1", 1, _poi()))

    assert len(events) == 1
    assert events[0]["kind"] == "error"
    assert "拒绝" in events[0]["error_message"]


class PendingRclpyFuture:
    """手动触发的 rclpy future:模拟 goal 已发出、goal_handle 还没回来。"""

    def __init__(self):
        self._callbacks = []
        self._value = None
        self._done = False

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def set_value(self, value):
        self._done = True
        self._value = value
        for callback in self._callbacks:
            callback(self)

    def result(self):
        return self._value


def test_cancel_during_retry_sleep_does_not_republish(monkeypatch):
    # nav WS v1.1 abort:第 1 次失败进入消抖等待期间被 cancel(abort),
    # 不能再把旧 goal 重发出去(否则"停下后车又自己走了")。
    events, callback = _collect_events()
    bridge = HarnessBridge([6, 4], callback, goal_retry_delay_sec=60.0)

    async def scenario():
        entered_sleep = asyncio.Event()
        real_sleep = asyncio.sleep

        async def fake_sleep(delay, *args, **kwargs):
            entered_sleep.set()
            await real_sleep(3600)

        monkeypatch.setattr(asyncio, "sleep", fake_sleep)
        task = asyncio.create_task(bridge.start_navigation("req_1", 1, _poi()))
        await asyncio.wait_for(entered_sleep.wait(), timeout=5.0)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(scenario())

    assert len(bridge.client.sent_goals) == 1  # 没有重发
    assert events == []  # 也没有以 error/paused 冒充终态


def test_cancel_while_goal_send_inflight_cancels_late_arriving_handle():
    # abort 恰好落在 send_goal_async 已发出、goal_handle 还没回来的窗口:
    # handle 晚到时必须补一刀 cancel,否则 nav2 拿着这个 goal 继续开车,
    # 而 executor 侧已经没人盯着它了。
    events, callback = _collect_events()
    bridge = HarnessBridge([], callback)
    pending = PendingRclpyFuture()

    class InflightClient:
        def __init__(self):
            self.sent_goals = []

        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal, feedback_callback=None):
            self.sent_goals.append(goal)
            return pending

    class LateHandle:
        def __init__(self):
            self.accepted = True
            self.cancel_calls = 0

        def cancel_goal_async(self):
            self.cancel_calls += 1
            return FakeRclpyFuture(None)

    bridge.client = InflightClient()
    handle = LateHandle()

    async def scenario():
        task = asyncio.create_task(bridge.start_navigation("req_1", 1, _poi()))
        for _ in range(5):
            await asyncio.sleep(0)  # 让 task 走到 await goal_handle
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        pending.set_value(handle)  # goal_handle 迟到

    asyncio.run(scenario())

    assert len(bridge.client.sent_goals) == 1
    assert handle.cancel_calls == 1
    assert events == []


def test_retry_waits_configured_delay(monkeypatch):
    recorded = []
    real_sleep = asyncio.sleep

    async def fake_sleep(delay, *args, **kwargs):
        recorded.append(delay)
        await real_sleep(0)

    monkeypatch.setattr(asyncio, "sleep", fake_sleep)
    events, callback = _collect_events()
    bridge = HarnessBridge([6, 4], callback, goal_retry_delay_sec=1.0)

    asyncio.run(bridge.start_navigation("req_1", 1, _poi()))

    assert recorded == [1.0]
    assert [e["kind"] for e in events] == ["arrived"]
