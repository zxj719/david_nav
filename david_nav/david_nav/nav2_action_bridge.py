from __future__ import annotations

import asyncio
import concurrent.futures
import contextlib
import time

from david_nav.navigation_types import StoredPoi

# nav WS v1 checklist:on_progress 建议 1Hz。nav2 bt_navigator 的 feedback
# 跟 BT tick 走(10Hz+),不节流会把 server 刷爆。
DEFAULT_PROGRESS_MIN_INTERVAL_SEC = 1.0

# 失败消抖(2026-07-24):nav2 偶发单次 ABORTED(控制器抽风/costmap 瞬时
# 占用/启动初期 action server 未就绪)会造成假失败。单次失败不上报 agent,
# 等 DEFAULT_GOAL_RETRY_DELAY_SEC 后重发同一个导航点;连败耗尽重试预算才
# 发 on_error。CANCELED 是主动取消(abort/preempt),不算失败、不重试。
DEFAULT_GOAL_RETRY_COUNT = 1
DEFAULT_GOAL_RETRY_DELAY_SEC = 1.0

# action_msgs/msg/GoalStatus 常量 → 可读名,失败日志用
GOAL_STATUS_NAMES = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}
_STATUS_SUCCEEDED = 4
_STATUS_CANCELED = 5


def _log(message: str):
    print(f"[nav2_action_bridge] {message}", flush=True)


class Nav2ActionBridge:
    def __init__(
        self,
        node,
        action_name: str,
        server_timeout: float,
        event_callback,
        progress_min_interval_sec: float = DEFAULT_PROGRESS_MIN_INTERVAL_SEC,
        goal_retry_count: int = DEFAULT_GOAL_RETRY_COUNT,
        goal_retry_delay_sec: float = DEFAULT_GOAL_RETRY_DELAY_SEC,
    ):
        self.node = node
        self.action_name = action_name
        self.server_timeout = server_timeout
        self.event_callback = event_callback
        self.progress_min_interval_sec = progress_min_interval_sec
        self.goal_retry_count = max(0, int(goal_retry_count))
        self.goal_retry_delay_sec = float(goal_retry_delay_sec)

        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        self._navigate_to_pose = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, action_name)
        self._current_goal_handle = None

    async def _await_rclpy_future(self, future):
        wrapped = concurrent.futures.Future()

        def done_callback(done_future):
            # abort(task.cancel)后 wrapped 已被 wrap_future 连带取消,rclpy
            # future 稍后完成时 set_result 会炸 InvalidStateError 并打进
            # rclpy spin —— 静默丢弃即可(结果没人要了)。
            if wrapped.cancelled():
                return
            try:
                result = done_future.result()
            except Exception as exc:  # pragma: no cover - defensive bridge path
                with contextlib.suppress(concurrent.futures.InvalidStateError):
                    wrapped.set_exception(exc)
                return
            with contextlib.suppress(concurrent.futures.InvalidStateError):
                wrapped.set_result(result)

        future.add_done_callback(done_callback)
        return await asyncio.wrap_future(wrapped)

    @staticmethod
    def _describe_goal(poi: StoredPoi) -> str:
        pose = poi.map_pose
        return (
            f"target={poi.poi_id}({poi.name}) "
            f"goal={pose.frame_id}(x={pose.x:.2f}, y={pose.y:.2f}, yaw={pose.yaw:.2f}rad)"
        )

    async def start_navigation(self, request_id: str, sub_id: int, poi: StoredPoi) -> None:
        total_attempts = 1 + self.goal_retry_count
        for attempt in range(1, total_attempts + 1):
            failure_detail = await self._attempt_navigation(
                request_id, sub_id, poi, attempt, total_attempts
            )
            if failure_detail is None:
                return
            _log(
                f"NAV FAILURE attempt {attempt}/{total_attempts} "
                f"request_id={request_id} sub_id={sub_id}: {failure_detail}"
            )
            if attempt < total_attempts:
                # 消抖:这次失败不上报 server,等 goal_retry_delay_sec 后
                # 重发同一个导航点(goal stamp 重打,见 _attempt_navigation)。
                _log(
                    f"suppressing failure report, re-publishing goal in "
                    f"{self.goal_retry_delay_sec:.1f}s "
                    f"(attempt {attempt + 1}/{total_attempts})"
                )
                await asyncio.sleep(self.goal_retry_delay_sec)
                continue
            _log(
                f"retry budget exhausted ({total_attempts} attempts) — "
                f"reporting on_error to server: {failure_detail}"
            )
            await self.event_callback(
                {
                    "kind": "error",
                    "request_id": request_id,
                    "sub_id": sub_id,
                    "error_message": (
                        f"导航失败(共尝试{total_attempts}次): {failure_detail}"
                    ),
                }
            )
            return

    async def _attempt_navigation(
        self,
        request_id: str,
        sub_id: int,
        poi: StoredPoi,
        attempt: int,
        total_attempts: int,
    ) -> str | None:
        """发一次 NavigateToPose goal 并等到终态。

        arrived / canceled 事件在这里直接发给 core,返回 None(不重试);
        失败时**不发任何事件**,只返回详细失败原因字符串,由 start_navigation
        决定是消抖重试还是上报 on_error。
        """
        attempt_started = time.monotonic()

        if not self.client.wait_for_server(timeout_sec=self.server_timeout):
            return (
                f"Nav2 action server '{self.action_name}' 在 "
                f"{self.server_timeout:.1f}s 内不可用(未起来或 lifecycle 未 Activate), "
                f"{self._describe_goal(poi)}"
            )

        goal = self._navigate_to_pose.Goal()
        goal.pose.header.frame_id = poi.map_pose.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = poi.map_pose.x
        goal.pose.pose.position.y = poi.map_pose.y
        goal.pose.pose.position.z = poi.map_pose.z
        orientation = poi.map_pose.orientation()
        goal.pose.pose.orientation.x = orientation["x"]
        goal.pose.pose.orientation.y = orientation["y"]
        goal.pose.pose.orientation.z = orientation["z"]
        goal.pose.pose.orientation.w = orientation["w"]

        last_progress_monotonic = 0.0
        last_remaining_distance: float | None = None

        def feedback_callback(feedback_msg):
            nonlocal last_progress_monotonic, last_remaining_distance
            feedback = feedback_msg.feedback
            last_remaining_distance = float(
                getattr(feedback, "distance_remaining", 0.0)
            )
            now = time.monotonic()
            if now - last_progress_monotonic < self.progress_min_interval_sec:
                return
            last_progress_monotonic = now
            asyncio.create_task(
                self.event_callback(
                    {
                        "kind": "progress",
                        "request_id": request_id,
                        "sub_id": sub_id,
                        "remaining_distance": last_remaining_distance,
                        "status": "running",
                    }
                )
            )

        send_goal_future = self.client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        try:
            goal_handle = await self._await_rclpy_future(send_goal_future)
        except asyncio.CancelledError:
            # abort 恰好落在 goal 已发出、goal_handle 还没回来的窗口:此时
            # cancel_current 够不着这个 goal(handle 还不存在),不补刀的话
            # nav2 拿着它继续开车而 executor 已经没人盯着。handle 迟到时
            # 立即 cancel。
            def _cancel_late_handle(done_future):
                try:
                    handle = done_future.result()
                except Exception:  # pragma: no cover - send 本身失败,无 goal 可取消
                    return
                if handle is not None and getattr(handle, "accepted", False):
                    _log("goal handle arrived after abort — cancelling it")
                    handle.cancel_goal_async()

            send_goal_future.add_done_callback(_cancel_late_handle)
            raise
        self._current_goal_handle = goal_handle
        if goal_handle is None or not goal_handle.accepted:
            return (
                f"Nav2 拒绝 goal(bt_navigator rejected;常见于 TF 未就绪/上一个 "
                f"goal 未清), {self._describe_goal(poi)}"
            )

        result = await self._await_rclpy_future(goal_handle.get_result_async())
        status = getattr(result, "status", None)
        elapsed = time.monotonic() - attempt_started

        if status == _STATUS_SUCCEEDED:
            await self.event_callback(
                {"kind": "arrived", "request_id": request_id, "sub_id": sub_id}
            )
            return None
        if status == _STATUS_CANCELED:
            # 主动取消(abort_navigation / navigate_to preempt),不算失败。
            await self.event_callback(
                {
                    "kind": "progress",
                    "request_id": request_id,
                    "sub_id": sub_id,
                    "remaining_distance": 0.0,
                    "status": "paused",
                }
            )
            return None

        status_name = GOAL_STATUS_NAMES.get(status, "?")
        remaining_text = (
            f"{last_remaining_distance:.2f}m"
            if last_remaining_distance is not None
            else "n/a(无 feedback,可能 planner 直接失败)"
        )
        return (
            f"Nav2 终态 status={status}({status_name}), "
            f"{self._describe_goal(poi)}, "
            f"剩余距离={remaining_text}, 本次耗时={elapsed:.1f}s"
        )

    async def cancel_current(self) -> None:
        if self._current_goal_handle is None:
            return
        await self._await_rclpy_future(self._current_goal_handle.cancel_goal_async())
