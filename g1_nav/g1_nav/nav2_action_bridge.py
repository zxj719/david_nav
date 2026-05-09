from __future__ import annotations

import asyncio
import concurrent.futures

from g1_nav.navigation_types import StoredPoi


class Nav2ActionBridge:
    def __init__(self, node, action_name: str, server_timeout: float, event_callback):
        self.node = node
        self.action_name = action_name
        self.server_timeout = server_timeout
        self.event_callback = event_callback

        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        self._navigate_to_pose = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, action_name)
        self._current_goal_handle = None

    async def _await_rclpy_future(self, future):
        wrapped = concurrent.futures.Future()

        def done_callback(done_future):
            try:
                wrapped.set_result(done_future.result())
            except Exception as exc:  # pragma: no cover - defensive bridge path
                wrapped.set_exception(exc)

        future.add_done_callback(done_callback)
        return await asyncio.wrap_future(wrapped)

    async def start_navigation(self, request_id: str, sub_id: int, poi: StoredPoi) -> None:
        if not self.client.wait_for_server(timeout_sec=self.server_timeout):
            await self.event_callback(
                {
                    "kind": "error",
                    "request_id": request_id,
                    "sub_id": sub_id,
                    "error_message": (
                        "Nav2 action server is not available within "
                        f"{self.server_timeout:.1f}s"
                    ),
                }
            )
            return

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

        def feedback_callback(feedback_msg):
            feedback = feedback_msg.feedback
            remaining_distance = getattr(feedback, "distance_remaining", 0.0)
            asyncio.create_task(
                self.event_callback(
                    {
                        "kind": "progress",
                        "request_id": request_id,
                        "sub_id": sub_id,
                        "remaining_distance": float(remaining_distance),
                        "status": "running",
                    }
                )
            )

        goal_handle = await self._await_rclpy_future(
            self.client.send_goal_async(goal, feedback_callback=feedback_callback)
        )
        self._current_goal_handle = goal_handle
        if goal_handle is None or not goal_handle.accepted:
            await self.event_callback(
                {
                    "kind": "error",
                    "request_id": request_id,
                    "sub_id": sub_id,
                    "error_message": "Nav2 rejected the goal",
                }
            )
            return

        result = await self._await_rclpy_future(goal_handle.get_result_async())
        status = getattr(result, "status", None)
        status_succeeded = 4
        status_canceled = 5

        if status == status_succeeded:
            await self.event_callback(
                {"kind": "arrived", "request_id": request_id, "sub_id": sub_id}
            )
        elif status == status_canceled:
            await self.event_callback(
                {
                    "kind": "progress",
                    "request_id": request_id,
                    "sub_id": sub_id,
                    "remaining_distance": 0.0,
                    "status": "paused",
                }
            )
        else:
            await self.event_callback(
                {
                    "kind": "error",
                    "request_id": request_id,
                    "sub_id": sub_id,
                    "error_message": f"Nav2 finished with status {status}",
                }
            )

    async def cancel_current(self) -> None:
        if self._current_goal_handle is None:
            return
        await self._await_rclpy_future(self._current_goal_handle.cancel_goal_async())
