from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time

from g1_nav.navigation_protocol import (
    build_arrived_event,
    build_error_event,
    build_mark_poi_ack,
    build_mark_poi_error,
    build_mark_poi_success_event,
    build_progress_event,
    parse_command,
)
from g1_nav.navigation_types import StoredPoi


class ExecutorState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    STARTING = "starting"
    NAVIGATING = "navigating"
    ABORTING = "aborting"
    ERROR = "error"


@dataclass
class ActiveTask:
    request_id: str
    sub_id: int
    target_id: str
    anchor_poi: StoredPoi
    current_goal_poi: StoredPoi
    phase: str = "to_anchor"
    remaining_distance: float = 0.0


@dataclass(frozen=True)
class ResolvedNavigationGoal:
    poi: StoredPoi
    used_snap: bool


class DirectNavigationGoalResolver:
    def resolve_initial_goal(self, poi: StoredPoi) -> ResolvedNavigationGoal:
        return ResolvedNavigationGoal(poi=poi, used_snap=False)

    def is_anchor_admissible(self, poi: StoredPoi) -> bool:
        return True


class NavigationExecutorCore:
    def __init__(
        self,
        bridge,
        pose_provider,
        poi_store,
        goal_resolver=None,
        pose_stale_after_sec: float = 0.5,
        now_fn=None,
    ):
        self.bridge = bridge
        self.pose_provider = pose_provider
        self.poi_store = poi_store
        self.goal_resolver = goal_resolver or DirectNavigationGoalResolver()
        self.pose_stale_after_sec = pose_stale_after_sec
        self.now_fn = now_fn or (lambda: time.time())
        self.state = ExecutorState.IDLE
        self.active_task: ActiveTask | None = None

    async def handle_message(self, payload: dict) -> list[dict]:
        command = parse_command(payload)
        if command.action == "update_poi_list":
            self.poi_store.apply_semantic_directory(command.poi_list)
            return []
        if command.action == "navigate_to":
            try:
                poi = self.poi_store.get_required(command.target_id)
            except KeyError:
                self.state = ExecutorState.IDLE
                self.active_task = None
                return [
                    build_error_event(
                        command.request_id,
                        command.sub_id,
                        f"目标点不存在: {command.target_id}",
                    )
                ]
            if self.active_task is not None:
                await self.bridge.cancel_current()
            resolved_goal = self.goal_resolver.resolve_initial_goal(poi)
            if resolved_goal is None:
                self.state = ExecutorState.IDLE
                self.active_task = None
                return [
                    build_error_event(
                        command.request_id,
                        command.sub_id,
                        "目标点不可达，且未找到可行扩圈点",
                    )
                ]
            self.active_task = ActiveTask(
                command.request_id,
                command.sub_id,
                command.target_id,
                anchor_poi=poi,
                current_goal_poi=resolved_goal.poi,
                phase="to_snap" if resolved_goal.used_snap else "to_anchor",
            )
            self.state = ExecutorState.STARTING
            await self.bridge.start_navigation(
                command.request_id,
                command.sub_id,
                resolved_goal.poi,
            )
            return []
        if command.action == "abort_navigation":
            if self.active_task is None:
                return []
            if self.active_task.request_id != command.request_id:
                return []
            await self.bridge.cancel_current()
            self.state = ExecutorState.IDLE
            paused = build_progress_event(
                self.active_task.request_id,
                self.active_task.sub_id,
                self.active_task.remaining_distance,
                "paused",
            )
            self.active_task = None
            return [paused]
        if command.action == "mark_current_poi":
            outbound = [build_mark_poi_ack(command.request_id, command.sub_id)]
            try:
                captured = await self.pose_provider.capture_for_poi()
            except Exception as exc:
                outbound.append(
                    build_mark_poi_error(
                        command.request_id,
                        command.sub_id,
                        str(exc),
                    )
                )
                return outbound
            if self.now_fn() - captured.map_pose.stamp_sec > self.pose_stale_after_sec:
                outbound.append(
                    build_mark_poi_error(
                        command.request_id,
                        command.sub_id,
                        "当前位置位姿数据过期",
                    )
                )
                return outbound
            stored = self.poi_store.upsert_marked_poi(
                poi_id=command.poi_id,
                name=command.poi_name,
                map_pose=captured.map_pose,
                odom_pose=captured.odom_pose,
                slam_session_id=captured.slam_session_id,
            )
            outbound.append(
                build_mark_poi_success_event(
                    command.request_id,
                    command.sub_id,
                    stored,
                )
            )
            return outbound
        raise ValueError(f"unsupported command in core: {command.action}")

    async def handle_bridge_event(self, event: dict) -> list[dict]:
        if self.active_task is None:
            return []
        if event["kind"] == "progress":
            self.state = ExecutorState.NAVIGATING
            self.active_task.remaining_distance = float(event["remaining_distance"])
            return [
                build_progress_event(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                    self.active_task.remaining_distance,
                    event.get("status", "running"),
                )
            ]
        if event["kind"] == "arrived":
            if self.active_task.phase == "to_snap":
                if not self.goal_resolver.is_anchor_admissible(self.active_task.anchor_poi):
                    outbound = [
                        build_error_event(
                            self.active_task.request_id,
                            self.active_task.sub_id,
                            "到达扩圈可行点后，目标点仍不可达",
                        )
                    ]
                    self.active_task = None
                    self.state = ExecutorState.IDLE
                    return outbound
                self.active_task.phase = "to_anchor"
                self.active_task.current_goal_poi = self.active_task.anchor_poi
                self.state = ExecutorState.STARTING
                await self.bridge.start_navigation(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                    self.active_task.anchor_poi,
                )
                return []
            outbound = [
                build_arrived_event(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                )
            ]
            self.active_task = None
            self.state = ExecutorState.IDLE
            return outbound
        if event["kind"] == "error":
            outbound = [
                build_error_event(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                    event["error_message"],
                )
            ]
            self.active_task = None
            self.state = ExecutorState.IDLE
            return outbound
        return []
