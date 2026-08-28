from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import Enum
import time

from david_nav.navigation_protocol import (
    CommandParseError,
    build_arrived_event,
    build_error_event,
    build_mark_poi_ack,
    build_mark_poi_error,
    build_mark_poi_success_event,
    build_progress_event,
    parse_command,
)
from david_nav.navigation_types import StoredPoi


def _log(message: str):
    print(f"[navigation_executor_core] {message}", flush=True)


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

    def is_anchor_static_blocked(self, poi: StoredPoi) -> bool:
        return False


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
        # nav WS v1.1(2026-07-27):bridge.start_navigation 会 await 到 nav2
        # 终态,inline await 会把 WS 收包循环整个卡死 → abort_navigation 读
        # 不进来,"随时可中断"不成立。导航后台 task 化,core 持有引用以便
        # abort/preempt 时 cancel(否则任务还挂在旧 goal 的重试等待上,
        # 会把旧目标重新发出去)。
        self._nav_task: asyncio.Task | None = None

    def _spawn_navigation(self, request_id: str, sub_id: int, poi: StoredPoi) -> None:
        self._nav_task = asyncio.create_task(
            self.bridge.start_navigation(request_id, sub_id, poi)
        )
        self._nav_task.add_done_callback(self._on_nav_task_done)

    @staticmethod
    def _on_nav_task_done(task: asyncio.Task) -> None:
        # 后台任务的异常必须落日志,否则只剩 "Task exception was never
        # retrieved" 且导航悄悄死掉。终态帧本身由 bridge 的 event_callback
        # 链路负责,这里只兜底记录代码级异常。
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            _log(f"navigation task crashed: {exc!r}")

    async def _cancel_active_navigation(self) -> None:
        """停掉当前后台导航:先 cancel task(封死重试重发窗口),再 cancel
        nav2 goal(停车),最后收尸。任何状态下调用都安全。"""
        task = self._nav_task
        self._nav_task = None
        if task is not None and not task.done():
            task.cancel()
        await self.bridge.cancel_current()
        if task is not None:
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception as exc:
                # 被中断的任务怎么死都不能反噬 abort 流程(v1.1 幂等要求)
                _log(f"navigation task raised during cancel: {exc!r}")

    async def handle_message(self, payload: dict) -> list[dict]:
        try:
            command = parse_command(payload)
        except CommandParseError as exc:
            # v1 checklist:字段缺失/类型错 → on_error,不启车,状态不动。
            # request_id 都没有时无从回帧,交给上层记日志(re-raise)。
            if exc.request_id is None:
                raise
            return [build_error_event(exc.request_id, exc.sub_id, str(exc))]
        if command.action == "update_poi_list":
            self.poi_store.apply_semantic_directory(command.poi_list)
            return []
        if command.action == "navigate_to":
            target_id = command.target_id
            try:
                poi = self.poi_store.get_required(command.target_id)
            except KeyError:
                # on_error 即终态,agent 端 future 已 resolve;回 IDLE 等下一条。
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
                # preempt:除了 cancel nav2 goal,还要 cancel 旧的后台导航
                # task,否则它可能还在失败重试等待里,醒来把旧目标重发。
                await self._cancel_active_navigation()
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
                target_id,
                anchor_poi=poi,
                current_goal_poi=resolved_goal.poi,
                phase="to_snap" if resolved_goal.used_snap else "to_anchor",
            )
            self.state = ExecutorState.STARTING
            # 后台 task 化(v1.1):handle_message 立刻返回,WS 循环继续收
            # abort/ping。sleep(0) 让新任务先跑到第一个 await(真实 bridge
            # 在此已把 goal 发给 nav2)。
            self._spawn_navigation(
                command.request_id,
                command.sub_id,
                resolved_goal.poi,
            )
            await asyncio.sleep(0)
            return []
        if command.action == "abort_navigation":
            # v1.1:任意状态下 abort → idle,且幂等(空闲/重复/已终态的
            # abort 不崩、不回帧)。request_id 不匹配 = 过期帧,不能误杀
            # 正在跑的新导航(agent 同一时刻只维护一个 active request)。
            if self.active_task is None:
                self.state = ExecutorState.IDLE
                return []
            if self.active_task.request_id != command.request_id:
                return []
            self.state = ExecutorState.ABORTING
            await self._cancel_active_navigation()
            # v1 §三:abort 的确认帧是 on_progress(status="paused"),agent 收到
            # 后立刻把 future resolve 为 cancelled(≤5s deadline)。这里 cancel
            # 一发出去就同步回 paused,不等 nav2 的 canceled 结果回调(那条
            # 事件因 active_task 已清而被丢弃,不会重复发)。
            outbound = [
                build_progress_event(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                    self.active_task.remaining_distance,
                    "paused",
                )
            ]
            self.state = ExecutorState.IDLE
            self.active_task = None
            return outbound
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
        # 只认当前任务的事件。preempt(navigate_to 顶掉旧任务)后,旧 goal 的
        # canceled 结果会晚到,不过滤的话会以新 request_id 发出 paused,被
        # agent 当成新导航被取消(v1 §五:paused 只能跟在 abort 之后)。
        event_request_id = event.get("request_id")
        if event_request_id is not None and event_request_id != self.active_task.request_id:
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
                    # 2026-07-27：到达扩圈可行点 = 导航成功，不再区分
                    # "静态占用（永久）/costmap 动态占用（可能散去）"。
                    # 实测旧语义下 anchor 落在膨胀区（静态图自由但 costmap
                    # 代价>0，永远不会变 admissible）时报"仍不可达"，
                    # 服务端只会原地重试同样失败。resolver 连续外扩选出的
                    # 本来就是最近可达点，到点即是该目标的最好结果。
                    outbound = [
                        build_arrived_event(
                            self.active_task.request_id,
                            self.active_task.sub_id,
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
            # v1 里任务生命周期完全由事件承载(on_error 即终态),state 只是
            # 本地 STATUS 日志用,不再上传;错误后回 IDLE 等下一条 navigate_to。
            self.state = ExecutorState.IDLE
            return outbound
        return []
