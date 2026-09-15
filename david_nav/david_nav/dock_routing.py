"""把指定 POI 的导航改走精准入位（alias 0，2026-09-15）。

navigation_executor 平时通过 Nav2ActionBridge 把 POI 发给 bt_navigator 的
/navigate_to_pose。--dock-poi-id 列出的 POI 改发 dock_navigator 的 dock_to_pose
动作（同为 NavigateToPose 类型，内部是两段式对齐入位），因此：

* 不做目标扩圈（snap）：POI 位姿就是要精确到达的入位位姿，挪开就失去意义；
* 失败不静默重发：dock_navigator 约定入位失败后不再下发任何位移目标，
  重发等于再走一遍两段式位移（由创建 dock bridge 时 goal_retry_count=0 保证）。

其余 POI 的行为与原来完全相同。
"""
from __future__ import annotations

from david_nav.navigation_executor_core import ResolvedNavigationGoal
from david_nav.navigation_types import StoredPoi


class DockRoutingBridge:
    """对 core 暴露与 Nav2ActionBridge 相同的接口，按 POI 选择底层 bridge。"""

    def __init__(self, default_bridge, dock_bridge, dock_poi_ids):
        self.default_bridge = default_bridge
        self.dock_bridge = dock_bridge
        self.dock_poi_ids = frozenset(dock_poi_ids)
        self._active_bridge = None

    def bridge_for(self, poi: StoredPoi):
        if poi.poi_id in self.dock_poi_ids:
            return self.dock_bridge
        return self.default_bridge

    async def start_navigation(self, request_id: str, sub_id: int, poi: StoredPoi) -> None:
        bridge = self.bridge_for(poi)
        # 记住当前目标走的是哪条链路：abort / preempt 时只取消这一条。
        self._active_bridge = bridge
        await bridge.start_navigation(request_id, sub_id, poi)

    async def cancel_current(self) -> None:
        bridge = self._active_bridge
        if bridge is not None:
            await bridge.cancel_current()


class DockAwareGoalResolver:
    """精准入位 POI 永不扩圈，其余 POI 交给原 resolver。"""

    def __init__(self, inner, dock_poi_ids):
        self.inner = inner
        self.dock_poi_ids = frozenset(dock_poi_ids)

    def resolve_initial_goal(self, poi: StoredPoi) -> ResolvedNavigationGoal | None:
        if poi.poi_id in self.dock_poi_ids:
            return ResolvedNavigationGoal(poi=poi, used_snap=False)
        return self.inner.resolve_initial_goal(poi)

    def is_anchor_admissible(self, poi: StoredPoi) -> bool:
        return self.inner.is_anchor_admissible(poi)

    def is_anchor_static_blocked(self, poi: StoredPoi) -> bool:
        return self.inner.is_anchor_static_blocked(poi)
