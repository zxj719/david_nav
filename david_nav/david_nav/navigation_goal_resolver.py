from __future__ import annotations

import math

from david_nav.grid_map_view import GridMapView
from david_nav.navigation_executor_core import ResolvedNavigationGoal
from david_nav.navigation_types import PoseRecord, StoredPoi


def _distance_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class NavigationGoalResolver:
    def __init__(
        self,
        node,
        pose_provider,
        map_topic: str = "/lightning/grid_map",
        global_costmap_topic: str = "/global_costmap/costmap",
        snap_radius: float = 0.5,
        fallback_snap_radius: float = 1.0,
        goal_clearance_radius: float = 0.1,
        max_snap_radius: float = 3.0,
        snap_step: float = 0.25,
    ):
        self.node = node
        self.pose_provider = pose_provider
        self.snap_radius = snap_radius
        self.fallback_snap_radius = fallback_snap_radius
        self.goal_clearance_radius = goal_clearance_radius
        # 2026-07-24: 两档固定半径(snap/fallback) → 连续外扩。POI 压在
        # static costmap 障碍里时，从 snap_radius 起按 snap_step 逐圈外扩，
        # 直到 max_snap_radius，取第一个可达点（圈内按离机器人最近排序）。
        # fallback_snap_radius 保留作兼容：外扩上限取两者较大值。
        self.max_snap_radius = max(max_snap_radius, fallback_snap_radius)
        self.snap_step = max(snap_step, 0.05)
        self._map_msg = None
        self._global_costmap_msg = None

        occupancy_grid = __import__(
            "nav_msgs.msg", fromlist=["OccupancyGrid"]
        ).OccupancyGrid
        self._map_sub = self.node.create_subscription(
            occupancy_grid,
            map_topic,
            self._on_map,
            10,
        )
        self._global_costmap_sub = self.node.create_subscription(
            occupancy_grid,
            global_costmap_topic,
            self._on_global_costmap,
            10,
        )

    def _on_map(self, msg):
        self._map_msg = msg

    def _on_global_costmap(self, msg):
        self._global_costmap_msg = msg

    def _grid_view(self, msg) -> "GridMapView | None":
        return GridMapView.from_occupancy_grid(msg)

    def _is_goal_admissible(self, world_xy: tuple[float, float]) -> bool:
        grid_map = self._grid_view(self._map_msg)
        global_costmap = self._grid_view(self._global_costmap_msg)
        if grid_map is None or global_costmap is None:
            return True

        grid_cell = grid_map.world_to_cell(*world_xy)
        global_cell = global_costmap.world_to_cell(*world_xy)
        if grid_cell is None or global_cell is None:
            return False

        if not grid_map.is_goal_free(*grid_cell) or not global_costmap.is_goal_free(*global_cell):
            return False

        if self.goal_clearance_radius <= 0.0:
            return True

        radius_cells = max(
            0,
            int(math.ceil(self.goal_clearance_radius / global_costmap.resolution)),
        )
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x = global_cell[0] + dx
                y = global_cell[1] + dy
                if not global_costmap.in_bounds(x, y):
                    return False
                candidate_xy = global_costmap.cell_center_world(x, y)
                if (
                    _distance_xy(candidate_xy, world_xy) <= self.goal_clearance_radius
                    and not global_costmap.is_goal_free(x, y)
                ):
                    return False
        return True

    def _ring_candidates(
        self, anchor_xy: tuple[float, float], snap_radius: float
    ) -> list[tuple[float, float]]:
        global_costmap = self._grid_view(self._global_costmap_msg)
        if global_costmap is None:
            return []
        anchor_cell = global_costmap.world_to_cell(*anchor_xy)
        if anchor_cell is None:
            return []

        radius_cells = max(1, int(math.ceil(snap_radius / global_costmap.resolution)))
        tolerance = 0.5 * global_costmap.resolution + 1e-9
        candidates = []
        for dy in range(-radius_cells - 1, radius_cells + 2):
            for dx in range(-radius_cells - 1, radius_cells + 2):
                x = anchor_cell[0] + dx
                y = anchor_cell[1] + dy
                if not global_costmap.in_bounds(x, y):
                    continue
                world_xy = global_costmap.cell_center_world(x, y)
                if abs(_distance_xy(world_xy, anchor_xy) - snap_radius) <= tolerance:
                    candidates.append(world_xy)
        return candidates

    def _robot_xy(self) -> tuple[float, float] | None:
        try:
            pose = self.pose_provider.current_map_pose()
        except Exception:
            return None
        return (pose.x, pose.y)

    def _snapped_poi(
        self, anchor_poi: StoredPoi, candidate_xy: tuple[float, float]
    ) -> StoredPoi:
        snapped_map_pose = PoseRecord(
            frame_id=anchor_poi.map_pose.frame_id,
            x=candidate_xy[0],
            y=candidate_xy[1],
            z=anchor_poi.map_pose.z,
            yaw=anchor_poi.map_pose.yaw,
            stamp_sec=anchor_poi.map_pose.stamp_sec,
        )
        return StoredPoi(
            poi_id=anchor_poi.poi_id,
            name=anchor_poi.name,
            map_pose=snapped_map_pose,
            odom_pose=anchor_poi.odom_pose,
            slam_session_id=anchor_poi.slam_session_id,
        )

    def resolve_initial_goal(self, poi: StoredPoi) -> ResolvedNavigationGoal | None:
        anchor_xy = (poi.map_pose.x, poi.map_pose.y)
        if self._is_goal_admissible(anchor_xy):
            return ResolvedNavigationGoal(poi=poi, used_snap=False)

        # 2026-07-24: 连续外扩搜索。POI 在 static costmap / 全局代价里时，
        # 逐圈向外找第一个可达点为止（圈内候选按离机器人最近优先）。
        robot_xy = self._robot_xy()
        radius = max(self.snap_radius, self.snap_step)
        while radius <= self.max_snap_radius + 1e-9:
            candidates = self._ring_candidates(anchor_xy, radius)
            if robot_xy is not None:
                candidates.sort(key=lambda xy: _distance_xy(xy, robot_xy))
            for candidate_xy in candidates:
                if self._is_goal_admissible(candidate_xy):
                    return ResolvedNavigationGoal(
                        poi=self._snapped_poi(poi, candidate_xy),
                        used_snap=True,
                    )
            radius += self.snap_step
        return None

    def is_anchor_admissible(self, poi: StoredPoi) -> bool:
        return self._is_goal_admissible((poi.map_pose.x, poi.map_pose.y))

    def is_anchor_static_blocked(self, poi: StoredPoi) -> bool:
        """anchor 是否被【静态地图】占用（= 永久不可达）。

        2026-07-24 需求：POI 压在静态图障碍里（标在货架/墙上的语义点）时，
        扩圈可行点就是物理上能到的最好位置 —— core 据此把"到达扩圈点"
        判为导航成功；只有"静态图自由、costmap 动态占用"（障碍可能散去）
        才保留"仍不可达"报错。静态图未就绪时返回 False（信息不足，
        维持原报错语义，不误判成功）。
        """
        grid_map = self._grid_view(self._map_msg)
        if grid_map is None:
            return False
        cell = grid_map.world_to_cell(poi.map_pose.x, poi.map_pose.y)
        if cell is None:
            return True   # 出静态图边界，同样永久不可达
        return not grid_map.is_goal_free(*cell)
