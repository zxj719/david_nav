"""Pure legacy-equivalent free-space ring search, independent of ROS subscriptions."""

from dataclasses import dataclass, replace
import math
from typing import Callable

from nav_tasks.core.ports import PoseSource
from nav_tasks.domain.grid import GridMapView
from nav_tasks.domain.tasks import ResolvedGoal
from nav_tasks.domain.types import StoredPoi

Point = tuple[float, float]
GridSource = Callable[[], tuple[GridMapView | None, GridMapView | None]]


def _distance_xy(first: Point, second: Point) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


@dataclass(frozen=True)
class SnapSearch:
    """Search concentric costmap-cell rings using the existing traversal order.

    Inputs retain the legacy axis-aligned GridMapView convention. Missing either
    grid allows the anchor, and cost values other than zero are not free. This
    extraction deliberately does not change those characterized policies.

    Attributes:
        fallback_snap_radius: 后备搜索半径（米）。
        goal_clearance_radius: 候选点周边净空半径（米）。
        max_snap_radius: 扩圈半径上限（米）。
        snap_radius: 初始目标搜索半径（米）。
        snap_step: 扩圈半径增量（米）。
    """

    snap_radius: float = 0.5
    fallback_snap_radius: float = 1.0
    goal_clearance_radius: float = 0.1
    max_snap_radius: float = 3.0
    snap_step: float = 0.25

    def __post_init__(self) -> None:
        """Retain the legacy fallback upper bound and minimum radial step."""
        object.__setattr__(self, 'max_snap_radius', max(
            self.max_snap_radius, self.fallback_snap_radius))
        object.__setattr__(self, 'snap_step', max(self.snap_step, 0.05))

    def resolve(
        self, anchor_xy: Point, robot_xy: Point | None,
        grid: GridMapView | None, costmap: GridMapView | None,
    ) -> Point | None:
        """Return the anchor or the first admissible point on an expanding ring.

        Args:
            anchor_xy: Requested world-coordinate anchor.
            robot_xy: Current robot position; None preserves row-major candidate order.
            grid: Static occupancy grid snapshot, if available.
            costmap: Global costmap snapshot, if available.

        Returns:
            Admissible world coordinates, or None when the search is exhausted.
        """
        if self.is_admissible(anchor_xy, grid, costmap):
            return anchor_xy
        radius = max(self.snap_radius, self.snap_step)
        while radius <= self.max_snap_radius + 1e-9:
            candidates = self._ring_candidates(anchor_xy, radius, costmap)
            if robot_xy is not None:
                candidates.sort(key=lambda xy: _distance_xy(xy, robot_xy))
            for candidate in candidates:
                if self.is_admissible(candidate, grid, costmap):
                    return candidate
            radius += self.snap_step
        return None

    def is_admissible(
        self, world_xy: Point, grid: GridMapView | None, costmap: GridMapView | None,
    ) -> bool:
        """Apply the legacy zero-cost and circular-clearance checks.

        Args:
            world_xy: Candidate in world coordinates.
            grid: Static occupancy grid snapshot.
            costmap: Global costmap snapshot.

        Returns:
            Whether the candidate is admissible under the existing policy.
        """
        if grid is None or costmap is None:
            return True
        grid_cell = grid.world_to_cell(*world_xy)
        global_cell = costmap.world_to_cell(*world_xy)
        if grid_cell is None or global_cell is None:
            return False
        if not grid.is_goal_free(*grid_cell) or not costmap.is_goal_free(*global_cell):
            return False
        if self.goal_clearance_radius <= 0.0:
            return True
        radius_cells = max(0, int(math.ceil(self.goal_clearance_radius / costmap.resolution)))
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x, y = global_cell[0] + dx, global_cell[1] + dy
                if not costmap.in_bounds(x, y):
                    return False
                candidate_xy = costmap.cell_center_world(x, y)
                if (_distance_xy(candidate_xy, world_xy) <= self.goal_clearance_radius
                        and not costmap.is_goal_free(x, y)):
                    return False
        return True

    @staticmethod
    def _ring_candidates(
        anchor_xy: Point, snap_radius: float, costmap: GridMapView | None,
    ) -> list[Point]:
        if costmap is None:
            return []
        anchor_cell = costmap.world_to_cell(*anchor_xy)
        if anchor_cell is None:
            return []
        radius_cells = max(1, int(math.ceil(snap_radius / costmap.resolution)))
        tolerance = 0.5 * costmap.resolution + 1e-9
        candidates = []
        for dy in range(-radius_cells - 1, radius_cells + 2):
            for dx in range(-radius_cells - 1, radius_cells + 2):
                x, y = anchor_cell[0] + dx, anchor_cell[1] + dy
                if not costmap.in_bounds(x, y):
                    continue
                world_xy = costmap.cell_center_world(x, y)
                if abs(_distance_xy(world_xy, anchor_xy) - snap_radius) <= tolerance:
                    candidates.append(world_xy)
        return candidates


class DirectGoalResolver:
    """Resolve a POI directly when the assembly root disables snapping."""

    def resolve(self, poi: StoredPoi) -> ResolvedGoal:
        """Return the unchanged anchor.

        Args:
            poi: Requested POI.

        Returns:
            Direct non-snapped resolution.
        """
        return ResolvedGoal(poi, False)

    def is_admissible(self, poi: StoredPoi) -> bool:
        """Accept the anchor without occupancy inspection.

        Args:
            poi: Requested POI, unused by this policy.

        Returns:
            True, matching the legacy direct resolver.
        """
        return True


class GridGoalResolver:
    """Adapt pure grid search to POI values without subscribing to anything."""

    def __init__(
        self, grids: GridSource, poses: PoseSource, search: SnapSearch = SnapSearch(),
    ) -> None:
        """Inject data snapshots and search configuration.

        Args:
            grids: Provider of static-grid and costmap snapshots.
            poses: Current-pose provider used to order snap candidates.
            search: Pure search policy.
        """
        self._grids = grids
        self._poses = poses
        self._search = search

    def resolve(self, poi: StoredPoi) -> ResolvedGoal | None:
        """Find a target while preserving all nonposition POI fields.

        Args:
            poi: Requested anchor.

        Returns:
            Direct/snapped POI, or None when no candidate is admissible.
        """
        anchor_xy = (poi.map_pose.x, poi.map_pose.y)
        grid, costmap = self._grids()
        if self._search.is_admissible(anchor_xy, grid, costmap):
            return ResolvedGoal(poi, False)
        try:
            pose = self._poses.current_map_pose()
            robot_xy = (pose.x, pose.y)
        except Exception:
            # Legacy policy: missing robot pose removes distance sorting only.
            robot_xy = None
        candidate = self._search.resolve(anchor_xy, robot_xy, grid, costmap)
        if candidate is None:
            return None
        snapped = replace(poi, map_pose=replace(poi.map_pose, x=candidate[0], y=candidate[1]))
        return ResolvedGoal(snapped, True)

    def is_admissible(self, poi: StoredPoi) -> bool:
        """Recheck an anchor using the latest grid snapshots.

        Args:
            poi: Original POI anchor.

        Returns:
            Whether the anchor is now admissible.
        """
        grid, costmap = self._grids()
        return self._search.is_admissible((poi.map_pose.x, poi.map_pose.y), grid, costmap)
