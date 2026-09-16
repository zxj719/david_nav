"""Pure read-only grid access shared by goal resolution and path sparsification."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class GridMapView:
    """Read-only view over a nav_msgs/OccupancyGrid (values 0..100, -1 unknown).

    Lifted out of navigation_goal_resolver._GridMapView so the sparsifier can
    share the same world<->cell math. is_goal_free is kept for the resolver's
    existing (cost==0) admissibility check; is_blocked adds a thresholded check
    for line-of-sight.

    Attributes:
        cells: 按行排列的格值列表；冻结对象不会深冻结该列表。
        height: 栅格行数。
        origin_x: 旧轴对齐栅格模型的原点 x（米）。
        origin_y: 旧轴对齐栅格模型的原点 y（米）。
        resolution: 每格边长（米）。
        width: 栅格列数。
    """

    cells: list
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def in_bounds(self, x: int, y: int) -> bool:
        """Check integer cell coordinates against the grid dimensions.

        Args:
            x: Column index.
            y: Row index.

        Returns:
            Whether the cell lies inside the grid.
        """
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, x: int, y: int) -> int:
        """Read a row-major cell without adding bounds or occupancy policy.

        Args:
            x: Column index, already checked by the caller.
            y: Row index, already checked by the caller.

        Returns:
            Stored occupancy/cost value, including negative unknown values.
        """
        return self.cells[y * self.width + x]

    def is_goal_free(self, x: int, y: int) -> bool:
        """Apply the existing strict zero-cost goal admissibility rule.

        Args:
            x: Column index.
            y: Row index.

        Returns:
            Whether the stored value is exactly zero.
        """
        return self.value(x, y) == 0

    def is_blocked(self, x: int, y: int, threshold: int,
                   block_unknown: bool) -> bool:
        """Apply the caller's threshold and unknown-cell policy.

        Args:
            x: Column index.
            y: Row index.
            threshold: Inclusive occupied-value threshold.
            block_unknown: Whether negative values block line of sight.

        Returns:
            Whether the cell is blocked under that policy.
        """
        v = self.value(x, y)
        if v < 0:
            return block_unknown
        return v >= threshold

    def world_to_cell(self, wx: float, wy: float) -> tuple[int, int] | None:
        """Convert a world point using the legacy axis-aligned origin model.

        Args:
            wx: World x coordinate.
            wy: World y coordinate.

        Returns:
            Column/row indices, or None when the point is outside the grid.
        """
        x = int(math.floor((wx - self.origin_x) / self.resolution))
        y = int(math.floor((wy - self.origin_y) / self.resolution))
        if not self.in_bounds(x, y):
            return None
        return (x, y)

    def cell_center_world(self, x: int, y: int) -> tuple[float, float]:
        """Convert integer cell coordinates to the corresponding world center.

        Args:
            x: Column index.
            y: Row index.

        Returns:
            World x/y coordinates of the cell center.
        """
        return (
            (float(x) + 0.5) * self.resolution + self.origin_x,
            (float(y) + 0.5) * self.resolution + self.origin_y,
        )

    @classmethod
    def from_occupancy_grid(cls, msg: Any) -> "GridMapView | None":
        """Copy a structurally compatible grid message without importing ROS.

        Args:
            msg: Object with info/data fields, or None.

        Returns:
            An independent view, or None for absent/empty/nonpositive dimensions.
        """
        if msg is None:
            return None
        info = msg.info
        if info.width <= 0 or info.height <= 0 or not msg.data:
            return None
        return cls(
            cells=list(msg.data),
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
        )
