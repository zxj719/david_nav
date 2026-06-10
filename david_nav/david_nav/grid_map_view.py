from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GridMapView:
    """Read-only view over a nav_msgs/OccupancyGrid (values 0..100, -1 unknown).

    Lifted out of navigation_goal_resolver._GridMapView so the sparsifier can
    share the same world<->cell math. is_goal_free is kept for the resolver's
    existing (cost==0) admissibility check; is_blocked adds a thresholded check
    for line-of-sight.
    """

    cells: list
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, x: int, y: int) -> int:
        return self.cells[y * self.width + x]

    def is_goal_free(self, x: int, y: int) -> bool:
        return self.value(x, y) == 0

    def is_blocked(self, x: int, y: int, threshold: int,
                   block_unknown: bool) -> bool:
        v = self.value(x, y)
        if v < 0:
            return block_unknown
        return v >= threshold

    def world_to_cell(self, wx: float, wy: float):
        x = int(math.floor((wx - self.origin_x) / self.resolution))
        y = int(math.floor((wy - self.origin_y) / self.resolution))
        if not self.in_bounds(x, y):
            return None
        return (x, y)

    def cell_center_world(self, x: int, y: int):
        return (
            (float(x) + 0.5) * self.resolution + self.origin_x,
            (float(y) + 0.5) * self.resolution + self.origin_y,
        )

    @classmethod
    def from_occupancy_grid(cls, msg) -> "GridMapView | None":
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
