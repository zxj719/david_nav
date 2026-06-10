# Waypoint Follower (Nav2-planned, position-controller-driven) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `waypoint_follower` node that asks Nav2 to *plan only* (`ComputePathToPose`), sparsifies the global path into waypoints where every consecutive pair is a collision-free straight line ≤3m, and drives each segment with the existing `position_controller` via its `MoveTo` action.

**Architecture:** Nav2 stays as a pure global planner; the local controller (DWB) is bypassed. A pure-logic sparsifier (greedy farthest-reachable + Bresenham line-of-sight over the inflated global costmap) picks waypoints. The node converts each waypoint to a robot-relative `MoveTo` POSE goal computed from the live `map` pose (immune to map↔odom drift) and drives segments sequentially. Static-map assumption: segments are validated once at plan time, no runtime monitoring/replanning.

**Tech Stack:** Python 3, rclpy, ROS 2 Humble, `nav2_msgs/action/{ComputePathToPose,NavigateToPose}`, `david_nav_msgs/action/MoveTo`, `nav_msgs/OccupancyGrid`, tf2_ros. Pure logic unit-tested with pytest (no colcon build needed).

Spec: [docs/superpowers/specs/2026-06-09-waypoint-follower-position-controller-design.md](../specs/2026-06-09-waypoint-follower-position-controller-design.md)

---

## File Structure

| File | Responsibility |
|---|---|
| `david_nav/david_nav/grid_map_view.py` | **New, pure.** `GridMapView` lifted out of `navigation_goal_resolver.py` + `is_blocked()` + `from_occupancy_grid()` factory. Shared cell/world conversion. |
| `david_nav/david_nav/navigation_goal_resolver.py` | **Modify.** Import `GridMapView` instead of the local `_GridMapView` dataclass; behavior unchanged. |
| `david_nav/david_nav/waypoint_sparsifier.py` | **New, pure.** Bresenham line-of-sight, greedy farthest-reachable sparsify, relative-target geometry. No ROS imports. |
| `david_nav/david_nav/waypoint_follower.py` | **New.** rclpy node: `ComputePathToPose` client + costmap sub + sparsify + per-segment `MoveTo` driver + `~/navigate` action server + `~/goal_pose` topic + cancel. |
| `david_nav/config/waypoint_follower.yaml` | **New.** Parameters. |
| `david_nav/launch/waypoint_follower.launch.py` | **New.** Brings up `waypoint_follower` + `position_controller` + `cmd_vel_mux`. |
| `david_nav/CMakeLists.txt` | **Modify.** Add `install(PROGRAMS david_nav/waypoint_follower.py … RENAME waypoint_follower)`. |
| `david_nav/test/test_grid_map_view.py` | **New.** Unit tests for view + `is_blocked`. |
| `david_nav/test/test_waypoint_sparsifier.py` | **New.** Unit tests for Bresenham / sparsify / geometry. |

All paths below are relative to `/home/agx/ros2_ws/src/david_nav`.

> **Workspace note:** `build/` and `install/` are `lrx`-owned (being chowned to `agx`). All **unit tests run with plain `pytest` — no colcon build required** (matches existing `test/test_position_control_core.py` workflow). The integration/sim step (Task 7) and `ros2 run` need a `colcon build --packages-select david_nav`; run that as whoever owns `build/install`.

---

## Task 1: `GridMapView` shared module

**Files:**
- Create: `david_nav/grid_map_view.py`
- Test: `test/test_grid_map_view.py`

- [ ] **Step 1: Write the failing test**

Create `test/test_grid_map_view.py`:

```python
import math

from david_nav.grid_map_view import GridMapView


def _grid(cells, w, h, res=1.0, ox=0.0, oy=0.0):
    return GridMapView(cells=cells, width=w, height=h, resolution=res,
                       origin_x=ox, origin_y=oy)


def test_in_bounds_and_value():
    g = _grid([0, 1, 2, 3], 2, 2)
    assert g.in_bounds(0, 0)
    assert g.in_bounds(1, 1)
    assert not g.in_bounds(2, 0)
    assert not g.in_bounds(-1, 0)
    assert g.value(1, 1) == 3  # y*width + x = 1*2 + 1


def test_world_to_cell_and_center():
    g = _grid([0] * 16, 4, 4, res=0.5, ox=-1.0, oy=-1.0)
    # world (0.0, 0.0) -> floor((0 - -1)/0.5) = 2
    assert g.world_to_cell(0.0, 0.0) == (2, 2)
    # out of bounds returns None
    assert g.world_to_cell(100.0, 0.0) is None
    cx, cy = g.cell_center_world(2, 2)
    assert math.isclose(cx, 0.25) and math.isclose(cy, 0.25)


def test_is_blocked_threshold_and_unknown():
    # values on the OccupancyGrid 0..100 scale, -1 = unknown
    g = _grid([0, 50, 99, -1], 2, 2)
    assert not g.is_blocked(0, 0, threshold=99, block_unknown=True)   # free
    assert not g.is_blocked(1, 0, threshold=99, block_unknown=True)   # 50 < 99
    assert g.is_blocked(0, 1, threshold=99, block_unknown=True)       # 99 >= 99
    assert g.is_blocked(1, 1, threshold=99, block_unknown=True)       # unknown
    assert not g.is_blocked(1, 1, threshold=99, block_unknown=False)  # unknown allowed


def test_from_occupancy_grid_factory():
    class _Info:
        width = 2
        height = 2
        resolution = 0.5

        class origin:
            class position:
                x = -1.0
                y = -2.0

    class _Msg:
        info = _Info()
        data = [0, 1, 2, 3]

    g = GridMapView.from_occupancy_grid(_Msg())
    assert g is not None
    assert g.width == 2 and g.height == 2
    assert math.isclose(g.resolution, 0.5)
    assert math.isclose(g.origin_x, -1.0) and math.isclose(g.origin_y, -2.0)

    class _Empty:
        class info:
            width = 0
            height = 0
            resolution = 0.0

            class origin:
                class position:
                    x = 0.0
                    y = 0.0
        data = []

    assert GridMapView.from_occupancy_grid(_Empty()) is None
    assert GridMapView.from_occupancy_grid(None) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/test_grid_map_view.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'david_nav.grid_map_view'`

- [ ] **Step 3: Write minimal implementation**

Create `david_nav/grid_map_view.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/test_grid_map_view.py -q`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/agx/ros2_ws/src/david_nav
git add david_nav/grid_map_view.py test/test_grid_map_view.py
git commit -m "feat(nav): add shared GridMapView with is_blocked for line-of-sight

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Point `navigation_goal_resolver` at the shared `GridMapView`

**Files:**
- Modify: `david_nav/navigation_goal_resolver.py:10-39` (remove local `_GridMapView`), `:87-100` (`_grid_view` delegates to factory)

- [ ] **Step 1: Confirm existing resolver tests pass first (baseline)**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/ -q -k "navigation or resolver" 2>&1 | tail -5`
Expected: existing tests pass (or "no tests ran" if none target the resolver — that's fine, the resolver is exercised by the executor tests). Note the result as the baseline.

- [ ] **Step 2: Replace the local dataclass with an import**

In `david_nav/navigation_goal_resolver.py`, delete the entire local `@dataclass(frozen=True) class _GridMapView: …` block (lines 10–39, from `@dataclass` through `cell_center_world`'s return) and add the import near the top (after the existing `import math`):

```python
from david_nav.grid_map_view import GridMapView
```

The remaining top-of-file imports should read:

```python
from __future__ import annotations

from dataclasses import dataclass
import math

from david_nav.grid_map_view import GridMapView
from david_nav.navigation_executor_core import ResolvedNavigationGoal
from david_nav.navigation_types import PoseRecord, StoredPoi
```

(`dataclass` import stays only if still used elsewhere in the file; if `_distance_xy` and the rest no longer use it, leave it — an unused import is harmless and keeps the diff minimal. The `@dataclass` for `_GridMapView` is the only one being removed.)

- [ ] **Step 3: Delegate `_grid_view` to the factory**

Replace the body of `_grid_view` (lines ~87–100) with:

```python
    def _grid_view(self, msg) -> "GridMapView | None":
        return GridMapView.from_occupancy_grid(msg)
```

- [ ] **Step 4: Verify behavior unchanged**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -c "import ast; ast.parse(open('david_nav/navigation_goal_resolver.py').read()); print('parse ok')"`
Expected: `parse ok`

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/ -q 2>&1 | tail -8`
Expected: same pass/fail set as the Step 1 baseline (no new failures). The resolver still exposes `world_to_cell`, `is_goal_free`, `in_bounds`, `cell_center_world`, `resolution` via the shared class — its callers (`_is_goal_admissible`, `_ring_candidates`) are unchanged.

- [ ] **Step 5: Commit**

```bash
cd /home/agx/ros2_ws/src/david_nav
git add david_nav/navigation_goal_resolver.py
git commit -m "refactor(nav): use shared GridMapView in navigation_goal_resolver

Behavior unchanged; lifts the grid view into a reusable module.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Bresenham line-of-sight

**Files:**
- Create: `david_nav/waypoint_sparsifier.py`
- Test: `test/test_waypoint_sparsifier.py`

- [ ] **Step 1: Write the failing test**

Create `test/test_waypoint_sparsifier.py`:

```python
import math

from david_nav.grid_map_view import GridMapView
from david_nav.waypoint_sparsifier import (
    bresenham_cells,
    line_clear,
)


def _grid(cells, w, h, res=1.0, ox=0.0, oy=0.0):
    return GridMapView(cells=cells, width=w, height=h, resolution=res,
                       origin_x=ox, origin_y=oy)


def test_bresenham_straight_horizontal():
    assert bresenham_cells(0, 0, 3, 0) == [(0, 0), (1, 0), (2, 0), (3, 0)]


def test_bresenham_diagonal_endpoints_inclusive():
    cells = bresenham_cells(0, 0, 2, 2)
    assert cells[0] == (0, 0)
    assert cells[-1] == (2, 2)
    assert (1, 1) in cells


def test_bresenham_reversed_direction():
    assert bresenham_cells(3, 0, 0, 0) == [(3, 0), (2, 0), (1, 0), (0, 0)]


def test_line_clear_on_empty_grid():
    g = _grid([0] * 25, 5, 5)
    # centers of cells: world (x+0.5)
    assert line_clear(g, (0.5, 0.5), (4.5, 0.5), threshold=99,
                      block_unknown=True)


def test_line_clear_blocked_by_obstacle():
    cells = [0] * 25
    cells[2 * 5 + 2] = 100  # (x=2,y=2) lethal
    g = _grid(cells, 5, 5)
    # diagonal passes through (2,2) -> blocked
    assert not line_clear(g, (0.5, 0.5), (4.5, 4.5), threshold=99,
                          block_unknown=True)
    # a horizontal line on y=0 avoids it -> clear
    assert line_clear(g, (0.5, 0.5), (4.5, 0.5), threshold=99,
                      block_unknown=True)


def test_line_clear_out_of_bounds_is_blocked():
    g = _grid([0] * 25, 5, 5)
    assert not line_clear(g, (0.5, 0.5), (100.0, 0.5), threshold=99,
                          block_unknown=True)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/test_waypoint_sparsifier.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'david_nav.waypoint_sparsifier'`

- [ ] **Step 3: Write minimal implementation**

Create `david_nav/waypoint_sparsifier.py`:

```python
from __future__ import annotations

import math


def bresenham_cells(x0: int, y0: int, x1: int, y1: int) -> list:
    """Integer cells on the line from (x0,y0) to (x1,y1), endpoints inclusive."""
    cells = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return cells


def line_clear(grid, a_world, b_world, threshold: int,
               block_unknown: bool) -> bool:
    """True iff the straight segment a->b crosses only free cells.

    Both endpoints must map into the grid; any out-of-bounds or
    >=threshold (or unknown, when block_unknown) cell on the line blocks it.
    """
    ca = grid.world_to_cell(a_world[0], a_world[1])
    cb = grid.world_to_cell(b_world[0], b_world[1])
    if ca is None or cb is None:
        return False
    for (x, y) in bresenham_cells(ca[0], ca[1], cb[0], cb[1]):
        if not grid.in_bounds(x, y):
            return False
        if grid.is_blocked(x, y, threshold, block_unknown):
            return False
    return True
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/test_waypoint_sparsifier.py -q`
Expected: PASS (6 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/agx/ros2_ws/src/david_nav
git add david_nav/waypoint_sparsifier.py test/test_waypoint_sparsifier.py
git commit -m "feat(nav): Bresenham line-of-sight over the costmap

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Greedy farthest-reachable sparsify

**Files:**
- Modify: `david_nav/waypoint_sparsifier.py` (add `sparsify`)
- Modify: `test/test_waypoint_sparsifier.py` (add tests)

- [ ] **Step 1: Write the failing test**

Append to `test/test_waypoint_sparsifier.py`:

```python
from david_nav.waypoint_sparsifier import sparsify


def _path(xs):
    return [(float(x), 0.5) for x in xs]  # along y=0.5 (cell row 0 on res=1)


def test_sparsify_empty_and_single():
    g = _grid([0] * 25, 5, 5)
    assert sparsify([], g, max_segment=3.0, threshold=99,
                    block_unknown=True) == []
    one = [(0.5, 0.5)]
    assert sparsify(one, g, max_segment=3.0, threshold=99,
                    block_unknown=True) == one


def test_sparsify_enforces_max_segment():
    g = _grid([0] * 25, 5, 5)
    # dense points 0.5,1.5,2.5,3.5,4.5 along clear row; max_segment 3.0
    path = _path([0.5, 1.5, 2.5, 3.5, 4.5])
    wps = sparsify(path, g, max_segment=3.0, threshold=99, block_unknown=True)
    # every emitted segment (from start) must be <= 3.0
    anchor = path[0]
    for wp in wps:
        assert math.hypot(wp[0] - anchor[0], wp[1] - anchor[1]) <= 3.0 + 1e-9
        anchor = wp
    assert wps[-1] == path[-1]  # goal always included


def test_sparsify_splits_at_obstacle():
    # obstacle at cell (2,1): blocks straight line across the middle of a
    # 5-wide, 3-tall grid. Force a detour-style dense path that goes around.
    cells = [0] * 15  # 5x3
    cells[1 * 5 + 2] = 100  # (x=2,y=1) lethal
    g = _grid(cells, 5, 3)
    # dense path: (0.5,0.5)->(2.5,0.5)->(2.5,2.5)->(4.5,2.5) (an L around obstacle)
    path = [(0.5, 0.5), (1.5, 0.5), (2.5, 0.5),
            (2.5, 1.5), (2.5, 2.5),
            (3.5, 2.5), (4.5, 2.5)]
    wps = sparsify(path, g, max_segment=10.0, threshold=99, block_unknown=True)
    # Straight (0.5,0.5)->(4.5,2.5) would cross near the obstacle; the
    # sparsifier must NOT collapse the whole L into one segment.
    assert len(wps) >= 2
    assert wps[-1] == path[-1]
    # each emitted segment is line-clear from its anchor
    anchor = path[0]
    for wp in wps:
        assert line_clear(g, anchor, wp, threshold=99, block_unknown=True)
        anchor = wp


def test_sparsify_collapses_clear_straight_run():
    g = _grid([0] * 25, 5, 5)
    path = _path([0.5, 1.5, 2.5])  # all within 3.0 and clear
    wps = sparsify(path, g, max_segment=3.0, threshold=99, block_unknown=True)
    assert wps == [(2.5, 0.5)]  # one segment straight to the goal
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/test_waypoint_sparsifier.py -q`
Expected: FAIL — `ImportError: cannot import name 'sparsify'`

- [ ] **Step 3: Write minimal implementation**

Append to `david_nav/waypoint_sparsifier.py`:

```python
def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def sparsify(path_xy, grid, max_segment: float, threshold: int,
             block_unknown: bool) -> list:
    """Greedy farthest-reachable waypoint selection.

    path_xy: dense world points, path_xy[0] = start, path_xy[-1] = goal.
    Returns the waypoints to DRIVE TO (start excluded, goal always last).
    Each emitted segment anchor->waypoint is line_clear AND <= max_segment.

    Defensive fallback: a segment always advances by at least one dense point
    even if that adjacent point fails the checks (planner guarantees adjacent
    dense points are clear and far under max_segment, so this never fires in
    practice; it only guards against an infinite loop).
    """
    n = len(path_xy)
    if n == 0:
        return []
    if n == 1:
        return [path_xy[0]]

    waypoints = []
    anchor_idx = 0
    while anchor_idx < n - 1:
        anchor = path_xy[anchor_idx]
        best = anchor_idx + 1  # always advance at least one (fallback)
        j = anchor_idx + 1
        while j < n:
            cand = path_xy[j]
            if (_dist(anchor, cand) <= max_segment
                    and line_clear(grid, anchor, cand, threshold,
                                   block_unknown)):
                best = j
                j += 1
            else:
                break
        waypoints.append(path_xy[best])
        anchor_idx = best
    return waypoints
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/test_waypoint_sparsifier.py -q`
Expected: PASS (10 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/agx/ros2_ws/src/david_nav
git add david_nav/waypoint_sparsifier.py test/test_waypoint_sparsifier.py
git commit -m "feat(nav): greedy farthest-reachable waypoint sparsify

Each segment is line-clear and <= max_segment; goal always last.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Relative-target geometry helpers

**Files:**
- Modify: `david_nav/waypoint_sparsifier.py` (add `relative_target`, `relative_yaw`)
- Modify: `test/test_waypoint_sparsifier.py` (add tests)

- [ ] **Step 1: Write the failing test**

Append to `test/test_waypoint_sparsifier.py`:

```python
from david_nav.waypoint_sparsifier import relative_target, relative_yaw


def test_relative_target_pure_translation():
    # robot at origin facing +x (yaw=0): waypoint 2m ahead -> (2,0) in body
    rx, ry = relative_target(cur_x=0.0, cur_y=0.0, cur_yaw=0.0,
                             wp_x=2.0, wp_y=0.0)
    assert math.isclose(rx, 2.0, abs_tol=1e-9)
    assert math.isclose(ry, 0.0, abs_tol=1e-9)


def test_relative_target_with_rotation():
    # robot facing +y (yaw=pi/2): a waypoint at map (0,2) is straight ahead
    rx, ry = relative_target(cur_x=0.0, cur_y=0.0, cur_yaw=math.pi / 2,
                             wp_x=0.0, wp_y=2.0)
    assert math.isclose(rx, 2.0, abs_tol=1e-9)
    assert math.isclose(ry, 0.0, abs_tol=1e-9)
    # a waypoint at map (-1,0) (robot's left when facing +y is -x... ) :
    # facing +y, map -x is to the robot's FRONT-LEFT? compute: body x = ahead(+y),
    # body y = left(-x). map(-1,0): rel = R(-pi/2)*(-1,0) = (0*... ) -> ry should be +1
    rx2, ry2 = relative_target(cur_x=0.0, cur_y=0.0, cur_yaw=math.pi / 2,
                               wp_x=-1.0, wp_y=0.0)
    assert math.isclose(rx2, 0.0, abs_tol=1e-9)
    assert math.isclose(ry2, 1.0, abs_tol=1e-9)


def test_relative_yaw_wraps():
    assert math.isclose(relative_yaw(cur_yaw=0.0, goal_yaw=math.pi / 2),
                        math.pi / 2, abs_tol=1e-9)
    # wrap across pi: cur=3.0, goal=-3.0 -> shortest is +0.283... (2pi-6)
    r = relative_yaw(cur_yaw=3.0, goal_yaw=-3.0)
    assert -math.pi <= r <= math.pi
    assert math.isclose(r, (2 * math.pi - 6.0), abs_tol=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/test_waypoint_sparsifier.py -q`
Expected: FAIL — `ImportError: cannot import name 'relative_target'`

- [ ] **Step 3: Write minimal implementation**

Append to `david_nav/waypoint_sparsifier.py`:

```python
def _wrap_to_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def relative_target(cur_x: float, cur_y: float, cur_yaw: float,
                    wp_x: float, wp_y: float) -> tuple:
    """Map-frame waypoint -> robot-body-frame (x_ahead, y_left).

    Rotates the map-frame delta by -cur_yaw so the result is relative to the
    robot's current heading. Fed to MoveTo POSE with relative=true, which the
    position controller locks into odom at goal acceptance — immune to
    map<->odom drift because it is recomputed from the live map pose each
    segment.
    """
    dx = wp_x - cur_x
    dy = wp_y - cur_y
    c = math.cos(-cur_yaw)
    s = math.sin(-cur_yaw)
    rx = dx * c - dy * s
    ry = dx * s + dy * c
    return (rx, ry)


def relative_yaw(cur_yaw: float, goal_yaw: float) -> float:
    """Shortest signed rotation (rad) from cur_yaw to goal_yaw, in (-pi, pi]."""
    return _wrap_to_pi(goal_yaw - cur_yaw)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -m pytest test/test_waypoint_sparsifier.py -q`
Expected: PASS (13 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/agx/ros2_ws/src/david_nav
git add david_nav/waypoint_sparsifier.py test/test_waypoint_sparsifier.py
git commit -m "feat(nav): relative-target geometry helpers for MoveTo segments

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: `waypoint_follower` node

**Files:**
- Create: `david_nav/waypoint_follower.py`
- Create: `config/waypoint_follower.yaml`

This node has no pure unit test (it is ROS glue over already-tested logic); it is verified by `ast.parse` here and by the sim in Task 7. Keep ALL decision logic delegated to the tested `waypoint_sparsifier` functions.

- [ ] **Step 1: Write the node**

Create `david_nav/waypoint_follower.py`:

```python
#!/usr/bin/env python3
"""waypoint_follower — Nav2 plans, position_controller drives.

Asks Nav2 to PLAN ONLY (ComputePathToPose), sparsifies the global path into
waypoints where every consecutive pair is a collision-free straight line
<= max_segment (default 3 m), then drives each segment with the existing
position_controller via its MoveTo action. The local controller (DWB) is
bypassed; blind straight-line driving is safe because each segment was
verified clear on the global costmap.

Static-map assumption: segments are validated once at plan time. No runtime
obstacle monitoring or replanning.

Interfaces:
  Action ~/navigate : nav2_msgs/action/NavigateToPose (reuses the type; this is
                      NOT nav2's bt_navigator — point tools here explicitly)
  Topic  ~/goal_pose: geometry_msgs/PoseStamped (convenience, map frame)
"""

from __future__ import annotations

import math
import threading

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from david_nav_msgs.action import MoveTo

from david_nav.grid_map_view import GridMapView
from david_nav.waypoint_sparsifier import (
    relative_target,
    relative_yaw,
    sparsify,
)


def _yaw_from_quat(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class WaypointFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        self.declare_parameter("planner_action", "compute_path_to_pose")
        self.declare_parameter("move_action", "/position_controller/move")
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("max_segment", 3.0)
        self.declare_parameter("los_cost_threshold", 99)
        self.declare_parameter("block_unknown", True)
        self.declare_parameter("intermediate_heading", False)
        self.declare_parameter("planner_server_timeout", 5.0)
        self.declare_parameter("move_server_timeout", 5.0)
        self.declare_parameter("costmap_wait", 5.0)

        g = self.get_parameter
        self.planner_action = str(g("planner_action").value)
        self.move_action = str(g("move_action").value)
        self.costmap_topic = str(g("global_costmap_topic").value)
        self.map_frame = str(g("map_frame").value)
        self.robot_base_frame = str(g("robot_base_frame").value)
        self.planner_id = str(g("planner_id").value)
        self.max_segment = float(g("max_segment").value)
        self.los_threshold = int(g("los_cost_threshold").value)
        self.block_unknown = bool(g("block_unknown").value)
        self.intermediate_heading = bool(g("intermediate_heading").value)
        self.planner_timeout = float(g("planner_server_timeout").value)
        self.move_timeout = float(g("move_server_timeout").value)
        self.costmap_wait = float(g("costmap_wait").value)

        self._cb = ReentrantCallbackGroup()
        self._costmap = None
        self._costmap_lock = threading.Lock()
        self._drive_lock = threading.Lock()
        self._cancel_evt = threading.Event()
        self._active_move_handle = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, self.costmap_topic, self._on_costmap, 1,
            callback_group=self._cb)
        self.create_subscription(
            PoseStamped, "~/goal_pose", self._on_goal_pose, 10,
            callback_group=self._cb)

        self._planner = ActionClient(
            self, ComputePathToPose, self.planner_action,
            callback_group=self._cb)
        self._mover = ActionClient(
            self, MoveTo, self.move_action, callback_group=self._cb)

        self._server = ActionServer(
            self, NavigateToPose, "~/navigate",
            execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=self._on_cancel,
            callback_group=self._cb)

        self.get_logger().info(
            f"waypoint_follower up: plan via '{self.planner_action}', drive via "
            f"'{self.move_action}', max_segment={self.max_segment:.1f}m, "
            f"los_threshold={self.los_threshold}")

    # ---- costmap ----
    def _on_costmap(self, msg) -> None:
        with self._costmap_lock:
            self._costmap = msg

    def _grid(self):
        with self._costmap_lock:
            return GridMapView.from_occupancy_grid(self._costmap)

    # ---- tf ----
    def _map_pose(self):
        tf = self.tf_buffer.lookup_transform(
            self.map_frame, self.robot_base_frame, rclpy.time.Time())
        t = tf.transform.translation
        r = tf.transform.rotation
        return (float(t.x), float(t.y),
                _yaw_from_quat(r.x, r.y, r.z, r.w))

    # ---- planning ----
    def _wait_grid(self) -> "GridMapView | None":
        deadline = self.get_clock().now().nanoseconds + int(self.costmap_wait * 1e9)
        grid = self._grid()
        while grid is None and rclpy.ok():
            if self.get_clock().now().nanoseconds > deadline:
                return None
            self._cancel_evt.wait(0.1)
            grid = self._grid()
        return grid

    def _compute_path(self, goal_pose: PoseStamped):
        """Blocking call to ComputePathToPose. Returns list[(x,y)] or None."""
        if not self._planner.wait_for_server(timeout_sec=self.planner_timeout):
            self.get_logger().error("planner action server unavailable")
            return None
        req = ComputePathToPose.Goal()
        req.goal = goal_pose
        req.planner_id = self.planner_id
        req.use_start = False  # let the planner use the robot's current pose
        done = threading.Event()
        box = {}

        def _result_cb(fut):
            box["result"] = fut.result()
            done.set()

        def _goal_cb(fut):
            gh = fut.result()
            if not gh.accepted:
                box["result"] = None
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        self._planner.send_goal_async(req).add_done_callback(_goal_cb)
        done.wait()
        wrapped = box.get("result")
        if wrapped is None or wrapped.result is None:
            return None
        path = wrapped.result.path
        return [(p.pose.position.x, p.pose.position.y) for p in path.poses]

    # ---- driving one segment ----
    def _drive_segment(self, rx, ry, ryaw) -> bool:
        """Send one MoveTo POSE (relative). Returns True on SUCCEEDED."""
        if not self._mover.wait_for_server(timeout_sec=self.move_timeout):
            self.get_logger().error("position_controller move server unavailable")
            return False
        goal = MoveTo.Goal()
        goal.mode = MoveTo.Goal.MODE_POSE
        goal.target.x = float(rx)
        goal.target.y = float(ry)
        goal.yaw = float(ryaw)  # NaN => position-only (skip final in-place turn)
        goal.relative = True
        done = threading.Event()
        box = {}

        def _result_cb(fut):
            box["result"] = fut.result()
            done.set()

        def _goal_cb(fut):
            gh = fut.result()
            if not gh.accepted:
                box["result"] = None
                done.set()
                return
            self._active_move_handle = gh
            gh.get_result_async().add_done_callback(_result_cb)

        self._mover.send_goal_async(goal).add_done_callback(_goal_cb)
        done.wait()
        self._active_move_handle = None
        wrapped = box.get("result")
        if wrapped is None or wrapped.result is None:
            return False
        return bool(wrapped.result.success)

    # ---- the shared follow routine ----
    def _follow(self, goal_pose: PoseStamped, goal_handle=None) -> bool:
        if not self._drive_lock.acquire(blocking=False):
            self.get_logger().warn("waypoint_follower busy — goal rejected")
            if goal_handle is not None:
                goal_handle.abort()
            return False
        self._cancel_evt.clear()
        try:
            grid = self._wait_grid()
            if grid is None:
                self.get_logger().error("no global costmap — abort")
                if goal_handle is not None:
                    goal_handle.abort()
                return False

            goal_yaw = _yaw_from_quat(
                goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
                goal_pose.pose.orientation.z, goal_pose.pose.orientation.w)

            path_xy = self._compute_path(goal_pose)
            if not path_xy:
                self.get_logger().error("planner returned no path — abort")
                if goal_handle is not None:
                    goal_handle.abort()
                return False

            waypoints = sparsify(
                path_xy, grid, self.max_segment, self.los_threshold,
                self.block_unknown)
            n = len(waypoints)
            self.get_logger().info(
                f"path {len(path_xy)} pts -> {n} waypoints "
                f"(<= {self.max_segment:.1f}m, clear)")

            for i, wp in enumerate(waypoints):
                if self._cancel_evt.is_set() or (
                        goal_handle is not None
                        and goal_handle.is_cancel_requested):
                    self._cancel_active_move()
                    if goal_handle is not None:
                        goal_handle.canceled()
                    self.get_logger().info("follow canceled")
                    return False

                cx, cy, cyaw = self._map_pose()
                rx, ry = relative_target(cx, cy, cyaw, wp[0], wp[1])
                is_last = (i == n - 1)
                if is_last and not math.isnan(goal_yaw):
                    ryaw = relative_yaw(cyaw, goal_yaw)
                elif self.intermediate_heading:
                    # face toward the segment direction
                    ryaw = math.atan2(ry, rx)
                else:
                    ryaw = float("nan")

                self.get_logger().info(
                    f"  segment {i + 1}/{n} -> body({rx:+.2f},{ry:+.2f}) "
                    f"yaw={'nan' if math.isnan(ryaw) else f'{ryaw:+.2f}'}")
                if not self._drive_segment(rx, ry, ryaw):
                    self.get_logger().error(
                        f"segment {i + 1}/{n} failed — abort follow")
                    if goal_handle is not None:
                        goal_handle.abort()
                    return False

                if goal_handle is not None:
                    fb = NavigateToPose.Feedback()
                    fb.number_of_recoveries = 0
                    try:
                        fb.distance_remaining = float(
                            sum(math.hypot(
                                waypoints[k + 1][0] - waypoints[k][0],
                                waypoints[k + 1][1] - waypoints[k][1])
                                for k in range(i, n - 1)))
                    except Exception:
                        fb.distance_remaining = 0.0
                    goal_handle.publish_feedback(fb)

            if goal_handle is not None:
                goal_handle.succeed()
            self.get_logger().info("follow done: success")
            return True
        finally:
            self._drive_lock.release()

    def _cancel_active_move(self) -> None:
        gh = self._active_move_handle
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass

    # ---- action callbacks ----
    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self._cancel_evt.set()
        self._cancel_active_move()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        success = self._follow(goal_handle.request.pose, goal_handle)
        result = NavigateToPose.Result()
        return result

    # ---- topic convenience ----
    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self.get_logger().info(
            f"goal_pose -> follow target=({msg.pose.position.x:.2f},"
            f"{msg.pose.position.y:.2f}) frame={msg.header.frame_id or 'map'}")
        threading.Thread(target=self._follow, args=(msg, None),
                         daemon=True).start()


def main(argv=None):
    rclpy.init(args=argv)
    node = WaypointFollowerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify the node parses and imports resolve**

Run: `cd /home/agx/ros2_ws/src/david_nav && python3 -c "import ast; ast.parse(open('david_nav/waypoint_follower.py').read()); print('parse ok')"`
Expected: `parse ok`

> Note: a full `import david_nav.waypoint_follower` needs a sourced ROS env + built `david_nav_msgs`; that is exercised in Task 7. `ast.parse` is the build-free check here.

- [ ] **Step 3: Write the config**

Create `config/waypoint_follower.yaml`:

```yaml
waypoint_follower:
  ros__parameters:
    planner_action: "compute_path_to_pose"
    move_action: "/position_controller/move"
    global_costmap_topic: "/global_costmap/costmap"
    map_frame: "map"
    robot_base_frame: "base_footprint"
    planner_id: ""                 # "" => planner_server default (e.g. GridBased)
    max_segment: 3.0               # hard cap per segment (m)
    los_cost_threshold: 99         # OccupancyGrid 0..100 scale; 99 = inscribed
    block_unknown: true            # treat unknown (-1) cells as blocking
    intermediate_heading: false    # NaN heading on intermediate waypoints
    planner_server_timeout: 5.0
    move_server_timeout: 5.0
    costmap_wait: 5.0
```

- [ ] **Step 4: Commit**

```bash
cd /home/agx/ros2_ws/src/david_nav
git add david_nav/waypoint_follower.py config/waypoint_follower.yaml
git commit -m "feat(nav): waypoint_follower node (Nav2 plans, position ctrl drives)

ComputePathToPose -> sparsify (clear straight <=3m) -> per-segment MoveTo.
Reuses NavigateToPose action type; ~/goal_pose topic convenience; cancel
propagates to in-flight MoveTo. Static-map assumption.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: Install target + launch file + sim verification

**Files:**
- Modify: `CMakeLists.txt` (after the `cmd_vel_mux` install block, ~line 198)
- Create: `launch/waypoint_follower.launch.py`

- [ ] **Step 1: Add the install(PROGRAMS) block**

In `CMakeLists.txt`, immediately after the `install(PROGRAMS david_nav/cmd_vel_mux.py … RENAME cmd_vel_mux)` block (ends ~line 198), add:

```cmake
install(PROGRAMS
  david_nav/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_follower
)
```

- [ ] **Step 2: Write the launch file**

Create `launch/waypoint_follower.launch.py`:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("david_nav")
    wf_params = os.path.join(pkg, "config", "waypoint_follower.yaml")
    pc_params = os.path.join(pkg, "config", "position_controller.yaml")

    return LaunchDescription([
        Node(
            package="david_nav",
            executable="position_controller",
            name="position_controller",
            output="screen",
            parameters=[pc_params],
        ),
        Node(
            package="david_nav",
            executable="cmd_vel_mux",
            name="cmd_vel_mux",
            output="screen",
        ),
        Node(
            package="david_nav",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[wf_params],
        ),
    ])
```

- [ ] **Step 3: Build (run as the build/install owner)**

Run:
```bash
cd /home/agx/ros2_ws
colcon build --packages-select david_nav_msgs david_nav
source install/setup.bash
```
Expected: build succeeds; `ros2 run david_nav waypoint_follower --ros-args -p max_segment:=3.0` starts and logs `waypoint_follower up: …`. Ctrl-C to stop.

> If `colcon build` fails on ownership/permissions, see the workspace note at the top — run as the `build/install` owner. The algorithm is already fully covered by the Task 1–5 unit tests, which need no build.

- [ ] **Step 4: Sim smoke test (planner + drive end-to-end)**

This reuses `scripts/sim_fake_chassis.py` (integrates `/cmd_vel_position` -> `/odom`). A full Nav2 planner_server + costmap is heavy; the minimal end-to-end check is:

```bash
export ROS_DOMAIN_ID=77
# terminal 1: ideal chassis sim
python3 ~/ros2_ws/src/david_nav/scripts/sim_fake_chassis.py &
# terminal 2: bring up follower + position controller + mux
ros2 launch david_nav waypoint_follower.launch.py &
# terminal 3: confirm the node is wired up
ros2 node info /waypoint_follower            # shows ~/navigate, ~/goal_pose, costmap sub
ros2 action list | grep -E "navigate|move|compute_path"
```
Expected: `/waypoint_follower/navigate` (NavigateToPose) and `/position_controller/move` (MoveTo) present; the follower subscribes to `/global_costmap/costmap`.

> A complete drive test needs a real `planner_server` + `/global_costmap/costmap` + `map->base` TF. Run it inside the standard nav bringup (CLAUDE.md §4): start Nav2, set the initial pose in RViz, then
> `ros2 action send_goal /waypoint_follower/navigate nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: <gx>, y: <gy>}, orientation: {w: 1.0}}}}"`
> and watch the `segment k/n` logs and `/odom` converge. Verify in the logs that each emitted segment is ≤3m.

- [ ] **Step 5: Commit**

```bash
cd /home/agx/ros2_ws/src/david_nav
git add CMakeLists.txt launch/waypoint_follower.launch.py
git commit -m "feat(nav): install waypoint_follower + bringup launch

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: Docs

**Files:**
- Create: `docs/waypoint_follower_README.md`
- Modify: `README.md` (add a one-line pointer if it indexes nodes)

- [ ] **Step 1: Write the README**

Create `docs/waypoint_follower_README.md` covering: the concept (Nav2 plans, position ctrl drives), the two waypoint conditions (clear straight + ≤3m) and why they make blind driving safe, interfaces (`~/navigate` NavigateToPose, `~/goal_pose`), params (point at `config/waypoint_follower.yaml`), the static-map assumption + that DWB/separator are bypassed, and how to run (launch + action send_goal example from Task 7). Link back to the spec.

- [ ] **Step 2: Commit**

```bash
cd /home/agx/ros2_ws/src/david_nav
git add docs/waypoint_follower_README.md README.md
git commit -m "docs(nav): waypoint_follower README

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Verification checklist (whole feature)

- [ ] `python3 -m pytest test/test_grid_map_view.py test/test_waypoint_sparsifier.py -q` → all pass (no build needed).
- [ ] `python3 -m pytest test/ -q` → no regressions vs. the Task 2 baseline.
- [ ] `colcon build --packages-select david_nav_msgs david_nav` succeeds (build owner).
- [ ] `ros2 run david_nav waypoint_follower` starts and logs its config line.
- [ ] In nav bringup: a `NavigateToPose` goal to `/waypoint_follower/navigate` produces `n` waypoints, each segment ≤3m and clear, and `/odom` converges to the goal; cancel stops the robot.
```
