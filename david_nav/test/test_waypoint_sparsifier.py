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


from david_nav.waypoint_sparsifier import (
    insert_approach_waypoint,
    pose_window_stable,
    sparsify,
)


# ---------- 定位收敛判定(2026-06-10) ----------
def test_pose_stable_when_samples_tight():
    s = [(1.00, 2.00, 0.10), (1.01, 2.00, 0.11), (1.00, 2.01, 0.10)]
    assert pose_window_stable(s, eps_xy=0.03, eps_yaw=0.03)


def test_pose_unstable_when_position_creeps():
    s = [(1.00, 2.00, 0.10), (1.03, 2.00, 0.10), (1.06, 2.00, 0.10)]
    assert not pose_window_stable(s, eps_xy=0.03, eps_yaw=0.03)


def test_pose_unstable_when_yaw_creeps():
    s = [(1.0, 2.0, 0.10), (1.0, 2.0, 0.16), (1.0, 2.0, 0.22)]
    assert not pose_window_stable(s, eps_xy=0.03, eps_yaw=0.03)


def test_pose_stable_handles_pi_wrap():
    """±π 跳变不能误判为不稳定。"""
    s = [(0.0, 0.0, math.pi - 0.01), (0.0, 0.0, -math.pi + 0.01)]
    assert pose_window_stable(s, eps_xy=0.03, eps_yaw=0.05)


def test_pose_stable_needs_at_least_two_samples():
    assert not pose_window_stable([(0.0, 0.0, 0.0)], 0.03, 0.03)


def _path(xs):
    return [(float(x), 0.5) for x in xs]  # along y=0.5 (cell row 0 on res=1)


# ---------- 直行入位:goal 后方插进场点(2026-06-10) ----------
def test_approach_wp_inserted_along_goal_heading():
    """goal 朝 +x ⇒ 进场点在 goal 正后方 1m,最后一段方向 == 最终朝向。"""
    g = _grid([0] * 25, 5, 5)
    wps = [(2.5, 0.5), (4.5, 0.5)]
    out = insert_approach_waypoint((0.5, 0.5), wps, 0.0, g,
                                   approach_dist=1.0, max_segment=2.0,
                                   threshold=99, block_unknown=True)
    assert len(out) == 3
    assert out[-1] == (4.5, 0.5)                      # goal 不变
    ax, ay = out[-2]
    assert math.isclose(ax, 3.5) and math.isclose(ay, 0.5)  # A = goal 后方 1m
    assert out[0] == (2.5, 0.5)                        # 前面的 waypoint 不动


def test_approach_wp_single_segment_uses_start_as_prev():
    """只有一个 waypoint(robot→goal 直达)时,prev 用 start 校验。"""
    g = _grid([0] * 25, 5, 5)
    out = insert_approach_waypoint((0.5, 0.5), [(4.5, 0.5)], 0.0, g,
                                   approach_dist=1.0, max_segment=4.5,
                                   threshold=99, block_unknown=True)
    assert len(out) == 2
    assert math.isclose(out[0][0], 3.5)


def test_approach_wp_skipped_when_line_blocked():
    """A→goal 或 prev→A 被占据格挡住 ⇒ 原样返回(退化为旧行为)。"""
    cells = [0] * 25
    cells[0 * 5 + 3] = 100   # (x=3, y=0) 占据 —— 挡住 A(3.5) 与 goal(4.5) 之间
    g = _grid(cells, 5, 5)
    wps = [(2.5, 0.5), (4.5, 0.5)]
    out = insert_approach_waypoint((0.5, 0.5), wps, 0.0, g,
                                   approach_dist=1.0, max_segment=2.0,
                                   threshold=99, block_unknown=True)
    assert out == wps


def test_approach_wp_skipped_when_prev_to_a_exceeds_max_segment():
    """prev→A 超过 max_segment ⇒ 不插(盲走段不能超长)。"""
    g = _grid([0] * 100, 10, 10)
    wps = [(0.5, 0.5), (9.5, 0.5)]
    out = insert_approach_waypoint((0.5, 0.5), wps, 0.0, g,
                                   approach_dist=1.0, max_segment=2.0,
                                   threshold=99, block_unknown=True)
    assert out == wps   # prev(0.5)→A(8.5) = 8m > 2m


def test_approach_wp_off_when_dist_zero():
    g = _grid([0] * 25, 5, 5)
    wps = [(4.5, 0.5)]
    out = insert_approach_waypoint((0.5, 0.5), wps, 0.0, g,
                                   approach_dist=0.0, max_segment=2.0,
                                   threshold=99, block_unknown=True)
    assert out == wps


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
    # 5x5 grid with a vertical wall at x=2 for y=0..3, gap at (2,4). A straight
    # start->goal diagonal would cross the wall; the dense path detours up
    # through the gap and back down. Every dense point sits in a FREE cell
    # (as a real planner guarantees), so the defensive fallback never fires.
    cells = [0] * 25  # 5x5
    for y in range(4):  # (2,0),(2,1),(2,2),(2,3) lethal; (2,4) is the gap
        cells[y * 5 + 2] = 100
    g = _grid(cells, 5, 5)
    path = [
        (0.5, 0.5), (0.5, 1.5), (0.5, 2.5), (0.5, 3.5), (0.5, 4.5),  # up col x=0
        (1.5, 4.5), (2.5, 4.5), (3.5, 4.5),                          # through gap
        (4.5, 4.5),                                                  # goal
    ]
    wps = sparsify(path, g, max_segment=10.0, threshold=99, block_unknown=True)
    # Straight (0.5,0.5)->(4.5,4.5) crosses the wall, so it cannot collapse to
    # a single segment.
    assert len(wps) >= 2
    assert wps[-1] == path[-1]  # goal always included
    # each emitted segment is genuinely line-clear from its anchor
    anchor = path[0]
    for wp in wps:
        assert line_clear(g, anchor, wp, threshold=99, block_unknown=True)
        anchor = wp


def test_sparsify_collapses_clear_straight_run():
    g = _grid([0] * 25, 5, 5)
    path = _path([0.5, 1.5, 2.5])  # all within 3.0 and clear
    wps = sparsify(path, g, max_segment=3.0, threshold=99, block_unknown=True)
    assert wps == [(2.5, 0.5)]  # one segment straight to the goal


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
    # facing +y, a waypoint at map (-1,0) is to the robot's left (+y body)
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
