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


def insert_approach_waypoint(start_xy, waypoints, goal_yaw: float, grid,
                             approach_dist: float, max_segment: float,
                             threshold: int, block_unknown: bool) -> list:
    """直行入位(2026-06-10):在 goal 正后方(沿最终朝向反向)approach_dist 处
    插一个进场点 A,使最后一段 A→goal 的行驶方向 == 最终朝向。这样末端不需要
    原地大转(大终转会蹭点 + 转角期间 odom 角漂把 map 朝向带歪,2026-06-10 实
    车日志里 0.46~0.88 rad 终转后 map 朝向偏 20°~44°)。

    安全条件(任一不满足就原样返回,退化为旧行为):
      * prev→A 与 A→goal 都 line_clear(prev = goal 前一个 waypoint,只有一段
        时用 start_xy)
      * prev→A ≤ max_segment(盲走段不超长)
    """
    if approach_dist <= 0.0 or not waypoints:
        return waypoints
    gx, gy = waypoints[-1]
    a = (gx - approach_dist * math.cos(goal_yaw),
         gy - approach_dist * math.sin(goal_yaw))
    prev = waypoints[-2] if len(waypoints) >= 2 else start_xy
    if _dist(prev, a) > max_segment:
        return waypoints
    if not line_clear(grid, prev, a, threshold, block_unknown):
        return waypoints
    if not line_clear(grid, a, (gx, gy), threshold, block_unknown):
        return waypoints
    return list(waypoints[:-1]) + [a, (gx, gy)]


def plan_waypoints(path_xy, grid, max_segment: float, threshold: int,
                   block_unknown: bool, goal_yaw: float,
                   approach_dist: float) -> tuple:
    """入场点锚定的 waypoint 生成(2026-06-22)。返回 (waypoints, approach_anchored)。

    旧法:从机器人前向 sparsify,短余量段落在 goal 一侧,A 由 insert_approach_waypoint
    事后塞入(prev→A 长度随机)。新法:以"终点前入场点 A"(goal 沿最终朝向反向
    approach_dist 处)为锚,沿规划路径向机器人方向按 max_segment 等距铺,短余量段
    自然落在机器人一侧(起步段);A、goal 都是干净 waypoint,A→goal 为直行入位段。

    退化(approach_anchored=False):approach_dist<=0 / goal_yaw=NaN / A→goal 不可达
    → 回到普通前向 sparsify(后者再尝试旧式 insert_approach_waypoint)。
    """
    if not path_xy:
        return [], False
    if (approach_dist <= 0.0 or goal_yaw is None or math.isnan(goal_yaw)):
        return sparsify(path_xy, grid, max_segment, threshold, block_unknown), False

    start = path_xy[0]
    gx, gy = path_xy[-1]
    a = (gx - approach_dist * math.cos(goal_yaw),
         gy - approach_dist * math.sin(goal_yaw))

    # A→goal 直行入位段不可达 → 退化为旧行为(普通 sparsify + 事后插 A)
    if not line_clear(grid, a, (gx, gy), threshold, block_unknown):
        wps = sparsify(path_xy, grid, max_segment, threshold, block_unknown)
        out = insert_approach_waypoint(start, wps, goal_yaw, grid, approach_dist,
                                       max_segment, threshold, block_unknown)
        return out, (len(out) > len(wps))

    # robot→A 子路径:保留离 goal 远于 approach_dist 的 path 点,末端换成 A
    sub = [p for p in path_xy if _dist(p, (gx, gy)) > approach_dist] or [start]
    if _dist(sub[-1], a) > 1e-9:
        sub.append(a)

    # 从 A 反向贪心铺(短余量段落在机器人一侧),再翻回正向;去掉当前位姿 start
    rev_wps = sparsify(list(reversed(sub)), grid, max_segment, threshold,
                       block_unknown)            # 含末项 start, 不含锚点 A
    fwd = list(reversed(rev_wps))                # [start, ..., 近 A 点]
    if fwd and _dist(fwd[0], start) <= 1e-9:
        fwd = fwd[1:]
    return fwd + [a, (gx, gy)], True


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


def pose_window_stable(samples, eps_xy: float, eps_yaw: float) -> bool:
    """一窗口 (x,y,yaw) 采样是否已收敛(定位静稳判据,2026-06-10)。

    位置:窗口内 x/y 跨度的合成 ≤ eps_xy;朝向:相对首样本的 wrap 差全部
    ≤ eps_yaw(±π 跳变不会误判)。少于 2 个样本视为未收敛。
    背景:Lightning 重定位在运动中/刚停时位姿是脏的,静止后才收敛
    (实车停车间隙 map 朝向自行修正过 42°);读位姿前先等这个判据为真。
    """
    if len(samples) < 2:
        return False
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) > eps_xy:
        return False
    ref = samples[0][2]
    return all(abs(_wrap_to_pi(s[2] - ref)) <= eps_yaw for s in samples)
