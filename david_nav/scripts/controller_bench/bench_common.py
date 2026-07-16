#!/usr/bin/env python3
"""controller_bench 共享纯函数 —— 无 ROS 依赖,可单测。

放这里的东西:参考路径加载、body-frame 变换(与 waypoint_sparsifier 一致)、
跟踪误差(cross-track)、终点分布统计(CEP / bias-spread)、循环量 yaw 统计。

真值约定:所有"实际位姿"来自 slam_toolbox 的 map->base TF(见 record_track.py),
统一当绝对真值,忽略定位漂移(用户决定)。位置含 yaw。
"""

from __future__ import annotations

import math
from statistics import fmean, pstdev
from typing import Iterable, Sequence


# --------------------------------------------------------------------------
# 角度 / body-frame 变换(inlined from waypoint_sparsifier,保持 bench 自包含)
# --------------------------------------------------------------------------

def wrap_to_pi(a: float) -> float:
    """(-pi, pi]。"""
    return math.atan2(math.sin(a), math.cos(a))


def relative_target(cur_x: float, cur_y: float, cur_yaw: float,
                    wp_x: float, wp_y: float) -> tuple[float, float]:
    """map 系 waypoint -> 机体系 (x_ahead, y_left)。绕 -cur_yaw 旋转 map 系增量。

    喂给 MoveTo POSE(relative=True),position_controller 在 goal 受理时锁进
    odom,因此对 map<->odom 漂移免疫(每段用实时 map 位姿重算)。
    """
    dx = wp_x - cur_x
    dy = wp_y - cur_y
    c = math.cos(-cur_yaw)
    s = math.sin(-cur_yaw)
    return (dx * c - dy * s, dx * s + dy * c)


def relative_yaw(cur_yaw: float, goal_yaw: float) -> float:
    """cur_yaw -> goal_yaw 的最短带符号旋转(rad)。"""
    return wrap_to_pi(goal_yaw - cur_yaw)


# --------------------------------------------------------------------------
# 折线 -> 拐角圆弧化 dense path(DWB FollowPath 用)
# --------------------------------------------------------------------------

def densify_with_fillets(polyline, spacing: float, corner_radius: float,
                         goal_yaw: float, min_seg_frac: float = 0.50,
                         arc_spacing: float = 0.05) -> list:
    """折线 -> 拐角圆弧化的 dense (x, y, yaw) 序列。

    为什么(2026-07-08 iter3):尖角折线要求路径切向瞬间跳变,差速底盘不可能连续
    跟踪 —— DWB 只能停下原地转(停-转-走)或冲出绕环。每个内部顶点用切圆弧替代
    尖角后,"不停车过弯"成为可行解(r=0.3, w_max=0.5 → 弧上 v=r*w≈0.15 m/s,
    正落在 vx 采样档上)。

    - corner_radius <= 0 时完全退回尖角行为(A/B 对照用)。
    - 每个拐角的切点距按相邻段长 clamp(min_seg_frac),短段自动缩小半径,
      相邻拐角不重叠;l_in 用"上一个圆弧出口(cursor)到本顶点"的剩余长度。
      iter4(2026-07-14):min_seg_frac 0.45→0.50 —— 让 0.6m 末段的 C3 弧从
      0.27 抬到 0.30,与 C1/C2 一致;配合 max_vel_x 0.15,w=v/r=0.5=w_max,
      每个弧在巡航速度下恰好可行(此前 C3 弧不可行 → 过头螺旋绕回)。
    - 近直线(<3°)/近折返(>160°)顶点不加弧(折返角 tan(θ/2)→∞,加弧无意义)。
    - 直线段按 spacing 采样,圆弧按 min(arc_spacing, spacing) 采样(朝向连续);
      每点 yaw = 该处切向;末点强制 (终点, goal_yaw)。
    纯函数,无 ROS 依赖,可离线单测。
    """
    # 去重(合并 <1e-9 的重复点,如"纯原地转"退化段)
    pts = []
    for p in polyline:
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-9:
            pts.append((float(p[0]), float(p[1])))
    if len(pts) < 2:
        raise ValueError('densify_with_fillets: polyline 需要 >=2 个不同点')

    out: list = []

    def emit(x, y, yaw):
        if out and math.hypot(x - out[-1][0], y - out[-1][1]) < 1e-6:
            out[-1] = (out[-1][0], out[-1][1], yaw)  # 同点只更新朝向
            return
        out.append((x, y, yaw))

    def emit_straight(a, b, yaw, include_end=False):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < 1e-9:
            return
        n = max(1, int(math.ceil(d / spacing)))
        last = n if include_end else n - 1
        for k in range(0, last + 1):
            t = k / n
            emit(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), yaw)

    cursor = pts[0]
    for i in range(1, len(pts) - 1):
        p_i, p_next = pts[i], pts[i + 1]
        l_in = math.hypot(p_i[0] - cursor[0], p_i[1] - cursor[1])
        l_out = math.hypot(p_next[0] - p_i[0], p_next[1] - p_i[1])
        ux, uy = p_i[0] - pts[i - 1][0], p_i[1] - pts[i - 1][1]
        nu = math.hypot(ux, uy)
        ux, uy = ux / nu, uy / nu
        vx, vy = p_next[0] - p_i[0], p_next[1] - p_i[1]
        nv = math.hypot(vx, vy)
        vx, vy = vx / nv, vy / nv
        cross = ux * vy - uy * vx
        dot = ux * vx + uy * vy
        theta = math.atan2(abs(cross), dot)          # 转角 [0, pi]
        yaw_in = math.atan2(uy, ux)

        if (corner_radius <= 1e-6 or theta < math.radians(3.0)
                or theta > math.radians(160.0) or l_in < 1e-9):
            emit_straight(cursor, p_i, yaw_in)
            cursor = p_i
            continue

        tan_half = math.tan(theta / 2.0)
        t_max = min_seg_frac * min(l_in, l_out)
        r_eff = min(corner_radius, t_max / tan_half)
        t_eff = r_eff * tan_half
        A = (p_i[0] - ux * t_eff, p_i[1] - uy * t_eff)
        B = (p_i[0] + vx * t_eff, p_i[1] + vy * t_eff)
        side = 1.0 if cross > 0 else -1.0
        C = (A[0] + side * (-uy) * r_eff, A[1] + side * ux * r_eff)

        emit_straight(cursor, A, yaw_in)             # 进弧前直线
        arc_len = r_eff * theta
        m = max(2, int(math.ceil(arc_len / max(1e-3, min(arc_spacing, spacing)))))
        ang0 = math.atan2(A[1] - C[1], A[0] - C[0])
        for k in range(0, m + 1):
            f = k / m
            ang = ang0 + side * theta * f
            emit(C[0] + r_eff * math.cos(ang), C[1] + r_eff * math.sin(ang),
                 wrap_to_pi(yaw_in + side * theta * f))
        cursor = B

    last_yaw = math.atan2(pts[-1][1] - cursor[1], pts[-1][0] - cursor[0]) \
        if math.hypot(pts[-1][0] - cursor[0], pts[-1][1] - cursor[1]) > 1e-9 \
        else goal_yaw
    emit_straight(cursor, pts[-1], last_yaw, include_end=True)
    if not out:
        out.append((pts[-1][0], pts[-1][1], goal_yaw))
    out[-1] = (pts[-1][0], pts[-1][1], float(goal_yaw))  # 末点=终点+goal朝向
    return out


# --------------------------------------------------------------------------
# 跟踪误差:实际轨迹点到参考折线的最近垂距(cross-track)
# --------------------------------------------------------------------------

def _point_to_segment_dist(px: float, py: float,
                           ax: float, ay: float,
                           bx: float, by: float) -> float:
    """点 (px,py) 到线段 AB 的欧氏距离。AB 退化为点时退化为点距。"""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return math.hypot(apx, apy)
    t = (apx * abx + apy * aby) / denom
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * abx, ay + t * aby
    return math.hypot(px - cx, py - cy)


def cross_track_series(actual_xy: Sequence[tuple[float, float]],
                       ref_polyline: Sequence[tuple[float, float]]) -> list[float]:
    """每个实际轨迹点到参考折线(取所有段的最小垂距)的横向误差序列。"""
    if len(ref_polyline) < 2:
        return []
    out = []
    for (px, py) in actual_xy:
        d = min(
            _point_to_segment_dist(px, py, ref_polyline[i][0], ref_polyline[i][1],
                                   ref_polyline[i + 1][0], ref_polyline[i + 1][1])
            for i in range(len(ref_polyline) - 1)
        )
        out.append(d)
    return out


def rms(xs: Iterable[float]) -> float:
    xs = list(xs)
    if not xs:
        return float('nan')
    return math.sqrt(fmean(x * x for x in xs))


# --------------------------------------------------------------------------
# 终点分布:稳定性(围绕质心的散布) vs 精确性(相对 goal 的误差)
# bias(系统性偏移) 和 spread(随机散布) 必须分开报(见设计文档)。
# --------------------------------------------------------------------------

def circular_mean(yaws: Sequence[float]) -> float:
    """yaw 的循环均值(rad)。"""
    if not yaws:
        return float('nan')
    s = fmean(math.sin(a) for a in yaws)
    c = fmean(math.cos(a) for a in yaws)
    return math.atan2(s, c)


def circular_std(yaws: Sequence[float]) -> float:
    """围绕循环均值的标准差(rad),用 wrap 差。少于 2 个样本 -> nan。"""
    if len(yaws) < 2:
        return float('nan')
    m = circular_mean(yaws)
    return pstdev([wrap_to_pi(a - m) for a in yaws])


def percentile(sorted_vals: Sequence[float], q: float) -> float:
    """线性插值分位数;q in [0,1]。输入需已排序。"""
    if not sorted_vals:
        return float('nan')
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = q * (len(sorted_vals) - 1)
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return sorted_vals[lo]
    frac = idx - lo
    return sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac


def endpoint_stats(ends: Sequence[tuple[float, float, float]],
                   goal: tuple[float, float, float]) -> dict:
    """ends: 成功跑的终点 (x,y,yaw) 列表(map/slam 真值系)。goal: (x,y,yaw)。

    返回:
      n
      # 稳定性(围绕样本质心的重复性)
      centroid_x/y, std_x, std_y, yaw_std,
      cep50, cep95            # 围绕质心的径向分位
      # 精确性(相对 goal)
      bias_x, bias_y, bias_pos, bias_yaw   # 质心 - goal(系统性)
      err_pos_mean, err_pos_max            # 每次终点到 goal 的径向误差
      err_yaw_mean, err_yaw_max            # |wrap(yaw - goal_yaw)|
    """
    n = len(ends)
    if n == 0:
        return {'n': 0}
    xs = [e[0] for e in ends]
    ys = [e[1] for e in ends]
    yaws = [e[2] for e in ends]
    gx, gy, gyaw = goal

    cx, cy = fmean(xs), fmean(ys)
    cyaw = circular_mean(yaws)

    # 稳定性:围绕质心
    radial_c = sorted(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys))
    # 精确性:相对 goal
    err_pos = [math.hypot(x - gx, y - gy) for x, y in zip(xs, ys)]
    err_yaw = [abs(wrap_to_pi(a - gyaw)) for a in yaws]

    return {
        'n': n,
        'centroid_x': cx, 'centroid_y': cy, 'centroid_yaw': cyaw,
        'std_x': pstdev(xs) if n > 1 else 0.0,
        'std_y': pstdev(ys) if n > 1 else 0.0,
        'yaw_std': circular_std(yaws),
        'cep50': percentile(radial_c, 0.50),
        'cep95': percentile(radial_c, 0.95),
        'bias_x': cx - gx, 'bias_y': cy - gy,
        'bias_pos': math.hypot(cx - gx, cy - gy),
        'bias_yaw': wrap_to_pi(cyaw - gyaw),
        'err_pos_mean': fmean(err_pos), 'err_pos_max': max(err_pos),
        'err_yaw_mean': fmean(err_yaw), 'err_yaw_max': max(err_yaw),
    }


# --------------------------------------------------------------------------
# 参考路径加载
# --------------------------------------------------------------------------

def load_reference(path: str) -> dict:
    """读 reference_path.yaml -> dict。含 start / waypoints / tolerances。

    附带算好 polyline = [start] + [每个 waypoint 的 (x,y)],供 cross-track 用。
    """
    import yaml  # 延迟导入:离线指标脚本才不强依赖 ROS 环境的 import 时机
    with open(path, 'r') as f:
        ref = yaml.safe_load(f)
    s = ref['start']
    poly = [(float(s['x']), float(s['y']))]
    for w in ref['waypoints']:
        poly.append((float(w['x']), float(w['y'])))
    ref['polyline'] = poly
    ref['goal'] = ref['waypoints'][-1]
    return ref
