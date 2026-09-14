#!/usr/bin/env python3
"""dock_common —— dock_bench 的纯函数层：入位几何 + dock_navigator 判词解析 + 流畅性。

无 ROS、无文件 IO 依赖，可以直接单测。凡是"能离线算"的都放这里，
run_dock_bench.py / dock_metrics.py / plot_dock_run.py 只做 IO 和编排。

与 controller_bench/bench_common.py 的分工：
  bench_common  —— 通用：终点统计(bias/spread/CEP)、折线 cross-track、rms
  dock_common   —— 入位专属：理想入位直线的**有符号**横向偏差、沿线剩余距离、
                   阶段切分、dock_navigator 判词分类
入位段的基准是**一条无限长直线**(过 G、方向 yaw_G)，不是折线，所以不能复用
bench_common.cross_track_series —— 那个按线段算，会把阶段1 落点的容差算进阶段2。
"""

from __future__ import annotations

import math
import re

# runs_dock.csv 的字段表。放在这里而不是 run_dock_bench.py 里，是因为写方(runner,
# 要 rclpy)和读方(dock_metrics / 造数脚本, 不要 rclpy)都得认它 —— 放在带 ROS 依赖
# 的模块里会逼着离线工具去 import rclpy。
RUN_FIELDS = [
    'session', 'iteration', 'goal_id', 'timestamp',
    'goal_x', 'goal_y', 'goal_yaw',
    'prep_x', 'prep_y',
    'start_x', 'start_y', 'start_yaw', 'start_err',      # 本轮实际起点(协变量)
    'end_x', 'end_y', 'end_yaw',                          # 判词时刻的 TF 真值
    'exy', 'eyaw', 'cross_track_end',
    'verdict_class', 'verdict_detail',
    'stage1_status', 'stage2_status', 'n_spin',
    't_dock_sent', 't_stage1_start', 't_stage2_start', 't_verdict',
    'stage2_dur', 'total_dur',
    'node_exy', 'node_eyaw_deg',                          # 节点自评，用于交叉验证
    'battery_v', 'note',
]

# ---------------------------------------------------------------- 基础几何


def wrap(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of_quat(x: float, y: float, z: float, w: float) -> float:
    """与 workspace 其它节点一致的 atan2 形式。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def prep_point(gx: float, gy: float, gyaw: float, approach_dist: float = 0.5):
    """阶段1 预备点：目标沿自身航向后退 approach_dist。

    必须与 dock_navigator.execute_docking 里的公式**逐字一致**，否则
    bench 画的理想直线和车实际走的不是一回事。
    """
    return (gx - approach_dist * math.cos(gyaw),
            gy - approach_dist * math.sin(gyaw))


def signed_cross_track(px: float, py: float,
                       gx: float, gy: float, gyaw: float) -> float:
    """点 (px,py) 到「过 G、方向 gyaw」这条直线的**有符号**横向偏差。

    左正右负（以 gyaw 前进方向看）。取符号是为了让"总偏一侧"这种系统性误差
    在均值上直接现形 —— 只取绝对值的话左右各偏一半会互相抵消成"看起来很准"。
    """
    dx, dy = px - gx, py - gy
    # 直线方向 (cos,sin)，其左法向量 (-sin,cos)
    return -math.sin(gyaw) * dx + math.cos(gyaw) * dy


def along_track(px: float, py: float,
                gx: float, gy: float, gyaw: float) -> float:
    """点在直线上的投影坐标，以 G 为原点、gyaw 为正方向。

    入位过程中该值从 -approach_dist 走向 0，所以"剩余距离" = -along_track。
    """
    dx, dy = px - gx, py - gy
    return math.cos(gyaw) * dx + math.sin(gyaw) * dy


def remaining_dist(px: float, py: float,
                   gx: float, gy: float, gyaw: float) -> float:
    """沿理想直线到 G 的剩余距离（车在 G 后方时为正）。"""
    return -along_track(px, py, gx, gy, gyaw)


# ---------------------------------------------------- dock_navigator 判词解析

# dock_navigator 的日志前缀是稳定接口（见 david_nav/dock_navigator.py）。
# 这里只匹配前缀/关键片段，不解析数值，数值一律由 bench 自己从 TF 复算 ——
# 避免"用被测对象的自评当评测结果"。
_RE_GOAL_RECEIVED = re.compile(r'\[dock\] ===== 收到目标')
_RE_STAGE1_START = re.compile(r'\[dock\] 阶段1 预备点')
_RE_STAGE2_START = re.compile(r'\[dock\] 阶段2 入位 ->')
_RE_STAGE_DONE = re.compile(r'\[dock\] (阶段[12])\([^)]*\): (.+)')
_RE_SPIN = re.compile(r'\[dock\] 第(\d+)次纯旋转修正')
_RE_VERDICT = re.compile(r'\[dock\] ===== (成功|失败): (.*)')
_RE_MEASURED = re.compile(
    r'\[dock\] 入位实测 exy=([-\d.]+)m eyaw=([-\d.]+)°')

# 判词 -> 分类。顺序有意义：先匹配到的先算。
_VERDICT_RULES = (
    ('一次入位达标', 'SUCCESS'),
    ('旋转后达标', 'SUCCESS'),
    ('阶段1 未到预备点', 'STAGE1_FAILED'),
    ('xy 超差', 'XY_OUT_OF_TOL'),
    ('仍未收敛', 'YAW_NOT_CONVERGED'),
    ('入位后取不到位姿', 'POSE_LOOKUP_FAILED'),
    ('旋转后取不到位姿', 'POSE_LOOKUP_FAILED'),
    ('编排线程异常', 'NODE_EXCEPTION'),
    # 2026-09-10 dock_navigator 新增：等不到车停稳就不做补偿（nav2 的 Spin 对
    # 运动中的车会秒过，补偿只会把误差推大）。这是"没测成"，不是"没到位"。
    ('车未停稳', 'NOT_SETTLED'),
)


def classify_verdict(kind: str, detail: str) -> str:
    """把 dock_navigator 的终局判词映射成分类标签。

    kind = '成功' / '失败'；detail = 判词正文（可能带 "(实测 exy=... )" 后缀）。
    认不出来的失败一律 UNKNOWN_FAILURE —— 宁可显式暴露，也不要静默归到某一类里
    污染统计。
    """
    for needle, label in _VERDICT_RULES:
        if needle in detail:
            return label
    return 'SUCCESS' if kind == '成功' else 'UNKNOWN_FAILURE'


def parse_dock_timeline(rows):
    """从 (t_unix, msg) 序列里切出**一次**入位的时间线。

    rows 必须已经按时间升序、且已过滤成 dock_navigator 的消息。
    返回 dict；缺失的时间戳为 None（比如阶段1 就失败时没有 stage2_start）。
    """
    tl = {
        'goal_received': None,
        'stage1_start': None,
        'stage1_status': None,
        'stage2_start': None,
        'stage2_status': None,
        'spin_times': [],
        'verdict_t': None,
        'verdict_kind': None,
        'verdict_detail': None,
        'verdict_class': None,
        'node_exy': None,
        'node_eyaw_deg': None,
    }
    for t, msg in rows:
        if tl['goal_received'] is None and _RE_GOAL_RECEIVED.search(msg):
            tl['goal_received'] = t
            continue
        if tl['stage1_start'] is None and _RE_STAGE1_START.search(msg):
            tl['stage1_start'] = t
            continue
        if tl['stage2_start'] is None and _RE_STAGE2_START.search(msg):
            tl['stage2_start'] = t
            continue
        m = _RE_STAGE_DONE.search(msg)
        if m:
            key = 'stage1_status' if m.group(1) == '阶段1' else 'stage2_status'
            if tl[key] is None:
                tl[key] = m.group(2).strip()
            continue
        m = _RE_SPIN.search(msg)
        if m:
            tl['spin_times'].append(t)
            continue
        m = _RE_MEASURED.search(msg)
        if m:
            tl['node_exy'] = float(m.group(1))
            tl['node_eyaw_deg'] = float(m.group(2))
            continue
        m = _RE_VERDICT.search(msg)
        if m and tl['verdict_t'] is None:
            tl['verdict_t'] = t
            tl['verdict_kind'] = m.group(1)
            tl['verdict_detail'] = m.group(2).strip()
            tl['verdict_class'] = classify_verdict(m.group(1), m.group(2))
    tl['n_spin'] = len(tl['spin_times'])
    return tl


# ------------------------------------------------------------------ 流畅性


def stop_events(samples, v_thresh=0.02, min_stop=0.25):
    """停顿事件：|cmd_v| < v_thresh 连续持续 ≥ min_stop 秒的区段。

    samples = [(t, v), ...] 按时间升序。返回 [(t_start, t_end, duration), ...]。
    口径与 controller_bench/analyze_smoothness.py 一致，保证两套 bench 的数可比。
    """
    out = []
    run_start = None
    prev_t = None
    for t, v in samples:
        slow = abs(v) < v_thresh if v == v else False  # NaN -> False
        if slow and run_start is None:
            run_start = t
        elif not slow and run_start is not None:
            dur = prev_t - run_start
            if dur >= min_stop:
                out.append((run_start, prev_t, dur))
            run_start = None
        prev_t = t
    if run_start is not None and prev_t is not None:
        dur = prev_t - run_start
        if dur >= min_stop:
            out.append((run_start, prev_t, dur))
    return out


def sign_reversals(values, dead=1e-3):
    """符号反转次数。|v| <= dead 视作 0，不参与符号判定（避免零附近抖动刷数）。"""
    last = 0
    n = 0
    for v in values:
        if v != v or abs(v) <= dead:
            continue
        s = 1 if v > 0 else -1
        if last != 0 and s != last:
            n += 1
        last = s
    return n


def jitter_rms(values):
    """相邻差的 RMS —— 抖动强度。序列 < 2 个点返回 nan。"""
    vals = [v for v in values if v == v]
    if len(vals) < 2:
        return float('nan')
    diffs = [vals[i + 1] - vals[i] for i in range(len(vals) - 1)]
    return math.sqrt(sum(d * d for d in diffs) / len(diffs))


def yaw_convergence_bins(samples, gyaw, edges=(0.5, 0.4, 0.3, 0.2, 0.1, 0.0)):
    """航向收敛曲线：按"沿线剩余距离"分箱，报每箱 |eyaw| 中位数(度)。

    samples = [(remaining_dist, yaw), ...]。edges 从大到小，箱 i = [edges[i+1], edges[i])。
    看朝向是一路收敛过来的，还是全靠末端 Spin 硬掰。
    """
    bins = []
    for i in range(len(edges) - 1):
        hi, lo = edges[i], edges[i + 1]
        vals = [abs(math.degrees(wrap(y - gyaw)))
                for s, y in samples if lo <= s < hi]
        vals.sort()
        med = float('nan')
        if vals:
            n = len(vals)
            med = vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2])
        bins.append((hi, lo, len(vals), med))
    return bins


def median(vals):
    v = sorted(x for x in vals if x == x)
    if not v:
        return float('nan')
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])
