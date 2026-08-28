#!/usr/bin/env python3
"""滚动态 wz / v 执行增益标定（外部真值阶梯版：slam_gt / lightning）。

背景（docs/reports/2026-07-17-dwb-straight-line-limit-cycle-analysis.md §5.1）：
DWB 直线正弦振荡的第一根因是底盘执行增益未标定 —— 原地 wz 实测 ~2.5×、
v ~1.2×，而**滚动态 wz 增益从未量过**。本脚本直接向 /cmd_vel 发阶跃
（不经 david_move/separator = 纯 plant），用两路观测同时记录：

  * 外部真值（slam_gt 或 lightning，见下方阶梯）      ← 增益的分子
  * /odom (Odometry, wheeltec 轮速计)                ← 底盘自报，另列对比

真值 = lightning（2026-07-17 定稿：slam_gt 不稳退役；loc/odometry 实测偶发
单帧 6.9m/π 重定位跳变 → 降为末位兜底）。阶梯自动选择、运行中打印所用源：
  1. /lightning/odometry      激光里程计 —— 主源。连续积分无重定位跳变；
                              odom 系，秒级测量窗内漂移可忽略，home 点微漂
  2. TF odom→base_*           兜底（lightning pub_tf）
  3. /lightning/loc/odometry  末位兜底（分析层有跳变缝合）
  0. --truth-topic 显式传入时（如 slam_gt /groundtruth/pose）才参与，最高优先
注：wz 增益对刚体上任意坐标系不变；v 增益只用纯直行段 → 阶梯各源等价可用。
分析层稳健化：跳变缝合（单帧 >0.12m/0.35rad 视为跳变不计入轨迹）+
Theil-Sen 斜率（抗 ≲29% 离群），端点法已废弃。

三组测试：
  v       直行阶跃  vx ∈ {0.15, 0.20}，交替正/反向防漂出场地
  wzstat  原地旋转  wz ∈ {±0.25, ±0.50, ±0.75}（复验 2.5×）
  wzroll  滚动旋转  先直行 1.5s，再叠加 wz ∈ {±0.10, ±0.25, ±0.50}（关键缺口）

每段输出：gain_gt = 真值响应/cmd、gain_odom = 轮速计/cmd、纯滞后估计、死区标记。
汇总给出 per-channel 中位增益 + 建议修正系数（落 wheeltec 固件或 twist_transform）。

安全（70kg 重载差速底盘，人必须守急停）：
  * 以启动时 GT 位姿为中心的 ±box/2 安全框，越框 → 立即零速 + 段中止；
  * GT 新鲜度 > 0.6s（slam 掉了）→ 立即零速 + 段中止；
  * 段间基于 GT 闭环自动回位（转向→回中→回航向），也受安全框约束；
  * Ctrl-C 任何时刻 → 5 连发零速退出（已录数据仍会分析）。

前置（二选一；不要起 nav2/键盘遥控，它们会抢 /cmd_vel）：
  slam_gt 流程 : alias 2 (lidar) → 5 (wheeltec) → 6 (slam_gt)
  lightning 流程: alias 2 (lidar) → 5 (wheeltec) → 3 (lightning) → 手动给初始位姿
推荐入口：scripts/calibrate_plant_gains.sh（含 preflight），即 alias 9。

干跑（不发 cmd，只验证话题/新鲜度/打印计划）：
  python3 scripts/calibrate_plant_gains.py --dry-run
方向确认（低速短脉冲验证 vx/wz 符号）：
  python3 scripts/calibrate_plant_gains.py --sign-check
实跑 / 只跑某组 / 重分析已有 CSV：
  python3 scripts/calibrate_plant_gains.py
  python3 scripts/calibrate_plant_gains.py --tests wzroll --repeats 5
  python3 scripts/calibrate_plant_gains.py --analyze /tmp/david_nav_logs/calibrate_plant_gains_XXX.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import threading
import time

# --------------------------------------------------------------------------- #
# logging tee（沿用 chassis_characterize 约定：时间戳日志 + _latest 软链）
# --------------------------------------------------------------------------- #
class _Tee:
    def __init__(self, stream, logfile):
        self.stream = stream
        self.logfile = logfile

    def write(self, data):
        self.stream.write(data)
        self.logfile.write(data)
        self.logfile.flush()

    def flush(self):
        self.stream.flush()
        self.logfile.flush()


def setup_logging(log_dir: str):
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, f"calibrate_plant_gains_{stamp}.log")
    fh = open(log_path, "w", buffering=1)
    fh.write(f"# calibrate_plant_gains {stamp}\n# argv: {' '.join(sys.argv)}\n")
    sys.stdout = _Tee(sys.__stdout__, fh)
    sys.stderr = _Tee(sys.__stderr__, fh)
    latest = "/tmp/calibrate_plant_gains_latest.log"
    try:
        if os.path.islink(latest) or os.path.exists(latest):
            os.remove(latest)
        os.symlink(os.path.abspath(log_path), latest)
    except OSError:
        pass
    return log_path, stamp


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def ang_norm(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# --------------------------------------------------------------------------- #
# 分析（纯函数：吃 CSV 行，不依赖 ROS —— --analyze 模式复用）
# --------------------------------------------------------------------------- #
CSV_FIELDS = [
    "t", "seg", "group", "phase", "cmd_vx", "cmd_wz",
    "gt_t", "gt_x", "gt_y", "gt_yaw", "gt_age",
    "odom_vx", "odom_wz", "odom_x", "odom_y", "odom_yaw",
]

NO_RESP_W = 0.02   # rad/s 以下视为无响应（死区）
NO_RESP_V = 0.02   # m/s


def gt_series(rows, stitch=True):
    """去重 GT 序列，yaw 增量解缠；stitch=True 时缝合重定位跳变。

    实测（2026-07-17 052343 run）：/lightning/loc/odometry 正常帧间 Δpos
    中位 7mm，但偶发单帧 6.9m / π rad 的重定位跳变 —— 一次跳变就把
    端点法速率打成 3.1 m/s。缝合 = 跳变步的位移/角度不计入轨迹。

    返回 ([(gt_t, row_t, x, y_cont, yaw_cont)], jump_count)：
    速率计算用 gt_t（ROS 钟）；延迟计算用 row_t（与 cmd 同 monotonic 钟）。
    """
    out = []
    jumps = 0
    last_t = None
    prev = None                    # (gt_t, x_raw, y_raw, yaw_raw)
    off_x = off_y = 0.0
    yaw_acc = 0.0
    for r in rows:
        gt = r["gt_t"]
        if gt is None or gt == last_t:
            continue
        last_t = gt
        x, y, yaw = r["gt_x"], r["gt_y"], r["gt_yaw"]
        if prev is None:
            yaw_acc = yaw
        else:
            dt = max(gt - prev[0], 1e-3)
            scale = min(max(dt / 0.05, 1.0), 4.0)   # 丢帧时按 dt 放宽阈值
            dx, dy = x - prev[1], y - prev[2]
            dyaw = ang_norm(yaw - prev[3])
            # 0.12m/50ms=2.4m/s、0.35rad/50ms=7rad/s，均远超物理上限 → 只可能是跳变
            if stitch and (math.hypot(dx, dy) > 0.12 * scale
                           or abs(dyaw) > 0.35 * scale):
                jumps += 1
                off_x -= dx
                off_y -= dy
                # yaw_acc 不累加跳变步
            else:
                yaw_acc += dyaw
        prev = (gt, x, y, yaw)
        out.append((gt, r["t"], x + off_x, y + off_y, yaw_acc))
    return out, jumps


def theil_sen(pairs):
    """成对斜率的中位数（Theil–Sen，对 ≲29% 离群点稳健）。pairs=[(t, val)]。"""
    n = len(pairs)
    if n < 5:
        return None
    slopes = []
    for i in range(n):
        ti, vi = pairs[i]
        for j in range(i + 1, n):
            dt = pairs[j][0] - ti
            if dt > 1e-3:
                slopes.append((pairs[j][1] - vi) / dt)
    return statistics.median(slopes) if slopes else None


def analyze_segment(step_rows, settle, hold):
    """一段 step 相位的行 → 指标 dict（gain_gt/gain_odom/delay/flags）。"""
    if len(step_rows) < 6:
        return None
    cmd_vx = step_rows[0]["cmd_vx"]
    cmd_wz = step_rows[0]["cmd_wz"]
    t0 = step_rows[0]["t"]
    t1 = step_rows[-1]["t"]
    win_start = t0 + max(settle, 0.4 * hold)
    win = [r for r in step_rows if r["t"] >= win_start]
    if len(win) < 4:
        win = step_rows[len(step_rows) // 2:]

    res = {"cmd_vx": cmd_vx, "cmd_wz": cmd_wz, "t0": t0, "dur": t1 - t0, "flags": []}

    # -- GT 通道：跳变缝合 + Theil-Sen 稳健斜率（抗 loc 重定位跳/抖动）--
    g, jumps = gt_series(win)
    if jumps:
        res["flags"].append(f"GT-JUMP×{jumps}")
    if len(g) >= 5 and g[-1][0] - g[0][0] > 0.2:
        sx = theil_sen([(e[0], e[2]) for e in g])
        sy = theil_sen([(e[0], e[3]) for e in g])
        sw = theil_sen([(e[0], e[4]) for e in g])
        if sx is not None and sy is not None:
            v_sign = 1.0 if cmd_vx >= 0 else -1.0
            res["gt_v"] = v_sign * math.hypot(sx, sy)
        if sw is not None:
            res["gt_wz"] = sw
    else:
        res["flags"].append("GT-SPARSE")

    # -- odom 通道（轮速计自报）--
    ov = [r["odom_vx"] for r in win if r["odom_vx"] is not None]
    ow = [r["odom_wz"] for r in win if r["odom_wz"] is not None]
    if ov:
        res["odom_v"] = statistics.median(ov)
    if ow:
        res["odom_wz"] = statistics.median(ow)

    # -- 增益 --
    if abs(cmd_vx) > 1e-6 and "gt_v" in res:
        res["gain_v_gt"] = res["gt_v"] / cmd_vx
        if "odom_v" in res:
            res["gain_v_odom"] = res["odom_v"] / cmd_vx
        if abs(res["gt_v"]) < NO_RESP_V:
            res["flags"].append("V-NO-RESP")
    if abs(cmd_wz) > 1e-6 and "gt_wz" in res:
        res["gain_w_gt"] = res["gt_wz"] / cmd_wz
        if "odom_wz" in res:
            res["gain_w_odom"] = res["odom_wz"] / cmd_wz
        if abs(res["gt_wz"]) < NO_RESP_W:
            res["flags"].append("W-NO-RESP(死区)")

    # -- 纯滞后（粗估 ±~0.1s）--
    if abs(cmd_wz) > 1e-6:
        g_all, _ = gt_series(step_rows)
        steady = res.get("gt_wz", 0.0)
        thr = max(0.04, 0.3 * abs(steady))
        for i in range(1, len(g_all) - 1):
            dt = g_all[i + 1][0] - g_all[i - 1][0]
            if dt <= 0:
                continue
            rate = (g_all[i + 1][4] - g_all[i - 1][4]) / dt
            if math.copysign(1.0, cmd_wz) * rate > thr:
                res["delay"] = g_all[i][1] - t0   # row_t 与 t0 同钟
                break
    elif abs(cmd_vx) > 1e-6:
        thr = 0.5 * abs(cmd_vx)
        for r in step_rows:
            if r["odom_vx"] is not None and math.copysign(1.0, cmd_vx) * r["odom_vx"] > thr:
                res["delay"] = r["t"] - t0
                break

    return res


def load_csv(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            rows.append(line)
    rdr = csv.DictReader(rows)
    out = []
    for d in rdr:
        row = {}
        for k in CSV_FIELDS:
            v = d.get(k, "")
            if k in ("seg", "group", "phase"):
                row[k] = v
            else:
                row[k] = float(v) if v not in ("", "nan") else None
        out.append(row)
    return out


def analyze_all(rows, settle, hold_v, hold_w, print_rows=True):
    """按 seg 分组分析 step 相位，打印明细 + 汇总，返回汇总 dict。"""
    segs = {}
    for r in rows:
        if r["phase"] != "step":
            continue
        segs.setdefault(r["seg"], []).append(r)

    results = []
    for seg_id in sorted(segs, key=lambda s: segs[s][0]["t"]):
        srows = segs[seg_id]
        group = srows[0]["group"]
        hold = hold_v if group == "v" else hold_w
        res = analyze_segment(srows, settle, hold)
        if res is None:
            continue
        res["seg"] = seg_id
        res["group"] = group
        results.append(res)

    if print_rows:
        print("\n── 分段明细 ─────────────────────────────────────────────────────")
        print(f"{'seg':<14}{'cmd_vx':>8}{'cmd_wz':>8}{'gt_v':>8}{'gt_wz':>8}"
              f"{'odom_v':>8}{'odom_wz':>9}{'g_v':>7}{'g_w':>7}{'delay':>7}  flags")
        for r in results:
            print(f"{r['seg']:<14}"
                  f"{r['cmd_vx']:>8.3f}{r['cmd_wz']:>8.3f}"
                  f"{r.get('gt_v', float('nan')):>8.3f}{r.get('gt_wz', float('nan')):>8.3f}"
                  f"{r.get('odom_v', float('nan')):>8.3f}{r.get('odom_wz', float('nan')):>9.3f}"
                  f"{r.get('gain_v_gt', float('nan')):>7.2f}{r.get('gain_w_gt', float('nan')):>7.2f}"
                  f"{r.get('delay', float('nan')):>7.2f}  {','.join(r['flags'])}")

    # -- 汇总 --
    def agg(vals):
        vals = [v for v in vals if v is not None and not math.isnan(v)]
        if not vals:
            return None
        return (statistics.median(vals), min(vals), max(vals), len(vals))

    summary = {}
    print("\n── 汇总（中位 [min..max] (n)）─────────────────────────────────────")
    for group, key, label in (
        ("v", "gain_v_gt", "v 增益   (GT真值/cmd)"),
        ("v", "gain_v_odom", "v 增益   (轮速计/cmd)"),
        ("wzstat", "gain_w_gt", "wz原地增益(GT真值/cmd)"),
        ("wzstat", "gain_w_odom", "wz原地增益(轮速计/cmd)"),
        ("wzroll", "gain_w_gt", "wz滚动增益(GT真值/cmd)  ★关键缺口"),
        ("wzroll", "gain_w_odom", "wz滚动增益(轮速计/cmd)"),
    ):
        a = agg([r.get(key) for r in results if r["group"] == group])
        if a:
            med, lo, hi, n = a
            summary[f"{group}.{key}"] = med
            print(f"  {label:<28} {med:6.2f}  [{lo:5.2f} .. {hi:5.2f}]  (n={n})")

    # 分档明细（看线性度/死区）
    print("\n── 按档位（GT 真值增益中位）───────────────────────────────────────")
    by_level = {}
    for r in results:
        lvl = r["cmd_wz"] if r["group"] != "v" else r["cmd_vx"]
        key = (r["group"], round(abs(lvl), 3))
        g = r.get("gain_w_gt") if r["group"] != "v" else r.get("gain_v_gt")
        dead = any("NO-RESP" in f for f in r["flags"])
        by_level.setdefault(key, []).append((g, dead))
    for (group, lvl) in sorted(by_level):
        entries = by_level[(group, lvl)]
        gains = [g for g, _ in entries if g is not None and not math.isnan(g)]
        ndead = sum(1 for _, d in entries if d)
        gtxt = f"{statistics.median(gains):5.2f}" if gains else "  n/a"
        dtxt = f"  死区 {ndead}/{len(entries)}" if ndead else ""
        print(f"  {group:<8} |cmd|={lvl:<6} gain={gtxt}{dtxt}")

    delays = agg([r.get("delay") for r in results])
    if delays:
        print(f"\n  纯滞后估计: {delays[0]:.2f}s  [{delays[1]:.2f} .. {delays[2]:.2f}] (n={delays[3]}, 分辨率~0.1s)")

    # 建议
    print("\n── 建议 ───────────────────────────────────────────────────────────")
    gw_roll = summary.get("wzroll.gain_w_gt")
    gw_stat = summary.get("wzstat.gain_w_gt")
    gv = summary.get("v.gain_v_gt")
    if gv:
        print(f"  v 通道修正系数   k_v = 1/{gv:.2f} = {1.0 / gv:.3f}   (|gain-1|>{abs(gv-1):.0%})")
    if gw_stat:
        print(f"  wz原地修正系数  k_w = 1/{gw_stat:.2f} = {1.0 / gw_stat:.3f}")
    if gw_roll:
        print(f"  wz滚动修正系数  k_w = 1/{gw_roll:.2f} = {1.0 / gw_roll:.3f}  ← DWB 工况以此为准")
    if gw_roll and gw_stat and abs(gw_roll - gw_stat) > 0.3:
        print("  ⚠ 滚动/原地 wz 增益差 >0.3：非线性显著（静摩擦态相关），"
              "固件单系数只能取滚动值，原地行为需另行处理。")
    odom_v = summary.get("v.gain_v_odom")
    if gv and odom_v and abs(odom_v - gv) > 0.1:
        print(f"  ⚠ 轮速计自报 v 增益({odom_v:.2f}) 与真值({gv:.2f}) 不一致 → odom 标定也要修。")
    odom_w = summary.get("wzroll.gain_w_odom") or summary.get("wzstat.gain_w_odom")
    gw_any = gw_roll or gw_stat
    if gw_any and odom_w and abs(odom_w - gw_any) > 0.2:
        print(f"  ⚠ 轮速计自报 wz 增益({odom_w:.2f}) 与真值({gw_any:.2f}) 不一致 → "
              "track_width/wheel_R 参数在固件与 odom 计算里不自洽。")
    print("  修正落点优先级：wheeltec 固件运动学参数 > twist_transform 系数。"
          "改完重跑本脚本验收 |gain-1|<10%。")
    return summary


# --------------------------------------------------------------------------- #
# ROS 节点
# --------------------------------------------------------------------------- #
class Calibrator:
    def __init__(self, args):
        import rclpy
        from rclpy.node import Node  # noqa: F401
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry

        self.args = args
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("calibrate_plant_gains")
        self.Twist = Twist

        self.pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.lock = threading.Lock()
        # 真值阶梯（lightning-only，2026-07-17 调整）：
        #   lidar_odom /lightning/odometry —— 主源。激光里程计连续积分，
        #              不会重定位跳变（loc/odometry 实测偶发单帧 6.9m/π 跳）。
        #              odom 系，秒级测量窗内漂移可忽略，长会话 home 点微漂。
        #   tf         odom->base_*（同为 lightning 发布）
        #   loc_odom   /lightning/loc/odometry —— 末位兜底，分析层有跳变缝合。
        #   topic      /groundtruth/pose 仅在显式传 --truth-topic 时参与（最高优先）。
        self.truth = {}          # source -> (recv_mono, stamp_t, x, y, yaw)
        self.truth_hist = {}     # source -> [最近若干条 (recv_mono, t, x, y, yaw)]
        self.truth_order = (("topic",) if args.truth_topic else ()) + (
            "lidar_odom", "tf", "loc_odom")
        self.truth_label = {
            "topic": args.truth_topic or "(未启用)",
            "loc_odom": "/lightning/loc/odometry (兜底,有重定位跳变风险)",
            "lidar_odom": "/lightning/odometry",
            "tf": "TF",
        }
        self.active_truth = None
        self.odom = None        # (recv_mono, vx, wz, x, y, yaw)
        self._viol_count = 0    # 安全框连击计数（抗单帧跳变误触发）

        if args.truth_topic:
            self.node.create_subscription(
                PoseWithCovarianceStamped, args.truth_topic, self._on_gt, 10)
        self.node.create_subscription(
            Odometry, "/lightning/loc/odometry",
            lambda m: self._on_truth_odom("loc_odom", m), 10)
        self.node.create_subscription(
            Odometry, "/lightning/odometry",
            lambda m: self._on_truth_odom("lidar_odom", m), 10)
        self.node.create_subscription(Odometry, "/odom", self._on_odom, 10)

        # TF 兜底：第一条可解析且 stamp 在推进的链（odom 父系优先，map 系靠 loc 不稳）
        import tf2_ros
        from rclpy.time import Time as _RclTime
        self._RclTime = _RclTime
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.tf_pairs = [("odom", "base_footprint"), ("odom", "base_link"),
                         ("map", "base_footprint"), ("map", "base_link")]
        self.tf_pair = None
        self.last_tf_stamp = 0.0
        self.node.create_timer(0.05, self._poll_tf)

        self.rows = []
        self.abort = False

        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:
            pass

    def _store_truth(self, source, stamp_t, x, y, yaw):
        e = (time.monotonic(), stamp_t, x, y, yaw)
        with self.lock:
            self.truth[source] = e
            h = self.truth_hist.setdefault(source, [])
            h.append(e)
            if len(h) > 7:
                del h[0]

    def _on_gt(self, msg):
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation.x, p.orientation.y,
                            p.orientation.z, p.orientation.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._store_truth("topic", t, p.position.x, p.position.y, yaw)

    def _on_truth_odom(self, source, msg):
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation.x, p.orientation.y,
                            p.orientation.z, p.orientation.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._store_truth(source, t, p.position.x, p.position.y, yaw)

    def _poll_tf(self):
        pairs = [self.tf_pair] if self.tf_pair else self.tf_pairs
        for pair in pairs:
            try:
                tr = self.tf_buffer.lookup_transform(pair[0], pair[1],
                                                     self._RclTime())
            except Exception:
                continue
            self.tf_pair = pair
            st = tr.header.stamp.sec + tr.header.stamp.nanosec / 1e9
            if st > self.last_tf_stamp:   # stamp 不推进 = 上游死了，不算新鲜
                self.last_tf_stamp = st
                t = tr.transform.translation
                q = tr.transform.rotation
                self._store_truth("tf", st, t.x, t.y,
                                  yaw_from_quat(q.x, q.y, q.z, q.w))
            return

    def _on_odom(self, msg):
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation.x, p.orientation.y,
                            p.orientation.z, p.orientation.w)
        with self.lock:
            self.odom = (time.monotonic(), msg.twist.twist.linear.x,
                         msg.twist.twist.angular.z,
                         p.position.x, p.position.y, yaw)

    # ------------------------------------------------------------------ #
    def gt_fresh(self):
        """按阶梯优先级取新鲜真值；全过期/无源 → None。"""
        now = time.monotonic()
        with self.lock:
            snap = dict(self.truth)
        for src in self.truth_order:
            e = snap.get(src)
            if e and now - e[0] < self.args.gt_stale:
                if src != self.active_truth:
                    lbl = self.truth_label[src]
                    if src == "tf" and self.tf_pair:
                        lbl = f"TF {self.tf_pair[0]}->{self.tf_pair[1]}"
                    print(f"  [truth] 真值源: {lbl}")
                    self.active_truth = src
                return e
        return None

    def gt_latest(self):
        """无视新鲜度取最近一条（安全检查通过后的位姿读取用）。"""
        with self.lock:
            snap = list(self.truth.values())
        return max(snap, key=lambda e: e[0]) if snap else None

    def gt_pose_med(self):
        """当前活跃源最近≤5条的中值位姿 (x, y, yaw)。

        单帧重定位跳变被中值压掉；持续性位移 2-3 帧(~150ms)后被接受。
        homing / 安全框 / home 采集一律用它，不用裸样本。
        """
        e = self.gt_fresh()
        if e is None:
            e = self.gt_latest()
            if e is None:
                return None
        src = self.active_truth
        with self.lock:
            h = list(self.truth_hist.get(src, []))[-5:]
        if len(h) < 2:
            return (e[2], e[3], e[4])
        xs = sorted(v[2] for v in h)
        ys = sorted(v[3] for v in h)
        ref = h[-1][4]
        dyaws = sorted(ang_norm(v[4] - ref) for v in h)
        m = len(h) // 2
        return (xs[m], ys[m], ang_norm(ref + dyaws[m]))

    def wait_topics(self, timeout=20.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                od_ok = self.odom and time.monotonic() - self.odom[0] < 0.5
            if od_ok and self.gt_fresh():
                return True
            time.sleep(0.2)
        now = time.monotonic()
        with self.lock:
            ages = {s: now - e[0] for s, e in self.truth.items()}
            od = self.odom
        print("✗ 话题未就绪：")
        od_txt = f"age {now - od[0]:.1f}s" if od else "从未收到"
        print(f"    /odom (wheeltec, alias 5): {od_txt}")
        for src in self.truth_order:
            a_txt = f"age {ages[src]:.1f}s" if src in ages else "从未收到"
            print(f"    {self.truth_label[src]}: {a_txt}")
        print("  提示: lightning (alias 3) 需先手动给初始位姿(/initialpose)，"
              "定位未初始化前 loc/odometry 与 map TF 都不会更新；"
              "或改用 slam_gt (alias 6)。")
        return False

    def send(self, vx, wz):
        msg = self.Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def zero_burst(self, n=5):
        for _ in range(n):
            self.send(0.0, 0.0)
            time.sleep(0.02)

    # ------------------------------------------------------------------ #
    def record_row(self, seg, group, phase, vx, wz):
        gt = self.gt_latest()
        with self.lock:
            od = self.odom
        now = time.monotonic()
        row = {
            "t": now, "seg": seg, "group": group, "phase": phase,
            "cmd_vx": vx, "cmd_wz": wz,
            "gt_t": gt[1] if gt else None,
            "gt_x": gt[2] if gt else None,
            "gt_y": gt[3] if gt else None,
            "gt_yaw": gt[4] if gt else None,
            "gt_age": (now - gt[0]) if gt else None,
            "odom_vx": od[1] if od else None,
            "odom_wz": od[2] if od else None,
            "odom_x": od[3] if od else None,
            "odom_y": od[4] if od else None,
            "odom_yaw": od[5] if od else None,
        }
        self.rows.append(row)

    def safety_violation(self):
        """返回 None 或违规原因字符串。越框需连续 3 次读到（抗单帧跳变）。"""
        g = self.gt_fresh()
        if g is None:
            return f"真值超时(> {self.args.gt_stale}s) —— 定位/slam 掉了？"
        p = self.gt_pose_med()
        dx = p[0] - self.home[0]
        dy = p[1] - self.home[1]
        half = self.args.box / 2.0
        if abs(dx) > half or abs(dy) > half:
            self._viol_count += 1
            if self._viol_count >= 3:
                return f"越出安全框 |dx|={abs(dx):.2f} |dy|={abs(dy):.2f} > {half:.2f}"
        else:
            self._viol_count = 0
        return None

    def hold(self, seg, group, phase, vx, wz, dur, guarded=True):
        """以 rate 发送 (vx,wz) 共 dur 秒，逐 tick 记录 + 安全检查。"""
        period = 1.0 / self.args.rate
        t_end = time.monotonic() + dur
        while time.monotonic() < t_end:
            if guarded:
                why = self.safety_violation()
                if why:
                    self.zero_burst()
                    print(f"  ✗ 安全中止({phase}): {why}")
                    return False
            self.send(vx, wz)
            self.record_row(seg, group, phase, vx, wz)
            time.sleep(period)
        return True

    # ------------------------------------------------------------------ #
    def gt_yaw_rate(self):
        """活跃真值源最近 ~0.3s 的 yaw 速率（相邻差分中位，抗单帧跳变）。"""
        src = self.active_truth
        with self.lock:
            h = list(self.truth_hist.get(src, []))[-6:]
        rates = []
        for a, b in zip(h, h[1:]):
            dt = b[1] - a[1]
            if 1e-3 < dt < 0.5:
                rates.append(ang_norm(b[4] - a[4]) / dt)
        return statistics.median(rates) if rates else 0.0

    def rotate_to(self, target_yaw, timeout=None):
        """GT 闭环原地旋转（2026-07-17 重调）。

        实测问题：cmd=0.55 正处 stick-slip 边界（yaw 率中位 0.15、爆发 1.3
        rad/s），12s 超时转不完；且增益 ~2.5× 使固定提前角必然过冲。
        对策：w 提到 0.70 踢穿静摩擦；预测性提前停 lead=|ω_est|·lead_s
        （蠕动时 lead 收到 0.30，爆发时自动放大）；脉冲-滑行-再评估循环；
        容差放宽到 home_yaw_tol（回位不需要精确朝向）。
        """
        args = self.args
        timeout = timeout or args.home_rotate_timeout
        t0 = time.monotonic()
        attempts = 0
        while time.monotonic() - t0 < timeout and attempts < 5:
            p = self.gt_pose_med()
            if abs(ang_norm(target_yaw - p[2])) < args.home_yaw_tol:
                self.zero_burst(3)
                return True
            attempts += 1
            pulse_t0 = time.monotonic()
            while time.monotonic() - t0 < timeout:
                if self.safety_violation():
                    self.zero_burst()
                    return False
                p = self.gt_pose_med()
                err = ang_norm(target_yaw - p[2])
                lead = max(0.30, abs(self.gt_yaw_rate()) * args.home_lead_s)
                if abs(err) < lead or time.monotonic() - pulse_t0 > 4.0:
                    break
                w = math.copysign(args.home_w, err)
                self.send(0.0, w)
                self.record_row("home", "home", "rotate", 0.0, w)
                time.sleep(1.0 / args.rate)
            self.zero_burst(3)
            time.sleep(1.0)          # 等动量滑行完再评估
        p = self.gt_pose_med()
        ok = abs(ang_norm(target_yaw - p[2])) < args.home_yaw_tol * 1.5
        self.zero_burst()
        return ok

    def go_home(self):
        """回起始位姿：对准方位角 → 直行回中 → 回初始航向。

        直行段 P 增益 0.6 / wz 限 ±0.25（底盘 wz 增益 ~2.5×，给大了会绕圈）；
        方位偏差 >0.9 rad 时停车重对准（最多 3 轮），不再一条弧硬追。
        """
        args = self.args
        for _ in range(3):
            p = self.gt_pose_med()
            dist = math.hypot(self.home[0] - p[0], self.home[1] - p[1])
            if dist <= args.home_pos_tol:
                break
            bearing = math.atan2(self.home[1] - p[1], self.home[0] - p[0])
            if not self.rotate_to(bearing):
                return False
            t0 = time.monotonic()
            realign = False
            while time.monotonic() - t0 < args.home_drive_timeout:
                if self.safety_violation():
                    self.zero_burst()
                    return False
                p = self.gt_pose_med()
                dist = math.hypot(self.home[0] - p[0], self.home[1] - p[1])
                if dist < args.home_pos_tol:
                    break
                bearing = math.atan2(self.home[1] - p[1], self.home[0] - p[0])
                werr = ang_norm(bearing - p[2])
                if abs(werr) > 0.9:
                    realign = True     # 冲过头/漂太偏：停车重对准
                    break
                wcmd = max(-0.25, min(0.25, 0.6 * werr))
                self.send(args.home_v, wcmd)
                self.record_row("home", "home", "drive", args.home_v, wcmd)
                time.sleep(1.0 / args.rate)
            self.zero_burst(3)
            time.sleep(0.5)
            if not realign:
                break
        p = self.gt_pose_med()
        if math.hypot(self.home[0] - p[0], self.home[1] - p[1]) > args.home_pos_tol * 1.5:
            return False
        return self.rotate_to(self.home[2])

    def maybe_home(self):
        if self.args.no_home:
            return
        p = self.gt_pose_med()
        dist = math.hypot(self.home[0] - p[0], self.home[1] - p[1])
        yaw_err = abs(ang_norm(self.home[2] - p[2]))
        if dist > 0.6 or yaw_err > 0.7:
            print(f"  ↩ 回位 (偏移 {dist:.2f}m / {math.degrees(yaw_err):.0f}°)...")
            if not self.go_home():
                if self.args.no_confirm:
                    print("  ⚠ 自动回位未达容差 —— 继续下一段（增益测量不受影响，注意场地余量）")
                else:
                    input("  ⚠ 自动回位失败/超时 —— 请手动摆正机器人后按 ENTER 继续: ")

    # ------------------------------------------------------------------ #
    def run(self):
        args = self.args
        print("等待真值（lightning 阶梯：odometry > TF > loc/odometry）+ /odom ...")
        if not self.wait_topics():
            return 2
        time.sleep(0.3)               # 攒几帧供中值滤波
        p = self.gt_pose_med()
        self.home = p
        print(f"✓ 真值就绪 @({p[0]:.2f},{p[1]:.2f},{math.degrees(p[2]):.0f}°)  "
              f"odom 就绪。安全框 ±{args.box / 2.0:.1f}m。")

        # 组装分段计划
        plan = []   # (seg_id, group, vx, wz, lead, hold)
        tests = args.tests.split(",")
        for rep in range(args.repeats):
            sign = 1.0 if rep % 2 == 0 else -1.0
            if "v" in tests:
                for lvl in args.v_levels:
                    plan.append((f"v{lvl}#{rep}", "v", sign * lvl, 0.0, 0.0, args.hold_v))
            if "wzstat" in tests:
                for lvl in args.wstat_levels:
                    plan.append((f"ws{lvl}#{rep}", "wzstat", 0.0, sign * lvl, 0.0, args.hold_w))
            if "wzroll" in tests:
                for lvl in args.wroll_levels:
                    plan.append((f"wr{lvl}#{rep}", "wzroll", args.base_vx, sign * lvl,
                                 args.lead, args.hold_w))
        # 按组排序（v 全做完 → wzstat → wzroll），组内保持 rep 交替方向
        order = {"v": 0, "wzstat": 1, "wzroll": 2}
        plan.sort(key=lambda p: order[p[1]])

        print(f"\n计划 {len(plan)} 段: "
              + ", ".join(f"{g}×{sum(1 for p in plan if p[1] == g)}"
                          for g in ("v", "wzstat", "wzroll") if any(p[1] == g for p in plan)))
        if args.dry_run:
            for p in plan:
                print(f"  {p[0]:<12} {p[1]:<7} vx={p[2]:+.2f} wz={p[3]:+.2f} "
                      f"lead={p[4]:.1f}s hold={p[5]:.1f}s")
            print("dry-run 结束（未发送任何 cmd）。")
            return 0

        if args.sign_check:
            print("\n[sign-check] vx=+0.10 × 0.6s ...")
            self.hold("signv", "v", "step", 0.10, 0.0, 0.6)
            self.zero_burst()
            time.sleep(1.0)
            p2 = self.gt_pose_med()
            fwd = ((p2[0] - p[0]) * math.cos(p[2]) + (p2[1] - p[1]) * math.sin(p[2]))
            print(f"  GT 前向位移 {fwd:+.3f}m（应为正）")
            print("[sign-check] wz=+0.30 × 0.6s ...")
            self.hold("signw", "wzstat", "step", 0.0, 0.30, 0.6)
            self.zero_burst()
            time.sleep(1.0)
            p3 = self.gt_pose_med()
            print(f"  GT 航向变化 {math.degrees(ang_norm(p3[2] - p2[2])):+.1f}°（应为正=逆时针）")
            return 0

        cur_group = None
        skipped = set()
        done = 0
        for (seg_id, group, vx, wz, lead, hold_dur) in plan:
            if group != cur_group:
                cur_group = group
                desc = {"v": "直行阶跃（交替正反向）",
                        "wzstat": "原地旋转阶跃",
                        "wzroll": f"滚动旋转（先直行{args.lead}s 再叠加 wz）"}[group]
                if not args.no_confirm:
                    ans = input(f"\n■ 组 [{group}] {desc} —— ENTER 开始 / s 跳过组 / q 退出: ")
                    if ans.strip().lower() == "q":
                        break
                    if ans.strip().lower() == "s":
                        skipped.add(group)
                        continue
                else:
                    print(f"\n■ 组 [{group}] {desc}")
            if group in skipped:
                continue

            print(f"▶ {seg_id}: vx={vx:+.2f} wz={wz:+.2f}")
            ok = True
            if lead > 0:
                ok = self.hold(seg_id, group, "lead", vx, 0.0, lead)
            if ok:
                ok = self.hold(seg_id, group, "step", vx, wz, hold_dur)
            self.zero_burst(3)
            self.hold(seg_id, group, "stop", 0.0, 0.0, 0.6, guarded=False)
            done += 1
            if not ok:
                input("  ⚠ 段被安全中止 —— 摆正机器人（回到场地中心）后按 ENTER 继续: ")
            self.maybe_home()

        self.zero_burst()
        print(f"\n完成 {done}/{len(plan)} 段。")
        return 0

    def shutdown(self):
        try:
            self.zero_burst()
        except Exception:
            pass
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        f.write(f"# calibrate_plant_gains argv: {' '.join(sys.argv)}\n")
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if r[k] is None else r[k]) for k in CSV_FIELDS})


def parse_levels(s):
    return [float(x) for x in s.split(",") if x.strip()]


def main():
    ap = argparse.ArgumentParser(
        description="滚动态 wz/v 执行增益标定（slam_gt 真值 + 轮速计双观测）")
    ap.add_argument("--tests", default="v,wzstat,wzroll",
                    help="逗号分隔: v,wzstat,wzroll")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--v-levels", type=parse_levels, default=[0.15, 0.20],
                    help="直行档位 m/s（生产工作点）")
    ap.add_argument("--wstat-levels", type=parse_levels, default=[0.25, 0.50, 0.75],
                    help="原地 wz 档位 rad/s")
    ap.add_argument("--wroll-levels", type=parse_levels, default=[0.10, 0.25, 0.50],
                    help="滚动叠加 wz 档位 rad/s（DWB 实际使用区间）")
    ap.add_argument("--base-vx", type=float, default=0.15,
                    help="滚动组基础线速度 = DWB max_vel_x")
    ap.add_argument("--hold-v", type=float, default=4.0)
    ap.add_argument("--hold-w", type=float, default=2.0)
    ap.add_argument("--lead", type=float, default=1.5, help="滚动组前置直行秒数")
    ap.add_argument("--settle", type=float, default=0.8, help="稳态窗前的沉降期")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--box", type=float, default=3.0, help="安全框边长 m（中心=启动位姿）")
    ap.add_argument("--gt-stale", type=float, default=0.6)
    ap.add_argument("--home-v", type=float, default=0.15)
    ap.add_argument("--home-w", type=float, default=0.70,
                    help="回位原地转幅值。实测 0.55 卡 stick-slip 边界"
                         "(yaw率中位仅0.15rad/s)，0.70 才能可靠起转")
    ap.add_argument("--home-lead-s", type=float, default=0.45,
                    help="预测性提前停：提前角=|ω实测|×该秒数（吸收滑行动量）")
    ap.add_argument("--home-yaw-tol", type=float, default=0.6,
                    help="回位航向容差 rad（标定回位不需要精确朝向）")
    ap.add_argument("--home-pos-tol", type=float, default=0.4,
                    help="回位位置容差 m")
    ap.add_argument("--home-rotate-timeout", type=float, default=25.0)
    ap.add_argument("--home-drive-timeout", type=float, default=30.0)
    ap.add_argument("--truth-topic", default="",
                    help="显式指定最高优先真值话题(PoseWithCovarianceStamped,"
                         "如 slam_gt 的 /groundtruth/pose)。默认空=纯 lightning 阶梯")
    ap.add_argument("--no-confirm", action="store_true", help="组间不提示直接跑")
    ap.add_argument("--no-home", action="store_true", help="段间不自动回位")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--sign-check", action="store_true")
    ap.add_argument("--log-dir", default="/tmp/david_nav_logs")
    ap.add_argument("--analyze", default="",
                    help="只重分析已有 CSV，不连 ROS 不发 cmd")
    args = ap.parse_args()

    if args.analyze:
        rows = load_csv(args.analyze)
        print(f"重分析 {args.analyze}: {len(rows)} 行")
        analyze_all(rows, args.settle, args.hold_v, args.hold_w)
        return 0

    log_path, stamp = setup_logging(args.log_dir)
    csv_path = os.path.join(args.log_dir, f"calibrate_plant_gains_{stamp}.csv")
    print(f"log: {log_path}\ncsv: {csv_path}")
    print("⚠ 70kg 底盘将自主运动 —— 场地 ≥3×3m，人守急停。Ctrl-C 随时零速退出。")

    cal = Calibrator(args)
    rc = 1
    try:
        rc = cal.run()
    except KeyboardInterrupt:
        print("\nCtrl-C —— 发送零速退出。")
    finally:
        cal.shutdown()
        if cal.rows:
            write_csv(csv_path, cal.rows)
            print(f"\nCSV 已保存: {csv_path}  ({len(cal.rows)} 行)")
            try:
                analyze_all(cal.rows, args.settle, args.hold_v, args.hold_w)
            except Exception as e:  # 分析失败不吞数据
                print(f"⚠ 自动分析失败（数据已保存，可 --analyze 重跑）: {e}")
            print(f"\n重分析命令: python3 {os.path.abspath(__file__)} --analyze {csv_path}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
