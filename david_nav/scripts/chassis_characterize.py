#!/usr/bin/env python3
"""开环底盘速度区间表征（新底盘标定，3m×3m 场地，机器人居中）。

直接向 /cmd_vel 发**恒定** Twist（不经 separator/david_move = 纯 plant），用
**/odom 的 twist.twist（轮速编码器速度）作为真值**，分直行 / 旋转两类扫描，
找出：
  * 死区下限 min_cmd_v / min_cmd_w —— 最坏 caster 状态下仍能可靠起步的最低 cmd
  * 饱和上限 v_sat / w_sat       —— actual 不再随 cmd 增长的拐点（→ 有用 v_max/w_max）
  * 命令→实速 映射 + 加速度上限 a_max/alpha_max（斜坡阶跃斜率）

高噪声万向轮处理：每档重复 R 次；直行档前可做小幅原地转（甩定脚轮到逆向＝
ALIGN→DRIVE 最坏工况），旋转档前可走一小段直行（＝"直行后"旋转最坏工况）；
统计 min/median/max，死区下限取**最坏重复仍能动**的档。

安全（70kg 重载差速底盘）：
  * 启动时把 odom 位置当场地中心，±(field/2-margin) 安全框。直行 burst 还有
    一个更小的 soft 位移上限：超过就提前优雅减速；越安全框 → 立即发零速 + abort。
  * 直行用交替方向 + odom 闭环慢速回中，避免漂出框。
  * 全程无人工确认（--no-confirm 风格）；**人必须在机器人旁守急停**。

先干跑校验（不发任何 cmd，只验证 odom + 看门狗 + 打印计划）：
  python3 scripts/chassis_characterize.py --dry-run
方向/符号确认（一个极低速短脉冲）：
  python3 scripts/chassis_characterize.py --sign-check
实跑：
  python3 scripts/chassis_characterize.py --tests straight,rotate
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

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# --------------------------------------------------------------------------- #
# logging tee (沿用 field_test 约定：带时间戳日志 + _latest 软链)
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
    log_path = os.path.join(log_dir, f"chassis_characterize_{stamp}.log")
    fh = open(log_path, "w", buffering=1)
    fh.write(f"# chassis_characterize {stamp}\n# argv: {' '.join(sys.argv)}\n")
    sys.stdout = _Tee(sys.__stdout__, fh)
    sys.stderr = _Tee(sys.__stderr__, fh)
    latest = "/tmp/chassis_characterize_latest.log"
    try:
        if os.path.islink(latest) or os.path.exists(latest):
            os.remove(latest)
        os.symlink(os.path.abspath(log_path), latest)
    except OSError:
        latest = None
    return log_path, latest, stamp, fh


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# --------------------------------------------------------------------------- #
class Characterizer(Node):
    def __init__(self, half_safe: float, soft_disp: float) -> None:
        super().__init__("chassis_characterize")
        self.half_safe = half_safe        # 硬安全框半宽（越界→急停）
        self.soft_disp = soft_disp        # 直行 burst 软位移上限（超过→提前减速）
        self._lock = threading.Lock()
        self._x = self._y = self._yaw = 0.0
        self._vx = self._wz = 0.0         # /odom twist 轮速真值
        self._have_odom = False
        self.origin = None
        self.origin_yaw = 0.0
        self.breach = False               # 硬越界发生过
        self._odom_t = None

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        t = msg.twist.twist
        with self._lock:
            self._x, self._y = p.x, p.y
            self._yaw = quat_to_yaw(msg.pose.pose.orientation)
            self._vx = t.linear.x
            self._wz = t.angular.z
            self._have_odom = True
            self._odom_t = time.monotonic()
            if self.origin is None:
                self.origin = (p.x, p.y)
                self.origin_yaw = self._yaw

    # ---- accessors ----
    def state(self):
        with self._lock:
            return (self._x, self._y, self._yaw, self._vx, self._wz)

    def odom_fresh(self, max_age=0.4) -> bool:
        with self._lock:
            return self._odom_t is not None and (time.monotonic() - self._odom_t) < max_age

    def out_of_box(self) -> bool:
        with self._lock:
            if self.origin is None:
                return False
            return (abs(self._x - self.origin[0]) > self.half_safe
                    or abs(self._y - self.origin[1]) > self.half_safe)

    def disp_from_origin(self) -> float:
        with self._lock:
            if self.origin is None:
                return 0.0
            return math.hypot(self._x - self.origin[0], self._y - self.origin[1])

    def signed_forward_to_origin(self) -> float:
        """沿当前朝向，原点在身后为负 / 身前为正的投影距离（回中用）。"""
        with self._lock:
            if self.origin is None:
                return 0.0
            dx = self.origin[0] - self._x
            dy = self.origin[1] - self._y
            return dx * math.cos(self._yaw) + dy * math.sin(self._yaw)

    # ---- cmd helpers ----
    def send(self, v: float, w: float) -> None:
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub.publish(m)

    def stop(self, n=5, dt=0.05) -> None:
        for _ in range(n):
            self.send(0.0, 0.0)
            time.sleep(dt)


# --------------------------------------------------------------------------- #
class Aborted(Exception):
    pass


class Runner:
    """串行驱动测序；所有运动都过这里 → 统一看门狗 + 急停。"""

    def __init__(self, node: Characterizer, rate=20.0, dry=False):
        self.n = node
        self.dt = 1.0 / rate
        self.dry = dry

    def guard(self):
        """每个控制 tick 调用：越界或 odom 失联 → 急停 + Aborted。"""
        if self.n.out_of_box():
            self.n.breach = True
            self.n.stop()
            raise Aborted(f"硬越界 disp={self.n.disp_from_origin():.2f}m > "
                          f"±{self.n.half_safe:.2f}m — 急停")
        if not self.n.odom_fresh():
            self.n.stop()
            raise Aborted("odom 失联 >0.4s — 急停")

    def hold_cmd(self, v, w, duration, cap_disp=None, record=False):
        """以恒定 (v,w) 持续 duration 秒；可选记录 twist 真值样本。

        cap_disp: 直行 burst 软位移上限（m），超过→提前 break（返回 reason）。
        return: (samples_vx, samples_wz, reason)  reason∈{"done","cap","abort"}
        """
        sv, sw = [], []
        t_end = time.monotonic() + duration
        reason = "done"
        while time.monotonic() < t_end:
            self.guard()
            if cap_disp is not None and self.n.disp_from_origin() >= cap_disp:
                reason = "cap"
                break
            if not self.dry:
                self.n.send(v, w)
            if record:
                _, _, _, vx, wz = self.n.state()
                sv.append(vx)
                sw.append(wz)
            time.sleep(self.dt)
        return sv, sw, reason

    def recenter_linear(self, tol=0.08, vmax=0.30, vmin=0.18, timeout=15.0):
        """沿朝向开回原点（直行项每个 rep 前用；闭环 + 看门狗兜底）。

        速度比例收敛但**永远 ≥ vmin**——这台 caster 在 <0.18 常常起步不了，回中速度
        太低会卡在边界(上一轮 sweep 0.40/0.50 全 0 的根因就是 0.25 回中卡死)。"""
        if self.dry:
            return
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.guard()
            fwd = self.n.signed_forward_to_origin()
            if abs(fwd) <= tol:
                break
            v = clamp(1.0 * fwd, -vmax, vmax)
            if abs(v) < vmin:
                v = math.copysign(vmin, fwd)
            self.n.send(v, 0.0)
            time.sleep(self.dt)
        self.n.stop()

    def settle(self, sec=0.8):
        """发零速并等机身停稳。"""
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end:
            if not self.dry:
                self.n.send(0.0, 0.0)
            time.sleep(self.dt)


# --------------------------------------------------------------------------- #
STEADY_WIN_SEC = 0.7   # 稳态只取末尾这么多秒（丢掉起步 stick-slip lurch 整段）
SAMPLE_RATE = 20.0     # /odom 频率，用于把窗口秒换成样本数

def steady_stats(samples):
    """末尾 STEADY_WIN_SEC 的 |样本| → (median, min, max, n)。

    用 median 而非 mean：高噪声 caster 起步会暴冲(stick-slip lurch)，均值被尖峰
    污染（实测 0.15cmd 暴冲到 0.248）；中位数抗尖峰，更接近真稳态。
    """
    if not samples:
        return (0.0, 0.0, 0.0, 0)
    k = max(1, int(STEADY_WIN_SEC * SAMPLE_RATE))
    win = [abs(s) for s in samples[-k:]]
    return (statistics.median(win), min(win), max(win), len(win))


def main():
    ap = argparse.ArgumentParser(description="开环底盘速度区间表征 3x3m")
    ap.add_argument("--field", type=float, default=3.0)
    ap.add_argument("--margin", type=float, default=0.45,
                    help="安全余量(m,含内切半径~0.375)。安全框=±(field/2-margin)，默认±1.05m")
    ap.add_argument("--soft-disp", type=float, default=0.80,
                    help="直行 burst 软位移上限(m)，到此提前减速回中；< 安全框")
    ap.add_argument("--tests", default="straight,rotate")
    ap.add_argument("--rate", type=float, default=20.0, help="控制/采样频率(Hz)，匹配 odom 20Hz")
    ap.add_argument("--repeats", type=int, default=3, help="每档重复次数（抓 caster 噪声）")
    ap.add_argument("--hold", type=float, default=2.0,
                    help="每档恒速保持秒数（>break-away+稳态窗,默认2.0;高速档会被soft位移提前cap）")
    ap.add_argument("--settle", type=float, default=0.8)
    ap.add_argument("--v-levels", default="0.03,0.05,0.07,0.10,0.13,0.16,0.20,0.25,0.30,0.40,0.50",
                    help="直行 cmd 档(m/s)，逗号分隔，低→高")
    ap.add_argument("--w-levels", default="0.05,0.08,0.12,0.16,0.20,0.25,0.30,0.40,0.55,0.70,0.90",
                    help="旋转 cmd 档(rad/s)")
    ap.add_argument("--precondition", action=argparse.BooleanOptionalAction, default=True,
                    help="档前做最坏 caster 预置（直行前小转 / 旋转前小直行）")
    ap.add_argument("--dry-run", action="store_true", help="不发任何 cmd，只验证 odom/看门狗/打印计划")
    ap.add_argument("--sign-check", action="store_true",
                    help="只发一个极低速短脉冲（直行+旋转各一）确认方向符号后退出")
    ap.add_argument("--log-dir", default="/tmp/david_nav_logs")
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    log_path, latest, stamp, _fh = setup_logging(args.log_dir)
    if not args.csv:
        args.csv = os.path.join(args.log_dir, f"chassis_characterize_{stamp}.csv")
    print(f"日志: {log_path}" + (f"  (tail -f {latest})" if latest else ""))
    print(f"CSV : {args.csv}")

    global SAMPLE_RATE
    SAMPLE_RATE = args.rate
    half_safe = args.field / 2.0 - args.margin
    if half_safe <= 0.15:
        print(f"ERROR 安全框过小 ±{half_safe:.2f}m"); sys.exit(2)
    soft = min(args.soft_disp, half_safe - 0.15)
    v_levels = [float(x) for x in args.v_levels.split(",") if x.strip()]
    w_levels = [float(x) for x in args.w_levels.split(",") if x.strip()]
    tests = [t.strip() for t in args.tests.split(",") if t.strip()]

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init()
    node = Characterizer(half_safe, soft)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    print("等待 /odom ...")
    t0 = time.monotonic()
    while node.origin is None and time.monotonic() - t0 < 10.0:
        time.sleep(0.1)
    if node.origin is None:
        print("ERROR 10s 内无 /odom"); node.stop(); rclpy.shutdown(); sys.exit(2)
    print(f"场地中心(odom)=({node.origin[0]:.2f},{node.origin[1]:.2f}) "
          f"安全框=±{half_safe:.2f}m soft位移={soft:.2f}m  rate={args.rate}Hz")
    print(f"直行档: {v_levels}")
    print(f"旋转档: {w_levels}")
    print(f"重复×{args.repeats} hold={args.hold}s precondition={args.precondition} "
          f"{'[DRY-RUN 不发cmd]' if args.dry_run else ''}\n")

    runner = Runner(node, rate=args.rate, dry=args.dry_run)
    rows = []   # (kind, cmd, mean, vmin, vmax, n, reason, rep)

    def emergency_stop():
        node.stop(n=10, dt=0.02)

    try:
        if args.sign_check:
            print("SIGN-CHECK: 直行 +0.12 m/s × 0.8s（应前进, twist.vx>0）")
            sv, _, _ = runner.hold_cmd(0.12, 0.0, 0.8, cap_disp=soft, record=True)
            node.stop(); runner.settle(0.6)
            m, lo, hi, n = steady_stats(sv)
            print(f"  → 稳态 |vx|≈{m:.3f} (min{lo:.3f}/max{hi:.3f} n{n}); "
                  f"末样本符号 {'+' if (sv and sv[-1]>=0) else '-'}")
            runner.recenter_linear()
            print("SIGN-CHECK: 旋转 +0.25 rad/s × 0.8s（应左转/CCW, twist.wz>0）")
            _, sw, _ = runner.hold_cmd(0.0, 0.25, 0.8, record=True)
            node.stop(); runner.settle(0.6)
            m, lo, hi, n = steady_stats(sw)
            print(f"  → 稳态 |wz|≈{m:.3f} (min{lo:.3f}/max{hi:.3f} n{n}); "
                  f"末样本符号 {'+' if (sw and sw[-1]>=0) else '-'}")
            print("\nSIGN-CHECK 完成。确认符号/方向无误后去掉 --sign-check 实跑。")
            raise KeyboardInterrupt

        # -------- 直行 sweep --------
        if "straight" in tests and not args.dry_run:
            print("=== 直行 sweep ===")
            for vi, vc in enumerate(v_levels):
                for r in range(args.repeats):
                    # 每个 rep 先回中 → 满 soft 位移预算（修上一版高速档 cap0 全 0 的坑）
                    runner.recenter_linear()
                    if args.precondition:
                        # 最坏：先小幅原地转甩定脚轮（方向交替），再立刻直行
                        sgn = 1.0 if (r % 2 == 0) else -1.0
                        runner.hold_cmd(0.0, sgn * 0.4, 0.6)
                        node.stop(); runner.settle(0.4)
                    direction = 1.0 if (r % 2 == 0) else -1.0
                    sv, _, reason = runner.hold_cmd(direction * vc, 0.0, args.hold,
                                                    cap_disp=soft, record=True)
                    node.stop(); runner.settle(args.settle)
                    m, lo, hi, n = steady_stats(sv)
                    moved = m > 0.015   # 轮速>1.5cm/s 视为起步
                    print(f"  v_cmd={vc:.3f} rep{r+1} dir{'+' if direction>0 else '-'}: "
                          f"|vx|稳态≈{m:.3f} (min{lo:.3f}/max{hi:.3f} n{n}) "
                          f"{'MOVED' if moved else 'stuck'} [{reason}]")
                    rows.append(("straight", vc, m, lo, hi, n, reason, r + 1))
            print()

        # -------- 旋转 sweep --------
        if "rotate" in tests and not args.dry_run:
            print("=== 旋转 sweep ===")
            for wc in w_levels:
                for r in range(args.repeats):
                    if args.precondition:
                        # 最坏：先走一小段直行（"直行后"旋转工况），再原地转
                        runner.hold_cmd(0.12, 0.0, 0.5, cap_disp=soft)
                        node.stop(); runner.settle(0.4)
                        runner.recenter_linear()
                    sgn = 1.0 if (r % 2 == 0) else -1.0
                    _, sw, reason = runner.hold_cmd(0.0, sgn * wc, args.hold, record=True)
                    node.stop(); runner.settle(args.settle)
                    m, lo, hi, n = steady_stats(sw)
                    moved = m > 0.03
                    print(f"  w_cmd={wc:.3f} rep{r+1} sgn{'+' if sgn>0 else '-'}: "
                          f"|wz|稳态≈{m:.3f} (min{lo:.3f}/max{hi:.3f} n{n}) "
                          f"{'MOVED' if moved else 'stuck'} [{reason}]")
                    rows.append(("rotate", wc, m, lo, hi, n, reason, r + 1))
            print()

        if args.dry_run:
            print("DRY-RUN: odom/看门狗/计划已验证，未发任何 cmd。去掉 --dry-run 实跑。")

    except Aborted as e:
        print(f"\n!! ABORT: {e}")
        emergency_stop()
    except KeyboardInterrupt:
        print("\nCtrl-C / sign-check 结束 — 急停")
        emergency_stop()
    finally:
        emergency_stop()
        # 落 CSV
        if rows:
            with open(args.csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["kind", "cmd", "act_mean", "act_min", "act_max", "n", "reason", "rep"])
                w.writerows(rows)
            print(f"CSV 写入 {args.csv}")
            summarize(rows)
        print("\n完成。机器人四周安全框留好。")
        node.stop()
        time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        try:
            sys.stdout.flush(); sys.stderr.flush()
            sys.stdout, sys.stderr = sys.__stdout__, sys.__stderr__
            _fh.close()
        except Exception:
            pass


def summarize(rows):
    """按 cmd 聚合多次重复 → 死区下限(最坏仍动) + 饱和拐点。"""
    for kind, mv_thresh, unit in (("straight", 0.015, "m/s"), ("rotate", 0.03, "rad/s")):
        krows = [r for r in rows if r[0] == kind]
        if not krows:
            continue
        cmds = sorted(set(r[1] for r in krows))
        print(f"\n--- {kind} 汇总（真值=/odom twist, 单位{unit}）---")
        print(f"{'cmd':>7} {'act_med':>8} {'med_lo':>7} {'med_hi':>7} {'worst':>7} {'reps':>5}  起步?")
        agg = {}
        for c in cmds:
            cr = [r for r in krows if r[1] == c]
            meds = [r[2] for r in cr]            # 每次重复的稳态中位数
            med_lo, med_hi = min(meds), max(meds)
            worst = min(r[3] for r in cr)        # 任一重复稳态窗的全局最小
            med = statistics.median(meds)
            # 「可靠起步」= 每次重复的稳态中位数都过阈值（最坏重复也动）
            moved_worst = med_lo > mv_thresh
            agg[c] = (med, moved_worst)
            print(f"{c:>7.3f} {med:>8.3f} {med_lo:>7.3f} {med_hi:>7.3f} {worst:>7.3f} {len(cr):>5}  "
                  f"{'YES' if moved_worst else 'no'}")
        # 死区下限 = 最低的「每次重复都起步」档（且其上各档也都起步，排除单档侥幸）
        floor = None
        cl = sorted(cmds)
        for i, c in enumerate(cl):
            if all(agg[cc][1] for cc in cl[i:]):
                floor = c
                break
        # 饱和拐点 = 从 floor 之上起，median 单调上升斜率连续 2 档 <8% 的第一档
        sat = None
        above = [c for c in cl if floor is not None and c >= floor]
        for i in range(2, len(above)):
            a, b, d = agg[above[i-2]][0], agg[above[i-1]][0], agg[above[i]][0]
            if a > 0 and (b - a) / a < 0.08 and (d - b) / max(b, 1e-6) < 0.08:
                sat = above[i-1]
                break
        floor_s = f"{floor:.3f}" if floor is not None else "未捕获(都没动/都动)"
        sat_s = f"{sat:.3f}" if sat is not None else "未捕获(还在线性增长，可加高档)"
        print(f"  ▶ 死区下限(最坏仍起步) ≈ {floor_s} {unit}"
              f"  → 建议 min_cmd ≈ {floor_s}+裕量")
        print(f"  ▶ 饱和拐点 ≈ {sat_s} {unit}  → 建议 v_max/w_max 取此附近")


if __name__ == "__main__":
    main()
