#!/usr/bin/env python3
"""position_controller 实车/仿真验收测试脚本（3m×3m 场地）。

向 position_controller 的 ~/move action 依次发 MoveTo 目标，量化 SPEC §7 验收：
直线 / 原地转 / 定点重复散布 / 取消 / 限速，并打印汇总表（可选 CSV）。

安全（重载 70kg 差速底盘，务必看 README §6.1 死区）：
  * 把脚本启动时的 odom 位置当作 **场地中心**，按 `--field`（默认 3.0m）+ `--margin`
    （默认 0.6m）算一个安全方框；任何会越界的目标**拒发**。
  * 运动中 odom 一旦越出安全框 → 立即 cancel 当前 goal（位置环收到即发零速）。
  * 默认每个动作前要 [Enter] 确认；`--no-confirm` 跳过。Ctrl-C → cancel 当前 goal。

先在仿真里干跑（不接底盘）：
  ros2 run david_nav position_controller &
  python3 scripts/sim_fake_chassis.py &        # 理想底盘
  python3 scripts/position_controller_field_test.py --no-confirm --tests all

实车（确认 mux/底盘链路就绪、人在急停旁）：
  python3 scripts/position_controller_field_test.py --tests straight,rotate,pose,repeat
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from david_nav_msgs.action import MoveTo


class _Tee:
    """Duplicate writes to the console and a log file (transcript capture)."""

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


def setup_logging(log_dir: str, log_path: str):
    """Open the run log, tee stdout+stderr into it, refresh the _latest symlink.

    Follows the repo convention (`launch_nav_with_log.sh`): timestamped file
    under /tmp/david_nav_logs + a stable _latest symlink so it can be tail -f'd
    without knowing the timestamp. Returns (log_path, file_handle).
    """
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    if not log_path:
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, f"position_controller_field_test_{stamp}.log")
    else:
        os.makedirs(os.path.dirname(os.path.abspath(log_path)) or ".", exist_ok=True)
    fh = open(log_path, "w", buffering=1)
    fh.write(f"# position_controller_field_test  {stamp}\n")
    fh.write(f"# argv: {' '.join(sys.argv)}\n")
    sys.stdout = _Tee(sys.__stdout__, fh)
    sys.stderr = _Tee(sys.__stderr__, fh)
    latest = "/tmp/position_controller_field_test_latest.log"
    try:
        if os.path.islink(latest) or os.path.exists(latest):
            os.remove(latest)
        os.symlink(os.path.abspath(log_path), latest)
    except OSError:
        latest = None
    return log_path, latest, stamp, fh


def launch_controller(params_file: str, cmd_topic: str, log_dir: str,
                      stamp: str):
    """Spawn `ros2 run david_nav position_controller` as a child process.

    Its stdout/stderr go to position_controller_node_<stamp>.log next to the
    test log. start_new_session=True puts it in its own process group so we can
    SIGINT the whole group on teardown (clean exit → node._stop() zeros cmd).
    Returns (proc, node_log_path).
    """
    node_log = os.path.join(log_dir, f"position_controller_node_{stamp}.log")
    cmd = ["ros2", "run", "david_nav", "position_controller", "--ros-args",
           "--params-file", params_file,
           "-r", f"/cmd_vel_position:={cmd_topic}"]
    lf = open(node_log, "w", buffering=1)
    lf.write(f"# launched by field_test: {' '.join(cmd)}\n")
    proc = subprocess.Popen(cmd, stdout=lf, stderr=subprocess.STDOUT,
                            start_new_session=True)
    return proc, node_log


def stop_controller(proc) -> None:
    """SIGINT the controller's process group, wait, SIGKILL fallback."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        return
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FieldTester(Node):
    def __init__(self, action_ns: str, half_safe: float) -> None:
        super().__init__("position_controller_field_test")
        self.half_safe = half_safe          # safe box half-width (m) around origin
        self._lock = threading.Lock()
        self._x = self._y = self._yaw = 0.0
        self._have_odom = False
        self.origin = None                  # (x0, y0) captured at start = field center
        self.origin_yaw = 0.0               # heading captured at start (for re-home)

        # live-goal state for the bounds watchdog + sample capture
        self._active_gh = None
        self._cancel_sent = False
        self.samples = []                   # (t, x, y, yaw) during the active goal
        self.last_feedback = None
        self.breach = False                 # set True if a goal got bounds-canceled

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self._client = ActionClient(self, MoveTo, f"{action_ns}/move")

    # ---- odom + bounds watchdog ----
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        with self._lock:
            self._x, self._y = p.x, p.y
            self._yaw = quat_to_yaw(msg.pose.pose.orientation)
            self._have_odom = True
            if self.origin is None:
                self.origin = (p.x, p.y)
                self.origin_yaw = self._yaw
            if self._active_gh is not None:
                self.samples.append((time.monotonic(), self._x, self._y, self._yaw))
                if self._out_of_bounds(self._x, self._y) and not self._cancel_sent:
                    self._cancel_sent = True
                    self.breach = True
                    self.get_logger().error(
                        f"BOUNDS BREACH at ({self._x:.2f},{self._y:.2f}) — CANCEL")
                    self._active_gh.cancel_goal_async()

    def pose(self):
        with self._lock:
            return self._x, self._y, self._yaw

    def _out_of_bounds(self, x: float, y: float) -> bool:
        if self.origin is None:
            return False
        return (abs(x - self.origin[0]) > self.half_safe
                or abs(y - self.origin[1]) > self.half_safe)

    def predict_in_bounds(self, mode: int, distance: float, yaw: float,
                          tx: float, ty: float, relative: bool) -> tuple:
        """Return (ok, reason). Refuse goals whose endpoint leaves the safe box."""
        x, y, th = self.pose()
        if mode == MoveTo.Goal.MODE_ROTATE:
            return True, "in-place"
        if mode == MoveTo.Goal.MODE_DISTANCE:
            ex, ey = x + distance * math.cos(th), y + distance * math.sin(th)
        else:  # POSE
            if relative:
                c, s = math.cos(th), math.sin(th)
                ex, ey = x + tx * c - ty * s, y + tx * s + ty * c
            else:
                ex, ey = tx, ty
        if self._out_of_bounds(ex, ey):
            return False, (f"endpoint ({ex:.2f},{ey:.2f}) outside safe box "
                           f"±{self.half_safe:.2f}m around center {self.origin}")
        edge = self.half_safe - max(abs(ex - self.origin[0]),
                                    abs(ey - self.origin[1]))
        warn = f"  ⚠越界缓冲仅{edge*100:.0f}cm，实车请值守急停/后移机器人" if edge < 0.12 else ""
        return True, f"endpoint ({ex:.2f},{ey:.2f}) ok{warn}"

    # ---- action plumbing (executor spins in a background thread) ----
    def _await(self, fut, timeout: float):
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return None
            time.sleep(0.02)
        return fut.result()

    def run_goal(self, goal: MoveTo.Goal, label: str, timeout: float = 90.0):
        """Send one goal, watchdog-guarded; return a metrics dict."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("action server ~/move not available")
            return None
        self.samples = []
        self.last_feedback = None
        self._cancel_sent = False
        self.breach = False
        start = self.pose()
        t0 = time.monotonic()

        send_fut = self._client.send_goal_async(goal, feedback_callback=self._fb)
        gh = self._await(send_fut, 5.0)
        if gh is None or not gh.accepted:
            self.get_logger().error(f"[{label}] goal not accepted")
            return None
        self._active_gh = gh
        res = self._await(gh.get_result_async(), timeout)
        self._active_gh = None
        elapsed = time.monotonic() - t0
        end = self.pose()
        status = res.status if res is not None else None
        result = res.result if res is not None else None
        return {
            "label": label, "start": start, "end": end, "elapsed": elapsed,
            "status": status, "result": result, "breach": self.breach,
            "samples": list(self.samples),
        }

    def _fb(self, fb) -> None:
        self.last_feedback = fb.feedback

    def cancel_active(self):
        if self._active_gh is not None:
            self._active_gh.cancel_goal_async()


# --------------------------------------------------------------------------- #
# goal builders
# --------------------------------------------------------------------------- #
def g_distance(d: float) -> MoveTo.Goal:
    g = MoveTo.Goal(); g.mode = MoveTo.Goal.MODE_DISTANCE; g.distance = float(d)
    return g


def g_rotate(deg: float) -> MoveTo.Goal:
    g = MoveTo.Goal(); g.mode = MoveTo.Goal.MODE_ROTATE
    g.yaw = math.radians(deg); return g


def g_pose(dx: float, dy: float, dyaw_deg: float, relative=True) -> MoveTo.Goal:
    g = MoveTo.Goal(); g.mode = MoveTo.Goal.MODE_POSE
    g.target.x = float(dx); g.target.y = float(dy)
    g.yaw = math.radians(dyaw_deg); g.relative = relative
    return g


def g_pose_abs(tx: float, ty: float, yaw_rad: float) -> MoveTo.Goal:
    """Absolute (odom-frame) POSE — used for re-home regardless of current drift."""
    g = MoveTo.Goal(); g.mode = MoveTo.Goal.MODE_POSE
    g.target.x = float(tx); g.target.y = float(ty)
    g.yaw = float(yaw_rad); g.relative = False
    return g


# --------------------------------------------------------------------------- #
# metrics
# --------------------------------------------------------------------------- #
def metrics_distance(m, commanded):
    (x0, y0, th0), (x1, y1, _th1) = m["start"], m["end"]
    travelled = (x1 - x0) * math.cos(th0) + (y1 - y0) * math.sin(th0)
    lateral = -(x1 - x0) * math.sin(th0) + (y1 - y0) * math.cos(th0)
    peak = 0.0
    for _, sx, sy, _ in m["samples"]:
        proj = (sx - x0) * math.cos(th0) + (sy - y0) * math.sin(th0)
        peak = max(peak, proj)
    overshoot = max(0.0, peak - commanded)
    return {"residual": commanded - travelled, "lateral": lateral,
            "overshoot_m": overshoot}


def metrics_rotate(m, commanded_deg):
    (_x0, _y0, th0), (_x1, _y1, th1) = m["start"], m["end"]
    turned = wrap(th1 - th0)
    peak = 0.0
    for _, _sx, _sy, syaw in m["samples"]:
        peak = max(peak, abs(wrap(syaw - th0)))
    overshoot = max(0.0, math.degrees(peak) - abs(commanded_deg))
    return {"residual_deg": commanded_deg - math.degrees(turned),
            "overshoot_deg": overshoot}


def metrics_pose(m, dx, dy, dyaw_deg):
    (x0, y0, th0), (x1, y1, th1) = m["start"], m["end"]
    c, s = math.cos(th0), math.sin(th0)
    tx, ty = x0 + dx * c - dy * s, y0 + dx * s + dy * c
    t_yaw = wrap(th0 + math.radians(dyaw_deg))
    return {"pos_err": math.hypot(x1 - tx, y1 - ty),
            "yaw_err_deg": math.degrees(wrap(th1 - t_yaw)),
            "target": (tx, ty)}


# --------------------------------------------------------------------------- #
def confirm(msg: str, auto: bool) -> bool:
    if auto:
        print(f"  >> {msg} [auto]")
        return True
    try:
        return input(f"  >> {msg} — 回车执行 / 输入 s 跳过: ").strip().lower() != "s"
    except (EOFError, KeyboardInterrupt):
        return False


def main():
    ap = argparse.ArgumentParser(description="position_controller 3x3m field test")
    ap.add_argument("--field", type=float, default=3.0, help="场地边长(m)，默认3.0")
    ap.add_argument("--margin", type=float, default=0.45,
                    help="安全余量(m，已含机器人内切半径~0.375)。安全框=±(field/2-margin)。"
                         "默认0.45→3m场地±1.05m，恰好容纳居中起步的1m直线(留5cm越界缓冲)。")
    ap.add_argument("--action-ns", default="/position_controller",
                    help="action 命名空间，默认 /position_controller")
    ap.add_argument("--tests", default="straight,rotate,pose,repeat,cancel",
                    help="逗号分隔: straight,rotate,pose,repeat,cancel 或 all")
    ap.add_argument("--distance", type=float, default=1.0)
    ap.add_argument("--rotate-deg", type=float, default=90.0)
    ap.add_argument("--pose", default="0.5,0.5,0", help="相对 dx,dy,dyaw_deg")
    ap.add_argument("--repeat", type=int, default=5, help="定点重复次数")
    ap.add_argument("--no-confirm", action="store_true")
    ap.add_argument("--rehome", action=argparse.BooleanOptionalAction, default=True,
                    help="每项之间用绝对 POSE 回到场地中心+初始朝向（默认开），"
                         "防止单项失败把机器人留在角落导致后续越界级联跳过")
    ap.add_argument("--csv", default="", help="结果 CSV 路径；留空=自动写到日志目录")
    ap.add_argument("--log", default="",
                    help="日志文件路径；留空=自动 /tmp/david_nav_logs/...<时间戳>.log")
    ap.add_argument("--log-dir", default="/tmp/david_nav_logs",
                    help="自动日志目录（默认 /tmp/david_nav_logs，沿用本仓约定）")
    ap.add_argument("--launch-controller", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="启动脚本时一并拉起 position_controller 节点（默认开）。"
                         "已自行起节点或要走 mux 时用 --no-launch-controller。")
    ap.add_argument("--controller-params",
                    default="/home/agx/ros2_ws/src/david_nav/config/position_controller.yaml",
                    help="拉起 controller 用的 params yaml")
    ap.add_argument("--cmd-topic", default="/cmd_vel",
                    help="controller 输出 remap 目标（/cmd_vel_position:=此topic）；"
                         "纯位置环测试直发 /cmd_vel，与 Nav2 共存时改成 mux 输入")
    args = ap.parse_args()

    # ---- logging: tee 全部输出到带时间戳的日志 + _latest 软链 ----
    log_path, latest, stamp, _fh = setup_logging(args.log_dir, args.log)
    if not args.csv:
        args.csv = os.path.join(os.path.dirname(log_path),
                                f"position_controller_field_test_{stamp}.csv")
    print(f"日志: {log_path}")
    if latest:
        print(f"      tail -f {latest}")
    print(f"结果 CSV: {args.csv}")

    half_safe = args.field / 2.0 - args.margin
    if half_safe <= 0.1:
        print(f"ERROR: 安全框过小 (±{half_safe:.2f}m)；减小 --margin 或增大 --field")
        sys.exit(2)
    tests = ("straight,rotate,pose,repeat,cancel".split(",")
             if args.tests == "all" else [t.strip() for t in args.tests.split(",")])
    dx, dy, dyaw = [float(v) for v in args.pose.split(",")]

    # disable rclpy's SIGINT handler so Ctrl-C raises KeyboardInterrupt in the
    # main thread (clean teardown) instead of shutting the context out from under
    # the background spin thread (which otherwise crashes with a wait-set error).
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init()
    node = FieldTester(args.action_ns, half_safe)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    print("等待 /odom ...")
    t0 = time.monotonic()
    while node.origin is None and time.monotonic() - t0 < 10.0:
        time.sleep(0.1)
    if node.origin is None:
        print("ERROR: 10s 内没收到 /odom，退出")
        node.cancel_active(); rclpy.shutdown(); sys.exit(2)
    print(f"场地中心(odom) = ({node.origin[0]:.2f},{node.origin[1]:.2f}), "
          f"安全框 = ±{half_safe:.2f}m  (field={args.field} margin={args.margin})")
    print("提示：把机器人放在场地正中、四周留出安全框再开始。\n")

    # 可选：一并拉起 position_controller 节点（默认开）
    proc = None
    if args.launch_controller:
        proc, node_log = launch_controller(
            args.controller_params, args.cmd_topic,
            os.path.dirname(log_path), stamp)
        print(f"已拉起 position_controller (pid {proc.pid})")
        print(f"  params : {args.controller_params}")
        print(f"  输出   : /cmd_vel_position -> {args.cmd_topic}")
        print(f"  节点日志: {node_log}")
        if node._client.wait_for_server(timeout_sec=8.0):
            print("position_controller action server 就绪\n")
        else:
            print("WARN: 8s 内 action server 未就绪——controller 可能没起来，"
                  f"看节点日志 {node_log}\n")

    rows = []

    def do(label, goal, mode, **predict):
        ok, reason = node.predict_in_bounds(mode=mode, **predict)
        if not ok:
            print(f"[{label}] 跳过：{reason}")
            return None
        if not confirm(f"{label}: {reason}", args.no_confirm):
            print(f"[{label}] 用户跳过")
            return None
        m = node.run_goal(goal, label)
        if m is None:
            print(f"[{label}] 失败/无结果")
        return m

    def rehome(tag):
        """Drive back to the captured field center via an ABSOLUTE POSE goal.

        Absolute (not relative) so it recovers even if the previous test left
        the robot displaced — endpoint is the center, always in-bounds, so it
        never cascades into out-of-bounds skips.
        """
        if not args.rehome:
            return
        ox, oy = node.origin
        # position-only (yaw=NaN): heading doesn't matter for re-home, and a big
        # final in-place turn scrubs the point (caster trailing) → fails to
        # converge. Next test is relative to wherever we end, so heading is free.
        print(f"[{tag}] 回中 → ({ox:.2f},{oy:.2f}) 仅位置")
        node.run_goal(g_pose_abs(ox, oy, float("nan")), f"rehome[{tag}]")

    try:
        if "straight" in tests:
            m = do("straight", g_distance(args.distance),
                   MoveTo.Goal.MODE_DISTANCE, distance=args.distance, yaw=0.0,
                   tx=0.0, ty=0.0, relative=True)
            if m:
                mm = metrics_distance(m, args.distance)
                ok = m["result"] and m["result"].success and not m["breach"]
                print(f"  → 残余={mm['residual']*100:+.1f}cm 侧偏={mm['lateral']*100:+.1f}cm "
                      f"超调={mm['overshoot_m']*100:.1f}cm 用时={m['elapsed']:.1f}s "
                      f"success={ok}  [验收: |残余|≤2cm]")
                rows.append(("straight", mm, m))
            rehome("straight")

        if "rotate" in tests:
            for sgn, name in ((+1, "rotate+"), (-1, "rotate-")):
                m = do(name, g_rotate(sgn * args.rotate_deg),
                       MoveTo.Goal.MODE_ROTATE, distance=0.0,
                       yaw=0.0, tx=0.0, ty=0.0, relative=True)
                if m:
                    mm = metrics_rotate(m, sgn * args.rotate_deg)
                    ok = m["result"] and m["result"].success and not m["breach"]
                    print(f"  → 残余={mm['residual_deg']:+.2f}° 超调={mm['overshoot_deg']:.1f}° "
                          f"用时={m['elapsed']:.1f}s success={ok}  [验收: |残余|≤1.15°]")
                    rows.append((name, mm, m))

        if "pose" in tests:
            m = do("pose", g_pose(dx, dy, dyaw),
                   MoveTo.Goal.MODE_POSE, distance=0.0, yaw=math.radians(dyaw),
                   tx=dx, ty=dy, relative=True)
            if m:
                mm = metrics_pose(m, dx, dy, dyaw)
                ok = m["result"] and m["result"].success and not m["breach"]
                print(f"  → 位置误差={mm['pos_err']*100:.1f}cm 朝向误差={mm['yaw_err_deg']:+.2f}° "
                      f"用时={m['elapsed']:.1f}s success={ok}  [验收: ≤20cm / ≤20°]")
                rows.append(("pose", mm, m))
            rehome("pose")

        if "repeat" in tests:
            print(f"\n定点重复 ×{args.repeat}（每次去 ({dx},{dy}) 再回原点，量散布）:")
            errs = []
            for i in range(args.repeat):
                m = do(f"repeat[{i+1}]", g_pose(dx, dy, dyaw),
                       MoveTo.Goal.MODE_POSE, distance=0.0,
                       yaw=math.radians(dyaw), tx=dx, ty=dy, relative=True)
                if m:
                    mm = metrics_pose(m, dx, dy, dyaw)
                    errs.append(mm["pos_err"])
                    print(f"    #{i+1} 位置误差={mm['pos_err']*100:.1f}cm "
                          f"朝向={mm['yaw_err_deg']:+.2f}°")
                    rows.append((f"repeat{i+1}", mm, m))
                rehome(f"repeat{i+1}")
            if errs:
                mean = sum(errs) / len(errs)
                std = (sum((e - mean) ** 2 for e in errs) / len(errs)) ** 0.5
                print(f"  散布: 均值={mean*100:.1f}cm 标准差={std*100:.1f}cm "
                      f"最大={max(errs)*100:.1f}cm  (n={len(errs)})")

        if "cancel" in tests:
            # 取消测试：发一个会越界但不会真发(predict拦截)？不行——要真跑再取消。
            # 改为发一个安全范围内的长 DISTANCE，1s 后主动 cancel，量停车响应。
            safe_d = max(0.3, min(args.distance, half_safe * 0.8))
            print(f"\n取消测试：DISTANCE {safe_d:.2f}m，1.0s 后主动 cancel：")
            if confirm("cancel-test", args.no_confirm):
                node.samples = []
                node._cancel_sent = False
                node.breach = False
                gfut = node._client.send_goal_async(g_distance(safe_d),
                                                    feedback_callback=node._fb)
                gh = node._await(gfut, 5.0)
                if gh and gh.accepted:
                    node._active_gh = gh
                    time.sleep(1.0)
                    t_cancel = time.monotonic()
                    gh.cancel_goal_async()
                    # keep sampling briefly after the result to capture settling
                    time.sleep(0.5)
                    res = node._await(gh.get_result_async(), 10.0)
                    node._active_gh = None
                    # cancel→停车时间 = 最后一个"还在动"的样本相对 cancel 的时刻。
                    # 步长阈值按 dt≈0.02s × v_stop(0.01) ≈ 0.2mm；用 1mm 容噪。
                    post = [(t, x, y) for (t, x, y, _yaw) in node.samples if t >= t_cancel]
                    last_moving = 0.0
                    for i in range(1, len(post)):
                        if math.hypot(post[i][1] - post[i-1][1],
                                      post[i][2] - post[i-1][2]) > 1e-3:
                            last_moving = post[i][0] - t_cancel
                    stop_str = f"{last_moving:.2f}s" if post else "无样本"
                    print(f"  → cancel→停车 ≈{stop_str} "
                          f"(状态码 {res.status if res else 'n/a'})  [验收: ≤1 周期发零速]")
                else:
                    print("  cancel 测试：goal 未被接受")

    except KeyboardInterrupt:
        print("\nCtrl-C — cancel 当前 goal")
        node.cancel_active()
    finally:
        # 无论正常结束/异常/Ctrl-C，都要：写 CSV、停 goal、关 controller、收尾。
        if args.csv and rows:
            with open(args.csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["test", "metric", "value"])
                for name, mm, _m in rows:
                    for k, v in mm.items():
                        w.writerow([name, k, v])
            print(f"\nCSV 写入 {args.csv}")

        print("\n完成。把机器人四周安全框留好；越界会自动 cancel。")
        node.cancel_active()
        if proc is not None:
            print("关闭 position_controller (SIGINT；节点退出时会发零速) ...")
            stop_controller(proc)
        time.sleep(0.3)
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        print(f"日志: {log_path}")
        if args.csv and rows:
            print(f"结果 CSV: {args.csv}")
        # restore real streams BEFORE closing the log so the daemon executor
        # thread can't write into a closed file (avoids unraisable).
        try:
            sys.stdout.flush(); sys.stderr.flush()
            sys.stdout, sys.stderr = sys.__stdout__, sys.__stderr__
            _fh.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
