#!/usr/bin/env python3
"""位置环 PID 粗扫（Phase B）——套现有 position_controller_field_test.py。

对一组 (Kp, Kd) 候选，逐个：生成临时 position_controller.yaml（base + 覆盖被测
增益 + Phase A 标定的速度区间）→ 跑 field_test 的对应单项（rotate 或 straight）
→ 解析它写的 CSV → 聚合 超调/残余/重复散布 → 排序打印。

真值与安全完全沿用 field_test：/odom + 3×3 安全框看门狗 + --no-confirm。

  旋转环（主要矛盾：caster 停转超调）：
    python3 scripts/pid_sweep.py --loop yaw \
        --kp 1.5,2.0,2.5,3.0 --kd 0.0,0.15,0.3 --reps 2
  直行环：
    python3 scripts/pid_sweep.py --loop dist \
        --kp 0.8,1.2,1.6 --kd 0.0,0.1,0.2 --reps 2

每个候选：rotate 项 field_test 自带 ±90° 两个机动 = 2 样本；--reps 再整体重复。
直行项每次交替 ±distance（--no-rehome 时在中心附近来回，不漂出框）。
"""

from __future__ import annotations

import argparse
import csv
import itertools
import os
import re
import statistics
import subprocess
import sys
import tempfile
import time

FIELD_TEST = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "position_controller_field_test.py")


def load_base(path):
    with open(path) as f:
        return f.read()


def make_yaml(base_text, overrides: dict) -> str:
    """把 base yaml 里 `key: value` 行就地替换成 override 值，写临时文件返回路径。

    只改已存在的标量行（dist.Kp/yaw.Kp/v_max/...），不动结构。找不到的 key 追加到
    ros__parameters 段尾。"""
    lines = base_text.splitlines()
    remaining = dict(overrides)
    out = []
    key_re = {k: re.compile(rf"^(\s*){re.escape(k)}:\s*[^#\n]+(.*)$") for k in overrides}
    for ln in lines:
        replaced = False
        for k, rx in key_re.items():
            m = rx.match(ln)
            if m and k in remaining:
                out.append(f"{m.group(1)}{k}: {remaining.pop(k)}{('  ' + m.group(2).strip()) if m.group(2).strip() else ''}")
                replaced = True
                break
        if not replaced:
            out.append(ln)
    # 任何没匹配到的 key：插到 ros__parameters: 行后
    if remaining:
        for i, ln in enumerate(out):
            if ln.strip().endswith("ros__parameters:"):
                indent = re.match(r"^(\s*)", out[i + 1]).group(1) if i + 1 < len(out) else "      "
                for k, v in remaining.items():
                    out.insert(i + 1, f"{indent}{k}: {v}")
                break
    fd, p = tempfile.mkstemp(prefix="pid_sweep_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        f.write("\n".join(out) + "\n")
    return p


def run_field_test(loop, params, rotate_deg, distance, log_dir):
    """跑一次 field_test 单项，返回它写的 CSV 路径（解析用）。"""
    test = "rotate" if loop == "yaw" else "straight"
    csv_path = os.path.join(log_dir, f"pid_sweep_{loop}_{int(time.time()*1000)%100000}.csv")
    cmd = [sys.executable, FIELD_TEST,
           "--no-confirm", "--no-rehome",
           "--tests", test,
           "--controller-params", params,
           "--cmd-topic", "/cmd_vel",
           "--csv", csv_path,
           "--log-dir", log_dir]
    if test == "rotate":
        cmd += ["--rotate-deg", str(rotate_deg)]
    else:
        cmd += ["--distance", str(distance)]
    subprocess.run(cmd, timeout=180)
    return csv_path


def parse_csv(path, loop):
    """从 field_test CSV 抽指标。rotate: overshoot_deg/residual_deg(±两机动)。
    straight: overshoot_m/residual。返回 dict of lists。"""
    out = {"overshoot": [], "residual": []}
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for row in csv.DictReader(f):
            metric, val = row["metric"], row["value"]
            try:
                v = float(val)
            except ValueError:
                continue
            if loop == "yaw":
                if metric == "overshoot_deg":
                    out["overshoot"].append(abs(v))
                elif metric == "residual_deg":
                    out["residual"].append(abs(v))
            else:
                if metric == "overshoot_m":
                    out["overshoot"].append(abs(v))
                elif metric == "residual":
                    out["residual"].append(abs(v))
    return out


def main():
    ap = argparse.ArgumentParser(description="位置环参数粗扫（套 field_test）")
    ap.add_argument("--loop", choices=["yaw", "dist"], required=True,
                    help="yaw→rotate 项；dist→straight 项")
    ap.add_argument("--p1", default="", help="第一个被扫参数名（默认 yaw.Kp / dist.Kp）")
    ap.add_argument("--p1-vals", required=True, help="逗号分隔候选值")
    ap.add_argument("--p2", default="", help="第二个被扫参数名（默认 yaw.Kd / dist.Kd）")
    ap.add_argument("--p2-vals", required=True, help="逗号分隔候选值")
    ap.add_argument("--reps", type=int, default=2, help="每候选整体重复次数")
    ap.add_argument("--rotate-deg", type=float, default=90.0)
    ap.add_argument("--distance", type=float, default=1.0)
    ap.add_argument("--base",
                    default="/home/agx/ros2_ws/src/david_nav/config/position_controller.yaml")
    ap.add_argument("--limits", default="",
                    help="额外固定覆盖，形如 w_max=0.8,min_cmd_w=0.5")
    ap.add_argument("--log-dir", default="/tmp/david_nav_logs/pid_sweep")
    args = ap.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    base = load_base(args.base)
    p1k = args.p1 or ("yaw.Kp" if args.loop == "yaw" else "dist.Kp")
    p2k = args.p2 or ("yaw.Kd" if args.loop == "yaw" else "dist.Kd")
    p1s = [float(x) for x in args.p1_vals.split(",")]
    p2s = [float(x) for x in args.p2_vals.split(",")]
    extra = {}
    for kv in (x for x in args.limits.split(",") if x.strip()):
        k, v = kv.split("=")
        extra[k.strip()] = v.strip()

    kp_key, kd_key = p1k, p2k
    kps, kds = p1s, p2s

    print(f"粗扫 loop={args.loop}  {kp_key}∈{kps}  {kd_key}∈{kds}  reps={args.reps}")
    if extra:
        print(f"固定覆盖: {extra}")
    print(f"日志目录: {args.log_dir}\n")

    results = []
    for kp, kd in itertools.product(kps, kds):
        ov, res = [], []
        ovr = {kp_key: kp, kd_key: kd}
        ovr.update(extra)
        params = make_yaml(base, ovr)
        for r in range(args.reps):
            dist = args.distance * (1.0 if r % 2 == 0 else -1.0)
            csv_path = run_field_test(args.loop, params, args.rotate_deg, dist, args.log_dir)
            m = parse_csv(csv_path, args.loop)
            ov += m["overshoot"]
            res += m["residual"]
        os.unlink(params)
        unit = "°" if args.loop == "yaw" else "m"
        ov_med = statistics.median(ov) if ov else float("nan")
        ov_max = max(ov) if ov else float("nan")
        res_med = statistics.median(res) if res else float("nan")
        # 评分：超调中位 + 超调最坏*0.5 + 残余中位（越低越好）
        score = (ov_med + 0.5 * ov_max + res_med) if ov else float("inf")
        results.append((kp, kd, ov_med, ov_max, res_med, len(ov), score))
        print(f"  {kp_key}={kp:.2f} {kd_key}={kd:.2f}: "
              f"超调中位={ov_med:.2f}{unit} 超调最坏={ov_max:.2f}{unit} "
              f"残余中位={res_med:.2f}{unit} n={len(ov)} score={score:.3f}")

    print("\n=== 排序（score 越低越好）===")
    for kp, kd, om, ox, rm, n, sc in sorted(results, key=lambda x: x[6]):
        print(f"  {kp_key}={kp:.2f} {kd_key}={kd:.2f}  超调中位={om:.2f} 最坏={ox:.2f} "
              f"残余={rm:.2f} n={n}  score={sc:.3f}")
    best = min(results, key=lambda x: x[6])
    print(f"\n▶ 最优: {kp_key}={best[0]:.2f} {kd_key}={best[1]:.2f}  "
          f"（再人工在其邻域精修 1–2 轮确认 caster 稳健性）")


if __name__ == "__main__":
    main()
