#!/usr/bin/env python3
"""A 组（A1 死区 / A2 增益 / A4 动态）一键辨识 + 自动生成 md 报告。

判定一律用**轮速计 /odom 的速度值**（linear.x / angular.z），不使用 IMU。
场地约束：2x2 m。线速度试验默认值已收紧；旋转类原地不占地。

复用 david_cmd 现成节点 wheeltec_velocity_loop_eval（只读 /odom，不碰 IMU）：
  - A1 死区 + A2 增益 共用一次低→高 amp 扫描（--mode amp）
  - A4 动态 -> --mode step
A3（惰行）不做：速度环 + STM32 P 控会主动保持速度，无法撤力滑行。

前置：
  1) source ~/ros2_ws/install/setup.bash
  2) 只起底盘，不要起 lightning/nav2/david_move（本脚本直发 /cmd_vel 绕过桥测裸执行器）：
       ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
  3) 装满 70kg 上肢、关节锁死；机器人放场地中央，朝对角线方向。

用法：
  python3 run_a_group.py                 # 交互式（线速度试验前提示复位+预置脚轮）
  python3 run_a_group.py --no-prompt     # 无人值守
  python3 run_a_group.py --dry-run       # 只打印命令
  python3 run_a_group.py --tests a1,a2   # 只跑部分（a1/a2 共用扫描）
"""
import argparse
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
import time

PKG = "david_cmd"
CMD_TOPIC = "/cmd_vel"      # 直发 = 绕过 david_move/separator，测裸执行器
ODOM_TOPIC = "/odom"
VOLT_TOPIC = "/PowerVoltage"
CMD_YAML = os.path.expanduser("~/ros2_ws/src/david_cmd/config/cmd.yaml")


# ----------------------------- 基础工具 -----------------------------
def sh(cmd, timeout=15):
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, timeout=timeout)
        return p.returncode, p.stdout
    except subprocess.TimeoutExpired as e:
        return 124, (e.output or "")
    except Exception as e:  # noqa: BLE001
        return 1, str(e)


def run_streaming(cmd, log_path, timeout):
    """跑节点，实时打印 + 落盘日志，返回 (rc, text)。"""
    print("\n\033[36mRUN:\033[0m " + " ".join(cmd))
    lines = []
    with open(log_path, "w") as lf:
        lf.write("CMD: " + " ".join(cmd) + "\n\n")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1)
        start = time.time()
        try:
            for line in proc.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                lf.write(line)
                lines.append(line)
                if timeout and (time.time() - start) > timeout:
                    proc.terminate()
                    lf.write("\n[SCRIPT TIMEOUT -> terminate]\n")
                    print("\033[31m[超时，已终止该试验]\033[0m")
                    break
        finally:
            try:
                proc.wait(timeout=10)
            except Exception:  # noqa: BLE001
                proc.kill()
    return proc.returncode, "".join(lines)


def list_topics():
    rc, out = sh(["ros2", "topic", "list"], timeout=10)
    return set(out.split()) if rc == 0 else set()


def read_voltage():
    rc, out = sh(["ros2", "topic", "echo", VOLT_TOPIC, "--once"], timeout=8)
    m = re.search(r"data:\s*([-\d.]+)", out)
    return float(m.group(1)) if m else None


def prompt(msg, enabled):
    if enabled:
        try:
            input(f"\033[33m{msg}  [回车继续 / Ctrl-C 中止]\033[0m")
        except EOFError:
            pass
    else:
        print(f"\033[33m{msg} (--no-prompt: 跳过等待)\033[0m")


def f(x, nd=4):
    if x is None:
        return "n/a"
    try:
        return f"{float(x):.{nd}f}"
    except (TypeError, ValueError):
        return str(x)


def load_cmd_yaml_lift():
    """读 cmd.yaml 现行 deadzone-lift 生产值，用于对照。"""
    try:
        import yaml
        with open(CMD_YAML) as fh:
            d = yaml.safe_load(fh)
        p = d["david_move"]["ros__parameters"]
        return {
            "min_linear_x": p.get("min_linear_x"),
            "min_stationary_angular_z": p.get("min_stationary_angular_z"),
        }
    except Exception:  # noqa: BLE001
        return None


# ----------------------------- 解析（全部基于 /odom 速度值） -----------------------------
def load_vle_summary(out_dir, label):
    path = os.path.join(out_dir, f"{label}_summary.json")
    if not os.path.exists(path):
        return None
    with open(path) as fh:
        return json.load(fh)


def deadzone_from_summary(summary, floor):
    """死区 = 最小的正命令档，其 /odom 稳态实速 >= floor（超出噪声地板=破启）。

    返回 (破启命令, 该档实速)；全程没动返回 (None, None)。
    """
    if not summary:
        return None, None
    cand = []
    for s in summary.get("segments", []):
        t, ss = s["target_w"], s["ss_mean"]
        if t > 0 and ss is not None and ss >= floor:
            cand.append((t, ss))
    if not cand:
        return None, None
    return min(cand, key=lambda c: c[0])


def segment_veer(out_dir, label, seg_name, tail_frac=0.3):
    """读某 linear 段 CSV，取末尾 tail_frac 的 odom_ang_z 均值 = 左右不对称代理。"""
    path = os.path.join(out_dir, f"{label}_{seg_name}.csv")
    if not os.path.exists(path):
        return None
    try:
        with open(path) as fh:
            rows = [r.strip().split(",") for r in fh.readlines()[1:] if r.strip()]
        if not rows:
            return None
        n = max(1, int(len(rows) * tail_frac))
        wz = [float(r[2]) for r in rows[-n:]]  # 列: t,lin_x,ang_z,...
    except Exception:  # noqa: BLE001
        return None
    return sum(wz) / len(wz) if wz else None


def responded(target, ss_mean, frac=0.3):
    if ss_mean is None or target == 0:
        return False
    return (ss_mean * target) > 0 and abs(ss_mean) >= abs(target) * frac


def analyze_step(csv_path, axis=None, floor=0.01):
    """从一段阶跃 CSV 提取 PI 整定指标：稳态误差/超调/上升/整定/反冲/振荡。

    需 hold 足够长（已到稳态平台）才有意义。返回 dict 或 None。
    """
    try:
        with open(csv_path) as fh:
            rows = [r.strip().split(",") for r in fh.readlines()[1:] if r.strip()]
    except Exception:  # noqa: BLE001
        return None
    if len(rows) < 5:
        return None
    t = [float(r[0]) for r in rows]
    lin = [float(r[1]) for r in rows]
    ang = [float(r[2]) for r in rows]
    try:
        target = float(rows[0][3])
    except (IndexError, ValueError):
        target = None
    if axis == "linear":
        sig = lin
    elif axis == "angular":
        sig = ang
    else:
        sig = lin if max(map(abs, lin)) >= max(map(abs, ang)) else ang
    t0 = t[0]
    t = [x - t0 for x in t]
    sgn = (1.0 if target > 0 else -1.0) if target else \
          (1.0 if max(sig) >= -min(sig) else -1.0)
    x = [v * sgn for v in sig]                 # 朝命令方向为正
    tgt = abs(target) if target else None
    vmax = max(x)
    if vmax < floor:
        return {"target": tgt, "note": "no motion"}
    onset = next((i for i, v in enumerate(x) if v > floor), 0)
    plateau_end = max((i for i, v in enumerate(x) if v > 0.5 * vmax),
                      default=len(x) - 1)
    a = onset + int(0.75 * max(plateau_end - onset, 1))
    seg = x[a:plateau_end + 1] or [x[plateau_end]]
    steady = sum(seg) / len(seg)
    overshoot = (vmax - steady) / steady * 100 if steady > 1e-9 else None
    sse = (1 - steady / tgt) * 100 if tgt else None
    imax = x.index(vmax)

    def cross(level):
        for i in range(onset, imax + 1):
            if x[i] >= level:
                return t[i]
        return None
    t10, t90 = cross(0.1 * steady), cross(0.9 * steady)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    band = 0.05 * steady
    last_out = next((i for i in range(plateau_end, onset - 1, -1)
                     if abs(x[i] - steady) > band), None)
    settle = (t[last_out] - t[onset]) if last_out is not None else None
    tail = x[plateau_end + 1:]
    reversal = -min(tail) if (tail and min(tail) < 0) else 0.0
    region = x[a:plateau_end + 1]
    osc = sum(1 for i in range(1, len(region))
              if (region[i] - steady) * (region[i - 1] - steady) < 0)
    return {"target": tgt, "vmax": vmax, "steady": steady,
            "overshoot_pct": overshoot, "sse_pct": sse, "rise_s": rise,
            "settle_s": settle, "reversal": reversal, "osc_crossings": osc}


# ----------------------------- 报告生成 -----------------------------
def md_table(header, rows):
    out = ["| " + " | ".join(header) + " |",
           "|" + "|".join(["---"] * len(header)) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def build_report(meta, a1, a2_lin, a2_ang, a4, lift, out_dir, vle_dir, floors):
    L = []
    L.append("# A 组辨识报告（裸执行器 / 2×2 m / 仅轮速计）\n")
    L.append("> 直发 `/cmd_vel` 绕过 david_move/separator，测 STM32+电机+脚轮裸特性。"
             "判定全部基于轮速计 `/odom`（`linear.x`/`angular.z`），**未用 IMU**。"
             "A3 惰行不可做（P 控主动保持）。\n")

    L.append("## 0. 元数据")
    L.append(md_table(["项", "值"], [
        ["时间", meta["time"]],
        ["场地", "2×2 m（线速度试验已收紧）"],
        ["命令 topic", f"`{CMD_TOPIC}`（bypass）"],
        ["反馈", f"`{ODOM_TOPIC}`（chassis twist，仅轮速计）"],
        ["运动地板", f"线 {floors['lin']} m/s / 角 {floors['ang']} rad/s（超此=破启）"],
        ["起始电压", f(meta.get("volt_start"), 2) + " V"],
        ["结束电压", f(meta.get("volt_end"), 2) + " V"],
        ["操作员备注", meta.get("operator", "")],
    ]))
    L.append("")

    # A1（由扫描低速段得出）
    L.append("## 1. A1 — 速度死区（由低速 amp 扫描得出，仅 /odom）")
    if a1:
        L.append(md_table(["量", "破启命令", "该档实速", "单位"], [
            ["线速度", f(a1["lin_cmd"]), f(a1["lin_ss"]), "m/s"],
            ["角速度", f(a1["ang_cmd"]), f(a1["ang_ss"]), "rad/s"],
        ]))
        notnull = [a1["lin_cmd"], a1["ang_cmd"]]
        if all(v is None for v in notnull):
            L.append("\n_所有测试档均未越过运动地板：死区 > 最大测试命令，需提高 amplitudes 再测。_")
        if lift:
            ml, msa = lift.get("min_linear_x"), lift.get("min_stationary_angular_z")
            verdict = []
            if a1["lin_cmd"] is not None and ml is not None:
                gap = ml - a1["lin_cmd"]
                verdict.append(f"- 线速度：实测破启 **{f(a1['lin_cmd'])}** vs 生产 lift "
                               f"`min_linear_x={ml}` → 余量 {gap:+.3f} m/s "
                               f"({'偏保守' if gap > 0.03 else '偏紧/可能不够' if gap < 0 else '合适'})")
            if a1["ang_cmd"] is not None and msa is not None:
                gap = msa - a1["ang_cmd"]
                verdict.append(f"- 角速度：实测破启 **{f(a1['ang_cmd'])}** vs 生产 lift "
                               f"`min_stationary_angular_z={msa}` → 余量 {gap:+.3f} rad/s "
                               f"({'偏保守' if gap > 0.05 else '偏紧/可能不够' if gap < 0 else '合适'})")
            if verdict:
                L.append("\n**对照 cmd.yaml 生产 deadzone-lift：**\n" + "\n".join(verdict))
        L.append("")
    else:
        L.append("_未采集（a1 未选或扫描失败）_\n")

    # A2
    L.append("## 2. A2 — 命令→实速 增益（量化 P 控饱和）")
    for title, summ, axis, label, unit in [
        ("### 2.1 线速度 (linear.x)", a2_lin, "linear", "A2_lin", "m/s"),
        ("### 2.2 角速度 (angular.z)", a2_ang, "angular", "A2_ang", "rad/s"),
    ]:
        L.append(title)
        if not summ:
            L.append("_未采集_\n")
            continue
        rows = []
        for s in summ.get("segments", []):
            tw, ss = s["target_w"], s["ss_mean"]
            gain = (ss / tw) if (ss is not None and tw) else None
            row = [f(tw), f(ss), f(gain, 3), f(s["ss_std"], 3),
                   "✓" if responded(tw, ss) else "✗"]
            if axis == "linear":
                row.append(f(segment_veer(vle_dir, label, s["name"]), 3))
            rows.append(row)
        head = [f"命令 ({unit})", f"实速 ({unit})", "增益", "std", "响应"]
        if axis == "linear":
            head.append("跑偏 wz(rad/s)")
        L.append(md_table(head, rows))
        pos = [s for s in summ.get("segments", [])
               if s["ss_mean"] is not None and s["target_w"] > 0]
        if pos:
            hi = max(pos, key=lambda s: s["target_w"])
            g_hi = hi["ss_mean"] / hi["target_w"]
            L.append(f"\n→ 最高命令档 {f(hi['target_w'])} {unit} 增益 = **{g_hi:.2f}**"
                     f"（越低说明 P 控稳态饱和越重）。")
        if axis == "linear":
            L.append("\n→ 跑偏列 = 纯平移时 `/odom.angular.z`，非零即左右轮不对称代理"
                     "（无单轮编码器，只能这样看）。")
        L.append("")

    # A4
    L.append("## 3. A4 — 阶跃/动态响应")
    if a4:
        rows = []
        for s in a4.get("segments", []):
            tw, ss = s["target_w"], s["ss_mean"]
            gain = (ss / tw) if (ss is not None and tw) else None
            rows.append([s["name"], s.get("_axis", ""), f(tw),
                         f(s["start_latency"]), f(s["stop_latency"]),
                         f(ss), f(gain, 3)])
        L.append(md_table(
            ["段", "轴", "命令", "起步延迟(s)", "停车延迟(s)", "实速", "增益"], rows))
        L.append("\n→ 起步延迟≈死时间+上升沿（定起步脉冲上升沿）；"
                 "停车延迟≈惯性滑行（定提前刹车量）。\n")
    else:
        L.append("_未采集_\n")

    L.append("## 4. 产出去向")
    L.append("- A1 → 校核/refine `cmd.yaml` 的 `min_linear_x` / `min_stationary_angular_z`")
    L.append("- A2 → 命令反向补偿表（替代不可行的力矩前馈）；高端增益 = P 控饱和程度")
    L.append("- A4 → 轨迹加速度可行域、起步脉冲上升沿、提前刹车量\n")
    L.append("## 5. 本轮盲区（不可静默外推）")
    L.append("- **2×2 m**：线速度只测到低速档，高速加减速与拖尾未覆盖")
    L.append("- **无单轮编码器**：左右不对称仅以纯平移跑偏 `wz` 间接观测")
    L.append("- **未用 IMU**：判定纯靠轮速计；轮子打滑时 `/odom` 会高估实际位移")
    L.append("- **A3 惰行不可做**；**B 组**（脚轮拖尾/曲率）需激光定位真值，本轮未做\n")
    L.append(f"---\n_原始数据：`{out_dir}`（vle CSV/JSON + 日志）_")
    return "\n".join(L)


# ----------------------------- PI 扫描（人在回路） -----------------------------
def _avg(mp, mn, key):
    vals = [m.get(key) for m in (mp, mn) if m and m.get(key) is not None]
    return sum(vals) / len(vals) if vals else None


def pi_score(sse, ov, settle, rev, osc):
    """启发式评分（越低越好）：稳态误差 + 超调 + 整定×20 + 反冲×120 + 振荡×6。"""
    if sse is None:
        return None
    return (abs(sse) + max(0.0, ov or 0.0) + 20.0 * (settle if settle is not None else 1.5)
            + 120.0 * (rev or 0.0) + 6.0 * (osc or 0.0))


def parse_kpki(label):
    """从标签里抠出 Kp / Ki 数字，如 'Kp600_Ki40' -> ('600','40')。"""
    kp = re.search(r"kp[_\-]?(\d+\.?\d*)", label, re.I)
    ki = re.search(r"ki[_\-]?(\d+\.?\d*)", label, re.I)
    return (kp.group(1) if kp else "-", ki.group(1) if ki else "-")


def build_pi_report(results, axis, args):
    unit = "rad/s" if axis == "angular" else "m/s"
    L = [f"# PI 整定对比报告（{axis} 阶跃, 目标 {args.pi_target} {unit}）\n",
         f"> 流程：每组 PI 由人工 flash 进 STM32 → 长 hold({args.pi_hold}s) 阶跃 → 取 pos/neg 均值。\n"
         "> 评分(越低越好) = 稳态误差% + 超调% + 整定s×20 + 反冲×120 + 稳态振荡×6。\n"
         "> 增量式PI：Kp 作用在 Δ误差≈−Δ速度(噪声)→高 Kp 放大速度噪声致抖动/振荡；"
         "Ki·e 提供跟踪/消稳差。反冲=停车主动制动过冲(非 windup)。\n"]
    hold = float(args.pi_hold)
    rows, metrics = [], []
    for r in results:
        mp, mn = r["pos"], r["neg"]
        kp, ki = parse_kpki(r["label"])
        sse = _avg(mp, mn, "sse_pct"); ov = _avg(mp, mn, "overshoot_pct")
        rise = _avg(mp, mn, "rise_s"); st = _avg(mp, mn, "settle_s")
        rev = _avg(mp, mn, "reversal"); osc = _avg(mp, mn, "osc_crossings")
        sc = pi_score(sse, ov, st, rev, osc)
        # 整定时间逼近 hold = 根本没整定（持续振荡）
        never = (st is not None and st >= 0.8 * hold)
        ok = (ov is not None and ov < 25 and not never
              and osc is not None and osc < 2)
        metrics.append({"label": r["label"], "ov": ov, "st": st, "osc": osc,
                        "never": never, "ok": ok, "score": sc})
        st_disp = (f(st, 2) + ("⚠未整定" if never else "")) if st is not None else "n/a"
        rows.append([r["label"], kp, ki, f(sse, 1), f(ov, 1), f(rise, 2), st_disp,
                     f(rev, 3), f(osc, 1), f(sc, 1) if sc is not None else "n/a"])
    L.append(md_table(["PI 组", "Kp", "Ki", "稳态误差%", "超调%", "上升s", "整定s",
                       "反冲", "振荡", "评分"], rows))

    passed = [m for m in metrics if m["ok"]]
    valid = [m for m in metrics if m["score"] is not None]
    if passed:
        best = min(passed, key=lambda m: m["score"])
        L.append(f"\n**推荐：`{best['label']}`** —— 通过门槛(超调<25%、能整定、不振荡)中评分最低。")
    elif valid:
        worst_ov = min(valid, key=lambda m: (m["ov"] if m["ov"] is not None else 1e9))
        L.append(f"\n**⚠ 全部不合格**：没有一组通过门槛（超调<25% 且能整定 且不振荡）。")
        L.append(f"所有组都在**持续极限环振荡**（整定时间≈hold、超调极大）。相对最不烂的是 "
                 f"`{worst_ov['label']}`，但仍不可用。")
        L.append("\n说明这套速度环在当前增益区间根本稳不住，下一步二选一：")
        L.append("- (a) **继续往下降 Kp**（远低于已测最小值），看振荡是否消失；")
        L.append("- (b) 直接转**每轮位置(增量)控制** —— 增量式 Kp 放大速度噪声，"
                 "位置环不求导，从结构上消除该振荡。")
    L.append("\n判读提示（增量式 PI）：")
    L.append("- 持续振荡/整定不了 → **Kp 偏高**（放大速度噪声），优先大幅降 Kp")
    L.append("- 反冲>0 → 停车主动制动过冲，降 Kp/Ki 或加减速限幅（非 windup，不用动抗饱和）")
    L.append("- 稳态误差大但不抖 → **Ki 偏低**，往上加")
    L.append("- “超调列”在重载+stick-slip 下多为挣脱顿挫的尖峰，需结合波形看")
    return "\n".join(L)


def run_pi_sweep(args, out_dir, vle_dir, do_prompt):
    labels = [s.strip() for s in args.pi_sweep.split(",") if s.strip()]
    axis = args.pi_axis
    print(f"\nPI 扫描：{len(labels)} 组 × ({axis} 阶跃 target={args.pi_target} hold={args.pi_hold}s)")
    print("每组流程：你把 PI flash 进 STM32 → 回车 → 我跑阶跃并评分。\n")
    results = []
    for lab in labels:
        prompt(f"【PI = {lab}】把电机 PI 设成该组、flash 完成、确认底盘已重启生效；"
               f"机器人复位中央、预置脚轮", do_prompt)
        label = ("PI_" + lab).replace(" ", "").replace("/", "_")
        cmd = ["ros2", "run", PKG, "wheeltec_velocity_loop_eval",
               "--mode", "step", "--axis", axis, "--cmd-topic", CMD_TOPIC,
               "--odom-topic", ODOM_TOPIC, "--target-w", str(args.pi_target),
               "--hold-seconds", str(args.pi_hold), "--rest-seconds", str(args.rest_sec),
               "--output-dir", vle_dir, "--label", label]
        if args.dry_run:
            print("DRY-RUN:", " ".join(cmd))
            continue
        run_streaming(cmd, os.path.join(out_dir, f"{label}.log"), args.seg_timeout)
        mp = analyze_step(os.path.join(vle_dir, f"{label}_pos.csv"), axis=axis)
        mn = analyze_step(os.path.join(vle_dir, f"{label}_neg.csv"), axis=axis)
        results.append({"label": lab, "pos": mp, "neg": mn})
        if mp:
            print(f"  → {lab}: SSE={f(mp.get('sse_pct'),1)}% 超调={f(mp.get('overshoot_pct'),1)}% "
                  f"整定={f(mp.get('settle_s'),2)}s 反冲={f(mp.get('reversal'),3)} "
                  f"振荡={mp.get('osc_crossings')}")
    if args.dry_run:
        return
    rep = build_pi_report(results, axis, args)
    path = os.path.join(out_dir, "pi_tuning_report.md")
    with open(path, "w") as fh:
        fh.write(rep)
    print(f"\n\033[32mPI 整定报告：{path}\033[0m")


def build_dz_report(rows, breakcmd, axis, unit, precond_mode, lift):
    L = [f"# 速度死区报告（{axis}, 脚轮 precond={precond_mode}）\n",
         f"> 每档**先自动 precond** 把脚轮甩到 **{precond_mode}**，再**单方向**阶跃测——"
         "避免来回扫描让脚轮每段翻向（这是 amp 来回扫的硬伤）。\n"
         "> peak=该档 /odom 峰值速度，超运动地板即判“动”=已破启。\n"]
    L.append(md_table([f"命令({unit})", "峰值", "结果"], rows))
    if breakcmd is not None:
        L.append(f"\n**破启命令 ≈ {breakcmd} {unit}**（{precond_mode} 脚轮下能挣脱的最低命令）")
    else:
        L.append("\n**所有档都没动** → 死区 > 最大测试档，提高 --dz-amps 再测")
    if lift:
        key = "min_linear_x" if axis == "linear" else "min_stationary_angular_z"
        val = lift.get(key)
        if breakcmd is not None and val is not None:
            L.append(f"\n对照 cmd.yaml `{key}={val}`：余量 {val - breakcmd:+.3f} {unit}")
    return "\n".join(L)


# ----------------------------- 主流程 -----------------------------
def main():
    ap = argparse.ArgumentParser(description="A 组一键辨识 + md 报告 (2x2m, 仅轮速计)")
    ap.add_argument("--out", default=os.path.expanduser("~/a_group_results"))
    ap.add_argument("--tests", default="a1,a2,a4", help="逗号分隔: a1,a2,a4")
    ap.add_argument("--no-prompt", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    # 扫描档位：低端给 A1 死区，全段给 A2 增益
    # 注意：重载底盘有大死时间+慢加速，hold 必须够长才能到稳态（实测 1s 远不够）。
    # 2x2 限制：线速度只在低档(<=~0.12)能在长 hold 内到稳态又不撞墙；高速档稳态测不了。
    ap.add_argument("--amps-lin", default="0.04,0.06,0.08,0.10,0.12")
    ap.add_argument("--amps-ang", default="0.05,0.10,0.15,0.20,0.30,0.40,0.50")
    ap.add_argument("--hold-lin", type=float, default=3.0)  # 重载需 >=3s 才接近稳态
    ap.add_argument("--hold-ang", type=float, default=4.0)  # 旋转死时间更大，原地不占地可放长
    ap.add_argument("--rest-sec", type=float, default=2.0)  # >=2s 让底盘完全停，避免段间耦合
    ap.add_argument("--a4-lin-target", type=float, default=0.20)
    ap.add_argument("--a4-ang-target", type=float, default=0.40)
    # 运动地板（轮速计判定阈值，沿用 micro_eval 的 odom 阈值）
    ap.add_argument("--lin-floor", type=float, default=0.015)
    ap.add_argument("--ang-floor", type=float, default=0.03)
    # 自动预置脚轮（机器人自己甩，因 70kg 推不动）
    ap.add_argument("--precond", choices=["aligned", "adverse", "none"], default="none",
                    help="扫描前自动驱动一记把脚轮甩到 顺向/逆向；aligned=与待测方向同向先动")
    ap.add_argument("--precond-dur", type=float, default=1.3, help="预置动作持续秒")
    ap.add_argument("--precond-lin", type=float, default=0.30, help="预置线速度 m/s")
    ap.add_argument("--precond-ang", type=float, default=0.60, help="预置角速度 rad/s")
    # 脚轮可控的死区扫描：每档先 precond，单方向测，避免来回让脚轮翻向
    ap.add_argument("--dz-axis", choices=["angular", "linear"], default="angular")
    ap.add_argument("--dz-amps", default=None, help="死区档位，逗号分隔；默认按轴取")
    ap.add_argument("--seg-timeout", type=float, default=300.0)
    # PI 扫描（人在回路：你设 PI→flash，脚本跑阶跃→评分）
    ap.add_argument("--pi-sweep", metavar="LABELS",
                    help="逗号分隔 PI 组标签，如 'Kp300,Kp600,Kp900,Kp1200'；逐组提示你设值")
    ap.add_argument("--pi-axis", choices=["angular", "linear"], default="angular",
                    help="PI 扫描用哪个轴（默认 angular：原地、不占地、最干净）")
    ap.add_argument("--pi-target", type=float, default=0.3)
    ap.add_argument("--pi-hold", type=float, default=4.0)
    ap.add_argument("--analyze-step", metavar="CSV",
                    help="只分析一段阶跃 CSV 的 PI 整定指标后退出")
    ap.add_argument("--axis", choices=["linear", "angular"], default=None,
                    help="配合 --analyze-step 指定轴（默认自动选）")
    args = ap.parse_args()

    if args.analyze_step:
        m = analyze_step(args.analyze_step, axis=args.axis)
        if not m:
            sys.exit("无法分析该 CSV")
        print(f"目标={f(m.get('target'))}  峰值={f(m.get('vmax'))}  稳态={f(m.get('steady'))}")
        print(f"稳态误差={f(m.get('sse_pct'),1)}%  超调={f(m.get('overshoot_pct'),1)}%  "
              f"上升={f(m.get('rise_s'),2)}s  整定={f(m.get('settle_s'),2)}s")
        print(f"停车反冲={f(m.get('reversal'),3)}  稳态区过零次数={m.get('osc_crossings')}")
        return

    tests = {t.strip().lower() for t in args.tests.split(",") if t.strip()}
    do_prompt = not args.no_prompt
    if not shutil.which("ros2"):
        sys.exit("找不到 ros2，请先 source ~/ros2_ws/install/setup.bash")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(args.out, stamp)
    vle_dir = os.path.join(out_dir, "vle")
    os.makedirs(vle_dir, exist_ok=True)
    floors = {"lin": args.lin_floor, "ang": args.ang_floor}

    if not args.dry_run:
        topics = list_topics()
        if ODOM_TOPIC not in topics:
            print(f"\033[31m警告：{ODOM_TOPIC} 不在话题列表，底盘可能没起。\033[0m")
            prompt("确认底盘已起再继续", do_prompt)

    meta = {"time": stamp, "operator": ""}
    if do_prompt and not args.dry_run:
        try:
            meta["operator"] = input("操作员/地面/负载备注（可空）: ").strip()
        except EOFError:
            pass

    print("\n确认：① 70kg 上肢装好锁死 ② 机器人在场地中央朝对角线 ③ 已 source 工作空间")
    meta["volt_start"] = None if args.dry_run else read_voltage()
    print(f"起始电压: {f(meta['volt_start'],2)} V")

    # PI 扫描模式：独立流程，跑完即退
    if args.pi_sweep:
        run_pi_sweep(args, out_dir, vle_dir, do_prompt)
        if not args.dry_run:
            print(f"结束电压: {f(read_voltage(),2)} V")
        return

    a1 = a2_lin = a2_ang = a4 = None
    lift = load_cmd_yaml_lift()

    def maybe_run(cmd, log, timeout):
        if args.dry_run:
            print("DRY-RUN:", " ".join(cmd))
            return 0, ""
        return run_streaming(cmd, log, timeout)

    def precond(axis, mode):
        """扫描前用一记明显高于死区的命令把脚轮甩到已知朝向（机器人自驱，不用手推）。

        aligned = 朝待测正方向先动一下 → 脚轮甩到对该方向顺的位置；
        adverse = 朝反方向先动 → 脚轮指向相反，待测时需翻 180°（最坏情况）。
        """
        if mode == "none":
            return
        spd = args.precond_lin if axis == "linear" else args.precond_ang
        val = spd if mode == "aligned" else -spd
        field = ("linear: {x: %.3f}" % val) if axis == "linear" \
            else ("angular: {z: %.3f}" % val)
        msg = "{%s}" % field
        print(f"\033[35m[precond {axis} {mode}] {field} 持续 {args.precond_dur}s\033[0m")
        if args.dry_run:
            print("DRY-RUN precond:", msg)
            return
        # 持续发布 dur 秒（timeout 杀掉），再发一次 0 停车
        subprocess.run(["timeout", str(args.precond_dur), "ros2", "topic", "pub",
                        "-r", "20", CMD_TOPIC, "geometry_msgs/msg/Twist", msg],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["ros2", "topic", "pub", "--once", CMD_TOPIC,
                        "geometry_msgs/msg/Twist", "{}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.6)

    def amp_run(axis, amps, hold, label):
        cmd = ["ros2", "run", PKG, "wheeltec_velocity_loop_eval",
               "--mode", "amp", "--axis", axis,
               "--cmd-topic", CMD_TOPIC, "--odom-topic", ODOM_TOPIC,
               "--amplitudes", amps, "--hold-seconds", str(hold),
               "--rest-seconds", str(args.rest_sec), "--output-dir", vle_dir, "--label", label]
        maybe_run(cmd, os.path.join(out_dir, f"{label}.log"), args.seg_timeout)
        return None if args.dry_run else load_vle_summary(vle_dir, label)

    # ---- DZ: 脚轮可控的死区扫描（每档先 precond，单方向，避免来回翻向）----
    if "dz" in tests:
        dzaxis = args.dz_axis
        floor = args.lin_floor if dzaxis == "linear" else args.ang_floor
        unit = "m/s" if dzaxis == "linear" else "rad/s"
        default_amps = ("0.03,0.05,0.07,0.09,0.12,0.15" if dzaxis == "linear"
                        else "0.05,0.08,0.11,0.15,0.20,0.30")
        amps = [float(x) for x in (args.dz_amps or default_amps).split(",") if x.strip()]
        hold = args.hold_lin if dzaxis == "linear" else args.hold_ang
        dz_rows, dz_break = [], None
        for amp in amps:
            prompt(f"【死区档 {amp} {unit}】回车后自动 precond({args.precond})+单方向阶跃", do_prompt)
            precond(dzaxis, args.precond)
            label = f"DZ_{args.precond}_{amp:.3f}"
            cmd = ["ros2", "run", PKG, "wheeltec_velocity_loop_eval", "--mode", "step",
                   "--axis", dzaxis, "--cmd-topic", CMD_TOPIC, "--odom-topic", ODOM_TOPIC,
                   "--target-w", str(amp), "--hold-seconds", str(hold),
                   "--rest-seconds", str(args.rest_sec), "--output-dir", vle_dir,
                   "--label", label]
            if args.dry_run:
                print("DRY-RUN:", " ".join(cmd)); continue
            run_streaming(cmd, os.path.join(out_dir, f"{label}.log"), args.seg_timeout)
            mp = analyze_step(os.path.join(vle_dir, f"{label}_pos.csv"), axis=dzaxis)
            vmax = mp.get("vmax") if mp else None
            moved = vmax is not None and vmax >= floor
            dz_rows.append([f"{amp:.3f}", f(vmax, 3), "✓动" if moved else "✗停"])
            print(f"  → {amp} {unit}: peak={f(vmax,3)} {'动' if moved else '停'}")
            if moved and dz_break is None:
                dz_break = amp
        if not args.dry_run:
            rep = build_dz_report(dz_rows, dz_break, dzaxis, unit, args.precond, lift)
            with open(os.path.join(out_dir, "dz_report.md"), "w") as fh:
                fh.write(rep)
            print(f"\n\033[32m死区报告：{os.path.join(out_dir, 'dz_report.md')}\033[0m")
            print(f"结束电压: {f(read_voltage(),2)} V")
        return

    # ---- A1 + A2 共用 amp 扫描 ----
    if tests & {"a1", "a2"}:
        prompt("【线速度扫描 (A1死区+A2增益)】复位中央，确认前方净空（precond 会自动甩脚轮）", do_prompt)
        precond("linear", args.precond)
        a2_lin = amp_run("linear", args.amps_lin, args.hold_lin, "A2_lin")
        prompt("【角速度扫描 (A1死区+A2增益)】确认四周净空（precond 会自动甩脚轮）", do_prompt)
        precond("angular", args.precond)
        a2_ang = amp_run("angular", args.amps_ang, args.hold_ang, "A2_ang")
        if "a1" in tests and not args.dry_run:
            lc, ls = deadzone_from_summary(a2_lin, args.lin_floor)
            ac, as_ = deadzone_from_summary(a2_ang, args.ang_floor)
            a1 = {"lin_cmd": lc, "lin_ss": ls, "ang_cmd": ac, "ang_ss": as_}
        if "a2" not in tests:  # 只要 a1：报告里不展开增益表
            a2_lin = a2_ang = None

    # ---- A4 阶跃 ----
    if "a4" in tests:
        prompt("【A4 线速度阶跃】复位中央、预置脚轮", do_prompt)
        cmd = ["ros2", "run", PKG, "wheeltec_velocity_loop_eval",
               "--mode", "step", "--axis", "linear", "--cmd-topic", CMD_TOPIC,
               "--odom-topic", ODOM_TOPIC, "--target-w", str(args.a4_lin_target),
               "--hold-seconds", str(args.hold_lin), "--rest-seconds", str(args.rest_sec),
               "--output-dir", vle_dir, "--label", "A4_lin"]
        maybe_run(cmd, os.path.join(out_dir, "A4_lin.log"), args.seg_timeout)
        a4_lin = load_vle_summary(vle_dir, "A4_lin") if not args.dry_run else None

        prompt("【A4 角速度阶跃】原地旋转", do_prompt)
        cmd = ["ros2", "run", PKG, "wheeltec_velocity_loop_eval",
               "--mode", "step", "--axis", "angular", "--cmd-topic", CMD_TOPIC,
               "--odom-topic", ODOM_TOPIC, "--target-w", str(args.a4_ang_target),
               "--hold-seconds", str(args.hold_ang), "--rest-seconds", str(args.rest_sec),
               "--output-dir", vle_dir, "--label", "A4_ang"]
        maybe_run(cmd, os.path.join(out_dir, "A4_ang.log"), args.seg_timeout)
        a4_ang = load_vle_summary(vle_dir, "A4_ang") if not args.dry_run else None

        if a4_lin or a4_ang:
            segs = []
            for ax, s in (("linear", a4_lin), ("angular", a4_ang)):
                if s:
                    for seg in s.get("segments", []):
                        seg = dict(seg)
                        seg["_axis"] = ax
                        segs.append(seg)
            a4 = {"segments": segs}

    meta["volt_end"] = None if args.dry_run else read_voltage()
    print(f"结束电压: {f(meta['volt_end'],2)} V")

    if args.dry_run:
        print("\n[dry-run] 不生成报告。")
        return

    report = build_report(meta, a1, a2_lin, a2_ang, a4, lift, out_dir, vle_dir, floors)
    report_path = os.path.join(out_dir, "report.md")
    with open(report_path, "w") as fh:
        fh.write(report)
    print(f"\n\033[32m报告已生成：{report_path}\033[0m")


if __name__ == "__main__":
    main()
