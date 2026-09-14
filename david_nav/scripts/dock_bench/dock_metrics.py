#!/usr/bin/env python3
"""dock_metrics —— 把 N 轮入位汇总成五张表(markdown)。无 ROS 依赖，离线跑。

读：
  runs_dock.csv          run_dock_bench.py 每轮追加的一行（判词/终点/时间线）
  track_<session>.csv    record_track.py 的轨迹 + cmd_vel（阶段2 指标的原料）
  dock_case.yaml         goal / 容差（理想入位直线的基准）

五个维度：
  1 可靠性   成功率 + 失败分类 + Spin 次数分布 + INVALID（全程）
  2 入位精度 终点 vs goal：bias(系统性) / spread(随机) 分开 + CEP + 达标率
  3 稳定性   终点围绕质心的散布 —— 重复性
  4 贴合度   ★阶段2 专属：vs「过 G、方向 yaw_G」的理想入位直线的有符号横向偏差
  5 流畅性   ★阶段2 专属：停顿/蛇行/抖动/最低速度/耗时

★ 除 1 之外全部只在 [t_stage2_start, t_verdict] 窗口内计算 —— 评测重心是最后 0.5m，
  阶段1 怎么绕过来的不参与打分。

用法:
  python3 dock_metrics.py [--session dock_20260909_0312] [--out report.md]
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from collections import Counter

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'controller_bench'))

from dock_common import (jitter_rms, median, remaining_dist,  # noqa: E402
                         sign_reversals, signed_cross_track, stop_events, wrap)

try:
    # 复用 controller_bench 已验证的终点统计（bias/spread/CEP），
    # 保证入位精度这张表和 alias 8 的口径可比。
    from bench_common import endpoint_stats, rms  # noqa: E402
except ImportError as e:  # pragma: no cover
    raise SystemExit(
        f'导入 controller_bench/bench_common.py 失败: {e}\n'
        '本脚本要和 controller_bench 保持同一套终点统计口径，'
        '请确认 scripts/controller_bench/bench_common.py 存在。')


def _f(x, default=float('nan')):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def _fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return 'n/a'
    return f'{v:.{nd}f}'


def _deg(rad, nd=1):
    if rad is None or (isinstance(rad, float) and math.isnan(rad)):
        return 'n/a'
    return f'{math.degrees(rad):.{nd}f}'


# ------------------------------------------------------------------ 载入


def load_runs(path, session=None):
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if session:
        rows = [r for r in rows if r.get('session') == session]
    return rows


def _pick_cmd_cols(fieldnames):
    """cmd_vel 列名在 record_track 里按话题名派生。优先用最终下发给底盘的
    /cmd_vel（真正执行的那一条），退回 controller 原始输出。"""
    for base in ('cmd_vel', 'cmd_vel_smoothed', 'cmd_vel_nav'):
        if f'{base}_v' in fieldnames and f'{base}_w' in fieldnames:
            return f'{base}_v', f'{base}_w'
    return None, None


def load_track(path):
    """返回 (rows, v_col, w_col)。rows 每项 dict，已转 float 的常用列。"""
    if not os.path.exists(path):
        return [], None, None
    with open(path, newline='', encoding='utf-8') as f:
        rd = csv.DictReader(f)
        v_col, w_col = _pick_cmd_cols(rd.fieldnames or [])
        rows = []
        for r in rd:
            rows.append({
                't': _f(r.get('t_unix')),
                'x': _f(r.get('actual_x')), 'y': _f(r.get('actual_y')),
                'yaw': _f(r.get('actual_yaw')),
                'ok': r.get('tf_ok') in ('1', '1.0'),
                'v': _f(r.get(v_col)) if v_col else float('nan'),
                'w': _f(r.get(w_col)) if w_col else float('nan'),
            })
    return rows, v_col, w_col


def slice_stage2(track, run):
    """截出本轮阶段2 的窗口 [t_stage2_start, t_verdict]。"""
    t0, t1 = _f(run.get('t_stage2_start')), _f(run.get('t_verdict'))
    if math.isnan(t0) or math.isnan(t1) or t1 <= t0:
        return []
    return [r for r in track if t0 <= r['t'] <= t1]


# ------------------------------------------------------------------ 报告


def build_report(runs, track_dir, case, session_label):
    goal0 = case['goals'][0]
    gx, gy = float(goal0['x']), float(goal0['y'])
    gyaw = math.radians(float(goal0['yaw_deg']))
    xy_ok = float(case['tolerances']['xy_ok'])
    yaw_ok = math.radians(float(case['tolerances']['yaw_ok_deg']))

    L = []
    L.append(f'# dock_bench 精准入位评测报告 — {session_label}')
    L.append('')
    L.append(f'- 目标 `{goal0["id"]}`: ({gx:.3f}, {gy:.3f}) yaw={goal0["yaw_deg"]:.1f}°'
             f'，接近距离 {case.get("approach_dist", 0.5)} m')
    L.append(f'- 总轮次: {len(runs)}')
    L.append(f'- 达标阈值: xy ≤ {xy_ok:.2f} m 且 |yaw| ≤ {case["tolerances"]["yaw_ok_deg"]:.1f}°')
    L.append('- **真值 = lightning 的 `map->base_footprint`**。lightning 既是被测系统的'
             '定位器又是本报告唯一的尺子，所以下列全部是**系统自洽误差**，'
             '不是对地面的绝对精度；lightning 整体漂移多少，这套指标看不见。')
    L.append('')

    # ---------------- 1 可靠性 ----------------
    counts = Counter(r.get('verdict_class', '?') for r in runs)
    n_total = len(runs)
    n_invalid = counts.get('INVALID', 0)
    n_valid = n_total - n_invalid
    n_ok = counts.get('SUCCESS', 0)
    L.append('## 1. 可靠性（全程）')
    L.append('')
    if n_valid > 0:
        L.append(f'- **成功率: {n_ok}/{n_valid} = {100.0 * n_ok / n_valid:.0f}%**'
                 f'（分母已剔除 {n_invalid} 轮 INVALID）')
    else:
        L.append('- 无有效轮次')
    L.append('')
    L.append('| 结果 | 次数 | 说明 |')
    L.append('|---|---|---|')
    desc = {
        'SUCCESS': '入位达标',
        'STAGE1_FAILED': '阶段1 没到预备点，根本没进入位段',
        'XY_OUT_OF_TOL': '位置超差，按约定不再下发位移目标',
        'YAW_NOT_CONVERGED': 'xy 达标但 yaw 经 Spin 补偿仍未收敛',
        'POSE_LOOKUP_FAILED': '取不到 TF，无法判定（定位/TF 问题，不是控制问题）',
        'NODE_EXCEPTION': 'dock_navigator 编排线程抛异常',
        'NOT_SETTLED': '等不到车停稳，按约定不做补偿（是"没测成"，不是"没到位"）',
        'TIMEOUT': '窗口内没等到任何判词',
        'UNKNOWN_FAILURE': '判词无法归类 —— 检查 dock_navigator 是否改过日志',
        'INVALID': '回程没回到起点容差内，本轮作废',
    }
    for k, c in counts.most_common():
        L.append(f'| {k} | {c} | {desc.get(k, "")} |')
    L.append('')

    valid = [r for r in runs if r.get('verdict_class') != 'INVALID']
    spins = Counter(r.get('n_spin', '') for r in valid)
    L.append('**Spin 纯旋转补偿次数分布**（末端 yaw 收敛能力的直接体现）:')
    L.append('')
    L.append('| Spin 次数 | 轮数 |')
    L.append('|---|---|')
    for k in sorted(spins, key=lambda s: (s == '', s)):
        L.append(f'| {k or "n/a"} | {spins[k]} |')
    L.append('')

    ok_runs = [r for r in runs if r.get('verdict_class') == 'SUCCESS']
    if not ok_runs:
        L.append('_无成功轮次，精度/稳定性/贴合度/流畅性无法统计。_')
        return '\n'.join(L)

    # ---------------- 2 精度 + 3 稳定性 ----------------
    ends = [(_f(r['end_x']), _f(r['end_y']), _f(r['end_yaw'])) for r in ok_runs]
    ends = [e for e in ends if not any(math.isnan(v) for v in e)]
    st = endpoint_stats(ends, (gx, gy, gyaw))

    # ★达标率必须算在**所有有效轮**上，不能只算成功轮 —— 只算成功轮是同义反复
    #   (dock_navigator 正是"在容差内"才报成功)，会把 XY_OUT_OF_TOL / TIMEOUT
    #   这些真正没到位的轮次排除在分母外，得出永远 100% 的假指标。
    #   这里一律用 bench 自己从 TF 复算的 exy/eyaw，不采信节点自评。
    n_pass = sum(1 for r in valid
                 if _f(r['exy']) <= xy_ok and abs(_f(r['eyaw'])) <= yaw_ok)
    L.append(f'## 2. 入位精度')
    L.append('')
    L.append(f'达标率的分母是**全部 {len(valid)} 个有效轮**（含失败轮）；'
             f'bias/spread 等分布量沿用 controller_bench 的口径，'
             f'只在 {len(ends)} 个成功轮上统计（把失败轮混进去会污染重复性）。')
    L.append('')
    L.append('| 量 | 值 | 说明 |')
    L.append('|---|---|---|')
    L.append(f'| **达标率** | {n_pass}/{len(valid)}'
             f' = {100.0 * n_pass / len(valid):.0f}% | bench 复算：'
             f'xy≤{xy_ok:.2f}m 且 |yaw|≤{case["tolerances"]["yaw_ok_deg"]:.0f}° |')
    L.append(f'| bias 位置 | {_fmt(st["bias_pos"])} m '
             f'({_fmt(st["bias_x"])}, {_fmt(st["bias_y"])}) | 系统性偏移，可标定/调参消 |')
    L.append(f'| bias yaw | {_deg(st["bias_yaw"])}° | 系统性朝向偏 |')
    L.append(f'| 位置误差 均值/最大 | {_fmt(st["err_pos_mean"])} / '
             f'{_fmt(st["err_pos_max"])} m | |')
    L.append(f'| yaw 误差 均值/最大 | {_deg(st["err_yaw_mean"])} / '
             f'{_deg(st["err_yaw_max"])}° | |')
    L.append('')
    L.append('> bias 大 → 调参可消；spread（见下表）大 → plant 随机性/脚轮摩擦，难消。')
    if n_pass != n_ok:
        L.append('>')
        L.append(f'> ⚠ **判定不一致**：节点报了 {n_ok} 轮成功，但 bench 复算只有 '
                 f'{n_pass} 轮真在容差内。两者取 TF 的时刻不同（节点在判词前取、'
                 f'bench 在判词后取，其间车可能还在动），或 TF 有滞后。'
                 f'差异持续存在就去看下面"节点自评 vs bench 复算"那一节。')
    L.append('')

    L.append(f'## 3. 稳定性（终点重复性，围绕质心，n={st["n"]}）')
    L.append('')
    L.append('| 量 | 值 |')
    L.append('|---|---|')
    L.append(f'| 质心 (x,y) | ({_fmt(st["centroid_x"])}, {_fmt(st["centroid_y"])}) m |')
    L.append(f'| std_x / std_y | {_fmt(st["std_x"])} / {_fmt(st["std_y"])} m |')
    L.append(f'| yaw 质心 / std | {_deg(st["centroid_yaw"])}° / {_deg(st["yaw_std"])}° |')
    L.append(f'| CEP50 / CEP95 | {_fmt(st["cep50"])} / {_fmt(st["cep95"])} m |')
    L.append('')
    if st['n'] < 30:
        L.append(f'> ⚠ n={st["n"]}<30：均值/std 可用，CEP95 尾部不可靠（冒烟级）。')
        L.append('')

    # ---------------- 4 贴合度 + 5 流畅性（阶段2）----------------
    L.append('## 4. 轨迹贴合度（★仅阶段2 的最后 0.5m）')
    L.append('')
    L.append('基准 = 过 goal、方向 goal_yaw 的**理想入位直线**（不是 P→G 线段：'
             '阶段1 落点本身有容差，用线段会把阶段1 的误差算到阶段2 头上）。'
             '横向偏差**带符号**，左正右负 —— 均值显著非零即"总偏一侧"的系统性误差。')
    L.append('')
    L.append('| 轮次 | 采样点 | 有符号均值 (m) | RMS (m) | 最大侵入 (m) |')
    L.append('|---|---|---|---|---|')
    all_rms, all_max, all_mean = [], [], []
    conv_all = []
    for r in ok_runs:
        track, _, _ = load_track(
            os.path.join(track_dir, f'track_{r["session"]}.csv'))
        seg = [s for s in slice_stage2(track, r) if s['ok']]
        if len(seg) < 2:
            L.append(f'| {r["iteration"]} | {len(seg)} | n/a | n/a | n/a |')
            continue
        ct = [signed_cross_track(s['x'], s['y'], gx, gy, gyaw) for s in seg]
        mean_ct = sum(ct) / len(ct)
        r_rms, r_max = rms(ct), max(abs(c) for c in ct)
        all_rms.append(r_rms)
        all_max.append(r_max)
        all_mean.append(mean_ct)
        conv_all.extend(
            (remaining_dist(s['x'], s['y'], gx, gy, gyaw), s['yaw']) for s in seg)
        L.append(f'| {r["iteration"]} | {len(seg)} | {mean_ct:+.3f} | '
                 f'{_fmt(r_rms)} | {_fmt(r_max)} |')
    L.append('')
    if all_rms:
        L.append(f'- **跨轮 RMS 均值 {_fmt(rms(all_rms))} m**，最大侵入 '
                 f'{_fmt(max(all_max))} m，有符号均值的均值 '
                 f'{sum(all_mean) / len(all_mean):+.3f} m')
        L.append('')

    if conv_all:
        L.append('**航向收敛曲线**（按沿线剩余距离分箱，|eyaw| 中位数）—— '
                 '看朝向是一路收敛过来的，还是全靠末端 Spin 硬掰：')
        L.append('')
        L.append('| 剩余距离 (m) | 采样点 | \\|eyaw\\| 中位数 (°) |')
        L.append('|---|---|---|')
        edges = (0.5, 0.4, 0.3, 0.2, 0.1, 0.0)
        for i in range(len(edges) - 1):
            hi, lo = edges[i], edges[i + 1]
            vals = [abs(math.degrees(wrap(y - gyaw)))
                    for s, y in conv_all if lo <= s < hi]
            L.append(f'| [{lo:.1f}, {hi:.1f}) | {len(vals)} | '
                     f'{_fmt(median(vals), 1) if vals else "n/a"} |')
        L.append('')

    L.append('## 5. 流畅性（★仅阶段2）')
    L.append('')
    L.append(f'口径与 `controller_bench/analyze_smoothness.py` 一致（停顿 = |cmd_v|<0.02 '
             f'持续≥0.25s），保证两套 bench 可比。**末端为收 yaw 而停是设计行为**，'
             f'所以停顿只统计进入终点容差圈（{xy_ok:.2f} m）之前的部分，圈内单列。')
    L.append('')
    L.append('| 轮次 | 阶段2 耗时 (s) | 圈外停顿 次/秒 | 圈内停顿 次 | '
             'w 符号反转 | w 抖动 RMS | 最低\\|v\\| (m/s) |')
    L.append('|---|---|---|---|---|---|---|')
    for r in ok_runs:
        track, v_col, _ = load_track(
            os.path.join(track_dir, f'track_{r["session"]}.csv'))
        seg = slice_stage2(track, r)
        if len(seg) < 2 or v_col is None:
            L.append(f'| {r["iteration"]} | {r.get("stage2_dur", "n/a")} | '
                     'n/a | n/a | n/a | n/a | n/a |')
            continue
        outer = [s for s in seg
                 if not s['ok'] or math.hypot(s['x'] - gx, s['y'] - gy) > xy_ok]
        inner = [s for s in seg if s not in outer]
        ev_out = stop_events([(s['t'], s['v']) for s in outer])
        ev_in = stop_events([(s['t'], s['v']) for s in inner])
        ws = [s['w'] for s in seg]
        vs = [abs(s['v']) for s in outer if s['v'] == s['v']]
        L.append(f'| {r["iteration"]} | {r.get("stage2_dur", "n/a")} | '
                 f'{len(ev_out)} / {sum(e[2] for e in ev_out):.1f} | {len(ev_in)} | '
                 f'{sign_reversals(ws)} | {_fmt(jitter_rms(ws))} | '
                 f'{_fmt(min(vs)) if vs else "n/a"} |')
    L.append('')
    L.append('> 圈外停顿 > 0 = 入位过程中停车再走，是要消灭的；最低|v| > 0 = 全程不停车入位。')
    L.append('')

    # 节点自评 vs bench 复算的一致性
    diffs = [abs(_f(r['exy']) - _f(r['node_exy'])) for r in ok_runs
             if r.get('node_exy') not in (None, '')]
    diffs = [d for d in diffs if d == d]
    if diffs:
        L.append('## 附：节点自评 vs bench 复算')
        L.append('')
        L.append(f'`dock_navigator` 自己报的 exy 与 bench 从 TF 复算的最大差 '
                 f'{_fmt(max(diffs))} m。差值显著（>0.01 m）说明两者取 TF 的时刻不同 '
                 f'—— 判词时刻车还在动，或 TF 有滞后。')
        L.append('')
    return '\n'.join(L)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--runs', default=os.path.expanduser(
        '~/david_nav_logs/dock_bench/runs_dock.csv'))
    p.add_argument('--track-dir',
                   default=os.path.expanduser('~/david_nav_logs/dock_bench'))
    p.add_argument('--case', default=os.path.join(HERE, 'dock_case.yaml'))
    p.add_argument('--session', default='')
    p.add_argument('--out', default='')
    args = p.parse_args()

    with open(args.case, encoding='utf-8') as f:
        case = yaml.safe_load(f)
    runs = load_runs(args.runs, args.session or None)
    if not runs:
        raise SystemExit(f'{args.runs} 里没有数据'
                         + (f'（session={args.session}）' if args.session else ''))
    label = args.session or runs[-1].get('session', 'all')
    report = build_report(runs, args.track_dir, case, label)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(report + '\n')
        print(f'report written: {args.out}')
    else:
        print(report)


if __name__ == '__main__':
    main()
