#!/usr/bin/env python3
"""bench_metrics — 把 N 次 run 汇总成 稳定性 / 精确性 / 跟踪 三张表(markdown)。

无 ROS 依赖,离线跑。读:
  runs.csv                        —— run_bench.py 每次追加的一行(终点 + 失败分类)
  track_<run_id>.csv              —— record_track.py 的实际轨迹(map/slam 真值)
  reference_path.yaml             —— 参考折线(cross-track 基准)

三个维度(对应用户最终范围,已去掉"轨迹合理性/planner 质量"):
  可靠性  : 成功率 + 失败类型计数(稳定性的第一层)
  稳定性  : 终点 x/y/yaw 围绕质心的散布(std, CEP50/95)—— 重复性
  精确性  : 终点相对 goal 的误差(bias 系统性 / spread 随机 分开)+ yaw
  跟踪    : 实际轨迹 vs 固定参考折线的 cross-track(RMS/max)—— controller 跟踪质量

用法:
  ./bench_metrics.py --runs /tmp/david_nav_logs/bench/runs.csv \
                     --track-dir /tmp/david_nav_logs/bench \
                     --reference reference_path.yaml \
                     [--controller position_controller] [--out report.md]
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from collections import Counter

from bench_common import cross_track_series, endpoint_stats, load_reference, rms


def _f(x, default=float('nan')):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def _load_runs(path, controller=None):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if controller:
        rows = [r for r in rows if r.get('controller') == controller]
    return rows


def _load_track_xy(track_csv):
    """返回 tf_ok==1 的 (x,y) 序列。文件缺失/空 -> []。"""
    if not os.path.exists(track_csv):
        return []
    out = []
    with open(track_csv, newline='') as f:
        for r in csv.DictReader(f):
            if r.get('tf_ok') in ('1', '1.0', 1) and r.get('actual_x') not in (None, ''):
                x, y = _f(r['actual_x']), _f(r['actual_y'])
                if not (math.isnan(x) or math.isnan(y)):
                    out.append((x, y))
    return out


def _fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return 'n/a'
    return f'{v:.{nd}f}'


def _deg(rad):
    if rad is None or (isinstance(rad, float) and math.isnan(rad)):
        return 'n/a'
    return f'{math.degrees(rad):.1f}'


def build_report(runs, track_dir, ref, controller_label):
    lines = []
    lines.append(f'# controller_bench 指标报告 — {controller_label}')
    lines.append('')
    lines.append(f'- 参考路径: `{ref.get("map_name", "?")}`,'
                 f' {len(ref["waypoints"])} waypoints(S1-S4 覆盖)')
    lines.append(f'- 总 run 数: {len(runs)}')
    lines.append('- 真值: slam_toolbox map->base TF(忽略定位漂移)')
    lines.append('')

    # ---- 可靠性 ----
    status_counts = Counter(r.get('overall_status', '?') for r in runs)
    n_total = len(runs)
    n_ok = status_counts.get('SUCCEEDED', 0)
    lines.append('## 1. 可靠性')
    lines.append('')
    lines.append(f'- **成功率: {n_ok}/{n_total}'
                 f' = {100.0 * n_ok / n_total:.0f}%**' if n_total else '- 无数据')
    lines.append('')
    lines.append('| 结果 | 次数 |')
    lines.append('|---|---|')
    for st, c in status_counts.most_common():
        lines.append(f'| {st} | {c} |')
    lines.append('')

    ok_runs = [r for r in runs if r.get('overall_status') == 'SUCCEEDED']
    if not ok_runs:
        lines.append('_无成功 run,稳定性/精确性/跟踪无法统计。_')
        return '\n'.join(lines)

    # ---- 稳定性 + 精确性(终点)----
    ends = [(_f(r['end_x']), _f(r['end_y']), _f(r['end_yaw'])) for r in ok_runs]
    # goal 取第一条成功 run 记录的(各 run 相同)
    g = ok_runs[0]
    goal = (_f(g['goal_x']), _f(g['goal_y']), _f(g['goal_yaw']))
    st = endpoint_stats(ends, goal)

    lines.append(f'## 2. 稳定性(终点重复性,n={st["n"]} 成功 run)')
    lines.append('')
    lines.append('| 量 | 值 |')
    lines.append('|---|---|')
    lines.append(f'| 质心 (x,y) | ({_fmt(st["centroid_x"])}, {_fmt(st["centroid_y"])}) m |')
    lines.append(f'| std_x / std_y | {_fmt(st["std_x"])} / {_fmt(st["std_y"])} m |')
    lines.append(f'| yaw 质心 / std | {_deg(st["centroid_yaw"])} / {_deg(st["yaw_std"])} deg |')
    lines.append(f'| CEP50 / CEP95(围绕质心) | {_fmt(st["cep50"])} / {_fmt(st["cep95"])} m |')
    lines.append('')
    if st['n'] < 30:
        lines.append(f'> ⚠ n={st["n"]}<30:均值/std 可用,CEP95 尾部不可靠(冒烟级)。')
        lines.append('')

    lines.append('## 3. 精确性(相对 goal)')
    lines.append('')
    lines.append('| 量 | 值 | 说明 |')
    lines.append('|---|---|---|')
    lines.append(f'| bias 位置 | {_fmt(st["bias_pos"])} m '
                 f'({_fmt(st["bias_x"])}, {_fmt(st["bias_y"])}) | 系统性偏移(可标定/调容差消) |')
    lines.append(f'| bias yaw | {_deg(st["bias_yaw"])} deg | 系统性朝向偏 |')
    lines.append(f'| 位置误差 均值/最大 | {_fmt(st["err_pos_mean"])} / {_fmt(st["err_pos_max"])} m | 每次终点到 goal |')
    lines.append(f'| yaw 误差 均值/最大 | {_deg(st["err_yaw_mean"])} / {_deg(st["err_yaw_max"])} deg | |')
    lines.append('')
    lines.append('> bias(系统性) vs spread(std/CEP,随机):bias 大→调参可消;'
                 'spread 大→plant 随机性/摩擦,难消。')
    lines.append('')

    # ---- 跟踪 ----
    lines.append('## 4. 跟踪性能(实际 vs 固定参考折线,cross-track)')
    lines.append('')
    lines.append('| run_id | 点数 | RMS (m) | max (m) |')
    lines.append('|---|---|---|---|')
    all_rms, all_max = [], []
    poly = ref['polyline']
    for r in ok_runs:
        rid = r['run_id']
        xy = _load_track_xy(os.path.join(track_dir, f'track_{rid}.csv'))
        if len(xy) < 2:
            lines.append(f'| {rid} | {len(xy)} | n/a | n/a |')
            continue
        ct = cross_track_series(xy, poly)
        r_rms, r_max = rms(ct), max(ct)
        all_rms.append(r_rms)
        all_max.append(r_max)
        lines.append(f'| {rid} | {len(xy)} | {_fmt(r_rms)} | {_fmt(r_max)} |')
    lines.append('')
    if all_rms:
        lines.append(f'- **跨 run RMS 均值: {_fmt(rms(all_rms))} m**,'
                     f' 最大 cross-track: {_fmt(max(all_max))} m')
        lines.append('')
    lines.append('> cross-track 基准是固定参考折线;position_controller 是 '
                 'turn-drive-turn,段内直线,拐点附近 cross-track 天然偏大属正常。')
    lines.append('')
    return '\n'.join(lines)


def main():
    p = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    p.add_argument('--runs', default='/tmp/david_nav_logs/bench/runs.csv')
    p.add_argument('--track-dir', default='/tmp/david_nav_logs/bench')
    p.add_argument('--reference', default=os.path.join(here, 'reference_path.yaml'))
    p.add_argument('--controller', default='')
    p.add_argument('--out', default='')
    args = p.parse_args()

    ref = load_reference(args.reference)
    runs = _load_runs(args.runs, args.controller or None)
    if not runs:
        raise SystemExit(f'no runs in {args.runs}'
                         + (f' for controller {args.controller}' if args.controller else ''))
    label = args.controller or (runs[0].get('controller', 'unknown'))
    report = build_report(runs, args.track_dir, ref, label)

    if args.out:
        with open(args.out, 'w') as f:
            f.write(report + '\n')
        print(f'report written: {args.out}')
    else:
        print(report)


if __name__ == '__main__':
    main()
