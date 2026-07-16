#!/usr/bin/env python3
"""analyze_smoothness — 离线量化一次 run 的"流畅度":中途停顿、蛇行、过弯速度。

动机(2026-07-08 iter3):用户核心诉求是"中途不停顿转向(除最后入位)、转弯丝滑"。
本工具把这两点变成可对比的数,不靠肉眼:

  ① 中途停顿事件:cmd_v<0.02 持续 ≥min-stop 秒的区段(运动窗口内),报位置/时长/
     段内 max|w|,并分类:拐角停(停-转-走,要消灭) / 终点区(允许) / 直线段停(异常)。
  ② 拐角通过:每个拐角 0.45m 半径内的逗留时间 + 最低 cmd_v(不停车过弯 => min_v>0)。
  ③ 蛇行:cmd_w 符号反转次数 + w 抖动 RMS(相邻差)。
  ④ cross-track:vs 尖角折线(历史口径)和 vs commanded sidecar(圆弧化后真正的跟踪口径)。

无 ROS 依赖。用法:
  python3 analyze_smoothness.py                       # 分析目录里所有 dwb run
  python3 analyze_smoothness.py dwb_20260708_190558   # 指定 run id(可多个)
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_common import cross_track_series, load_reference, rms  # noqa: E402


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def _load_track(path):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            rows.append((
                _f(r.get('t_unix')),
                _f(r.get('actual_x')), _f(r.get('actual_y')),
                r.get('tf_ok') in ('1', '1.0'),
                _f(r.get('cmd_vel_v')), _f(r.get('cmd_vel_w')),
            ))
    return rows


def _load_xy_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        pts = [(_f(r['x']), _f(r['y'])) for r in csv.DictReader(f)]
    return [p for p in pts if not (math.isnan(p[0]) or math.isnan(p[1]))]


def analyze(run_id, track_dir, ref, min_stop=0.25, goal_zone=0.35,
            corner_zone=0.45):
    track = os.path.join(track_dir, f'track_{run_id}.csv')
    if not os.path.exists(track):
        print(f'[{run_id}] 缺 {track}')
        return
    S = _load_track(track)

    # 拐角 = 去重折线的内部顶点;终点 = 折线末点
    poly = []
    for p in ref['polyline']:
        if not poly or math.hypot(p[0] - poly[-1][0], p[1] - poly[-1][1]) > 1e-6:
            poly.append(p)
    corners = poly[1:-1]
    goal = poly[-1]

    # 运动窗口:首个 v>0.02 到 最后一个 (v>0.02 或 |w|>0.05)
    mv = [i for i, s in enumerate(S) if not math.isnan(s[4]) and s[4] > 0.02]
    if not mv:
        print(f'[{run_id}] 无运动(cmd_v 全 <0.02)')
        return
    i0 = mv[0]
    i1 = max(i for i, s in enumerate(S)
             if (not math.isnan(s[4]) and s[4] > 0.02)
             or (not math.isnan(s[5]) and abs(s[5]) > 0.05))

    print(f'\n===== {run_id} =====')
    print(f'运动窗口 {S[i1][0] - S[i0][0]:.1f}s')

    # ① 中途停顿事件
    events, cur = [], None
    for i in range(i0, i1 + 1):
        t, x, y, ok, v, w = S[i]
        if math.isnan(v):
            continue
        if v < 0.02:
            if cur is None:
                cur = {'t0': t, 't1': t, 'ws': [], 'pts': []}
            cur['t1'] = t
            if not math.isnan(w):
                cur['ws'].append(w)
            if ok:
                cur['pts'].append((x, y))
        elif cur is not None:
            events.append(cur)
            cur = None
    if cur is not None:
        events.append(cur)

    n_bad = 0
    for e in events:
        dur = e['t1'] - e['t0']
        if dur < min_stop:
            continue
        if e['pts']:
            px = sum(p[0] for p in e['pts']) / len(e['pts'])
            py = sum(p[1] for p in e['pts']) / len(e['pts'])
            d2g = math.hypot(px - goal[0], py - goal[1])
            dc = min((math.hypot(px - c[0], py - c[1]) for c in corners),
                     default=9.9)
        else:
            px = py = d2g = dc = float('nan')
        wmax = max((abs(w) for w in e['ws']), default=0.0)
        if not math.isnan(d2g) and d2g < goal_zone:
            tag = '终点区(允许)'
        elif not math.isnan(dc) and dc < corner_zone:
            tag = f'★拐角停-转(距角{dc:.2f}m)'
            n_bad += 1
        else:
            tag = '★直线段停(异常)'
            n_bad += 1
        print(f'  停顿 {dur:4.1f}s @({px:.2f},{py:.2f}) max|w|={wmax:.2f} -> {tag}')
    if not events or n_bad == 0:
        print(f'  ✅ 无"拐角/直线段"中途停顿(≥{min_stop}s)')

    # ② 拐角通过
    for ci, c in enumerate(corners):
        seg = [(s[0], s[4]) for s in S[i0:i1 + 1]
               if s[3] and not math.isnan(s[4])
               and math.hypot(s[1] - c[0], s[2] - c[1]) < corner_zone]
        if not seg:
            continue
        dwell = seg[-1][0] - seg[0][0]
        vmin = min(v for _, v in seg)
        mark = '✅连续' if vmin > 0.02 else '⚠有停'
        print(f'  拐角C{ci + 1}({c[0]:.2f},{c[1]:.2f}): 逗留{dwell:4.1f}s '
              f'min_v={vmin:.2f} {mark}')

    # ③ 蛇行
    ws = [s[5] for s in S[i0:i1 + 1] if not math.isnan(s[5])]
    rev = sum(1 for i in range(1, len(ws)) if ws[i - 1] * ws[i] < -1e-6)
    jit = math.sqrt(sum((ws[i] - ws[i - 1]) ** 2
                        for i in range(1, len(ws))) / max(1, len(ws) - 1))
    print(f'  蛇行: w反向 {rev} 次, w抖动RMS {jit:.3f} rad/s/样本')

    # ④ cross-track:双口径
    xy = [(s[1], s[2]) for s in S if s[3]
          and not (math.isnan(s[1]) or math.isnan(s[2]))]
    if len(xy) > 1:
        ct = cross_track_series(xy, poly)
        line = (f'  ct vs 尖角折线: RMS {rms(ct) * 100:.1f}cm '
                f'max {max(ct) * 100:.0f}cm')
        cmd = _load_xy_csv(os.path.join(track_dir,
                                        f'track_{run_id}_commanded.csv'))
        if len(cmd) > 1:
            ct2 = cross_track_series(xy, cmd)
            line += (f' | vs commanded(圆弧): RMS {rms(ct2) * 100:.1f}cm '
                     f'max {max(ct2) * 100:.0f}cm')
        print(line)


def main():
    p = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    p.add_argument('run_ids', nargs='*', help='不给则分析目录里所有 track_*.csv')
    p.add_argument('--track-dir',
                   default=os.path.expanduser('~/david_nav_logs/bench'))
    p.add_argument('--reference', default=os.path.join(here, 'reference_path.yaml'))
    p.add_argument('--min-stop', type=float, default=0.25)
    args = p.parse_args()

    ref = load_reference(args.reference)
    rids = args.run_ids
    if not rids:
        rids = sorted(
            f[len('track_'):-len('.csv')]
            for f in os.listdir(args.track_dir)
            if f.startswith('track_') and f.endswith('.csv')
            and not f.endswith(('_reference.csv', '_plan.csv', '_commanded.csv')))
    for rid in rids:
        analyze(rid, args.track_dir, ref, min_stop=args.min_stop)


if __name__ == '__main__':
    main()
