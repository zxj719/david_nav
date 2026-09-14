#!/usr/bin/env python3
"""plot_dock_run —— 每轮入位出一张两联图。

左：全局视角（地图底 + 阶段1 + 阶段2 + 理想入位直线 + goal 朝向箭头）
右：**最后 0.5m 放大**，横轴 = 沿线剩余距离，纵轴 = 有符号横向偏差 —— 这张才是
    真正看入位质量的图（左图那点偏差在整张地图上根本看不见）。

地图底直接复用 controller_bench/plot_run.py 的取图逻辑（活 /map 优先，退回磁盘 pgm），
不重复实现。

用法:
  python3 plot_dock_run.py --session dock_20260909_031200          # 该 session 全部轮次
  python3 plot_dock_run.py --session ... --iteration 3             # 只画第 3 轮
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'controller_bench'))

from dock_common import prep_point, remaining_dist, signed_cross_track  # noqa: E402
from dock_metrics import _f, load_runs, load_track, slice_stage2  # noqa: E402


def _map_background(ax, map_yaml):
    """复用 controller_bench 的取图；拿不到图就静默跳过（不影响右图）。"""
    try:
        from plot_run import grab_live_map, load_disk_map
    except ImportError:
        return None
    got = None
    try:
        got = grab_live_map(timeout_sec=5.0)
    except Exception:  # noqa: BLE001
        got = None
    if got is None and map_yaml and os.path.exists(map_yaml):
        try:
            got = load_disk_map(map_yaml)
        except Exception:  # noqa: BLE001
            got = None
    if got is None:
        return None
    img, res, ox, oy = got[0], got[1], got[2], got[3]
    h, w = np.array(img).shape[:2]
    ax.imshow(img, cmap='gray_r', origin='lower', alpha=0.35,
              extent=(ox, ox + w * res, oy, oy + h * res))
    return got


def plot_one(run, track, case, out_path, map_yaml):
    gx, gy = _f(run['goal_x']), _f(run['goal_y'])
    gyaw = _f(run['goal_yaw'])
    ad = float(case.get('approach_dist', 0.5))
    px, py = prep_point(gx, gy, gyaw, ad)
    xy_ok = float(case['tolerances']['xy_ok'])

    t_sent, t_s2, t_end = (_f(run.get('t_dock_sent')),
                           _f(run.get('t_stage2_start')),
                           _f(run.get('t_verdict')))
    whole = [r for r in track
             if r['ok'] and (math.isnan(t_sent) or r['t'] >= t_sent)
             and (math.isnan(t_end) or r['t'] <= t_end)]
    seg2 = [r for r in slice_stage2(track, run) if r['ok']]
    seg1 = [r for r in whole if math.isnan(t_s2) or r['t'] < t_s2]

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))

    # ---------------- 左：全局 ----------------
    _map_background(ax, map_yaml)
    if seg1:
        ax.plot([r['x'] for r in seg1], [r['y'] for r in seg1],
                '-', color='0.55', lw=1.4, label='阶段1 去预备点')
    if seg2:
        ax.plot([r['x'] for r in seg2], [r['y'] for r in seg2],
                '-', color='tab:blue', lw=2.2, label='阶段2 入位(被测段)')
    ax.plot([px, gx], [py, gy], '--', color='tab:green', lw=1.6,
            label='理想入位直线')
    ax.plot(px, py, 'o', color='tab:green', ms=6, label='预备点 P')
    ax.plot(gx, gy, '*', color='tab:red', ms=16, label='goal G')
    ax.annotate('', xy=(gx + 0.25 * math.cos(gyaw), gy + 0.25 * math.sin(gyaw)),
                xytext=(gx, gy),
                arrowprops=dict(arrowstyle='->', color='tab:red', lw=1.8))
    ax.add_patch(plt.Circle((gx, gy), xy_ok, fill=False, ls=':',
                            color='tab:red', lw=1.2))
    if not math.isnan(_f(run.get('end_x'))):
        ax.plot(_f(run['end_x']), _f(run['end_y']), 'x', color='k', ms=10,
                mew=2, label='终点')
    # 视野锁在入位区附近，别被整张地图稀释
    pad = 1.2
    ax.set_xlim(min(px, gx) - pad, max(px, gx) + pad)
    ax.set_ylim(min(py, gy) - pad, max(py, gy) + pad)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)'), ax.set_ylabel('y (m)')
    ax.legend(fontsize=8, loc='best')
    ax.set_title(f'第 {run["iteration"]} 轮 — {run.get("verdict_class", "?")}')

    # ---------------- 右：最后 0.5m 放大 ----------------
    if seg2:
        s = [remaining_dist(r['x'], r['y'], gx, gy, gyaw) for r in seg2]
        d = [signed_cross_track(r['x'], r['y'], gx, gy, gyaw) * 1000.0
             for r in seg2]                      # mm
        ax2.plot(s, d, '-', color='tab:blue', lw=1.8)
        ax2.plot(s[0], d[0], 'o', color='tab:green', ms=6, label='入位起点')
        ax2.plot(s[-1], d[-1], 'x', color='k', ms=10, mew=2, label='终点')
        ax2.axhline(0, color='tab:green', ls='--', lw=1.2, label='理想直线')
        ax2.axhspan(-xy_ok * 1000, xy_ok * 1000, color='tab:red', alpha=0.07)
        # y 轴按实际数据定，别让 ±100mm 的容差带把 ±20mm 的轨迹细节压成一条线。
        # 容差带铺满背景 = "全程在容差内"，本身就是要传达的信息。
        span = max(max(abs(v) for v in d) * 1.35, 5.0)
        ax2.set_ylim(-span, span)
        ax2.invert_xaxis()                       # 左=远，右=到位
        ax2.set_xlabel('沿理想直线的剩余距离 (m)  →  到位')
        ax2.set_ylabel('有符号横向偏差 (mm)   左正 / 右负')
        ax2.grid(alpha=0.3)
        ax2.legend(fontsize=8)
        rms_mm = math.sqrt(sum(v * v for v in d) / len(d))
        ax2.set_title(f'入位段 | RMS {rms_mm:.0f} mm | 最大 {max(abs(v) for v in d):.0f} mm'
                      f' | 耗时 {run.get("stage2_dur", "?")} s')
    else:
        ax2.text(0.5, 0.5, '本轮没有阶段2 数据\n（阶段1 就失败 / 超时 / 轨迹缺失）',
                 ha='center', va='center', transform=ax2.transAxes)
        ax2.set_axis_off()

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--runs', default=os.path.expanduser(
        '~/david_nav_logs/dock_bench/runs_dock.csv'))
    p.add_argument('--track-dir',
                   default=os.path.expanduser('~/david_nav_logs/dock_bench'))
    p.add_argument('--case', default=os.path.join(HERE, 'dock_case.yaml'))
    p.add_argument('--session', default='')
    p.add_argument('--iteration', type=int, default=0)
    p.add_argument('--map-yaml', default=os.path.expanduser(
        '~/ros2_ws/src/david_nav/maps/map.yaml'))
    args = p.parse_args()

    with open(args.case, encoding='utf-8') as f:
        case = yaml.safe_load(f)
    runs = load_runs(args.runs, args.session or None)
    if args.iteration:
        runs = [r for r in runs if str(r['iteration']) == str(args.iteration)]
    if not runs:
        raise SystemExit('没有匹配的轮次')

    # 用中文标签就得挑个有中文字形的字体，否则 matplotlib 画一堆豆腐块
    for fam in ('Noto Sans CJK JP', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei',
                'Droid Sans Fallback'):
        try:
            matplotlib.font_manager.findfont(fam, fallback_to_default=False)
            plt.rcParams['font.sans-serif'] = [fam]
            plt.rcParams['axes.unicode_minus'] = False
            break
        except Exception:  # noqa: BLE001
            continue

    for r in runs:
        track, _, _ = load_track(
            os.path.join(args.track_dir, f'track_{r["session"]}.csv'))
        out = os.path.join(args.track_dir,
                           f'plot_{r["session"]}_i{r["iteration"]}.png')
        plot_one(r, track, case, out, args.map_yaml)
        print(f'  {out}')


if __name__ == '__main__':
    main()
