#!/usr/bin/env python3
"""dock_common 的单测。无 ROS 依赖，本地/机器人上都能跑：python3 test_dock_common.py"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dock_common import (along_track, classify_verdict, jitter_rms,  # noqa: E402
                         parse_dock_timeline, prep_point, remaining_dist,
                         sign_reversals, signed_cross_track, stop_events,
                         wrap, yaw_convergence_bins)

FAILS = []


def check(name, cond, extra=''):
    if cond:
        print(f'  ok   {name}')
    else:
        print(f'  FAIL {name} {extra}')
        FAILS.append(name)


def close(a, b, tol=1e-3):
    return abs(a - b) <= tol


print('== prep_point 必须和 dock_navigator 逐字一致 ==')
# 取 2026-09-08 实机崩溃那次的真实数值：
#   [dock] ===== 收到目标 (7.975, 1.097) yaw=157.6°
#   [dock] 阶段1 预备点 (8.437, 0.906)
px, py = prep_point(7.975, 1.097, math.radians(157.6), 0.5)
check('实机复现 (7.975,1.097)@157.6° -> (8.437,0.906)',
      close(px, 8.437, 2e-3) and close(py, 0.906, 2e-3), f'got ({px:.3f},{py:.3f})')
# 预备点到目标的距离恒为 approach_dist
check('预备点间距恒 = approach_dist',
      close(math.hypot(px - 7.975, py - 1.097), 0.5))

print('== 理想入位直线几何 ==')
G = (2.0, 3.0, math.radians(90.0))   # 目标朝 +y
# 预备点应在 G 的 -y 方向 0.5m
p = prep_point(*G, 0.5)
check('yaw=90° 预备点在 G 正后方', close(p[0], 2.0) and close(p[1], 2.5),
      f'got {p}')
check('预备点横向偏差 = 0', close(signed_cross_track(p[0], p[1], *G), 0.0))
check('预备点剩余距离 = 0.5', close(remaining_dist(p[0], p[1], *G), 0.5))
check('G 处剩余距离 = 0', close(remaining_dist(G[0], G[1], *G), 0.0))
check('过冲后剩余距离为负', remaining_dist(2.0, 3.2, *G) < 0)
# 朝 +y 走时，车偏到 -x 侧应为"左"(正)
check('偏左为正', signed_cross_track(1.9, 2.8, *G) > 0)
check('偏右为负', signed_cross_track(2.1, 2.8, *G) < 0)
check('横向偏差量值正确', close(abs(signed_cross_track(2.1, 2.8, *G)), 0.1))
check('沿线走不产生横向偏差',
      close(signed_cross_track(2.0, 2.7, *G), 0.0))

print('== wrap ==')
check('wrap(370°)=10°', close(math.degrees(wrap(math.radians(370))), 10.0))
check('wrap(-190°)=170°', close(math.degrees(wrap(math.radians(-190))), 170.0))

print('== 判词分类：逐条对着 dock_navigator.py 的原文 ==')
cases = [
    ('成功', '一次入位达标  (实测 exy=0.043m eyaw=-1.2°)', 'SUCCESS'),
    ('成功', '第2次旋转后达标  (实测 exy=0.061m eyaw=3.0°)', 'SUCCESS'),
    ('失败', '阶段1 未到预备点，放弃（不做入位）', 'STAGE1_FAILED'),
    ('失败', 'xy 超差 0.284 > 0.100，按约定不再下发位移目标  (实测 exy=0.284m eyaw=2.0°)',
     'XY_OUT_OF_TOL'),
    ('失败', 'yaw 经 2 次旋转仍未收敛  (实测 exy=0.052m eyaw=11.4°)', 'YAW_NOT_CONVERGED'),
    ('失败', '入位后取不到位姿，无法判定', 'POSE_LOOKUP_FAILED'),
    ('失败', '旋转后取不到位姿', 'POSE_LOOKUP_FAILED'),
    ('失败', '编排线程异常，本次目标放弃：\nTraceback...', 'NODE_EXCEPTION'),
    ('失败', '车未停稳（8.0s 内角速度未降到 0.020rad/s 以下），按约定不做旋转补偿',
     'NOT_SETTLED'),
    ('失败', '某种我没见过的新判词', 'UNKNOWN_FAILURE'),
]
for kind, detail, want in cases:
    got = classify_verdict(kind, detail)
    check(f'{want:<18} <- {detail[:26]}...', got == want, f'got {got}')

print('== 时间线解析（一次带 1 次 Spin 补偿的成功入位）==')
log = [
    (100.0, '[dock] 就绪: approach_dist=0.50m xy_ok=0.100m ...'),
    (110.0, '[dock] ===== 收到目标 (7.975, 1.097) yaw=157.6°'),
    (110.1, '[dock] 阶段1 预备点 (8.437, 0.906) yaw=157.6°（沿目标航向后退 0.50m）'),
    (128.0, '[dock] 阶段1(预备点): SUCCEEDED'),
    (128.1, '[dock] 阶段2 入位 -> (7.975, 1.097) yaw=157.6°'),
    (136.0, '[dock] 阶段2(入位): SUCCEEDED'),
    (138.0, '[dock] 入位实测 exy=0.062m eyaw=8.5° (阶段2 SUCCEEDED)'),
    (138.1, '[dock] 第1次纯旋转修正: Spin(-8.5°)'),
    (143.0, '[dock] 旋转后 exy=0.063m eyaw=1.1°'),
    (143.2, '[dock] ===== 成功: 第1次旋转后达标  (实测 exy=0.063m eyaw=1.1°)'),
]
tl = parse_dock_timeline(log)
check('goal_received', close(tl['goal_received'], 110.0))
check('stage1_start', close(tl['stage1_start'], 110.1))
check('stage1_status=SUCCEEDED', tl['stage1_status'] == 'SUCCEEDED', tl['stage1_status'])
check('stage2_start', close(tl['stage2_start'], 128.1))
check('stage2_status=SUCCEEDED', tl['stage2_status'] == 'SUCCEEDED', tl['stage2_status'])
check('n_spin=1', tl['n_spin'] == 1, tl['n_spin'])
check('verdict_class=SUCCESS', tl['verdict_class'] == 'SUCCESS', tl['verdict_class'])
check('verdict_t', close(tl['verdict_t'], 143.2))
check('节点自评 exy 被抓到', close(tl['node_exy'], 0.062))
check('节点自评 eyaw 被抓到', close(tl['node_eyaw_deg'], 8.5))
check('阶段2 窗口 = [128.1, 143.2]',
      close(tl['verdict_t'] - tl['stage2_start'], 15.1))

print('== 时间线解析（阶段1 就失败，没有阶段2）==')
log2 = [
    (10.0, '[dock] ===== 收到目标 (1.0, 2.0) yaw=0.0°'),
    (10.1, '[dock] 阶段1 预备点 (0.5, 2.0) yaw=0.0°（沿目标航向后退 0.50m）'),
    (70.0, '[dock] 阶段1(预备点): status=6'),
    (70.1, '[dock] ===== 失败: 阶段1 未到预备点，放弃（不做入位）'),
]
tl2 = parse_dock_timeline(log2)
check('无 stage2_start', tl2['stage2_start'] is None)
check('stage1_status=status=6', tl2['stage1_status'] == 'status=6', tl2['stage1_status'])
check('分类 STAGE1_FAILED', tl2['verdict_class'] == 'STAGE1_FAILED')
check('n_spin=0', tl2['n_spin'] == 0)

print('== 超时（没有任何判词）==')
tl3 = parse_dock_timeline([(1.0, '[dock] ===== 收到目标 (1,2) yaw=0°')])
check('verdict_class 为 None -> 调用方判 TIMEOUT', tl3['verdict_class'] is None)

print('== 流畅性 ==')
# 0.0-0.5 走, 0.5-1.0 停(0.5s>0.25s 记一次), 1.0-1.5 走
samples = ([(t / 10.0, 0.15) for t in range(0, 5)]
           + [(t / 10.0, 0.001) for t in range(5, 11)]
           + [(t / 10.0, 0.15) for t in range(11, 16)])
ev = stop_events(samples, v_thresh=0.02, min_stop=0.25)
check('抓到 1 次停顿', len(ev) == 1, ev)
check('停顿时长 ~0.5s', ev and close(ev[0][2], 0.5, 0.06), ev)
check('短于 min_stop 的不算',
      len(stop_events([(0.0, 0.0), (0.1, 0.0), (0.2, 0.3)], min_stop=0.25)) == 0)
check('全程停也能收尾',
      len(stop_events([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)], min_stop=0.25)) == 1)

check('符号反转 2 次', sign_reversals([0.1, 0.2, -0.1, -0.2, 0.3]) == 2)
check('零附近抖动不计', sign_reversals([0.1, 0.0001, -0.0001, 0.2]) == 0)
check('全同号 0 次', sign_reversals([0.1, 0.2, 0.3]) == 0)
check('抖动 RMS', close(jitter_rms([0.0, 0.1, 0.0, 0.1]), 0.1))
check('单点抖动 RMS = nan', math.isnan(jitter_rms([0.5])))

print('== 航向收敛分箱 ==')
# 剩余 0.5m 时偏 20°，越近越正
conv = [(0.45, math.radians(20.0)), (0.35, math.radians(15.0)),
        (0.25, math.radians(10.0)), (0.15, math.radians(4.0)),
        (0.05, math.radians(1.0))]
bins = yaw_convergence_bins(conv, 0.0)
check('5 个箱', len(bins) == 5, len(bins))
check('第一箱 [0.5,0.4) 中位数 20°', close(bins[0][3], 20.0, 0.1), bins[0])
check('末箱 [0.1,0.0) 中位数 1°', close(bins[4][3], 1.0, 0.1), bins[4])
check('单调收敛', all(bins[i][3] >= bins[i + 1][3] for i in range(4)))
empty = yaw_convergence_bins([], 0.0)
check('空输入不崩', len(empty) == 5 and math.isnan(empty[0][3]))

print()
if FAILS:
    print(f'!! {len(FAILS)} 个用例失败: {FAILS}')
    sys.exit(1)
print('全部通过')
