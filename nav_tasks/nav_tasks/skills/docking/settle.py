"""Detect continuous settling from pose samples without ROS dependencies.

## 为什么需要它

2026-09-10 dock_bench 首次冒烟实测（5 轮，3 轮 YAW_NOT_CONVERGED）暴露：
`dock_navigator` 在阶段2 的 NavigateToPose 报 SUCCEEDED 的**那一瞬间**就取 TF 测
eyaw 并据此下发 Spin —— 而此时车仍以 **0.55 rad/s** 在转（末端 RotateToGoal 满速
收朝向的尾巴）。于是：

    25.66 阶段2 SUCCEEDED, 入位实测 eyaw=6.8°   ← 运动中的瞬时值
    25.67 Spin(-6.8°)
    25.77 Spin#1: SUCCEEDED                     ← 只用 100ms
    25.77 旋转后 eyaw=13.9°                      ← 惯性又转过去 7°
    25.88 Spin#2: ABORTED → 失败 eyaw=19.6°

整个失败序列 **220ms**，车根本没机会停。两次补偿把误差从 6.8° 推到 19.6°。

nav2 的 `Spin` 行为用 **`abs(relative_yaw)`** 判完成，所以对一台"还在往反方向转"
的车，累计转角的**绝对值**瞬间就超过目标量 → 立刻报 SUCCEEDED。也就是说：
**只要车没停稳，Spin 补偿在原理上就不可能起作用**，重试只会让误差更大。

结论：测量和补偿都必须发生在车真正静止之后。这个模块只回答"停稳了吗"。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Sequence

# 浮点容差：窗口覆盖判定用，避免 0.4 vs 0.39999999 这种误判
_EPS = 1e-9


@dataclass(frozen=True)
class SettleSpec:
    """Thresholds for a continuously settled trailing sample window.

    max_rate_rad_s : 窗口内任意相邻样本间的角速度上限。实测静态定位抖动
                     yaw std 仅 0.068°(≈0.0012 rad)，20Hz 下抖动带来的表观角速度
                     约 0.02-0.05 rad/s，所以阈值不能定得比这更严，否则永远等不到。
    window_sec     : 必须**连续**满足上限的时长。只看一个瞬时差值不够 —— 车在
                     换向的过零点上瞬时角速度也接近 0。
    sample_period_sec : 轮询周期。

    Attributes:
        max_rate_rad_s: 相邻样本允许的最大表观偏航角速度（弧度/秒）。
        window_sec: 连续满足角速度限制所需覆盖的窗口（秒）。
        sample_period_sec: 请求位姿样本的轮询周期（秒）。
    """

    max_rate_rad_s: float = 0.05
    window_sec: float = 0.4
    sample_period_sec: float = 0.05


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def is_settled(samples: Sequence[tuple[float, float]], spec: SettleSpec) -> bool:
    """Check continuous angular-rate stability across the complete trailing window.

    两条都必须满足，缺一不可：
      ① 尾部窗口**被数据铺满**（时间跨度 >= window_sec）—— 这条专门堵"刚停 100ms
         就宣布停稳"，正是实测里 Spin 秒过的那个坑；
      ② 窗口内每一对相邻样本的角速度都低于上限。
    角度差一律先 wrap，否则车停在 ±180° 附近时 +π/−π 的表示切换会被当成 2π 的
    高速旋转。

    Args:
        samples: Time/yaw samples ordered by ascending time.
        spec: Continuous-window and angular-rate thresholds.

    Returns:
        Whether the trailing window is covered and every measured rate satisfies the threshold.
    """
    if len(samples) < 2:
        return False

    t_end = samples[-1][0]
    t_cut = t_end - spec.window_sec

    # 取最后一个"不晚于窗口起点"的样本当**桥接样本**，窗口从它开始。
    # ⚠ 不能只取 t >= t_cut 的样本：那样窗口跨度是 Δ×floor(window/Δ)，
    #   只有采样间隔 Δ 整除 window 时才等于 window。真机上 pose_now() 自身有
    #   耗时，Δ 恒略大于 sample_period（0.05 -> 0.051），floor 掉一档，跨度
    #   永远差一点点，于是**永远判不出停稳** —— 2026-09-10 实测每轮都跑满 8s
    #   超时报 NOT_SETTLED，而同一时段录下来的 TF 明明是静止的。
    #   单测当时用完美的 0.05 等间隔（0.4/0.05=8 刚好整除）把这个 bug 放过去了。
    start = None
    for i, (t, _) in enumerate(samples):
        if t <= t_cut + _EPS:
            start = i
    if start is None:
        # 数据还不够老，窗口没被覆盖到 —— 这条正是"刚停 100ms 就宣布停稳"的闸门
        return False
    window = samples[start:]
    if len(window) < 2:
        return False

    for (t0, y0), (t1, y1) in zip(window, window[1:]):
        dt = t1 - t0
        if dt <= 0.0:
            continue
        if abs(_wrap(y1 - y0)) / dt > spec.max_rate_rad_s:
            return False
    return True


def wait_until_settled(
    pose_fn: Callable[[], tuple[float, float, float] | None],
    spec: SettleSpec,
    timeout_sec: float,
    now_fn: Callable[[], float] | None = None,
    sleep_fn: Callable[[float], None] | None = None,
) -> tuple[bool, tuple[float, float, float] | None]:
    """Poll until settled or timed out, retaining the last available pose.

    pose_fn : 无参可调用，返回 (x, y, yaw) 或 None（取不到位姿）。
    now_fn / sleep_fn : 注入时钟，方便确定性单测；默认用 time.monotonic / time.sleep。

    超时也把**最后一次拿到的位姿**交出去 —— 调用方需要它来报告实际误差，
    "等不到停稳"和"取不到位姿"是两回事。

    Args:
        pose_fn: Pose provider returning x/y/yaw or None.
        spec: Continuous-window and angular-rate thresholds.
        timeout_sec: Maximum polling duration.
        now_fn: Injected monotonic clock; defaults to time.monotonic.
        sleep_fn: Injected sleeper; defaults to time.sleep.

    Returns:
        Settled flag and the latest available pose, including when the wait times out.
    """
    now_fn = now_fn or time.monotonic
    sleep_fn = sleep_fn or time.sleep

    t_start = now_fn()
    samples = []
    last_pose = None
    while True:
        pose = pose_fn()
        if pose is not None:
            last_pose = pose
            samples.append((now_fn(), pose[2]))
            if is_settled(samples, spec):
                return True, last_pose
        if now_fn() - t_start >= timeout_sec:
            return False, last_pose
        sleep_fn(spec.sample_period_sec)
