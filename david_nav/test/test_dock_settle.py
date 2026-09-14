"""Unit tests for dock_settle (pure, no ROS).

背景：2026-09-10 dock_bench 冒烟实测，dock_navigator 在阶段2 结束、车仍以
0.55 rad/s 旋转时就取 yaw 做判定并下发 Spin，220ms 内连败两次、误差从 6.8°
被推到 19.6°。这里的模块负责回答"车真的停稳了吗"。
"""

import math

from david_nav.dock_settle import SettleSpec, is_settled, wait_until_settled


SPEC = SettleSpec(max_rate_rad_s=0.02, window_sec=0.4, sample_period_sec=0.05)


def _series(start_t, yaws, dt=0.05):
    return [(start_t + i * dt, y) for i, y in enumerate(yaws)]


# ---------------- is_settled ----------------
def test_not_settled_while_rotating_at_stage2_exit_speed():
    # 实测阶段2 结束瞬间就是这个速度
    rate = 0.55
    dt = 0.05
    yaws = [rate * i * dt for i in range(20)]      # 1.0s of rotation
    assert is_settled(_series(100.0, yaws, dt), SPEC) is False


def test_settled_when_stationary_for_longer_than_window():
    yaws = [1.5707] * 20                            # 1.0s 完全不动
    assert is_settled(_series(100.0, yaws), SPEC) is True


def test_not_settled_when_window_not_yet_covered():
    """刚停下 0.1s 不算停稳 —— 这正是实测里 Spin 100ms 就"成功"的那个坑。"""
    yaws = [1.5707] * 3                             # 只有 0.10s 的数据
    assert is_settled(_series(100.0, yaws), SPEC) is False


def test_not_settled_if_a_fast_step_falls_inside_the_window():
    # 前面很稳，但窗口内有一次 0.3 rad/0.05s = 6 rad/s 的跳变
    yaws = [1.0] * 10 + [1.3] + [1.3] * 5
    assert is_settled(_series(100.0, yaws), SPEC) is False


def test_wrap_around_is_not_mistaken_for_rotation():
    """车停在 ±180°，+π 和 −π 是同一个角度，绕回不能被当成 2π 的高速旋转。"""
    yaws = [math.pi, -math.pi] * 10
    assert is_settled(_series(100.0, yaws), SPEC) is True


def test_settled_with_irregular_sample_spacing():
    """★ 2026-09-10 真机回归：采样间隔不整除窗口长度时也必须能判出停稳。

    原实现把窗口取成 `t >= t_end - window_sec` 的那些样本，其跨度是
    Δ×floor(window/Δ)，只有 Δ 整除 window 时才等于 window。真机上
    pose_now() 自己有耗时，Δ 恒略大于 sample_period，floor 掉一档 ->
    跨度永远差一点 -> **永远返回 False**。实测表现为每轮都跑满 8s 超时
    判 NOT_SETTLED，而同一时段录下来的 TF 明明是静止的。
    """
    yaws = [1.5707] * 20
    assert is_settled(_series(100.0, yaws, dt=0.07), SPEC) is True


def test_settled_with_slow_polling():
    """轮询比窗口稀疏得多时（Δ=0.15s），只要数据够老、够稳，也算停稳。"""
    yaws = [1.5707] * 10
    assert is_settled(_series(100.0, yaws, dt=0.15), SPEC) is True


def test_not_settled_when_all_samples_are_newer_than_the_window():
    """数据不够老 —— 窗口没被覆盖，不能算停稳（这条是上面两条的反面约束）。"""
    yaws = [1.5707] * 3
    assert is_settled(_series(100.0, yaws, dt=0.07), SPEC) is False


def test_motion_just_outside_the_window_does_not_block_settling():
    """0.4s 之前转得再快也不该影响判定，只看尾部窗口。"""
    moving = [0.0, 0.3, 0.6, 0.9]          # 早期高速旋转
    still = [0.9] * 10                      # 之后静止 0.7s
    assert is_settled(_series(100.0, moving + still, dt=0.07), SPEC) is True


def test_empty_or_single_sample_is_not_settled():
    assert is_settled([], SPEC) is False
    assert is_settled([(100.0, 1.0)], SPEC) is False


# ---------------- wait_until_settled ----------------
class _FakeClock:
    """可控时钟 + sleep，让等待逻辑能被确定性地测。"""

    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


def test_returns_true_once_the_robot_actually_stops():
    clock = _FakeClock()
    # 前 0.5s 以 0.55 rad/s 转，之后彻底停住
    def pose_fn():
        if clock.t < 0.5:
            return (0.0, 0.0, 0.55 * clock.t)
        return (0.0, 0.0, 0.55 * 0.5)

    settled, pose = wait_until_settled(
        pose_fn, SPEC, timeout_sec=5.0, now_fn=clock.now, sleep_fn=clock.sleep)

    assert settled is True
    assert pose is not None
    # 必须等到停了之后再加一个完整窗口才判定
    assert clock.t >= 0.5 + SPEC.window_sec


def test_times_out_when_the_robot_never_stops():
    clock = _FakeClock()

    def pose_fn():
        return (0.0, 0.0, 0.55 * clock.t)

    settled, pose = wait_until_settled(
        pose_fn, SPEC, timeout_sec=1.0, now_fn=clock.now, sleep_fn=clock.sleep)

    assert settled is False
    assert pose is not None            # 超时也要把最后拿到的位姿交出去
    assert clock.t >= 1.0


def test_pose_lookup_failure_does_not_count_as_settled():
    clock = _FakeClock()

    settled, pose = wait_until_settled(
        lambda: None, SPEC, timeout_sec=0.5,
        now_fn=clock.now, sleep_fn=clock.sleep)

    assert settled is False
    assert pose is None
