"""Unit tests for dock_finish (pure, no ROS).

阶段2 结束之后的"测量 -> 判定 -> 纯旋转补偿"整段逻辑。用一个会带惯性的假底盘
驱动，专门盯 2026-09-10 实测暴露的那个 bug：**在车还在转的时候就测量并补偿**。
"""

import math

from david_nav.dock_finish import DockTolerance, finish_docking
from david_nav.dock_settle import SettleSpec

SPEC = SettleSpec(max_rate_rad_s=0.02, window_sec=0.4, sample_period_sec=0.05)
TOL = DockTolerance(xy_ok=0.10, yaw_ok=math.radians(5.0), max_yaw_retries=2)


class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class _FakeRobot:
    """带惯性的假底盘：停之前一直在转，Spin 之后还会多滑一段。"""

    def __init__(self, clock, w=0.0, stop_at=None, xy=(0.0, 0.0), yaw0=0.0,
                 spin_overshoot_rate=0.0, spin_overshoot_sec=0.0):
        self.clock = clock
        self.w = w
        self.stop_at = stop_at
        self.xy = xy
        self.yaw = yaw0
        self.last_t = clock.now()
        self.spins = []
        self._ov_rate = spin_overshoot_rate
        self._ov_sec = spin_overshoot_sec

    def _advance(self):
        # 分段积分：跨越 stop_at 的那一步，只把还在转的那部分时长算进去。
        # 早期版本是先清零再积分，会丢掉最后半个区间，且是否丢还取决于浮点
        # 累加的运气(0.05*10 = 0.49999... 就不会触发)，让断言值飘。
        t = self.clock.now()
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0:
            return
        if self.stop_at is not None and t >= self.stop_at:
            active = max(0.0, min(dt, self.stop_at - (t - dt)))
            self.yaw += self.w * active
            self.w = 0.0
        else:
            self.yaw += self.w * dt

    def pose(self):
        self._advance()
        return (self.xy[0], self.xy[1], self.yaw)

    def spin(self, target_yaw):
        self._advance()
        self.spins.append(target_yaw)
        self.yaw += target_yaw
        # Spin 结束后底盘不会瞬停，还要滑一段（实测就是这段把误差推大的）
        self.w = self._ov_rate
        self.stop_at = self.clock.now() + self._ov_sec
        return True


def _run(robot, clock, goal, tol=TOL, timeout=10.0):
    return finish_docking(goal, robot.pose, robot.spin, tol, SPEC,
                          settle_timeout_sec=timeout,
                          now_fn=clock.now, sleep_fn=clock.sleep)


# ---------------------------------------------------------------- 核心回归
def test_measures_the_settled_pose_not_the_moving_one():
    """★ 本次要修的 bug：车还在 0.55rad/s 转时测出的 yaw 不能拿来判定。

    车从 yaw=0 以 0.55rad/s 转到 t=0.5s 停在 0.275rad。目标 yaw 就是 0.275rad。
    - 若立刻测量：eyaw = -0.275rad(15.8°) > 5° -> 会白白下发 Spin
    - 若等停稳再测：eyaw ≈ 0 -> 一次入位达标，不需要任何补偿
    """
    clock = _Clock()
    robot = _FakeRobot(clock, w=0.55, stop_at=0.5)
    out = _run(robot, clock, goal=(0.0, 0.0, 0.55 * 0.5))

    assert out.ok is True
    assert out.n_spin == 0, f'不该下发 Spin，实际发了 {robot.spins}'
    assert abs(out.eyaw) < math.radians(1.0)


def test_no_spin_when_already_within_tolerance():
    clock = _Clock()
    robot = _FakeRobot(clock, yaw0=math.radians(2.0))
    out = _run(robot, clock, goal=(0.0, 0.0, 0.0))

    assert out.ok is True
    assert robot.spins == []


def test_spins_by_the_negative_of_the_settled_yaw_error():
    clock = _Clock()
    robot = _FakeRobot(clock, yaw0=math.radians(12.0))
    out = _run(robot, clock, goal=(0.0, 0.0, 0.0))

    assert len(robot.spins) == 1
    assert math.isclose(robot.spins[0], -math.radians(12.0), abs_tol=1e-6)
    assert out.ok is True


def test_waits_for_settle_after_each_spin_before_remeasuring():
    """Spin 之后底盘还会滑一段，必须等它停了再测，否则读到的又是运动中的值。"""
    clock = _Clock()
    # 初始差 12°，Spin 后以 0.2rad/s 多滑 0.5s = 再多转 5.7°
    robot = _FakeRobot(clock, yaw0=math.radians(12.0),
                       spin_overshoot_rate=0.2, spin_overshoot_sec=0.5)
    out = _run(robot, clock, goal=(0.0, 0.0, 0.0))

    # 第一次 Spin 把 12° 抵消掉，惯性又滑出 +5.7°，仍超 5° -> 必须再来一次
    assert len(robot.spins) == 2
    assert math.isclose(robot.spins[1], -0.2 * 0.5, abs_tol=1e-3), \
        f'第二次补偿量应等于滑出的角度，实际 {robot.spins[1]}'


def test_never_spins_when_xy_is_out_of_tolerance():
    """既有硬约定：入位位置没到就不再下发任何指令，只能调角度不能位移。"""
    clock = _Clock()
    robot = _FakeRobot(clock, xy=(0.30, 0.0), yaw0=math.radians(20.0))
    out = _run(robot, clock, goal=(0.0, 0.0, 0.0))

    assert out.ok is False
    assert out.reason == 'xy_out_of_tol'
    assert robot.spins == []


def test_gives_up_after_max_retries():
    clock = _Clock()
    # 每次 Spin 之后都滑出 20°，永远收不敛
    robot = _FakeRobot(clock, yaw0=math.radians(20.0),
                       spin_overshoot_rate=0.7, spin_overshoot_sec=0.5)
    out = _run(robot, clock, goal=(0.0, 0.0, 0.0))

    assert out.ok is False
    assert out.reason == 'yaw_not_converged'
    assert out.n_spin == TOL.max_yaw_retries


def test_zero_retries_never_spins():
    """max_yaw_retries=0 时一次 Spin 都不许发。

    2026-09-10 实测：david_move 把任何纯原地旋转指令抬到 min_stationary_angular_z
    =0.55rad/s（死区补偿），底盘停下来要惰走 ~13.6°，而 yaw 容差是 5°。于是每次
    Spin 都过冲反向：命令 5.9° 实际转 13.6°、命令 7.7° 实际转 32.9°。三版对比里
    "从不补偿"的 |eyaw| 均值 10.9° 反而好过"补偿两次"的 16.3° —— 在死区前馈落地
    之前，把重试次数设成 0 是净收益。本用例锁住这条路径。
    """
    clock = _Clock()
    robot = _FakeRobot(clock, yaw0=math.radians(20.0))
    tol = DockTolerance(xy_ok=0.10, yaw_ok=math.radians(5.0), max_yaw_retries=0)
    out = _run(robot, clock, goal=(0.0, 0.0, 0.0), tol=tol)

    assert robot.spins == []
    assert out.n_spin == 0
    assert out.ok is False
    assert out.reason == 'yaw_not_converged'
    # 误差要如实报出来，不能因为没补偿就丢掉
    assert abs(math.degrees(out.eyaw) - 20.0) < 0.5


def test_reports_no_pose_when_tf_is_unavailable():
    clock = _Clock()
    out = finish_docking((0.0, 0.0, 0.0), lambda: None, lambda _y: True,
                         TOL, SPEC, settle_timeout_sec=0.5,
                         now_fn=clock.now, sleep_fn=clock.sleep)

    assert out.ok is False
    assert out.reason == 'no_pose'


def test_not_settling_within_timeout_is_reported_but_still_measured():
    """等不到停稳 != 取不到位姿：要如实标记，同时把最后的位姿交出去。"""
    clock = _Clock()
    robot = _FakeRobot(clock, w=0.55, stop_at=None)   # 永远不停
    out = _run(robot, clock, goal=(0.0, 0.0, 0.0), timeout=1.0)

    assert out.settled is False
    assert out.pose is not None
    assert out.reason == 'not_settled'
    assert robot.spins == [], '没停稳就不该下发 Spin —— nav2 的 Spin 对运动中的车会秒过'
