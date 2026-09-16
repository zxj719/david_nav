"""Blocking two-stage Dock workflow for a supervised worker, with frozen audit lines.

This is not a ROS node or a task owner. The runtime must keep its worker joined
and bridge motion calls to the one Nav2Client. Do not invoke execute on the ROS
executor or asyncio loop thread: the unchanged settle algorithm deliberately
blocks its calling worker. Entry adapters, action deadlines and cleanup ownership
belong outside this workflow.
"""

from dataclasses import dataclass
import logging
import math
from typing import Callable, Literal, Mapping

from nav_tasks.domain.footprint import FootprintCheck
from nav_tasks.skills.docking.finish import DockTolerance, finish_docking
from nav_tasks.skills.docking.ports import DockLogger, DockMotionPort, PlanarPose
from nav_tasks.skills.docking.settle import SettleSpec

_FAILURE_REASONS = ('no_pose', 'not_settled', 'xy_out_of_tol', 'yaw_not_converged')
_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class DockConfig:
    """Explicit robot/mode policy; do not substitute helper defaults for David tuning.

    Attributes:
        approach_dist: 目标朝向后方的预备距离（米）。
        nav_timeout_sec: 单段导航期限（秒）。
        settle_spec: 连续停稳窗口和角速度限制。
        settle_timeout_sec: 每次停稳等待期限（秒）。
        spin_timeout_sec: 单次 Spin 时间预算（秒）。
        tolerance: 最终位置/偏航容差及补偿次数。
    """

    approach_dist: float
    tolerance: DockTolerance
    settle_spec: SettleSpec
    nav_timeout_sec: float
    spin_timeout_sec: float
    settle_timeout_sec: float


class DockWorkflow:
    """Keep the old synchronous sequence behind injected worker-safe ports."""

    def __init__(
        self, config: DockConfig, motion: DockMotionPort,
        pose_fn: Callable[[], PlanarPose | None],
        approach_check: Callable[[PlanarPose], FootprintCheck], logger: DockLogger, *,
        now_fn: Callable[[], float] | None = None,
        sleep_fn: Callable[[float], None] | None = None,
        on_event: Callable[[str, Mapping[str, object]], None] | None = None,
    ) -> None:
        """Inject robot policy and effects without starting threads or ROS resources.

        Args:
            config: Explicit stage timeouts, geometry and finish policy.
            motion: Blocking facade over the sole motion backend; owns action audit lines.
            pose_fn: Latest navigation-base global pose, or None when unavailable.
            approach_check: Mandatory full-footprint admission using the static map.
            logger: Thread-safe log sink; texts and levels retain legacy behavior.
            now_fn: Optional deterministic monotonic clock for finish tests.
            sleep_fn: Optional deterministic sleeper for finish tests.
            on_event: Optional worker-safe observation sink; errors do not change Dock policy.
        """
        self._config = config
        self._motion = motion
        self._pose_fn = pose_fn
        self._approach_check = approach_check
        self._logger = logger
        self._now = now_fn
        self._sleep = sleep_fn
        self._on_event = on_event

    def execute(self, goal: PlanarPose) -> Literal['succeeded', 'failed', 'canceled']:
        """Run stages, then settle and judge without changing retry or movement policy.

        No arrival is produced for the approach stage. Stage-two failure still
        permits settled-pose judgement, as before. Spin's boolean result remains
        advisory: measured pose is authoritative for the old finish algorithm.
        The external task owner must fence results when cancellation has begun.

        Args:
            goal: Global-frame navigation-base x/y/yaw, already decoded by ingress.

        Returns:
            Legacy final outcome, not a ROS action response or WS event.

        Raises:
            Exception: A port failed; the task owner must retain and clean up resources.
        """
        config = self._config
        gx, gy, gyaw = goal
        self._logger.info(
            '[dock] ===== 收到目标 (%.3f, %.3f) yaw=%.1f°' % (gx, gy, math.degrees(gyaw)))
        self._observe('started', x=gx, y=gy, yaw=gyaw)
        px = gx - config.approach_dist * math.cos(gyaw)
        py = gy - config.approach_dist * math.sin(gyaw)
        self._logger.info(
            '[dock] 阶段1 预备点 (%.3f, %.3f) yaw=%.1f°（沿目标航向后退 %.2fm）'
            % (px, py, math.degrees(gyaw), config.approach_dist))
        check = self._approach_check((px, py, gyaw))
        self._observe('approach_checked', clear=check.clear, reason=check.reason,
                      occupied_cell=check.occupied_cell, x=px, y=py, yaw=gyaw)
        if not check.clear:
            detail = check.reason
            if check.occupied_cell is not None:
                mx, my, value = check.occupied_cell
                detail += ' cell=(%d,%d) value=%d' % (mx, my, value)
            self._report(False, '预备点车身轮廓静态地图校验失败: %s' % detail)
            return 'failed'
        self._logger.info('[dock] 预备点车身轮廓静态地图校验通过')
        self._observe('stage_started', stage='approach', x=px, y=py, yaw=gyaw)
        stage1 = self._motion.navigate(
            (px, py, gyaw), '阶段1(预备点)', config.nav_timeout_sec, config.approach_dist)
        if not stage1.succeeded:
            if stage1.canceled:
                self._logger.warn('[dock] ===== 已取消: 阶段1 被取消，不做入位')
                return 'canceled'
            self._report(False, '阶段1 未到预备点，放弃（不做入位）')
            return 'failed'
        self._logger.info('[dock] 阶段2 入位 -> (%.3f, %.3f) yaw=%.1f°'
                          % (gx, gy, math.degrees(gyaw)))
        self._observe('stage_started', stage='final', x=gx, y=gy, yaw=gyaw)
        stage2 = self._motion.navigate(
            goal, '阶段2(入位)', config.nav_timeout_sec, 0.0)
        if stage2.canceled:
            self._logger.warn('[dock] ===== 已取消: 阶段2 被取消，不做判定')
            return 'canceled'
        if not stage2.succeeded:
            self._logger.warn('[dock] 阶段2 未 SUCCEEDED，仍按停稳后的实测位姿判定')
        spin_index = 0
        last_canceled = False

        def event(name, **values):
            nonlocal spin_index
            if name == 'spin':
                spin_index = values['index']
            self._log_event(name, **values)
            self._observe(name, **values)

        def spin(yaw):
            nonlocal last_canceled
            result = self._motion.spin(
                yaw, 'Spin#%d' % spin_index,
                config.spin_timeout_sec, config.spin_timeout_sec + 5.0)
            last_canceled = result.canceled
            return result.succeeded

        outcome = finish_docking(
            goal, self._pose_fn, spin, config.tolerance, config.settle_spec,
            settle_timeout_sec=config.settle_timeout_sec, on_event=event,
            now_fn=self._now, sleep_fn=self._sleep)
        if outcome.reason == 'no_pose':
            self._report(False, '入位后取不到位姿，无法判定')
        elif outcome.reason == 'not_settled':
            self._report(False, '车未停稳（%.1fs 内角速度未降到 %.3frad/s 以下），'
                         '按约定不做旋转补偿'
                         % (config.settle_timeout_sec, config.settle_spec.max_rate_rad_s),
                         outcome.pose, goal)
        elif outcome.reason == 'xy_out_of_tol':
            self._report(False, 'xy 超差 %.3f > %.3f，按约定不再下发位移目标'
                         % (outcome.exy, config.tolerance.xy_ok), outcome.pose, goal)
        elif outcome.reason == 'yaw_not_converged':
            self._report(False, 'yaw 经 %d 次旋转仍未收敛' % outcome.n_spin,
                         outcome.pose, goal)
        elif outcome.n_spin == 0:
            self._report(True, '一次入位达标', outcome.pose, goal)
        else:
            self._report(True, '第%d次旋转后达标' % outcome.n_spin, outcome.pose, goal)
        if outcome.reason in _FAILURE_REASONS:
            return 'canceled' if last_canceled else 'failed'
        return 'succeeded'

    def _observe(self, name, **values):
        if self._on_event is not None:
            try:
                self._on_event(name, dict(values))
            except Exception as error:
                _LOGGER.warning('Dock observation failed (%s)', type(error).__name__)

    def _report(self, ok, message, pose=None, goal=None):
        if pose is not None and goal is not None:
            exy = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
            eyaw = (pose[2] - goal[2] + math.pi) % (2.0 * math.pi) - math.pi
            message = '%s  (实测 exy=%.3fm eyaw=%.1f°)' % (message, exy, math.degrees(eyaw))
        if ok:
            self._logger.info('[dock] ===== 成功: %s' % message)
        else:
            self._logger.error('[dock] ===== 失败: %s' % message)

    def _log_event(self, name, **values):
        if name == 'measured':
            self._logger.info(
                '[dock] 入位实测 exy=%.3fm eyaw=%.1f° (停稳=%s)'
                % (values['exy'], math.degrees(values['eyaw']),
                   '是' if values['settled'] else '否'))
        elif name == 'spin':
            self._logger.info('[dock] 第%d次纯旋转修正: Spin(%.1f°)'
                              % (values['index'], math.degrees(values['target_yaw'])))
        elif name == 'after_spin':
            self._logger.info(
                '[dock] 旋转后 exy=%.3fm eyaw=%.1f° (停稳=%s)'
                % (values['exy'], math.degrees(values['eyaw']),
                   '是' if values['settled'] else '否'))
        elif name == 'no_pose':
            self._logger.warn('[dock] 停稳等待期间取不到位姿')
