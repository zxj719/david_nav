"""解码 Dock 请求并按显式 POI ID 路由，绕过普通吸附和位移重试."""

from dataclasses import dataclass
import math
from typing import Callable, Mapping

from nav_tasks.core.handlers.navigate_to import NavigateToPoi, NavigateToPoiRequest
from nav_tasks.core.ports import Clock, DockBackend
from nav_tasks.core.registry import TaskContext
from nav_tasks.core.retry import MonotonicClock, RetryFailure, RetryPolicy
from nav_tasks.domain.errors import ConfigError
from nav_tasks.domain.tasks import DockGoal, NavOutcome, NavStatus
from nav_tasks.domain.types import PoseRecord, StoredPoi


@dataclass(frozen=True)
class DockRequest:
    """保存 Dock 目标及可选 POI，供旧诊断文本和状态关联使用.

    数据类自身不校验字段；直接位姿入口的 decode 只验证平面数值，
    POI 路由则直接包装仓库位姿，不经过该直接位姿解码器。

    Attributes:
        poi: 按标识解析的 Dock POI，直接位姿入站时可为 None。
        pose: 直接目标或 POI 路由提供的 Dock 位姿，不在数据类中校验。
    """

    pose: PoseRecord
    poi: StoredPoi | None = None


class Dock:
    """只执行一次托管 Dock 流程，对标量进度限频但不改变完整 ROS 反馈."""

    def __init__(
        self, backend: DockBackend, *, clock: Clock | None = None,
        progress_min_interval_sec: float = 1.0, retry_delay_sec: float = 1.0,
        on_failure: Callable[[TaskContext, RetryFailure], None] | None = None,
    ) -> None:
        """注入 Dock 后端和报告策略，不改变工作线程的两段式入位政策.

        Args:
            backend: 使用唯一共享运动后端的托管 Dock 端口。
            clock: 单调报告/耗时诊断时钟。
            progress_min_interval_sec: 旧标量进度间隔，不是原始反馈过滤器。
            retry_delay_sec: 保留在旧失败观察中的间隔来源；重试次数固定为零，不实际等待重试。
            on_failure: 可选同步单次 POI 失败观察者。

        Raises:
            ConfigError: 报告间隔或重试观察延迟为负或非有限。
            TypeError: 间隔不能参与有限性检查。
        """
        if not math.isfinite(progress_min_interval_sec) or progress_min_interval_sec < 0:
            raise ConfigError('progress_min_interval_sec must be finite and nonnegative')
        self._backend = backend
        self._clock = clock if clock is not None else MonotonicClock()
        self._interval = progress_min_interval_sec
        self._retry = RetryPolicy(0, retry_delay_sec)
        self._on_failure = on_failure

    def decode(self, payload: Mapping[str, object]) -> DockRequest:
        """在准入前检查直接 Dock 位姿的类型及 x/y/yaw 有限性.

        不验证 frame_id、z 或 stamp_sec；坐标系解释由入站/工作线程边界负责。

        Args:
            payload: pose 字段为已解码 PoseRecord 的领域载荷。

        Returns:
            保留原位姿对象的冻结 DockRequest，不产生坐标变换。

        Raises:
            ConfigError: 位姿类型错误，或平面坐标不是有限的非布尔 int/float。
        """
        pose = payload.get('pose')
        if not isinstance(pose, PoseRecord) or any(
                type(value) not in (int, float) or not math.isfinite(value)
                for value in (pose.x, pose.y, pose.yaw)):
            raise ConfigError('dock requires a PoseRecord with finite planar coordinates')
        return DockRequest(pose)

    async def run(self, context: TaskContext, payload: object) -> NavOutcome:
        """执行一次直接或 POI Dock，屏蔽结束后的反馈并检查取消后成功.

        POI 失败保留旧单次诊断包装，字符串中的 ABORTED 描述不是重新读取的
        Nav2 状态；直接位姿失败保留后端结果。后端异常不改写成普通失败。

        Args:
            context: 唯一任务所有者的循环内回调和身份。
            payload: DockRequest，可带 POI 来源用于诊断。

        Returns:
            完整 Dock 结果，不是预备点结果；POI 失败按旧格式包装总尝试次数为一。

        Raises:
            TypeError: 载荷不是 DockRequest。
            asyncio.CancelledError: 取消封闭到达结果和后续动作。
            Exception: 后端、清理、时钟或观察回调失败，不能当作普通 Dock 失败继续。
        """
        if not isinstance(payload, DockRequest):
            raise TypeError('Dock requires DockRequest')
        context.check_cancelled()
        context.describe(payload.poi.poi_id if payload.poi else 'dock_pose', 'to_anchor')
        context.start_segment()
        goal = DockGoal(payload.pose, context.task_id, context.sub_id, context.generation)
        started = self._clock.now()
        latest = None
        last_reported = 0.0
        accepting = True

        def feedback(distance):
            nonlocal latest, last_reported
            if not accepting or context.cancel_requested():
                return
            latest = float(distance)
            now = self._clock.now()
            if now - last_reported >= self._interval:
                last_reported = now
                context.feedback(latest)

        async def attempt():
            """执行一次 Dock，并为 POI 失败补齐冻结格式的诊断文本.

            后端返回后再次检查显式取消；非失败或直接位姿目标原样返回。
            POI 失败文本固定写 status=6(ABORTED)，并非重新读取后端原始状态；
            距离取最新反馈，耗时取本次技能的起始时间。本闭包不自行重试。

            Returns:
                原始结果或带兼容诊断的失败结果。

            Raises:
                asyncio.CancelledError: 任务取消，不转换成普通失败。
                Exception: 后端、时钟或上下文错误向外传播。
            """
            outcome = await self._backend.dock(goal, feedback)
            context.check_cancelled()
            if outcome.status != NavStatus.FAILED or payload.poi is None:
                return outcome
            poi, pose = payload.poi, payload.pose
            remaining = (f'{latest:.2f}m' if latest is not None
                         else 'n/a(无 feedback,可能 planner 直接失败)')
            elapsed = self._clock.now() - started
            return NavOutcome(NavStatus.FAILED, (
                f'Nav2 终态 status=6(ABORTED), target={poi.poi_id}({poi.name}) '
                f'goal={pose.frame_id}(x={pose.x:.2f}, y={pose.y:.2f}, yaw={pose.yaw:.2f}rad), '
                f'剩余距离={remaining}, 本次耗时={elapsed:.1f}s'))

        def audit(failure):
            if self._on_failure is not None:
                self._on_failure(context, failure)

        try:
            if payload.poi is None:
                return await attempt()
            return await self._retry.run(attempt, self._clock, audit)
        finally:
            accepting = False


class DockAwareNavigation:
    """在普通吸附和重试之前，把显式配置的 POI ID 路由到 Dock."""

    def __init__(self, navigation: NavigateToPoi, dock: Dock, dock_ids: tuple[str, ...]) -> None:
        """共用普通点位解码器，仅按显式标识集合选择 Dock.

        Args:
            navigation: 普通 POI 技能及其仓库解码器。
            dock: 托管单次 Dock 技能。
            dock_ids: 显式 Dock ID；不从名称推断，构造时保存为 frozenset。
        """
        self._navigation = navigation
        self._dock = dock
        self._dock_ids = frozenset(dock_ids)

    def decode(self, payload: Mapping[str, object]) -> NavigateToPoiRequest:
        """委托普通解码器在准入前取得 POI，不执行 Dock 位姿解码.

        Args:
            payload: navigate_to 的领域载荷。

        Returns:
            包装仓库 POI 的请求，保留未知 ID 的准入失败语义。

        Raises:
            ConfigError: 目标 ID 不合法。
            PoiNotFoundError: 目标 ID 不存在。
            Exception: 仓库查询失败。
        """
        return self._navigation.decode(payload)

    async def run(self, context: TaskContext, payload: object) -> NavOutcome:
        """在同一任务所有者下选择 Dock 或普通导航，只执行其中一个技能.

        Args:
            context: 共享的任务上下文。
            payload: 已准备的 POI 请求；其它类型交给普通技能按其类型检查拒绝。

        Returns:
            被选中技能的最终结果，不在路由层重试或吸附。

        Raises:
            TypeError: 普通技能收到不支持的载荷类型。
            asyncio.CancelledError: 被选技能取消。
            Exception: 被选技能失败。
        """
        if isinstance(payload, NavigateToPoiRequest) and payload.poi.poi_id in self._dock_ids:
            return await self._dock.run(context, DockRequest(payload.poi.map_pose, payload.poi))
        return await self._navigation.run(context, payload)
