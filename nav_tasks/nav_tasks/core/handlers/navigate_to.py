"""实现共享单一运动后端的顺序 POI 导航、吸附分段与失败重试."""

from dataclasses import dataclass
import math
from typing import Callable, Mapping

from nav_tasks.core.ports import Clock, GoalResolver, NavigationBackend, PoiRepository
from nav_tasks.core.registry import TaskContext
from nav_tasks.core.retry import MonotonicClock, RetryFailure, RetryPolicy
from nav_tasks.domain.errors import ConfigError, PoiNotFoundError
from nav_tasks.domain.tasks import NavGoal, NavOutcome, NavStatus
from nav_tasks.domain.types import StoredPoi


@dataclass(frozen=True)
class NavigateToPoiRequest:
    """保存准入前按 ID 取得的锚点值，不自行校验坐标或深复制仓库对象.

    Attributes:
        poi: 按 ID 从仓库取得的导航 POI，此值对象不验证几何。
    """

    poi: StoredPoi


class NavigateToPoi:
    """解析锚点/吸附目标并顺序执行各段重试，不通过嵌套回调创建第二任务."""

    def __init__(
        self, backend: NavigationBackend, pois: PoiRepository, resolver: GoalResolver,
        retry: RetryPolicy = RetryPolicy(), clock: Clock | None = None,
        progress_min_interval_sec: float = 1.0,
        on_failure: Callable[[TaskContext, RetryFailure], None] | None = None,
    ) -> None:
        """注入共享端口、反馈间隔和单段重试策略，不创建动作客户端.

        Args:
            backend: 共享单次尝试后端，不是每技能独有客户端。
            pois: 在准入前按 ID 查询点位的仓库。
            resolver: 锚点/吸附策略；Dock 应在进入此技能前单独路由。
            retry: 每段导航分别使用的失败重试策略。
            clock: 可选单调间隔时钟，默认 MonotonicClock。
            progress_min_interval_sec: 每次尝试的最小反馈报告间隔，单位秒。
            on_failure: 可选同步失败观察者，不在此改变重试政策。

        Raises:
            ConfigError: 反馈间隔为负或非有限。
            TypeError: 间隔不支持有限性检查。
        """
        if not math.isfinite(progress_min_interval_sec) or progress_min_interval_sec < 0:
            raise ConfigError('progress_min_interval_sec must be finite and nonnegative')
        self._backend = backend
        self._pois = pois
        self._resolver = resolver
        self._retry = retry
        self._clock = clock if clock is not None else MonotonicClock()
        self._progress_interval = progress_min_interval_sec
        self._on_failure = on_failure

    def decode(self, payload: Mapping[str, object]) -> NavigateToPoiRequest:
        """在抢占已有任务前验证目标 ID 并查询仓库，不在此校验位姿几何.

        Args:
            payload: 含 target_id 的领域字段映射，不接收线协议消息类型。

        Returns:
            包装仓库所返回 POI 的冻结请求，不自动复制或规范化其坐标。

        Raises:
            ConfigError: ID 缺失、为空白或不是字符串。
            PoiNotFoundError: 仓库中不存在该 ID。
            Exception: 仓库查询失败。
        """
        target = payload.get('target_id')
        if not isinstance(target, str) or not target.strip():
            raise ConfigError('navigate_to requires a nonempty target_id')
        poi = self._pois.get(target)
        if poi is None:
            raise PoiNotFoundError(f'目标点不存在: {target}')
        return NavigateToPoiRequest(poi)

    async def run(self, context: TaskContext, payload: object) -> NavOutcome:
        """先导航到选定目标；吸附成功且锚点重新准入时才追加锚点段.

        吸附后锚点仍不准入时保留吸附段的成功结果，不把它误写为必然到达原锚点。
        各段失败独立套用重试预算；此技能不处理 Dock 两段式入位。

        Args:
            context: 任务身份及受代次保护的循环内回调。
            payload: decode 生成的 NavigateToPoiRequest。

        Returns:
            最后一段的结果；无候选为失败，吸附成功但锚点不准入仍为成功。

        Raises:
            TypeError: 注册表提供的领域载荷类型错误。
            asyncio.CancelledError: 取消传播并封闭重试和下一分段。
            Exception: 解析器、后端、回调或重试时钟失败。
        """
        if not isinstance(payload, NavigateToPoiRequest):
            raise TypeError('NavigateToPoi requires NavigateToPoiRequest')
        context.check_cancelled()
        anchor = payload.poi
        resolved = self._resolver.resolve(anchor)
        if resolved is None:
            return NavOutcome(NavStatus.FAILED, '目标点不可达，且未找到可行扩圈点')
        phase = 'to_snap' if resolved.used_snap else 'to_anchor'
        outcome = await self._navigate(context, resolved.poi, phase)
        if outcome.succeeded and resolved.used_snap and self._resolver.is_admissible(anchor):
            outcome = await self._navigate(context, anchor, 'to_anchor')
        return outcome

    async def _navigate(self, context: TaskContext, poi: StoredPoi, phase: str) -> NavOutcome:
        """执行一个导航段及其失败重试，并封闭已结束尝试的迟到反馈.

        Args:
            context: 当前任务身份、取消检查和米制进度回调。
            poi: 该段的不可变目标快照。
            phase: to_snap 或 to_anchor，用于任务状态与观察事件。

        Returns:
            重试策略给出的最终导航结果；本函数不决定是否继续去锚点。

        Raises:
            asyncio.CancelledError: 任务取消，禁止重试和后续导航段。
            Exception: 后端、观察函数或重试等待失败，由任务管理器处理。
        """
        context.check_cancelled()
        context.describe(poi.poi_id, phase)
        context.start_segment()
        goal = NavGoal(poi, context.task_id, context.sub_id)

        async def attempt() -> NavOutcome:
            """执行一次导航，并用独立反馈开关隔离上一轮尝试的迟到回调.

            每次重置节流时间与接收开关，在后端调用前后检查取消。
            finally 始终关闭该尝试的反馈；同任务 ID 的重试不能仅靠代际隔离。
            本闭包不决定重试预算，策略由外层 RetryPolicy 持有。

            Returns:
                后端单次导航结果。

            Raises:
                asyncio.CancelledError: 前后取消检查或后端传播的取消。
                Exception: 后端、时钟或反馈上下文的异常。
            """
            context.check_cancelled()
            last_reported = 0.0
            accepting = True

            def feedback(remaining: float) -> None:
                nonlocal last_reported
                if not accepting:
                    return
                now = self._clock.now()
                if now - last_reported < self._progress_interval:
                    return
                last_reported = now
                context.feedback(float(remaining))

            try:
                outcome = await self._backend.navigate(goal, feedback)
                context.check_cancelled()
                return outcome
            finally:
                # A callback from a previous retry/segment shares the task ID,
                # so the manager generation fence alone cannot reject it.
                accepting = False

        def observe(failure: RetryFailure) -> None:
            if self._on_failure is not None:
                self._on_failure(context, failure)

        return await self._retry.run(attempt, self._clock, observe)
