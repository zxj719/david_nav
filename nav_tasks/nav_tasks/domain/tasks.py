"""Transport-independent requests, outcomes and immutable task observations."""

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from nav_tasks.domain.types import PoseRecord, StoredPoi


class NavStatus(str, Enum):
    """Terminal result of one navigation attempt, not an asyncio cancellation.

    Attributes:
        CANCELED: 后端报告取消，不等于 asyncio 协程已清理。
        FAILED: 单次导航失败。
        SUCCEEDED: 单次导航成功。
    """

    SUCCEEDED = 'succeeded'
    CANCELED = 'canceled'
    FAILED = 'failed'


@dataclass(frozen=True)
class NavOutcome:
    """Terminal status and human-readable failure detail from a backend or skill.

    Attributes:
        detail: 失败或终态的可读说明。
        status: 导航后端或技能的终态分类。
    """

    status: NavStatus
    detail: str = ''

    @property
    def succeeded(self) -> bool:
        """Whether this attempt reached its target successfully."""
        return self.status == NavStatus.SUCCEEDED


@dataclass(frozen=True)
class NavGoal:
    """One navigation target and request identity used only for correlation.

    Attributes:
        poi: 本次导航目标 POI。
        request_id: 上游请求标识，用于关联事件。
        sub_id: 上游子任务编号。
    """

    poi: StoredPoi
    request_id: str
    sub_id: int = 0


@dataclass(frozen=True)
class TaskCompletion:
    """One post-cleanup receipt, separate from legacy progress/terminal wire events.

    Attributes:
        generation: 区分重复任务标识的所有权代次。
        outcome: 技能最终结果；此回执在清理后生成。
        sub_id: 该任务的子任务编号。
        task_id: 已清理任务的标识。
    """

    task_id: str
    sub_id: int
    generation: int
    outcome: NavOutcome


@dataclass(frozen=True)
class SpinGoal:
    """Signed relative yaw and Nav2's own execution allowance, both in SI units.

    The allowance is sent to the server, not a client cleanup deadline. A skill
    may separately impose its attempt deadline and must then await cancellation.

    Attributes:
        target_yaw: 有符号相对旋转角（弧度），不是绝对地图朝向。
        time_allowance_sec: 发给 Nav2 的执行预算（秒），不是客户端清理期限。
    """

    target_yaw: float
    time_allowance_sec: float


@dataclass(frozen=True)
class DockGoal:
    """Planar Dock target and owner identity for complete feedback correlation.

    Attributes:
        generation: 关联当前任务所有权的代次。
        pose: Dock 的目标位姿。
        request_id: 上游请求标识，用于关联事件。
        sub_id: 上游子任务编号。
    """

    pose: PoseRecord
    request_id: str
    sub_id: int = 0
    generation: int = 0


@dataclass(frozen=True)
class ResolvedGoal:
    """A navigable POI and whether it differs from the original anchor.

    Attributes:
        poi: 已选定的可导航 POI。
        used_snap: 是否使用了区别于原始锚点的吸附目标。
    """

    poi: StoredPoi
    used_snap: bool


class PreemptPolicy(str, Enum):
    """Ingress-selected admission policy; implicit motion preemption is forbidden.

    Attributes:
        ALLOW: 允许在完成旧任务清理后接纳新任务。
        REJECT: 已有运动任务时拒绝新任务。
    """

    ALLOW = 'allow'
    REJECT = 'reject'


class TaskState(str, Enum):
    """Motion ownership state independent of transport connectivity.

    Attributes:
        ABORTING: 正在取消并等待清理。
        CLOSED: 任务入口已关闭。
        ERROR: 任务/清理发生错误，不能据此推断停车或释放。
        IDLE: 没有活动运动任务。
        NAVIGATING: 任务正在执行或保留其运动所有权。
        STARTING: 已接纳任务，正在启动。
    """

    IDLE = 'idle'
    STARTING = 'starting'
    NAVIGATING = 'navigating'
    ABORTING = 'aborting'
    ERROR = 'error'
    CLOSED = 'closed'


@dataclass(frozen=True)
class TaskRequest:
    """Motion request; adapters translate their wire values before admission.

    Payload decoding is synchronous and completes before an existing task is
    preempted. Decoders must return a detached immutable domain object. Directory
    updates and POI marking are administrative operations, not motion requests.
    Source is opaque ingress metadata preserved in events; empty retains legacy use.

    Attributes:
        action: 注册处理器对应的动作名称。
        payload: 入站载荷；接纳前同步解码为独立领域对象。
        preempt: 明确指定的抢占接纳策略。
        source: 不解释内容的入站来源标签，空值保留旧行为。
        sub_id: 子任务编号。
        task_id: 入站任务标识。
    """

    task_id: str
    action: str
    payload: Mapping[str, object]
    sub_id: int = 0
    preempt: PreemptPolicy = PreemptPolicy.REJECT
    source: str = ''


@dataclass(frozen=True)
class TaskSnapshot:
    """Read-only ownership observation; generations distinguish reused task IDs.

    Attributes:
        generation: 用于区分重复标识的所有权代次。
        phase: 技能报告的当前阶段，未知时为 None。
        remaining_distance: 最近报告的剩余距离（米），不是到达证明。
        state: 当前任务所有权状态。
        sub_id: 当前子任务编号。
        target_id: 当前目标 POI 标识，未知时为 None。
        task_id: 当前任务标识，空闲时可为 None。
    """

    state: TaskState
    task_id: str | None = None
    sub_id: int = 0
    generation: int = 0
    remaining_distance: float = 0.0

    target_id: str | None = None
    phase: str | None = None


class EventKind(str, Enum):
    """Domain event categories, translated to wire frames by an ingress adapter.

    Attributes:
        FAILED: 任务失败事件。
        PROGRESS: 任务进度事件。
        SUCCEEDED: 任务成功事件。
    """

    PROGRESS = 'progress'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'


@dataclass(frozen=True)
class TaskEvent:
    """Correlated event with legacy progress semantics and opaque ingress source metadata.

    Attributes:
        detail: 终态或失败说明。
        generation: 关联的所有权代次。
        kind: 领域事件分类，由入站适配器转换为线协议。
        remaining_distance: 报告的剩余距离（米）。
        source: 原样保留的入站来源标签。
        status: 沿用旧协议的 running/paused 进度状态。
        sub_id: 关联的子任务编号。
        task_id: 关联的任务标识。
    """

    kind: EventKind
    task_id: str
    sub_id: int
    generation: int
    remaining_distance: float = 0.0
    status: str = 'running'
    detail: str = ''
    source: str = ''
