"""定义不依赖传输的任务请求、结果和冻结观察值.

数据类本身不执行任务、不验证字段，也不深冻结外部传入的可变载荷。
状态和事件描述软件所有权/后端结果，不能单凭它们证明实机到达或停稳。
"""

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from nav_tasks.domain.types import PoseRecord, StoredPoi


class NavStatus(str, Enum):
    """表示单次导航尝试的后端或技能终态，不表示 asyncio 清理完成.

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
    """保存后端或技能给出的终态与可读说明，不独立验证目标到达.

    Attributes:
        detail: 失败或终态的可读说明。
        status: 导航后端或技能的终态分类。
    """

    status: NavStatus
    detail: str = ''

    @property
    def succeeded(self) -> bool:
        """判断保存的状态是否等于 SUCCEEDED，不重新测量位置或停稳.

        Returns:
            status 与 NavStatus.SUCCEEDED 比较的布尔结果。
        """
        return self.status == NavStatus.SUCCEEDED


@dataclass(frozen=True)
class NavGoal:
    """保存单次导航目标和用于关联反馈的请求标识，不自行下发动作.

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
    """表示管理器释放任务所有权后交付的回执，区别于旧线协议终态事件.

    管理器按清理契约交付回执；单独构造该数据类不会执行清理或核验物理停车。

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
    """保存有符号相对旋转角和发给 Nav2 服务端的执行预算.

    预算不是客户端清理期限；技能可另设尝试期限，超时后仍须等待取消收尾。
    本类只保存字段，不执行旋转或验证预算合法性。

    Attributes:
        target_yaw: 有符号相对旋转角（弧度），不是绝对地图朝向。
        time_allowance_sec: 发给 Nav2 的执行预算（秒），不是客户端清理期限。
    """

    target_yaw: float
    time_allowance_sec: float


@dataclass(frozen=True)
class DockGoal:
    """保存 Dock 目标位姿及完整反馈关联所需的所有权标识.

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
    """保存解析器选定的 POI 和是否使用吸附结果的标记.

    此值对象不自行验证路径可达性或比较原始锚点；可行性检查由解析器负责。

    Attributes:
        poi: 由解析器选定的目标 POI；数据类本身不证明可达。
        used_snap: 解析器是否标记使用了吸附结果，本类不自行计算差异。
    """

    poi: StoredPoi
    used_snap: bool


class PreemptPolicy(str, Enum):
    """表示入站明确选择的任务准入策略，枚举本身不执行取消或抢占.

    Attributes:
        ALLOW: 允许在完成旧任务清理后接纳新任务。
        REJECT: 已有运动任务时拒绝新任务。
    """

    ALLOW = 'allow'
    REJECT = 'reject'


class TaskState(str, Enum):
    """表示与传输连接独立的任务运动所有权状态.

    状态由管理器维护；IDLE 或 CLOSED 不能单独用作实机静止的传感器证明。

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
    """保存适配器提交的运动请求，冻结字段不等于复制或深冻结 payload.

    管理器在抢占旧任务前同步调用注册解码器；解码器按契约应返回独立的不可变
    领域对象，但该数据类和注册表不会自动深复制或验证其不可变性。
    目录更新和 POI 标记属于管理操作，不是运动请求。source 是原样保留的
    入站元数据，空值保持旧行为。

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
    """保存任务所有权观察值，用代次区分重复使用的任务标识.

    这是软件状态快照，不证明真实目标到达、测量新鲜度或物理停稳。

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
    """定义领域事件分类，由入站适配器转换为各自的线协议帧.

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
    """保存关联任务代次的领域事件、旧进度语义和不解释内容的来源标签.

    该对象不是序列化线协议帧，也不表示已向外部服务器送达。

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
