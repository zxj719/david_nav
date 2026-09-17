"""提供进程内入站适配器共享的公共任务、点位和组装契约.

传输层通过 TaskApi/PoiApi 调用领域逻辑；组装根可使用这里的导出注册技能，
无需直接依赖内核路径。导入本模块不启动 ROS、线程或传输，具体资源只在
显式工厂调用时延迟加载。Protocol 描述实现契约，本身不执行清理或验证。
"""

import asyncio
import time
from typing import Callable, Iterable, Mapping, Protocol

from nav_tasks.core.handlers.poi_commands import MarkPoiResult
from nav_tasks.core.ports import PoseSource
from nav_tasks.core.registry import CommandRegistry, TaskContext
from nav_tasks.core.state import TaskManager
from nav_tasks.domain.errors import (
    BusyError, ClosedError, ConfigError, NavTaskError, PoseUnavailableError, RuntimeAssemblyError,
)
from nav_tasks.domain.runtime_config import DockNodeConfig, DockRuntimeConfig, RuntimeConfig
from nav_tasks.domain.tasks import (
    DockGoal, EventKind, NavOutcome, NavStatus, PreemptPolicy, TaskCompletion, TaskEvent,
    TaskRequest, TaskSnapshot, TaskState,
)
from nav_tasks.domain.types import CapturedPose, PoseRecord, StoredPoi, Transform2D


class TaskApi(Protocol):
    """定义限定在同一事件循环的运动所有权接口，点位管理另设接口.

    取消调用方的等待不等于撤回已托管的准入或取消操作；须显式取消任务。
    成功/失败事件不独立证明物理停稳，所有权释放以实现的清理契约为准。
    """

    async def submit(
        self, task: TaskRequest, *, on_settled: Callable[[TaskCompletion], None] | None = None,
    ) -> TaskSnapshot:
        """解码并接纳任务，不等待导航终态；抢占时仍需等待旧任务清理.

        Args:
            task: 领域请求，含显式抢占策略；同步解码先于抢占。
            on_settled: 所有权释放后在任务循环调用一次的可选同步回执回调。

        Returns:
            接纳时的快照，不保证调用方收到时技能尚未开始或结束。

        Raises:
            ConfigError: 请求身份、来源、策略或回执回调不合法。
            ClosedError: 入口已封闭，含等待抢占清理期间关闭。
            BusyError: 已有所有者且请求不允许抢占。
            UnknownActionError: 请求动作未注册。
            RuntimeError: 不在所有者事件循环调用。
            asyncio.CancelledError: 调用方停止等待；已托管准入仍继续。
            Exception: 解码或旧任务清理失败。
        """
        ...

    async def cancel(self, task_id: str | None = None, *, generation: int | None = None) -> bool:
        """匹配任务身份并等待取消清理，不把取消请求已接收当作完成.

        Args:
            task_id: 需要匹配的任务 ID；None 表示不限 ID。
            generation: 可选准入代次，与 ID 在转换锁内共同匹配。

        Returns:
            匹配且清理成功为 True；无活动任务或身份不匹配为 False。

        Raises:
            RuntimeError: 不在所有者事件循环调用。
            asyncio.CancelledError: 调用方停止等待；已托管清理仍归管理器。
            Exception: 后端或技能收尾失败，不能据此释放所有权。
        """
        ...

    def status(self) -> TaskSnapshot:
        """读取当前软件运动所有权，不进行实机位置或停稳测量.

        Returns:
            独立冻结的状态快照；没有活动任务时不保留上一任务身份。

        Raises:
            RuntimeError: 不在所有者事件循环调用。
        """
        ...

    def subscribe_events(self) -> asyncio.Queue[TaskEvent]:
        """订阅后续领域事件，不重放历史，不承诺持久交付.

        当前管理器使用无容量上限的内存队列；调用方应持续消费并在结束时解除订阅。
        关闭不向队列放结束标记，消费者不能仅靠 get 等待获知关闭。

        Returns:
            在任务循环使用的事件队列。

        Raises:
            ClosedError: 准入已开始关闭。
            RuntimeError: 不在所有者事件循环调用。
        """
        ...

    def unsubscribe_events(self, queue: asyncio.Queue[TaskEvent]) -> None:
        """解除事件消费者订阅，不清空队列中已入队的事件.

        Args:
            queue: 先前订阅的队列；当前实现忽略未登记队列。

        Raises:
            RuntimeError: 不在所有者事件循环调用。
        """
        ...

    async def close(self) -> None:
        """执行协程时封闭准入并等待自有运动清理，不重新开放入口.

        并发调用加入同一收尾；失败可再次调用重试，不能因等待取消提前销毁后端。

        Raises:
            RuntimeError: 不在所有者事件循环调用。
            asyncio.CancelledError: 调用方停止等待，已托管收尾不因此撤销。
            Exception: 运动清理失败，仍需保留所有者和后端。
        """
        ...


class DockIngressApi(Protocol):
    """定义 Dock ROS 入口的资源所有者，宿主负责多线程回调和分阶段关闭.

    先在任务循环构造空所有者，再 open；stop/close 保留资源，宿主确认回调
    静默后才 destroy。接口本身不能验证宿主 executor 已无回调尾部。
    """

    def open(self, node: object) -> None:
        """创建目标订阅和动作入口，不创建新运动客户端或 ROS 宿主.

        Args:
            node: 外部持有的节点，必须保留到入口销毁完成。

        Raises:
            RuntimeError: 所有者循环不匹配、已经打开或正在关闭。
            Exception: 资源构造失败；保留所有者并关闭已完成部分。
        """
        ...

    def stop(self) -> None:
        """在任务循环封闭本入口准入并唤醒自有请求，不停止 ROS 回调或销毁资源.

        Raises:
            RuntimeError: 不在所有者事件循环调用。
        """
        ...

    async def close(self) -> None:
        """等待本入口自有请求收尾，仍保留 ROS 资源供回调继续执行.

        活跃收尾供并发调用共用；已终止但尚未关闭的收尾可重试，不丢弃失败入口。

        Raises:
            RuntimeError: 不在所有者事件循环调用。
            asyncio.CancelledError: 等待者或入口收尾被取消，不证明可销毁。
            Exception: 入口收尾失败，宿主应保留所有者和回调能力。
        """
        ...

    def destroy(self) -> None:
        """在关闭成功且宿主回调已静默后释放入口资源，不销毁外部节点.

        宿主负责先确认 executor 静默；本方法只检查内部关闭状态，不能替代该证明。
        已成功释放的阶段不重复释放，失败阶段保留供重试。

        Raises:
            RuntimeError: 循环不匹配、尚未关闭或订阅销毁未确认。
            Exception: ROS 资源销毁失败。
        """
        ...

    def on_feedback(self, goal: DockGoal, raw: object, extra_distance: float) -> None:
        """仅向身份和代次匹配的活动 Dock 动作转发完整反馈.

        当前实现保留原反馈对象，复制字段后叠加后续阶段距离；转换或发布异常会
        标记本入口失败并唤醒收尾，不把丢失反馈当作任务成功。

        Args:
            goal: 任务循环中的 Dock 所有者身份，含准入代次。
            raw: 原始 ROS 导航反馈封装，不在此修改。
            extra_distance: 后续阶段剩余距离，单位米。

        Raises:
            RuntimeError: 不在所有者事件循环调用。
            Exception: 任务状态读取或异常报告自身失败。
        """
        ...


def create_dock_ingress(
    tasks: TaskApi, config: DockNodeConfig, *, cleanup_retry_sec: float = 1.0,
) -> DockIngressApi:
    """在当前任务循环创建未打开 ROS 资源的 Dock 入口所有者.

    不创建新运动客户端。宿主应让至少两线程的 MultiThreadedExecutor 保持
    回调能力直至 close 成功，再确认其静默后 destroy；open 失败也须保留句柄。

    Args:
        tasks: 当前循环的共享公共任务管理器。
        config: 显式 Dock 入口名称和运行策略。
        cleanup_retry_sec: 自有任务取消失败后的重试间隔，单位秒，必须有限且为正。

    Returns:
        尚未 open 的资源所有者。

    Raises:
        ConfigError: 接线、策略或重试间隔不合法。
        RuntimeError: 当前没有正在运行的事件循环。
    """
    from nav_tasks.adapters.ros.dock_ingress import DockIngress

    return DockIngress(tasks, config, cleanup_retry_sec=cleanup_retry_sec)


class PoiApi(Protocol):
    """定义不占用或抢占运动所有权的点位管理接口."""

    async def mark(self, poi_id: str, name: str) -> MarkPoiResult:
        """捕获并写入命名 POI，不抢占导航或额外等待任务空闲.

        当前实现只按地图位姿年龄大于阈值拒绝，不额外拒绝未来时间戳或 NaN；
        写入异常不转换为成功结果。成功返回不是外部传输 ACK。

        Args:
            poi_id: 入站已经解码的点位标识。
            name: 入站已经解码的显示名称。

        Returns:
            包含已存 POI 或可读捕获/过期错误的结果对象。

        Raises:
            asyncio.CancelledError: 捕获协程被取消。
            Exception: 时间读取、捕获后字段处理或仓库存储失败。
        """
        ...

    def update_directory(self, id_to_name: Iterable[tuple[str, str]]) -> None:
        """按输入顺序更新目录名称，不创建未测量的位姿.

        Args:
            id_to_name: 已解码的标识/名称对；迭代和存储行为由仓库实现负责。

        Raises:
            Exception: 输入迭代或仓库更新失败。
        """
        ...


class TaskRuntimeApi(Protocol):
    """定义进程内任务服务及其资源生命周期，ROS 宿主和入口仍由外层持有.

    先封闭入站、保持 ROS 回调直至 settle 成功；多线程宿主再静默回调，
    最后 close 释放客户端/订阅。关闭失败须保留句柄，不凭超时销毁宿主。

    Attributes:
        pois: 点位管理接口，提供捕获、标记和目录更新。
        poses: 共享位姿来源接口。
        tasks: 共享任务管理接口，拥有运动任务的接纳和清理。
    """

    tasks: TaskApi
    pois: PoiApi
    poses: PoseSource

    def start_loop_closure(self) -> None:
        """启动或复用一项受监督的点位闭环重投影观察任务.

        连接生命周期由调用方选择；不会启动定位栈或发送运动目标。

        Raises:
            ClosedError: 运行时已开始关闭。
            RuntimeError: 不在所有者事件循环调用。
        """
        ...

    async def stop_loop_closure(self) -> None:
        """取消并等待点位观察任务，不取消运动任务.

        当前实现消费观察任务的普通异常和 CancelledError，避免它们阻断后续清理；
        调用方取消等待也可能被消费，不应据返回推断运动所有权已释放。

        Raises:
            RuntimeError: 不在所有者事件循环调用。
        """
        ...

    async def settle(self) -> None:
        """封闭准入并确认运动收尾，保留客户端和订阅供后续分阶段销毁.

        外层先停止入站并保持 ROS 回调能力；并发调用共用收尾，等待取消不撤销它。
        某个顺序清理步骤失败会中止后续步骤，保留所有者供重试。

        Raises:
            RuntimeError: 不在所有者事件循环调用。
            asyncio.CancelledError: 调用方停止等待，不代表底层清理完成。
            Exception: 运动收尾失败。
        """
        ...

    async def close(self) -> None:
        """先确认运动收尾，再依序释放自有资源；失败保留句柄供重试.

        外部节点/listener/executor 不归本接口销毁。已完成的部分资源可能已经关闭，
        重试不是事务回滚；不能把异常解释为所有资源都保持原状。

        Raises:
            RuntimeError: 不在所有者事件循环调用。
            asyncio.CancelledError: 调用方停止等待，托管收尾仍归运行时。
            Exception: 运动或资源清理失败。
        """
        ...


def _runtime_log(message: str) -> None:
    print(f'[navigation_executor] {message}', flush=True)


def dock_config_from_parameters(
    parameters: Mapping[str, object], *, spin_action_name: str = 'spin',
    pose_timeout_sec: float = 2.0,
) -> DockNodeConfig:
    """转换并验证完整旧 Dock 参数，不打开运行时资源.

    Args:
        parameters: 恰好十六个旧 ROS 参数；输入 footprint 仍是字符串。
        spin_action_name: 共享 Spin 端点，默认保留旧相对名 spin。
        pose_timeout_sec: TF 获取预算，默认保留旧两秒值。

    Returns:
        已通过参数/策略检查且与输入映射分离的冻结入口配置；
        输出 footprint 是解析后的坐标元组，不再是输入字符串。

    Raises:
        ConfigError: 参数缺失、多余、格式错误或超出受检范围。
        OverflowError: 极大整数轮廓坐标不能完成浮点有限性检查。
    """
    from nav_tasks.app.dock_parameters import from_parameters

    return from_parameters(parameters, spin_action_name=spin_action_name,
                           pose_timeout_sec=pose_timeout_sec)


def declare_dock_parameters(
    node: object, defaults: Mapping[str, object], *, spin_action_name: str = 'spin',
    pose_timeout_sec: float = 2.0,
) -> DockNodeConfig:
    """延迟声明原有十六个 Dock 参数，并应用 ROS 覆盖后再次验证.

    不创建节点、订阅或动作客户端。默认值在第一次声明前校验；
    声明或生效值检查失败时，已声明参数由节点所有者负责处理。

    Args:
        node: 外部持有的 ROS 节点，失败后也由调用方保留和清理。
        defaults: 完整旧默认值映射，输入轮廓保持字符串格式。
        spin_action_name: 共享后端端点，不是新增 ROS 参数。
        pose_timeout_sec: TF 获取预算，不是新增 ROS 参数。

    Returns:
        经 ROS 覆盖和检查的冻结生效配置。

    Raises:
        ConfigError: 默认值或生效值不合法。
        Exception: 参数声明、读取或转换失败，调用方仍负责节点。
    """
    from nav_tasks.adapters.ros.dock_parameters import declare_parameters

    return declare_parameters(node, defaults, spin_action_name=spin_action_name,
                              pose_timeout_sec=pose_timeout_sec)


async def create_runtime(
    config: RuntimeConfig, node: object, tf_buffer: object, *,
    log: Callable[[str], None] = _runtime_log, now: Callable[[], float] = time.time,
    on_dock_feedback: Callable[[DockGoal, object, float], None] | None = None,
) -> TaskRuntimeApi:
    """延迟组装任务服务，让入站代码只依赖公共接口.

    导入本 API 不加载 ROS/YAML/传输实现；调用工厂会加载具体适配器并访问
    仓库、创建自有观察/动作资源，但不初始化 ROS、创建宿主节点/spin 或发送目标。
    外层保留 node/listener/spin 至关闭成功；Dock IDs 缺少配置时拒绝而不退回普通导航。

    Args:
        config: 显式机器人/模式配置；冻结值对象不等于已完成工厂验证。
        node: 外部创建和持有的 ROS 节点。
        tf_buffer: 外部持有的 TF 缓冲与监听连接。
        log: 不含前缀的启动和 POI 日志接收器。
        now: 与位姿时间戳同一时间基准的新鲜度时钟。
        on_dock_feedback: 可选任务循环回调，接收完整反馈及后续阶段距离。

    Returns:
        公共任务、点位和位姿接口及自有资源生命周期句柄。

    Raises:
        ConfigError: 配置未通过组装边界检查。
        RuntimeAssemblyError: 回滚未完成；保留异常并在销毁节点前成功 await cleanup。
        Exception: 仓库或资源组装失败；工厂尚未接纳运动任务。
    """
    from nav_tasks.app.runtime import assemble_runtime

    return await assemble_runtime(config, node, tf_buffer, log=log, now=now,
                                  on_dock_feedback=on_dock_feedback)


__all__ = [
    'BusyError', 'CapturedPose', 'ClosedError', 'CommandRegistry', 'ConfigError',
    'DockGoal', 'DockIngressApi', 'DockNodeConfig', 'DockRuntimeConfig', 'EventKind',
    'MarkPoiResult',
    'NavOutcome', 'NavStatus', 'NavTaskError', 'PoiApi',
    'PoseRecord', 'PoseUnavailableError', 'PreemptPolicy', 'StoredPoi', 'TaskApi',
    'TaskCompletion', 'TaskContext', 'TaskEvent',
    'TaskManager', 'TaskRequest', 'TaskSnapshot', 'TaskState', 'Transform2D',
    'PoseSource', 'RuntimeAssemblyError', 'RuntimeConfig', 'TaskRuntimeApi', 'create_runtime',
    'declare_dock_parameters', 'dock_config_from_parameters',
    'create_dock_ingress',
]
