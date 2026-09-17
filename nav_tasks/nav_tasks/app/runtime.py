"""组装任务服务并持有其资源，不接管外部 ROS executor 或传输生命周期."""

import asyncio
from contextlib import suppress
import inspect
import logging
from pathlib import Path
from typing import Callable

from nav_tasks.adapters.ros.dock_observations import DockObservations
from nav_tasks.adapters.ros.dock_worker import DockWorker
from nav_tasks.adapters.ros.event_publisher import NavigationEvents
from nav_tasks.adapters.ros.grid_source import RosGridSource
from nav_tasks.adapters.ros.tf_pose_source import TfPoseSource
from nav_tasks.app.dock_config import workflow_config
from nav_tasks.app.logging import RetryAudit
from nav_tasks.app.loop_closure import watch_loop_closure
from nav_tasks.core.goal_resolution import GridGoalResolver, SnapSearch
from nav_tasks.core.handlers.dock import Dock, DockAwareNavigation
from nav_tasks.core.handlers.navigate_to import NavigateToPoi
from nav_tasks.core.handlers.poi_commands import PoiCommands
from nav_tasks.core.registry import CommandRegistry
from nav_tasks.core.retry import RetryPolicy
from nav_tasks.core.state import TaskManager
from nav_tasks.core.supervisor import TaskSupervisor
from nav_tasks.domain.errors import ClosedError, ConfigError, RuntimeAssemblyError
from nav_tasks.domain.runtime_config import RuntimeConfig
from nav_tasks.domain.tasks import DockGoal
from nav_tasks.nav2_client import Nav2Client
from nav_tasks.nav_site_data.named_poses_importer import import_named_poses
from nav_tasks.nav_site_data.yaml_poi_repository import YamlPoiRepository

_LOGGER = logging.getLogger(__name__)


class _Rollback:
    """暂存构造阶段的逆序清理回调，成功构造后把资源交给最终所有者.

    回滚串行执行并保留失败回调；不管理文件写入事务，也不销毁外部 ROS 宿主。
    注册与使用由组装过程在同一循环协调，不提供任意跨线程修改保证。
    """

    def __init__(self):
        self._callbacks: list[Callable[[], object]] = []
        self._lock = asyncio.Lock()

    async def __aenter__(self):
        return self

    async def __aexit__(self, error_type, error, traceback):
        """成功时移交所有权，构造异常时尝试全部已登记资源的逆序回滚.

        嵌套构造异常的 cleanup 先加入回滚。清理成功时不抑制原构造异常；
        清理失败（包括取消）则包装原构造原因并保留可重试回调。

        Args:
            error_type: 上下文协议提供的异常类型，本实现不单独使用。
            error: 构造异常实例，正常退出时为 None。
            traceback: 上下文协议提供的 traceback，本实现不单独使用。

        Raises:
            RuntimeAssemblyError: 回滚失败，包含原构造原因与本对象的 cleanup。
        """
        if error is None:
            self._callbacks.clear()
            return
        if isinstance(error, RuntimeAssemblyError):
            self.track(error.cleanup)
        try:
            await self.cleanup()
        except BaseException:
            raise RuntimeAssemblyError(error, self.cleanup) from error

    def track(self, callback):
        """在回滚队列头部登记清理回调，使后构造资源先清理.

        不立即调用、不去重；调用方负责为已取得的资源登记正确清理职责。

        Args:
            callback: 无参清理函数，返回可等待对象时由 cleanup 等待。
        """
        self._callbacks.insert(0, callback)

    async def cleanup(self):
        """持锁尝试当前回调快照，移除成功项并保留失败项供以后重试.

        回调中的 BaseException（包括取消）不跳过剩余快照项；全部尝试后
        重新抛出最后一次失败。锁等待阶段取消会直接传播，不代表已执行清理。
        这里只确认回调正常返回，不独立核验实际资源状态。

        Raises:
            BaseException: 等待锁被取消，或回调失败后重抛最后一次异常。
        """
        async with self._lock:
            failure = None
            for callback in tuple(self._callbacks):
                try:
                    result = callback()
                    if inspect.isawaitable(result):
                        await result
                except BaseException as error:
                    failure = error
                else:
                    self._callbacks.remove(callback)
            if failure is not None:
                raise failure


class TaskRuntime:
    """持有已组装任务资源，外层保留 ROS 节点、回调与入口生命周期.

    构造只接收既有资源，不启动运动或回环观察任务。公共调用方应经
    nav_tasks.api.create_runtime 返回的 TaskRuntimeApi 使用本对象。
    收尾按步骤执行，失败时仍须保留本对象，不能把返回异常等同于资源释放。

    Attributes:
        pois: 捕获、标记与目录更新接口，共享位姿及存储端口。
        poses: 共享位姿来源接口。
        tasks: 唯一共享任务管理接口，负责运动任务接纳与清理。
    """

    def __init__(
        self, tasks: TaskManager, pois: PoiCommands, poses: TfPoseSource,
        grids: RosGridSource, backend: Nav2Client, repository: YamlPoiRepository,
        config: RuntimeConfig, *, events: NavigationEvents,
        dock_worker: DockWorker | None = None,
        dock_observations: DockObservations | None = None,
    ) -> None:
        """在当前 asyncio 循环接收已构造资源的所有权，不执行额外配置校验.

        Args:
            tasks: 使用共享运动后端的唯一任务管理器。
            pois: 使用相同位姿与存储端口的 POI 管理接口。
            poses: 本运行时持有的导航位姿观察适配器。
            grids: 本运行时持有的栅格观察适配器。
            backend: 本运行时持有的单次尝试动作后端。
            repository: 共享 POI 持久化端口。
            config: 不可变组装配置；本构造不重新校验。
            events: 本运行时持有的尽力发布任务与 Dock 观察事件的适配器。
            dock_worker: 可选的受管工作线程，使用同一导航后端。
            dock_observations: 可选的保留地图及导航旋转基座观察适配器。

        Raises:
            RuntimeError: 当前没有运行中的事件循环。
        """
        self.tasks = tasks
        self.pois = pois
        self.poses = poses
        self._grids = grids
        self._backend = backend
        self._repository = repository
        self._config = config
        self._events = events
        self._dock_worker = dock_worker
        self._dock_observations = dock_observations
        self._loop = asyncio.get_running_loop()
        self._supervisor = TaskSupervisor(self._on_crash)
        self._watcher: asyncio.Task | None = None
        self._close_task: asyncio.Task | None = None
        self._settle_task: asyncio.Task | None = None
        self._closing = False
        self._closed = False

    def start_loop_closure(self) -> None:
        """按外层选择的连接生命周期启动至多一个托管回环观察任务.

        活跃任务存在时不重复启动；已结束或失败时允许新建。WS 断开与重连时由
        外层决定 stop/start，不在此改变旧会话或重投影策略。

        Raises:
            ClosedError: 运行时已开始关闭或已经关闭。
            RuntimeError: 不在原所有者事件循环，或任务创建失败。
        """
        self._check_loop()
        if self._closing:
            raise ClosedError('task runtime is closing or closed')
        if self._watcher is None or self._watcher.done():
            self._watcher = self._supervisor.spawn(watch_loop_closure(
                self.poses, self._repository, self._config.loop_closure_poll_sec,
                translation_threshold_m=self._config.loop_closure_translation_m,
                yaw_threshold_rad=self._config.loop_closure_yaw_rad), 'loop-closure')

    async def stop_loop_closure(self) -> None:
        """取消并等待回环观察任务，不调用运动取消，也不抹去其崩溃报告.

        supervisor 已取出并报告观察任务异常；这里消费普通异常及 CancelledError，
        避免它们阻断后续运动收尾。等待过程不加 shield，因此也可能消费调用方取消；
        该行为不能用来宣称调用方的取消一定向上传播。
        只在保存句柄仍为本次等待对象时清空它，避免覆盖后续新观察任务。

        Raises:
            RuntimeError: 调用不在原所有者事件循环。
        """
        self._check_loop()
        task = self._watcher
        if task is not None:
            task.cancel()
            with suppress(asyncio.CancelledError, Exception):
                await task
            if self._watcher is task:
                self._watcher = None

    async def close(self) -> None:
        """等待可重试的有序资源关闭，外层等待者取消不丢弃自有收尾任务.

        外层先停止入口，在运动收尾成功前保持 ROS 回调；多线程宿主还需在销毁
        资源前等待回调静默。只有全部步骤成功才确认 closed，失败可以部分完成。
        重复调用共享活跃关闭；已终止且尚未确认关闭时可重试，不提前销毁宿主或放弃资源。

        Raises:
            RuntimeError: 调用不在原所有者事件循环。
            asyncio.CancelledError: 等待者或内部关闭任务被取消，不证明资源释放。
            Exception: 运动或订阅清理失败，仍须保留运行时及必要的 ROS 回调。
        """
        self._check_loop()
        self._closing = True
        if self._close_task is None or (self._close_task.done() and not self._closed):
            self._close_task = self._supervisor.spawn(self._close(), 'close-task-runtime')
        await asyncio.shield(self._close_task)

    async def settle(self) -> None:
        """封闭任务接纳并等待运动收尾，不销毁 ROS 资源.

        先停止入口，保留 ROS 回调直到本调用成功；多线程宿主随后可等待回调静默，
        再由 close 释放客户端和订阅。活跃收尾供重复调用共享，外层等待者取消
        不会取消它；已终止且失败或取消的收尾可重试，成功收尾不重复执行。
        失败重试仍从头按顺序调用各所有者，不能跳过尚未成功的清理步骤。

        Raises:
            RuntimeError: 调用不在原所有者事件循环。
            asyncio.CancelledError: 等待者或内部收尾被取消，不表示资源已释放。
            Exception: 运动清理失败，须保留 ROS 和运行时后重试。
        """
        self._check_loop()
        self._closing = True
        if self._settle_task is None or (
                self._settle_task.done() and (
                    self._settle_task.cancelled() or self._settle_task.exception() is not None)):
            self._settle_task = self._supervisor.spawn(self._settle(), 'settle-task-runtime')
        await asyncio.shield(self._settle_task)

    async def _settle(self) -> None:
        """按观察器、任务内核、Dock 工作线程、后端顺序确认运动收尾.

        不销毁 ROS 客户端或订阅；外层必须保持回调处理，直到后端确认结束。
        某一步失败则停止后续步骤，外层保留本运行时并可重试。

        Raises:
            asyncio.CancelledError: 收尾任务被取消，不能据此释放资源。
            Exception: 某个运动所有者或后端未能确认清理。
        """
        await self.stop_loop_closure()
        await self.tasks.close()
        if self._dock_worker is not None:
            await self._dock_worker.close()
        await self._backend.settle()

    async def _close(self) -> None:
        """先确认运动收尾，再依序释放后端、观察器和输入资源.

        仅全部关闭成功后写入 closed；外层负责持有并重试失败的关闭任务，
        并在适当阶段静默 ROS 回调，不能因调用方等待超时提前销毁宿主。

        Raises:
            asyncio.CancelledError: 等待收尾或后端关闭时被取消。
            Exception: 任一资源关闭失败，closed 标记仍未设置。
        """
        await self.settle()
        await self._backend.close()
        if self._dock_observations is not None:
            self._dock_observations.close()
        self._grids.close()
        self.poses.close()
        self._events.close()
        self._closed = True

    def _check_loop(self) -> None:
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('task runtime used from a different event loop')

    @staticmethod
    def _on_crash(name: str, error: BaseException) -> None:
        _LOGGER.error('Task runtime operation %s failed: %s', name, error)


async def assemble_runtime(
    config: RuntimeConfig, node: object, tf_buffer: object, *,
    log: Callable[[str], None], now: Callable[[], float],
    on_dock_feedback: Callable[[DockGoal, object, float], None] | None = None,
) -> TaskRuntime:
    """在暴露任务接纳前组装普通 POI 与显式启用的 Dock 服务.

    创建订阅、事件发布器及动作客户端，但不初始化 ROS、创建节点/TF监听器/
    executor、spin、传输连接、回环轮询任务或运动目标。
    先校验 Dock 组合/策略及重试参数，读取仓库并可逐条导入命名点；其它适配器和
    技能仍可能在后续构造时校验失败，不保证所有配置问题都在文件 IO 前发现。
    已完成 ROS 资源按逆序回滚；文件导入不回滚。若清理失败，必须保留异常中
    的 cleanup，在原循环完成清理后才销毁外部节点。

    Args:
        config: 显式机器人与模式配置；Dock ID 必须配完整 Dock 配置。
        node: 外层持有的 ROS 节点，清理成功前保持有效。
        tf_buffer: 外层持有的 TF 缓冲与监听连接。
        log: 无前缀的 POI 加载日志接收器，异常可中止组装。
        now: 与位姿时间戳兼容的时钟，用于管理接口新鲜度检查。
        on_dock_feedback: 可选的任务循环内原始反馈，附任务及代次身份。

    Returns:
        持有资源并提供任务、POI 及位姿接口的运行时，不代表已启动任务或观察器。

    Raises:
        ConfigError: Dock 接线不完整或某项策略未通过其所在阶段的校验。
        RuntimeAssemblyError: 构造回滚失败，异常保留可重试清理回调。
        asyncio.CancelledError: 构造被取消且回滚完成；回滚失败时按上项保留资源。
        Exception: 仓库读写、日志或资源构造失败；不统一包装所有错误。
    """
    if config.dock_poi_ids and config.dock is None:
        raise ConfigError('Dock runtime assembly requires explicit Dock configuration')
    dock_policy = workflow_config(config.dock) if config.dock is not None else None
    retry = RetryPolicy(config.nav_retry_count, config.nav_retry_delay_sec)
    search = SnapSearch(
        config.nav_goal_snap_radius, config.nav_goal_fallback_snap_radius,
        config.nav_goal_clearance_radius, config.nav_goal_max_snap_radius,
        config.nav_goal_snap_step)
    repository = YamlPoiRepository(
        config.poi_store_file, config.source_poi_file, map_frame=config.map_frame)
    repository.load()
    if config.pose_file is not None:
        path = Path(config.pose_file)
        if not path.exists():
            log(f'pose file not found: {config.pose_file}')
        else:
            count = import_named_poses(
                repository, path, map_frame=config.map_frame,
                odom_frame=config.odom_frame, log=log)
            log(f'loaded {count} POIs from {config.pose_file}')
    async with _Rollback() as rollback:
        events = NavigationEvents()
        rollback.track(events.close)
        events.open(node, dock=config.dock is not None)
        poses = TfPoseSource(
            node, tf_buffer, robot_base_frame=config.robot_base_frame,
            odom_topic=config.odom_topic, map_frame=config.map_frame, odom_frame=config.odom_frame,
            session_id=lambda: config.slam_session_id)
        rollback.track(poses.close)
        grids = RosGridSource(
            node, map_topic=config.grid_map_topic, costmap_topic=config.global_costmap_topic)
        rollback.track(grids.close)
        backend = Nav2Client(
            node, config.action_name, config.server_timeout_sec,
            cancel_timeout_sec=config.cancel_timeout_sec,
            **({'spin_action_name': config.dock.spin_action_name} if config.dock else {}))
        rollback.track(backend.close)
        resolver = GridGoalResolver(grids.snapshot, poses, search)
        skill = NavigateToPoi(
            backend, repository, resolver, retry=retry,
            progress_min_interval_sec=config.progress_min_interval_sec, on_failure=RetryAudit())
        registry = CommandRegistry()
        dock_worker, dock_observations = None, None
        if config.dock is not None:
            dock_config = config.dock
            logger = node.get_logger()
            dock_observations = DockObservations(
                node, tf_buffer, global_frame=dock_config.global_frame,
                robot_base_frame=dock_config.robot_base_frame,
                static_map_topic=dock_config.static_map_topic, footprint=dock_config.footprint,
                occupied_threshold=dock_config.static_occupied_threshold, logger=logger,
                pose_timeout_sec=dock_config.pose_timeout_sec)
            rollback.track(dock_observations.close)
            dock_worker = DockWorker(
                backend, dock_config.global_frame, dock_policy, dock_observations.pose,
                dock_observations.check_approach, logger, on_raw_feedback=on_dock_feedback,
                on_event=events.dock)
            rollback.track(dock_worker.close)
            dock_skill = Dock(
                dock_worker, progress_min_interval_sec=config.progress_min_interval_sec,
                retry_delay_sec=config.nav_retry_delay_sec, on_failure=RetryAudit())
            registry.register('dock', dock_skill.decode, dock_skill.run)
            skill = DockAwareNavigation(skill, dock_skill, config.dock_poi_ids)
        registry.register('navigate_to', skill.decode, skill.run)

        async def cancel_motion():
            if dock_worker is not None:
                await dock_worker.cancel()
            await backend.cancel()

        tasks = TaskManager(registry, cancel_motion, on_event=events.task)
        rollback.track(tasks.close)
        commands = PoiCommands(poses, repository, now, config.pose_stale_after_sec)
        runtime = TaskRuntime(
            tasks, commands, poses, grids, backend, repository, config,
            events=events, dock_worker=dock_worker, dock_observations=dock_observations)
        return runtime
