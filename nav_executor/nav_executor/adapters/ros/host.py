"""持有私有 ROS 上下文、节点和执行器，不自动安装信号关闭逻辑."""

import asyncio
import logging
import multiprocessing
from typing import Sequence

_LOGGER = logging.getLogger(__name__)


class RosHost:
    """保留已完成构造阶段的 ROS 资源，直到显式有序释放.

    Attributes:
        node: 本宿主持有的 ROS 节点，未创建或完成销毁后为 None。
        tf_buffer: 本宿主创建的 TF 缓冲区；未创建时为 None。
    """

    def __init__(self) -> None:
        """创建空资源所有者，把 ROS 导入与初始化延后到 open."""
        self.node = None
        self.tf_buffer = None
        self._context = None
        self._executor = None
        self._listener = None
        self._shutdown_incomplete = False
        self._dock = False
        self._quiesce_task = None
        self._quiesce_loop = None
        self._quiesced = False

    def open(self, ros_args: Sequence[str], *, dock: bool = False) -> None:
        """初始化私有上下文，不安装 rclpy 自动关闭信号处理器.

        逐阶段保存资源，构造失败后调用方仍须持有本对象并负责收尾。
        使用 Context 的公共初始化接口，不用默认执行器或 TF 自旋线程；
        Dock 模式按 CPU 数且至少两个线程创建回调执行器。

        Args:
            ros_args: 原样传递给 ROS 的命令行参数。
            dock: 是否使用既有 Dock 节点名和多线程回调执行器。

        Raises:
            RuntimeError: 已经创建过上下文，拒绝重复 open。
            Exception: ROS 构造失败，已完成阶段的资源仍由本对象持有。
        """
        if self._context is not None:
            raise RuntimeError('ROS host is already open')
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformListener

        self._dock = dock
        self._quiesce_task = self._quiesce_loop = None
        self._quiesced = self._shutdown_incomplete = False
        self._context = Context()
        self._context.init(args=list(ros_args))
        name = 'dock_navigator' if dock else 'navigation_executor'
        self.node = Node(name, context=self._context)
        if dock:
            from rclpy.executors import MultiThreadedExecutor
            # Preserve Humble's CPU-count policy, with two required for blocking
            # action execution plus the callbacks needed to complete that action.
            try:
                threads = max(2, multiprocessing.cpu_count())
            except NotImplementedError:
                threads = 2
            self._executor = MultiThreadedExecutor(num_threads=threads, context=self._context)
        else:
            self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self.node)
        self.tf_buffer = Buffer()
        self._listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

    def spin_once(self) -> None:
        """以零等待处理就绪 ROS 工作，不另外驱动第二个执行器.

        零等待只约束等待就绪的时间，不保证回调自身不阻塞。

        Raises:
            RuntimeError: 已开始静默等待，或上下文在任务清理前丢失。
            Exception: ROS 回调或执行器操作失败。
        """
        if self._quiesce_task is not None:
            raise RuntimeError('ROS host is quiescing; spinning must remain stopped')
        if self._context is None or not self._context.ok():
            raise RuntimeError('ROS context is unavailable before task cleanup')
        self._executor.spin_once(timeout_sec=0.0)

    async def quiesce(self) -> None:
        """在线程中等待回调退出，以 shield 保留被取消等待者背后的收尾任务.

        调用前须先封闭入口、在 ROS 存活时完成运动清理并停止 spin；这些不是
        本函数自动执行的步骤。等待期间保持 asyncio 循环运行，因为回调可能调用它。
        只处理已提交及 SDK 已排队的尾部任务，不接收新的 DDS 工作或保证远端交付。
        Humble 多线程执行器的线程池/队列私有访问集中于本宿主，布局不明则保留资源。

        Raises:
            RuntimeError: 跨事件循环调用，或无法证明回调已结束。
            Exception: SDK 收尾失败，节点与上下文仍被持有。
            asyncio.CancelledError: 当前等待者取消；已创建的共享收尾任务继续运行。
        """
        loop = asyncio.get_running_loop()
        if self._quiesce_loop is not None and self._quiesce_loop is not loop:
            raise RuntimeError('ROS host quiescence used from a different event loop')
        if self._quiesce_task is None:
            self._quiesce_loop = loop
            self._quiesce_task = loop.create_task(self._quiesce(), name='ros-host-quiesce')
            self._quiesce_task.add_done_callback(self._quiesce_finished)
        await asyncio.shield(self._quiesce_task)

    async def _quiesce(self) -> None:
        """在线程中等待回调退出，仅在确认完成后释放执行器引用.

        失败后置位不可重试标记；节点与上下文仍归本宿主持有，不能继续销毁。

        Raises:
            RuntimeError: 先前清理不完整，或无法证明本次回调已退出。
            BaseException: 线程清理失败或等待被取消，保留不完整标记后透传。
        """
        if self._shutdown_incomplete:
            raise RuntimeError('ROS executor shutdown was incomplete; retaining node/context')
        try:
            await asyncio.to_thread(self._shutdown_callbacks)
        except BaseException:
            # A second SDK shutdown can return True without waiting; never retry it.
            self._shutdown_incomplete = True
            raise
        self._executor = None
        self._quiesced = True

    def _shutdown_callbacks(self) -> None:
        """在辅助线程关闭执行器并排空已提交回调，不接收新的 DDS 工作.

        Dock 模式先检查 Humble 私有队列布局，再等待线程池和队列尾部任务；
        调用方必须已停止 spin，并保持 asyncio 所有者循环存活。

        Raises:
            RuntimeError: 执行器布局不受支持、关闭不完整或尾部任务未正常结束。
            Exception: ROS 执行器或线程池关闭失败，资源仍由宿主持有。
        """
        executor = self._executor
        if executor is None:
            return
        pool = getattr(executor, '_executor', None) if self._dock else None
        if self._dock and not callable(getattr(pool, 'shutdown', None)):
            raise RuntimeError('Unknown ROS callback pool; retaining node/context')
        if self._dock and (not isinstance(getattr(executor, '_pending_tasks', None), dict)
                           or not hasattr(executor, '_tasks_lock')):
            raise RuntimeError('Unknown ROS task queue; retaining node/context')
        if not executor.shutdown(timeout_sec=None):
            raise RuntimeError('ROS executor shutdown was incomplete; retaining node/context')
        if pool is not None:
            # Executor._work_tracker does not cover all create_task/action result tails.
            pool.shutdown(wait=True)
            self._drain_sdk_tasks(executor)

    @staticmethod
    def _drain_sdk_tasks(executor) -> None:
        """同步排空已停止执行器中可立即完成的 SDK 尾部任务.

        Args:
            executor: 已关闭 spin 和线程池、具有 Humble 任务队列的执行器。

        Raises:
            RuntimeError: 任务仍在执行、调用后仍挂起或已带异常结束；不能释放资源。
            Exception: 读取或调用 SDK 任务失败，交由宿主保留清理所有权。
        """
        # David's Humble Task.set_result schedules done callbacks through create_task,
        # not through the pool. No spin driver or pool worker may remain here. These
        # queued tasks run on this shutdown helper thread with the asyncio loop alive.
        # Do not take new DDS data, poll an unresolved coroutine, or assume it finished.
        while True:
            with executor._tasks_lock:
                pending = [task for task in executor._pending_tasks
                           if not task.done() and not task.cancelled()]
            if not pending:
                return
            for task in pending:
                if task.executing():
                    raise RuntimeError('ROS task still executing; retaining node/context')
                task()
                if not task.done() and not task.cancelled():
                    raise RuntimeError('ROS task remains suspended; retaining node/context')
                if not task.cancelled() and task.exception() is not None:
                    raise RuntimeError('Queued ROS task failed; retaining node/context') \
                        from task.exception()

    @staticmethod
    def _quiesce_finished(task: asyncio.Task) -> None:
        """取出已结束静默任务的异常并报告，不在回调中释放 ROS 资源.

        已取消任务不调用 exception；调用方须仅把已结束的任务交给此回调。
        记录错误不等于已确认回调静默，所有权仍由外层宿主管理。

        Args:
            task: 已完成的宿主静默任务。
        """
        if not task.cancelled() and task.exception() is not None:
            _LOGGER.error('ROS host quiescence failed; retaining resources: %s', task.exception())

    def close(self) -> None:
        """在任务和 spin 收尾后顺序释放宿主资源，成功一步才清除其引用.

        已标记回调收尾不完整，或要求静默却尚未完成时拒绝继续释放。
        执行器 shutdown 返回 False 会留下不可重试标记，不能把再次调用的 True
        当作回调结束证据。其它资源释放异常保留当前引用；不保证每种异常均可安全重试。

        Raises:
            RuntimeError: 回调收尾不完整或尚未证明静默，需保留资源并处理故障。
            Exception: 某阶段释放失败，后续同层阶段不继续，先前已释放阶段保持完成。
        """
        if self._shutdown_incomplete:
            raise RuntimeError('ROS executor shutdown was incomplete; retaining node/context')
        if ((self._dock and self._executor is not None) or self._quiesce_task is not None):
            if not self._quiesced:
                raise RuntimeError('ROS callback quiescence must finish before close')
        if self._executor is not None:
            if not self._executor.shutdown(timeout_sec=0.0):
                self._shutdown_incomplete = True
                raise RuntimeError('ROS executor callbacks remain; retaining node/context')
            self._executor = None
        if self._listener is not None:
            self._listener.unregister()
            self._listener = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        self.tf_buffer = None
        if self._context is not None:
            self._context.try_shutdown()
            self._context = None
