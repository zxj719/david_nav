"""One private ROS context/node/executor with no automatic signal shutdown."""

import asyncio
import logging
import multiprocessing
from typing import Sequence

_LOGGER = logging.getLogger(__name__)


class RosHost:
    """Retain every completed construction stage until explicit ordered cleanup."""

    def __init__(self) -> None:
        """Create an empty owner; imports and ROS initialization wait until open."""
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
        """Initialize a private context without installing rclpy shutdown handlers.

        The caller retains this owner even when a construction stage fails.
        Context.init is the public context API and does not alter signal handlers.
        No default/global executor or TF spin thread is used.

        Args:
            ros_args: Untouched ROS CLI arguments.
            dock: Use the legacy Dock node name and multithreaded callback executor.

        Raises:
            RuntimeError: This owner has already been opened.
            Exception: ROS construction failed; close must clean completed stages.
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
        """Process ready ROS callbacks without blocking or spinning a second executor.

        Raises:
            RuntimeError: The context was lost before orderly task cleanup.
        """
        if self._quiesce_task is not None:
            raise RuntimeError('ROS host is quiescing; spinning must remain stopped')
        if self._context is None or not self._context.ok():
            raise RuntimeError('ROS context is unavailable before task cleanup')
        self._executor.spin_once(timeout_sec=0.0)

    async def quiesce(self) -> None:
        """Join callback execution off-loop, retaining ownership if a waiter is canceled.

        First seal ingress, settle motion with ROS alive, and join the spin loop.
        Keep this asyncio loop alive while awaiting: callback threads may call it.
        This waits for submitted callbacks and already-queued SDK tasks, including
        action result send callbacks. It does not take new DDS work or promise
        delivery to remote clients. Destroy ingress and host only afterwards.

        Humble's MultiThreadedExecutor inherits a shutdown that does not join its
        ThreadPoolExecutor. Private pool/task-queue access is isolated here; an
        unknown binding layout fails closed, retaining the ROS resources.

        Raises:
            RuntimeError: Called on another loop or callback shutdown cannot be proven.
            Exception: SDK shutdown failed; node and context remain owned.
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
        if not task.cancelled() and task.exception() is not None:
            _LOGGER.error('ROS host quiescence failed; retaining resources: %s', task.exception())

    def close(self) -> None:
        """Release completed host stages after task cleanup and the spin loop have joined.

        Failure retains the current stage for retry. Humble executor shutdown sets
        its shutdown flag even when returning False; another call can report True
        without rechecking callbacks. Do not accept that as proof of safe teardown.

        Raises:
            RuntimeError: Executor callbacks did not finish; external review is required.
            Exception: A host resource failed cleanup; completed stages remain completed.
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
