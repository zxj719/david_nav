"""One private ROS context/node/executor with no automatic signal shutdown."""

from typing import Sequence


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

    def open(self, ros_args: Sequence[str]) -> None:
        """Initialize a private context without installing rclpy shutdown handlers.

        The caller retains this owner even when a construction stage fails.
        Context.init is the public context API and does not alter signal handlers.
        No default/global executor or TF spin thread is used.

        Args:
            ros_args: Untouched ROS CLI arguments.

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

        self._context = Context()
        self._context.init(args=list(ros_args))
        self.node = Node('navigation_executor', context=self._context)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self.node)
        self.tf_buffer = Buffer()
        self._listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

    def spin_once(self) -> None:
        """Process ready ROS callbacks without blocking or spinning a second executor.

        Raises:
            RuntimeError: The context was lost before orderly task cleanup.
        """
        if self._context is None or not self._context.ok():
            raise RuntimeError('ROS context is unavailable before task cleanup')
        self._executor.spin_once(timeout_sec=0.0)

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
