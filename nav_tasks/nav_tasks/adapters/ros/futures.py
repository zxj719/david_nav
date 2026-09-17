"""Bridge ROS futures without creating or spinning another ROS executor.

The owning runtime alone spins ROS. A second executor around an action client
previously corrupted its wait-set ownership; see ADR-0005. Async cancellation
only abandons this waiter, never the source future needed for late-goal cleanup.
"""

import asyncio
import concurrent.futures
import contextlib
import threading
import time
from typing import Any, Callable, Protocol


class FutureLike(Protocol):
    """Minimal callback future contract shared by ROS and test doubles."""

    def add_done_callback(self, callback: Callable[['FutureLike'], None]) -> None:
        """Register a completion callback, possibly invoked immediately.

        Args:
            callback: Callback receiving this completed future.
        """
        ...

    def result(self) -> Any:
        """Retrieve a completed result or re-raise its exception.

        Returns:
            Result object produced by the operation.
        """
        ...

    def done(self) -> bool:
        """Check whether the operation completed.

        Returns:
            Completion state; ROS reports cancellation separately.
        """
        ...


async def await_rclpy_future(future: FutureLike) -> Any:
    """Wait from asyncio without spinning or canceling the underlying ROS future.

    Args:
        future: ROS callback future; callbacks may run on another thread.

    Returns:
        Completed result, with source exceptions propagated to this waiter.

    Raises:
        asyncio.CancelledError: The waiter or source future was canceled.
    """
    wrapped: concurrent.futures.Future = concurrent.futures.Future()

    def completed(source: FutureLike) -> None:
        """从完成线程转交结果或取消异常，迟到时仍取出源异常.

        只更新本地桥接 Future，不取消源对象。外层等待者已取消时，
        本地 InvalidStateError 被消费；普通异常和 asyncio.CancelledError
        都转交等待者，不让后一种异常因属于 BaseException 而逃逸完成线程。

        Args:
            source: 已完成的源 Future，可用取消标志或异常表达取消。
        """
        # Always retrieve a late source exception, even if nobody awaits it now.
        try:
            if getattr(source, 'cancelled', lambda: False)():
                wrapped.cancel()
                return
            result = source.result()
        except (Exception, asyncio.CancelledError) as error:
            with contextlib.suppress(concurrent.futures.InvalidStateError):
                wrapped.set_exception(error)
            return
        with contextlib.suppress(concurrent.futures.InvalidStateError):
            wrapped.set_result(result)

    future.add_done_callback(completed)
    return await asyncio.wrap_future(wrapped)


def wait_rclpy_future(
    future: FutureLike, timeout_sec: float, should_cancel: Callable[[], bool] | None = None,
    poll_sec: float = 0.1,
) -> bool:
    """Wait on a threading event, preserving the existing dock wait policy.

    Args:
        future: Future completed by the runtime's own ROS executor.
        timeout_sec: Maximum wait duration.
        should_cancel: Optional cancellation predicate checked between event waits.
        poll_sec: Polling interval only when the predicate is provided.

    Returns:
        True for completion, False for timeout or requested cancellation.
    """
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if future.done():
        return True
    if should_cancel is None:
        return event.wait(timeout_sec)
    deadline = time.monotonic() + timeout_sec
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        if event.wait(min(poll_sec, remaining)):
            return True
        if should_cancel():
            return False
