"""桥接 ROS Future，不另建或驱动 ROS executor.

只有外层运行时驱动 ROS；第二个 executor 曾破坏 wait-set 所有权，
见 ADR-0005。等待者取消仅放弃本地等待，不取消迟到目标清理所需的源 Future。
"""

import asyncio
import concurrent.futures
import contextlib
import threading
import time
from typing import Any, Callable, Protocol


class FutureLike(Protocol):
    """描述 ROS 与测试替身共享的最小回调 Future 协议."""

    def add_done_callback(self, callback: Callable[['FutureLike'], None]) -> None:
        """注册完成回调，允许立即调用，也允许从另一线程调用.

        Args:
            callback: 接收本源 Future 的完成回调。
        """
        ...

    def result(self) -> Any:
        """读取完成结果或重新抛出源异常，调用方应先等待完成.

        协议不统一未完成时的行为：Humble Future 可能返回 None，
        而标准库 Future 可能阻塞；不得依赖两者在未完成时等价。

        Returns:
            源操作结果。

        Raises:
            BaseException: 源对象保存的异常，包括 asyncio.CancelledError。
        """
        ...

    def done(self) -> bool:
        """读取实现定义的完成标志，取消状态需另行考虑.

        Returns:
            完成标志；Humble 的取消与 done 分离，测试替身可能不同。
        """
        ...


async def await_rclpy_future(future: FutureLike) -> Any:
    """在 asyncio 中等待源 Future，既不驱动 ROS 也不取消源对象.

    通过标准库 Future 转交跨线程结果。普通异常和保存于源中的
    asyncio.CancelledError 都传递给等待者；迟到异常仍会被取出。
    没有隐含期限或动作清理，不统一处理其余 BaseException。

    Args:
        future: 由外层 ROS executor 或测试线程完成的回调 Future。

    Returns:
        源结果；本地等待取消后源操作仍可继续。

    Raises:
        asyncio.CancelledError: 等待者取消、源取消标志或源保存的取消异常。
        Exception: 回调注册失败或源普通异常。
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
    """阻塞当前线程等待完成通知，保留既有 Dock 轮询策略.

    只等待通知，不调用 result、不取消源对象。True 不证明结果成功、
    动作已清理或物理停稳。带取消谓词时先检查剩余时间，再等待事件，
    最后查询谓词；不能概括为所有零期限场景都优先返回完成。
    本函数不增加数值有限性校验，也不限制谓词自身的耗时。

    Args:
        future: 由外层既有 ROS executor 完成的 Future。
        timeout_sec: 本地等待预算，单位为秒。
        should_cancel: 可选取消谓词，在事件等待之间调用。
        poll_sec: 仅有取消谓词时采用的轮询间隔。

    Returns:
        True 表示已完成或收到通知；False 表示预算到期或谓词要求取消。

    Raises:
        Exception: 回调注册、等待参数或取消谓词的异常。
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
