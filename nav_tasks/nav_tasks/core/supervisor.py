"""为 Python 3.10 后台任务保留强引用并取出完成异常."""

import asyncio
from typing import Any, Callable, Coroutine, TypeVar

from nav_tasks.domain.errors import ClosedError

Result = TypeVar('Result')


class TaskSupervisor:
    """持有后台任务至完成，并报告非取消异常一次.

    要求在同一运行循环构造和使用；spawn 显式核对循环，其它访问不能当作线程安全 API。
    同步崩溃回调不得阻塞；回调自身的普通异常转交循环错误处理器。
    本类不代表运动所有权，也不证明底层资源已清理。
    """

    def __init__(self, on_crash: Callable[[str, BaseException], None]) -> None:
        """把空后台任务所有者绑定到当前事件循环.

        Args:
            on_crash: 同步接收任务名和异常的报告函数。

        Raises:
            RuntimeError: 当前没有运行中的事件循环。
        """
        self._loop = asyncio.get_running_loop()
        self._tasks: set[asyncio.Task] = set()
        self._on_crash = on_crash
        self._closed = False

    @property
    def active_count(self) -> int:
        """统计仍未完成的自有任务，不等同资源或运动数量.

        Returns:
            自有任务集合中 done 为 False 的数量。
        """
        return sum(not task.done() for task in self._tasks)

    def spawn(self, coro: Coroutine[Any, Any, Result], name: str) -> asyncio.Task[Result]:
        """创建强引用托管任务；在循环不匹配或已关闭的拒绝路径关闭传入协程.

        调用方应在本循环交付新协程。无运行循环时 get_running_loop 自身会先失败，
        以及底层 create_task 失败的路径不具有这里的显式协程关闭保证。

        Args:
            coro: 转交本函数所有权的新协程对象。
            name: 用于诊断和异常关联的任务名称。

        Returns:
            已登记并设置完成回调的任务，调用方也可以等待它。

        Raises:
            ClosedError: supervisor 已经关闭任务接纳。
            RuntimeError: 无运行循环、循环不匹配或底层任务创建失败。
            Exception: 任务创建或回调登记失败。
        """
        if asyncio.get_running_loop() is not self._loop:
            coro.close()
            raise RuntimeError('supervisor used from a different event loop')
        if self._closed:
            coro.close()
            raise ClosedError('supervisor is closed')
        task = self._loop.create_task(coro, name=name)
        self._tasks.add(task)
        task.add_done_callback(self._finished)
        return task

    async def cancel_and_wait(self) -> None:
        """封闭后台任务接纳，取消当前快照中的任务并等待结束.

        要求从所有者循环调用；不会主动核对循环身份，也没有屏蔽外部重复取消。
        子任务普通异常由完成回调报告，gather 汇总而不再次抛出；
        本方法不调用业务后端取消接口，不能替代运动所有者的清理流程。

        Raises:
            RuntimeError: 自有任务试图等待自己。
            asyncio.CancelledError: 调用方取消等待，可能继续向子任务传播取消。
        """
        if asyncio.current_task() in self._tasks:
            raise RuntimeError('an owned task cannot join its own supervisor')
        self._closed = True
        tasks = tuple(self._tasks)
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    def _finished(self, task: asyncio.Task) -> None:
        """移除已完成任务的强引用，取出非取消异常并报告一次.

        Args:
            task: 本 supervisor 创建且已完成的 asyncio 任务。

        崩溃回调自身失败时，把原异常与回调异常一起交给事件循环错误处理器。
        """
        self._tasks.discard(task)
        if task.cancelled():
            return
        error = task.exception()
        if error is None:
            return
        try:
            self._on_crash(task.get_name(), error)
        except Exception as callback_error:
            self._loop.call_exception_handler({
                'message': 'TaskSupervisor crash callback failed',
                'exception': callback_error, 'task': task, 'original_exception': error,
            })
