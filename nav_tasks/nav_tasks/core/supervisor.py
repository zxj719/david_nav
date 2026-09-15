"""Strong ownership and exception retrieval for Python 3.10 background tasks."""

import asyncio
from typing import Any, Callable, Coroutine, TypeVar

from nav_tasks.domain.errors import ClosedError

Result = TypeVar('Result')


class TaskSupervisor:
    """Own tasks until completion and report non-cancellation failures once.

    Construct and use on one running asyncio loop. Crash callbacks are synchronous
    and must not block; a broken callback is forwarded to the loop error handler.
    """

    def __init__(self, on_crash: Callable[[str, BaseException], None]) -> None:
        """Bind a background-task owner to the current event loop.

        Args:
            on_crash: Synchronous task-name and exception reporting callback.
        """
        self._loop = asyncio.get_running_loop()
        self._tasks: set[asyncio.Task] = set()
        self._on_crash = on_crash
        self._closed = False

    @property
    def active_count(self) -> int:
        """Number of owned tasks that have not completed."""
        return sum(not task.done() for task in self._tasks)

    def spawn(self, coro: Coroutine[Any, Any, Result], name: str) -> asyncio.Task[Result]:
        """Create a strongly owned task or close a rejected coroutine.

        Args:
            coro: Newly created coroutine whose ownership transfers to this call.
            name: Diagnostic task name.

        Returns:
            Owned task, also safe for a caller to await.

        Raises:
            ClosedError: The supervisor is closing or closed.
            RuntimeError: Called from a different event loop.
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
        """Seal admission, cancel all owned tasks and wait for their cleanup.

        Raises:
            RuntimeError: An owned task attempts to join itself.
        """
        if asyncio.current_task() in self._tasks:
            raise RuntimeError('an owned task cannot join its own supervisor')
        self._closed = True
        tasks = tuple(self._tasks)
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    def _finished(self, task: asyncio.Task) -> None:
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
