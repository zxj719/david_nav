"""核验源 Future 的取消异常不会逃逸完成线程或丢失等待通知."""

import asyncio
from concurrent.futures import Future
import threading

import pytest

from nav_tasks.adapters.ros.futures import await_rclpy_future


class _Source(Future):
    def __init__(self):
        super().__init__()
        self.reads = 0

    def result(self, *args, **kwargs):
        self.reads += 1
        return super().result(*args, **kwargs)


@pytest.mark.parametrize('completion, abandon', [
    ('ready', False),
    ('loop', False),
    ('thread', False),
    ('loop', True),
    ('thread', True),
])
def test_stored_cancelled_error_is_bridged_without_canceling_source(completion, abandon):
    """异常形式的取消也必须通知等待者；迟到时取出异常但不污染完成线程.

    Args:
        completion: 预先完成、循环线程完成或独立线程完成。
        abandon: 是否在源完成前取消本地等待者。
    """
    async def scenario():
        source = _Source()
        error = asyncio.CancelledError('source failure')
        callback_errors = []
        waiter = None

        def complete():
            try:
                source.set_exception(error)
            except BaseException as caught:
                callback_errors.append(caught)

        try:
            if completion == 'ready':
                complete()
            waiter = asyncio.create_task(await_rclpy_future(source))
            await asyncio.sleep(0)
            if abandon:
                waiter.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await waiter
                assert not source.done()
            if completion == 'thread':
                thread = threading.Thread(target=complete)
                thread.start()
                thread.join(timeout=1)
                assert not thread.is_alive()
            elif completion == 'loop':
                complete()
            assert callback_errors == []
            if not abandon:
                with pytest.raises(asyncio.CancelledError):
                    await asyncio.wait_for(waiter, 1)
            assert source.done() and not source.cancelled()
            assert source.exception() is error
            assert source.reads == 1
        finally:
            if waiter is not None:
                if not waiter.done():
                    waiter.cancel()
                await asyncio.gather(waiter, return_exceptions=True)

    asyncio.run(scenario())
