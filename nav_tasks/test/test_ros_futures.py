"""No-ROS tests for callback-future adapters and cross-thread completion."""

import asyncio
from concurrent.futures import Future
import threading

import pytest

from nav_tasks.adapters.ros.futures import await_rclpy_future, wait_rclpy_future


def test_completed_value_and_exception():
    """Already-completed futures preserve result and exception identity."""
    async def scenario():
        future = Future()
        future.set_result('value')
        assert await await_rclpy_future(future) == 'value'
        error = ValueError('source failed')
        failed = Future()
        failed.set_exception(error)
        with pytest.raises(ValueError) as caught:
            await await_rclpy_future(failed)
        assert caught.value is error

    asyncio.run(scenario())


def test_completion_from_worker_thread():
    """The ROS-executor-thread equivalent safely wakes the asyncio loop."""
    async def scenario():
        future = Future()
        waiter = asyncio.create_task(await_rclpy_future(future))
        await asyncio.sleep(0)
        thread = threading.Thread(target=lambda: future.set_result(42))
        thread.start()
        assert await waiter == 42
        thread.join()

    asyncio.run(scenario())


@pytest.mark.parametrize('failure', [False, True])
def test_canceled_waiter_does_not_cancel_source_or_break_late_completion(failure):
    """Late completion remains available to an independent cleanup waiter."""
    async def scenario():
        source = Future()
        abandoned = asyncio.create_task(await_rclpy_future(source))
        await asyncio.sleep(0)
        abandoned.cancel()
        with pytest.raises(asyncio.CancelledError):
            await abandoned
        assert not source.cancelled()
        if failure:
            source.set_exception(RuntimeError('late error'))
            with pytest.raises(RuntimeError, match='late error'):
                await await_rclpy_future(source)
        else:
            source.set_result('late handle')
            assert await await_rclpy_future(source) == 'late handle'

    asyncio.run(scenario())


def test_source_cancellation_propagates():
    """A canceled source does not look like a successful None result."""
    async def scenario():
        source = Future()
        source.cancel()
        with pytest.raises(asyncio.CancelledError):
            await await_rclpy_future(source)

    asyncio.run(scenario())


def test_thread_wait_completion_timeout_and_cancel_without_spin():
    """Event-based waits neither own an executor nor alter the source future."""
    source = Future()
    assert not wait_rclpy_future(source, 0)
    assert not wait_rclpy_future(source, 1, should_cancel=lambda: True, poll_sec=0.001)
    assert not source.cancelled()
    thread = threading.Thread(target=lambda: source.set_result(42))
    thread.start()
    assert wait_rclpy_future(source, 1)
    thread.join()
    assert wait_rclpy_future(source, 0)
