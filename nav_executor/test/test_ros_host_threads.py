"""Humble-like callback pools without ROS initialization, DDS or motion goals."""

import asyncio
from concurrent.futures import ThreadPoolExecutor
import sys
import threading

import pytest

from nav_executor.adapters.ros.host import RosHost

from test_ros_host import ros_bindings  # noqa: F401


class _Task:
    def __init__(self, callback):
        self.callback = callback
        self.finished = False
        self.error = None

    def __call__(self):
        try:
            self.callback()
        except Exception as error:
            self.error = error
        self.finished = True

    def done(self):
        return self.finished

    def cancelled(self):
        return False

    def executing(self):
        return False

    def exception(self):
        return self.error


@pytest.fixture
def threaded_bindings(request, monkeypatch):
    """Model Humble shutdown that does not join its callback thread pool."""
    state = request.getfixturevalue('ros_bindings')
    state.node_name = 'dock_navigator'
    state.pools = []
    state.shutdown_threads = []
    bindings = sys.modules['rclpy.executors']
    single = bindings.SingleThreadedExecutor

    class Multi(single):
        def __init__(self, *, num_threads, context):
            assert num_threads >= 2
            state.num_threads = num_threads
            super().__init__(context)
            self._executor = ThreadPoolExecutor(max_workers=2)
            self._tasks_lock = threading.Lock()
            self._pending_tasks = {}
            state.pools.append(self._executor)
            state.executor = self

        def shutdown(self, timeout_sec=None):
            assert timeout_sec is None
            state.events.append('executor shutdown')
            state.shutdown_threads.append(threading.get_ident())
            if state.fail == 'executor shutdown':
                raise RuntimeError('shutdown failed')
            return state.shutdown_ok

    monkeypatch.setattr(bindings, 'MultiThreadedExecutor', Multi, raising=False)
    yield state
    for pool in state.pools:
        # Call the real class method even when a test replaced this instance method.
        ThreadPoolExecutor.shutdown(pool, wait=True)


@pytest.mark.parametrize('cancel_waiter', [False, True])
def test_quiesce_joins_sdk_tail_without_blocking_loop(threaded_bindings, cancel_waiter):
    """Action result tails may call the loop and must finish before ROS destruction."""
    state = threaded_bindings

    async def scenario():
        host = RosHost()
        host.open(['--ros-args'], dock=True)
        loop = asyncio.get_running_loop()
        callback_started = asyncio.Event()
        tail_started = asyncio.Event()
        release = threading.Event()
        owner = threading.get_ident()

        def callback():
            loop.call_soon_threadsafe(callback_started.set)
            release.wait()
            # The SDK continues after the user execute callback has returned.
            loop.call_soon_threadsafe(tail_started.set)
            acknowledged = asyncio.run_coroutine_threadsafe(asyncio.sleep(0), loop)
            acknowledged.result(timeout=2)
            state.events.append('SDK result stored')

        running = state.executor._executor.submit(callback)
        await asyncio.wait_for(callback_started.wait(), 1)
        waiter = asyncio.create_task(host.quiesce())
        try:
            await asyncio.sleep(0.03)
            assert state.shutdown_threads and state.shutdown_threads[0] != owner
            assert not waiter.done() and not running.done()
            assert 'node destroy' not in state.events
            with pytest.raises(RuntimeError, match='quiesc'):
                host.close()
            with pytest.raises(RuntimeError, match='quiesc'):
                host.spin_once()
            if cancel_waiter:
                waiter.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await waiter
        finally:
            release.set()
            await asyncio.wait_for(host.quiesce(), 2)
            await asyncio.wait_for(tail_started.wait(), 1)
            await asyncio.gather(waiter, return_exceptions=True)
        host.close()
        host.close()
        assert state.events.index('SDK result stored') < state.events.index('node destroy')
        assert state.events.count('executor shutdown') == 1
        assert not any(thread.is_alive() for thread in state.executor._executor._threads)

    asyncio.run(scenario())


def test_quiesce_drains_result_callbacks_enqueued_after_pool_work(threaded_bindings):
    """Setting an action result queues a further send callback outside the worker pool."""
    async def scenario():
        host = RosHost()
        host.open([], dock=True)
        executor = threaded_bindings.executor
        loop = asyncio.get_running_loop()
        sent = asyncio.Event()

        def send():
            loop.call_soon_threadsafe(sent.set)
            asyncio.run_coroutine_threadsafe(asyncio.sleep(0), loop).result(timeout=2)

        def result_tail():
            with executor._tasks_lock:
                executor._pending_tasks[_Task(send)] = None

        executor._pending_tasks[_Task(result_tail)] = None
        await host.quiesce()
        assert sent.is_set()
        assert all(task.done() for task in executor._pending_tasks)
        host.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('stage', [
    'context init', 'node create', 'executor create',
    'node add', 'buffer create', 'listener create'])
def test_dock_partial_open_can_quiesce_then_close(threaded_bindings, stage):
    """Every completed construction stage remains owned during failed startup."""
    async def scenario():
        host = RosHost()
        threaded_bindings.fail = stage
        with pytest.raises(RuntimeError, match=stage):
            host.open([], dock=True)
        threaded_bindings.fail = None
        await host.quiesce()
        host.close()
        assert host.node is None and host._context is None
        assert threaded_bindings.events[-1] == 'context shutdown'

    asyncio.run(scenario())


@pytest.mark.parametrize('failure', ['false', 'exception', 'pool'])
def test_unproven_shutdown_is_sticky_even_if_retried(threaded_bindings, failure):
    """A later optimistic SDK shutdown result cannot prove the first attempt finished."""
    state = threaded_bindings

    async def scenario():
        host = RosHost()
        host.open([], dock=True)
        if failure == 'false':
            state.shutdown_ok = False
        elif failure == 'exception':
            state.fail = 'executor shutdown'
        else:
            def broken(*, wait):
                raise RuntimeError('pool shutdown failed')
            state.executor._executor.shutdown = broken
        with pytest.raises(RuntimeError):
            await host.quiesce()
        state.fail = None
        state.shutdown_ok = True
        with pytest.raises(RuntimeError):
            await host.quiesce()
        with pytest.raises(RuntimeError, match='incomplete'):
            host.close()
        assert state.events.count('executor shutdown') == 1
        assert 'node destroy' not in state.events and 'context shutdown' not in state.events

    asyncio.run(scenario())


def test_dock_close_requires_explicit_quiescence(threaded_bindings):
    """Synchronous close cannot block the asyncio loop on a threaded action callback."""
    host = RosHost()
    host.open([], dock=True)
    with pytest.raises(RuntimeError, match='quiesc'):
        host.close()
    assert 'executor shutdown' not in threaded_bindings.events
    asyncio.run(host.quiesce())
    host.close()


@pytest.mark.parametrize('failure', ['unknown_queue', 'suspended', 'callback_error', 'executing'])
def test_unprovable_queued_work_retains_resources(threaded_bindings, failure):
    """Unsupported SDK layouts or unfinished queued work cannot authorize ROS teardown."""
    async def scenario():
        host = RosHost()
        host.open([], dock=True)
        executor = threaded_bindings.executor
        if failure == 'unknown_queue':
            del executor._pending_tasks
        else:
            def callback():
                if failure == 'callback_error':
                    raise ValueError('send failed')
            task = _Task(callback)
            if failure == 'suspended':
                task.done = lambda: False
            if failure == 'executing':
                task.executing = lambda: True
            executor._pending_tasks[task] = None
        with pytest.raises(RuntimeError):
            await host.quiesce()
        with pytest.raises(RuntimeError):
            host.close()
        assert host.node is not None and host._context is not None
        assert 'node destroy' not in threaded_bindings.events

    asyncio.run(scenario())


@pytest.mark.parametrize('cpu_count', [1, None])
def test_dock_keeps_two_threads_even_on_single_cpu_or_unknown_count(
    threaded_bindings, monkeypatch, cpu_count,
):
    """A blocking Dock execute callback must leave a second thread for action/TF callbacks."""
    def count():
        if cpu_count is None:
            raise NotImplementedError
        return cpu_count

    monkeypatch.setattr('nav_executor.adapters.ros.host.multiprocessing.cpu_count', count)
    host = RosHost()
    host.open([], dock=True)
    assert threaded_bindings.num_threads == 2
    asyncio.run(host.quiesce())
    host.close()


def test_quiesce_is_bound_to_one_event_loop(threaded_bindings):
    """An already-finished owner is not silently reused across different loops."""
    host = RosHost()
    host.open([], dock=True)
    asyncio.run(host.quiesce())
    with pytest.raises(RuntimeError, match='event loop'):
        asyncio.run(host.quiesce())
    host.close()
