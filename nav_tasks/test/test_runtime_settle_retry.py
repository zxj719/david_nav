"""用内存资源核验运行时停用的取消重试，不创建 ROS 或下发动作."""

import asyncio
from types import SimpleNamespace

import pytest

from nav_tasks.app.runtime import TaskRuntime


def _runtime(failure_stage=None, error_type=None, *, persistent=False, gate=None):
    """创建只记录调用及注入失败的资源，保留真实运行时收尾实现.

    Args:
        failure_stage: 首次或持续失败的步骤名称，None 表示不注入。
        error_type: 注入的异常类型。
        persistent: 是否每次运行到该步骤都失败。
        gate: 后端 settle 的可选阻塞门闩。

    Returns:
        真实 TaskRuntime、调用日志及后端等待起点队列。
    """
    calls, entered = [], asyncio.Queue()
    failed = False

    async def step(name):
        nonlocal failed
        calls.append(name)
        if name == failure_stage and (persistent or not failed):
            failed = True
            raise error_type('fixture')
        if name == 'backend.settle' and gate is not None:
            entered.put_nowait(None)
            await gate.wait()

    async def tasks_close():
        await step('tasks.close')

    async def dock_close():
        await step('dock.close')

    async def backend_settle():
        await step('backend.settle')

    async def backend_close():
        await step('backend.close')

    owner = TaskRuntime(
        SimpleNamespace(close=tasks_close), None,
        SimpleNamespace(close=lambda: calls.append('poses.close')),
        SimpleNamespace(close=lambda: calls.append('grids.close')),
        SimpleNamespace(settle=backend_settle, close=backend_close),
        None, None, events=SimpleNamespace(close=lambda: calls.append('events.close')),
        dock_worker=SimpleNamespace(close=dock_close),
        dock_observations=SimpleNamespace(close=lambda: calls.append('observations.close')))
    return owner, calls, entered


@pytest.mark.parametrize('stage', ['tasks.close', 'dock.close', 'backend.settle'])
@pytest.mark.parametrize('error_type', [RuntimeError, asyncio.CancelledError])
def test_terminated_settle_retries_before_destroying_resources(stage, error_type):
    """停用步骤失败或取消后，close 必须重新完成停用才能销毁资源.

    Args:
        stage: 注入首次失败的停用步骤。
        error_type: 普通异常或取消异常。
    """
    async def scenario():
        owner, calls, _ = _runtime(stage, error_type)
        sequence = ['tasks.close', 'dock.close', 'backend.settle']
        prefix = sequence[:sequence.index(stage) + 1]
        try:
            with pytest.raises(error_type):
                await asyncio.wait_for(owner.settle(), 1)
            failed = owner._settle_task
            assert failed.done() and owner._closing and not owner._closed
            assert calls == prefix
            await asyncio.wait_for(owner.close(), 1)
            assert owner._settle_task is not failed and owner._closed
            assert calls == prefix + sequence + [
                'backend.close', 'observations.close', 'grids.close',
                'poses.close', 'events.close']
            closed, settled, snapshot = owner._close_task, owner._settle_task, list(calls)
            await owner.settle()
            await owner.close()
            assert owner._settle_task is settled and owner._close_task is closed
            assert calls == snapshot
        finally:
            await owner._supervisor.cancel_and_wait()

    asyncio.run(scenario())


def test_canceled_waiter_keeps_one_live_settle_and_success_is_not_repeated():
    """外层等待者取消不影响活跃停用，完成后重复调用不重跑资源步骤."""
    async def scenario():
        gate = asyncio.Event()
        owner, calls, entered = _runtime(gate=gate)
        first = asyncio.create_task(owner.settle())
        second = None
        try:
            await asyncio.wait_for(entered.get(), 1)
            owned = owner._settle_task
            first.cancel()
            with pytest.raises(asyncio.CancelledError):
                await first
            assert not owned.done() and not owner._closed
            second = asyncio.create_task(owner.settle())
            await asyncio.sleep(0)
            assert owner._settle_task is owned and not second.done()
            assert entered.empty() and calls == ['tasks.close', 'dock.close', 'backend.settle']
            gate.set()
            await asyncio.wait_for(second, 1)
            await owner.close()
            assert owner._settle_task is owned and owner._closed
            assert calls.count('backend.settle') == 1
        finally:
            gate.set()
            await asyncio.gather(first, *([second] if second else []), return_exceptions=True)
            await owner._supervisor.cancel_and_wait()

    asyncio.run(scenario())


def test_repeated_internal_cancellation_keeps_resources_owned():
    """持续内部取消必须如实失败，每次可重试但不得绕过停用或销毁门禁."""
    async def scenario():
        owner, calls, _ = _runtime('dock.close', asyncio.CancelledError, persistent=True)
        try:
            for count in (1, 2):
                previous = owner._settle_task
                with pytest.raises(asyncio.CancelledError):
                    await asyncio.wait_for(owner.close(), 1)
                assert owner._settle_task is not previous
                assert calls == ['tasks.close', 'dock.close'] * count
                assert owner._closing and not owner._closed
        finally:
            await owner._supervisor.cancel_and_wait()

    asyncio.run(scenario())
