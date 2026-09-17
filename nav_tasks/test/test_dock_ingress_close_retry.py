"""用内存入口任务核验 Dock 关闭重试，不创建 ROS 或下发动作."""

import asyncio
from types import SimpleNamespace

import pytest

from nav_tasks.api import create_dock_ingress, dock_config_from_parameters


def _owner():
    """在当前循环创建未 open 的入口所有者，无节点或动作客户端.

    Returns:
        具有完整测试配置但不持有 ROS 资源的 Dock 入口。
    """
    config = dock_config_from_parameters(dict(
        approach_dist=0.5, xy_ok=0.1, yaw_ok=0.2, max_yaw_retries=0,
        goal_topic='/fixture_dock', action_name='fixture_dock_action',
        robot_base_frame='navigation_center', global_frame='world',
        nav_timeout_sec=10.0, spin_timeout_sec=5.0, static_map_topic='/fixture_map',
        static_occupied_threshold=50, footprint='[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]',
        settle_max_rate_rad_s=0.05, settle_window_sec=0.4, settle_timeout_sec=8.0))
    return create_dock_ingress(None, config)


def _instrument_close(owner):
    """保留真实收尾实现，只增加等待起点门闩与调用计数.

    Args:
        owner: 本测试创建的入口所有者。

    Returns:
        收尾起点队列；每次真实 _close 开始时入队一次。
    """
    original = owner._close
    started = asyncio.Queue()

    async def observed():
        started.put_nowait(None)
        await original()

    owner._close = observed
    return started


def _entry(owner, name, gate, error=None, *, retire=True):
    """安装一个有界可释放的内存入口任务，错误与回收策略由用例显式提供.

    Args:
        owner: 当前循环内的测试入口所有者。
        name: 该入口记录的唯一名称。
        gate: 控制任务终止的内存事件。
        error: 终止时抛出的异常，None 表示正常退出。
        retire: 是否在 finally 中移除已释放记录。

    Returns:
        被入口 supervisor 持有的异步任务。
    """
    async def run():
        try:
            await gate.wait()
            if error is not None:
                raise error
        finally:
            if retire:
                owner._entries.pop(name, None)

    task = owner._supervisor.spawn(run(), name)
    owner._entries[name] = SimpleNamespace(inbox=asyncio.Queue(), task=task)
    return task


@pytest.mark.parametrize('error_type', [RuntimeError, asyncio.CancelledError])
def test_failed_close_can_retry_after_entry_has_released(error_type):
    """任务已回收后，失败或取消的旧 close 不能永久毒化后续关闭.

    Args:
        error_type: 首次入口任务终止时的异常种类。
    """
    async def scenario():
        owner, gate = _owner(), asyncio.Event()
        started = _instrument_close(owner)
        entry = _entry(owner, 'failed', gate, error_type())
        waiter = asyncio.create_task(owner.close())
        try:
            await asyncio.wait_for(started.get(), 1)
            gate.set()
            with pytest.raises(error_type):
                await asyncio.wait_for(waiter, 1)
            failed = owner._close_task
            assert failed.done() and owner._closing and not owner._closed
            assert not owner._entries
            with pytest.raises(RuntimeError, match='before destroying'):
                owner.destroy()
            await asyncio.wait_for(owner.close(), 1)
            assert owner._close_task is not failed
            assert owner._closed and started.qsize() == 1
            completed = owner._close_task
            await owner.close()
            assert owner._close_task is completed and started.qsize() == 1
            owner.destroy()
        finally:
            gate.set()
            await asyncio.gather(entry, waiter, return_exceptions=True)
            await owner._supervisor.cancel_and_wait()

    asyncio.run(scenario())


def test_canceled_waiter_rejoins_live_close_without_restarting():
    """取消外层等待者不取消旧收尾任务，重复 close 仍加入同一次收尾."""
    async def scenario():
        owner, gate = _owner(), asyncio.Event()
        started = _instrument_close(owner)
        entry = _entry(owner, 'pending', gate)
        first = asyncio.create_task(owner.close())
        second = None
        try:
            await asyncio.wait_for(started.get(), 1)
            owned_close = owner._close_task
            first.cancel()
            with pytest.raises(asyncio.CancelledError):
                await first
            assert not owned_close.done() and not entry.done()
            second = asyncio.create_task(owner.close())
            await asyncio.sleep(0)
            assert owner._close_task is owned_close and started.empty()
            assert not owner._closed and not second.done()
            gate.set()
            await asyncio.wait_for(second, 1)
            assert owner._closed
            owner.destroy()
        finally:
            gate.set()
            await asyncio.gather(entry, first, *([second] if second else []),
                                 return_exceptions=True)
            await owner._supervisor.cancel_and_wait()

    asyncio.run(scenario())


def test_retry_still_waits_for_other_live_entry():
    """失败后的重试必须等待剩余入口，不得因重新创建 close 就提前报已关闭."""
    async def scenario():
        owner = _owner()
        failed_gate, live_gate = asyncio.Event(), asyncio.Event()
        started = _instrument_close(owner)
        failed_entry = _entry(owner, 'failed', failed_gate, RuntimeError('fixture'))
        live_entry = _entry(owner, 'live', live_gate)
        first = asyncio.create_task(owner.close())
        second = None
        try:
            await asyncio.wait_for(started.get(), 1)
            failed_gate.set()
            with pytest.raises(RuntimeError, match='fixture'):
                await asyncio.wait_for(first, 1)
            second = asyncio.create_task(owner.close())
            await asyncio.wait_for(started.get(), 1)
            assert not live_entry.done() and not second.done() and not owner._closed
            assert set(owner._entries) == {'live'}
            with pytest.raises(RuntimeError, match='before destroying'):
                owner.destroy()
            live_gate.set()
            await asyncio.wait_for(second, 1)
            assert owner._closed and not owner._entries
            owner.destroy()
        finally:
            failed_gate.set()
            live_gate.set()
            await asyncio.gather(failed_entry, live_entry, first,
                                 *([second] if second else []), return_exceptions=True)
            await owner._supervisor.cancel_and_wait()

    asyncio.run(scenario())


def test_failed_retained_entry_is_not_silently_discarded_on_retry():
    """仍有失败记录时重试必须继续失败，不能清空所有权假报可销毁."""
    async def scenario():
        owner, gate = _owner(), asyncio.Event()
        started = _instrument_close(owner)
        entry = _entry(owner, 'retained', gate, RuntimeError('retained'), retire=False)
        first = asyncio.create_task(owner.close())
        try:
            await asyncio.wait_for(started.get(), 1)
            gate.set()
            with pytest.raises(RuntimeError, match='retained'):
                await asyncio.wait_for(first, 1)
            old = owner._close_task
            with pytest.raises(RuntimeError, match='retained'):
                await asyncio.wait_for(owner.close(), 1)
            assert owner._close_task is not old and started.qsize() == 1
            assert not owner._closed and set(owner._entries) == {'retained'}
            with pytest.raises(RuntimeError, match='before destroying'):
                owner.destroy()
        finally:
            gate.set()
            await asyncio.gather(entry, first, return_exceptions=True)
            await owner._supervisor.cancel_and_wait()

    asyncio.run(scenario())
