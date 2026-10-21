"""保护运行时及存储说明，并用内存回调核验构造回滚的失败保留."""

import asyncio

import pytest

from nav_tasks.app.runtime import _Rollback


@pytest.mark.parametrize('error_type', [RuntimeError, asyncio.CancelledError])
def test_rollback_attempts_all_callbacks_and_retries_only_failure(error_type):
    """回滚中的普通异常或取消不跳过剩余资源，成功回调不会重复执行.

    Args:
        error_type: 中间异步回调首次抛出的异常类型。
    """
    async def scenario():
        owner, calls = _Rollback(), []
        failed = False
        error = error_type('fixture')

        async def middle():
            nonlocal failed
            calls.append('middle')
            if not failed:
                failed = True
                raise error

        owner.track(lambda: calls.append('first'))
        owner.track(middle)
        owner.track(lambda: calls.append('last'))
        with pytest.raises(error_type) as caught:
            await owner.cleanup()
        assert caught.value is error
        assert calls == ['last', 'middle', 'first']
        assert owner._callbacks == [middle]
        await owner.cleanup()
        assert calls == ['last', 'middle', 'first', 'middle']
        assert owner._callbacks == []
        await owner.cleanup()
        assert calls == ['last', 'middle', 'first', 'middle']

    asyncio.run(scenario())
