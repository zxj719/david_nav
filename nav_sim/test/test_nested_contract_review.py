"""核验自测启动门禁拒绝不健康证据，所有端口均为内存替身."""

import asyncio

import pytest

from nav_sim.selftest_coordinator import drive
from test_selftest_coordinator import _ports


@pytest.mark.parametrize('fault', ['peer', 'severity'])
def test_ready_startup_still_rejects_unhealthy_evidence(tmp_path, fault):
    """只读就绪不能掩盖连接错误或不健康任务事件，且始终不发送目标.

    Args:
        tmp_path: 测试私有证据目录。
        fault: 注入连接错误或任务事件严重程度错误。
    """
    scene, trace, peer, observer, child, state = _ports(tmp_path)
    if fault == 'peer':
        peer.error = 'connection_failed'
    else:
        trace.nav.append({'source': 'tasks', 'event_type': 'task_admitted',
                          'severity': 'warn'})
    with trace, pytest.raises(RuntimeError, match=(
            'connection_failed' if fault == 'peer' else 'unhealthy task observation')):
        asyncio.run(drive(scene, trace, peer, observer, child, 1, True))
    assert state.sent == []
