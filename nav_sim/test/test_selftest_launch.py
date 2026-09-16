"""Launcher shutdown ownership and event-loop fairness without ROS or real signals."""

import asyncio
import signal
from types import SimpleNamespace

import pytest

from nav_sim.selftest_launch import close_fixture, main


def test_shutdown_keeps_loop_available_until_peer_can_close(monkeypatch):
    """A scheduled peer response can finish before the synchronous final drain."""
    async def scenario():
        calls = []
        state = SimpleNamespace(status=None)
        child = SimpleNamespace(pid=123, poll=lambda: state.status,
                                close=lambda **kw: calls.append(('drain', kw)) or state.status)

        def interrupt(pid, signum):
            calls.append((pid, signum))
            asyncio.get_running_loop().call_later(0.001, setattr, state, 'status', 130)

        monkeypatch.setattr('nav_sim.selftest_launch.os.killpg', interrupt)
        assert await close_fixture(child, 1) == 130
        assert calls == [(123, signal.SIGINT), ('drain', {'timeout_sec': 0})]

    asyncio.run(scenario())


def test_shutdown_timeout_drains_owned_group_but_is_not_success(monkeypatch):
    """An unresponsive leader cannot turn forced cleanup into a successful result."""
    calls = []
    child = SimpleNamespace(pid=123, poll=lambda: None,
                            close=lambda **kw: calls.append(kw) or -9)
    monkeypatch.setattr('nav_sim.selftest_launch.os.killpg', lambda *args: None)
    with pytest.raises(TimeoutError, match='forced'):
        asyncio.run(close_fixture(child, 0.001))
    assert calls == [{'timeout_sec': 0}]


@pytest.mark.parametrize('timeout', [0, -1, float('inf'), float('nan'), True])
def test_invalid_shutdown_grace_never_touches_a_process(timeout):
    """Bad public API values are rejected before even observing a child."""
    with pytest.raises(ValueError):
        asyncio.run(close_fixture(None, timeout))


def test_non_sandbox_launcher_refuses_before_plan_io(tmp_path, monkeypatch):
    """A nominal start switch cannot override the hard domain check."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    with pytest.raises(RuntimeError, match='99'):
        main(['--run-dir', str(tmp_path / 'missing'), '--plan-sha256', 'bad',
              '--start-verified-fixture'])


def test_missing_start_switch_is_not_execution_authority(tmp_path):
    """A prepared path and hash do not by themselves authorize process creation."""
    with pytest.raises(SystemExit) as error:
        main(['--run-dir', str(tmp_path / 'missing'), '--plan-sha256', 'bad'])
    assert error.value.code == 2
