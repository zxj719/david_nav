"""Ownership/failure/cancellation checks with no ROS, sockets or child processes."""

import asyncio
import json
import sys
from types import SimpleNamespace

import pytest

from nav_sim import selftest_runtime as runtime
from test_selftest_evidence import CASES, _trace


@pytest.mark.parametrize('fault', ['domain', 'authority', 'timeout', 'idle'])
def test_runtime_refuses_before_preparation_or_ros(tmp_path, monkeypatch, fault):
    """Internal entry points repeat CLI safety gates before any caller callback."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '0' if fault == 'domain' else '99')
    monkeypatch.setitem(sys.modules, 'nav_sim.selftest_ros', None)

    def guard():
        raise RuntimeError('host busy')

    monkeypatch.setattr(runtime, 'require_idle_host', guard)
    with pytest.raises((RuntimeError, ValueError)):
        asyncio.run(runtime.execute(lambda uri: pytest.fail('unexpected preparation'),
                                    0 if fault == 'timeout' else 1, False, fault != 'authority'))
    assert list(tmp_path.iterdir()) == []


def _fakes(tmp_path, monkeypatch, fault):
    state = SimpleNamespace(closed=False, drained=False, server_closed=False, task=None)

    class Server:
        sockets = [SimpleNamespace(getsockname=lambda: ('127.0.0.1', 18101))]

        async def __aenter__(self):
            return self

        async def __aexit__(self, *_args):
            self.close()
            await self.wait_closed()

        def close(self):
            state.server_closed = True

        async def wait_closed(self):
            if fault == 'cancel_during_ws_close' and state.task is not None:
                task, state.task = state.task, None
                asyncio.get_running_loop().call_later(0.001, task.cancel)
                await asyncio.sleep(0.02)

    class Observer:
        def __init__(self, *_args):
            if fault == 'observer_start':
                raise RuntimeError('observer could not start')

        def step(self):
            pass

        def close(self):
            state.closed = True

    async def cleanup(_child):
        await asyncio.sleep(0)
        state.drained = True
        return 143 if fault == 'bad_exit' else 0

    async def drive(scene, trace, *args):
        state.task = asyncio.current_task()
        if fault == 'drive':
            raise RuntimeError('experiment failure')
        if fault == 'cancel':
            raise asyncio.CancelledError()
        nav, ws = _trace()
        for row in nav:
            trace.receive('nav', json.dumps(row))
        for row in ws:
            trace.receive('ws', json.dumps(row))
        return [{'task_id': case.task_id, 'pose': case.goal} for case in CASES]

    monkeypatch.setenv('ROS_DOMAIN_ID', '99')
    monkeypatch.setattr(runtime, 'require_idle_host', lambda: None)
    monkeypatch.setattr(runtime.websockets, 'serve', lambda *a, **kw: Server())
    monkeypatch.setitem(sys.modules, 'nav_sim.selftest_ros', SimpleNamespace(Observer=Observer))
    monkeypatch.setattr(runtime, 'OwnedProcess', lambda *a, **kw: object())
    monkeypatch.setattr(runtime, 'close_fixture', cleanup)
    monkeypatch.setattr(runtime, 'drive', drive)
    monkeypatch.setattr(runtime, 'load_scene', lambda *a: None)
    monkeypatch.setattr(runtime, 'audit_processes', lambda *a, **kw: True)
    scene = SimpleNamespace(root=tmp_path, digest='private', cases=CASES)
    return state, scene


@pytest.mark.parametrize('fault', [None, 'drive', 'observer_start', 'bad_exit'])
def test_runtime_result_depends_on_execution_and_owned_cleanup(tmp_path, monkeypatch, fault):
    """The real orchestration cannot promote a failed phase to a complete result."""
    state, scene = _fakes(tmp_path, monkeypatch, fault)
    result = asyncio.run(runtime.execute(lambda uri: scene, 1, False, True))
    assert result['passed'] is result['complete'] is (fault is None)
    assert json.loads(json.dumps(result)) == json.loads((tmp_path / 'result.json').read_text())
    assert state.server_closed
    if fault != 'observer_start':
        assert state.closed and state.drained


@pytest.mark.parametrize('fault', ['cancel', 'cancel_during_ws_close'])
def test_cancellation_cannot_skip_observer_or_child_cleanup(tmp_path, monkeypatch, fault):
    """Cancellation throughout teardown remains failure, but every owned resource is closed."""
    state, scene = _fakes(tmp_path, monkeypatch, fault)
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(runtime.execute(lambda uri: scene, 1, False, True))
    assert state.closed and state.drained and state.server_closed
    result = json.loads((tmp_path / 'result.json').read_text())
    assert not result['passed'] and not result['complete']
    assert 'CancelledError' in result['errors']
