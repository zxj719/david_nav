"""Frozen legacy run_action audit compared with the actual shared-backend thread bridge."""

import ast
import asyncio
from concurrent.futures import Future
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest

from nav_tasks.adapters.ros.dock_motion import _DockMotion
from nav_tasks.core.supervisor import TaskSupervisor
from test_action_session import _Client, _done, _Handle, _settle
from test_dock_worker import _bounded, _Log
from test_nav2_client import _Clock
from test_spin_backend import _backend

LEGACY = Path(__file__).resolve().parents[2] / 'david_nav/david_nav/dock_navigator.py'


def _legacy(case, label):
    tree = ast.parse(LEGACY.read_text())
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == 'DockNavigator')
    method = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                  and item.name == 'run_action')
    scope = {'GoalStatus': NS(STATUS_SUCCEEDED=4)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(LEGACY), 'exec'), scope)
    log = _Log()
    flag = [case == 'pre_canceled']
    handle = _Handle(status=int(case) if case.isdigit() else 4, accepted=case != 'rejected')
    client = _Client(_done(handle))
    client.wait_for_server = lambda timeout_sec: case != 'unavailable'

    def wait(future, timeout, should_cancel=None):
        if future is client.send:
            return case != 'send_timeout'
        if future is handle.result_future and case in ('result_timeout', 'canceled'):
            flag[0] = case == 'canceled'
            return False
        return True

    old = NS(_cancel_check=lambda: flag[0], get_logger=lambda: log, wait_future=wait)
    result = scope['run_action'](old, client, object(), label, 0)
    return result, old._last_run_canceled, log.lines


@pytest.mark.parametrize('kind,label', [('navigate', '阶段1(预备点)'), ('spin', 'Spin#1')])
@pytest.mark.parametrize('case', [
    '4', '5', '6', 'unavailable', 'rejected', 'send_timeout', 'result_timeout',
    'canceled', 'pre_canceled'])
def test_shared_worker_action_audit_matches_old_run_action(kind, label, case):
    """Preserve every old action log branch while strengthening motion cleanup ownership."""
    async def scenario():
        expected = _legacy(case, label)
        handle = _Handle(
            status=int(case) if case.isdigit() else None, accepted=case != 'rejected')
        send = Future() if case == 'send_timeout' else _done(handle)
        client = _Client(send, ready=case != 'unavailable')
        backend = _backend(nav=client, spin=client, clock=_Clock())
        stop = threading.Event()
        if case == 'pre_canceled':
            stop.set()

        class Log(_Log):
            def error(self, text):
                super().error(text)
                if case == 'send_timeout' and '没有应答' in text:
                    send.set_result(handle)

        log = Log()
        supervisor = TaskSupervisor(lambda name, error: None)
        motion = _DockMotion(backend, 'fixture_global', stop, supervisor, log, None, 5, 0)

        def call():
            timeout = 0 if case == 'result_timeout' else 30
            if kind == 'navigate':
                return motion.navigate((1, 2, 0), label, timeout, 0)
            return motion.spin(-0.7, label, 30, timeout)

        worker = supervisor.spawn(asyncio.to_thread(call), 'golden-worker')
        if case == 'canceled':
            await _bounded(client.sent.wait())
            await _settle()
            motion.request_cancel()
        result = await _bounded(worker)
        assert (result.succeeded, result.canceled, log.lines) == expected
        assert not backend._session.busy
        await backend.close()
        await supervisor.cancel_and_wait()

    asyncio.run(scenario())
