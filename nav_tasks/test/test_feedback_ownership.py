"""R3 queued feedback ownership without ROS, including GC and stale callbacks."""

import ast
import asyncio
import gc
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import weakref

import pytest

from nav_tasks.adapters.ros.action_session import ActionSession

from test_action_session import _Client, _done, _Handle, _settle


PACKAGE = Path(__file__).resolve().parents[1] / 'nav_tasks'


def _assert_owned_task_spawns(path):
    tree = ast.parse(path.read_text())
    bare = [node.lineno for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and ((isinstance(node.func, ast.Attribute)
                  and node.func.attr in ('create_task', 'ensure_future'))
                 or (isinstance(node.func, ast.Name)
                     and node.func.id in ('create_task', 'ensure_future')))]
    assert not bare, f'{path}: unowned task spawn at lines {bare}'


def test_task_runtime_creates_tasks_only_through_supervisor():
    """Feedback and other task paths cannot regress to untracked task construction."""
    for path in PACKAGE.rglob('*.py'):
        if path != PACKAGE / 'core/supervisor.py':
            _assert_owned_task_spawns(path)


def test_feedback_burst_is_strongly_queued_without_creating_async_tasks(monkeypatch):
    """ROS-thread messages survive GC while queued and are released after delivery."""
    async def scenario():
        loop = asyncio.get_running_loop()
        owner_thread = threading.get_ident()
        session, handle = ActionSession(), _Handle()
        client, received, references = _Client(_done(handle)), [], []

        class Message:
            def __init__(self, value):
                self.value = value

        def feedback(message):
            received.append((message.value, threading.get_ident()))

        def produce():
            for value in range(100):
                message = Message(value)
                references.append(weakref.ref(message))
                client.feedback(message)

        navigation = asyncio.create_task(session.execute(
            client, object, feedback, server_timeout_sec=0))
        try:
            await client.sent.wait()
            await _settle()
            tasks = set(asyncio.all_tasks())
            with monkeypatch.context() as patch:
                patch.setattr(loop, 'create_task',
                              lambda *a, **k: pytest.fail('feedback created an async task'))
                producer = threading.Thread(target=produce)
                producer.start()
                producer.join(timeout=1)
                assert not producer.is_alive()
                gc.collect()
                assert not received and all(reference() is not None for reference in references)
                await _settle()
                assert received == [(value, owner_thread) for value in range(100)]
                assert set(asyncio.all_tasks()) == tasks
                gc.collect()
                assert all(reference() is None for reference in references)
        finally:
            await session.close()
            await asyncio.gather(navigation, return_exceptions=True)

    asyncio.run(scenario())


@pytest.mark.parametrize('ending', ['cancel', 'terminal'])
def test_queued_and_stale_feedback_never_changes_a_replacement(ending):
    """Cancellation seals queued feedback; completed attempts cannot feed a new owner."""
    async def scenario():
        session, received = ActionSession(), []
        first_handle, second_handle = _Handle(), _Handle()
        first_client, second_client = _Client(_done(first_handle)), _Client(_done(second_handle))
        first = asyncio.create_task(session.execute(
            first_client, object, lambda raw: received.append(('first', raw)),
            server_timeout_sec=0))
        second = None
        try:
            await first_client.sent.wait()
            await _settle()
            if ending == 'cancel':
                first_client.feedback('queued-before-cancel')
                # cancel marks the attempt before yielding to the already queued callback.
                await session.cancel()
                await first
                assert received == []
            else:
                first_handle.result_future.set_result(SimpleNamespace(status=4))
                await first
            second = asyncio.create_task(session.execute(
                second_client, object, lambda raw: received.append(('second', raw)),
                server_timeout_sec=0))
            await second_client.sent.wait()
            first_client.feedback('stale')
            second_client.feedback('current')
            await _settle()
            assert received == [('second', 'current')]
        finally:
            await session.close()
            await asyncio.gather(*(task for task in (first, second) if task is not None),
                                 return_exceptions=True)

    asyncio.run(scenario())


if __name__ == '__main__':
    # Read-only diagnostic: apply the identical contract to the legacy source.
    _assert_owned_task_spawns(Path(sys.argv[1]))
