"""Spin and navigation share one motion owner; all ROS I/O is in-memory fake data."""

import asyncio
from concurrent.futures import Future
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from nav_tasks.adapters.ros.action_session import ActionSession
from nav_tasks.domain.errors import BusyError, ClosedError, ConfigError, RuntimeAssemblyError
from nav_tasks.domain.tasks import NavStatus, SpinGoal
from nav_tasks.nav2_client import Nav2Client

from test_action_session import _Client, _done, _Handle, _settle
from test_nav2_client import _Clock, _goal, _message


def _spin_message():
    return NS(target_yaw=0.0, time_allowance=NS(sec=0, nanosec=0))


def _backend(nav=None, spin=None, **kwargs):
    nav = nav if nav is not None else _Client()
    spin = spin if spin is not None else _Client()
    node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: 123)))
    return Nav2Client(
        node, 'fixture_nav', 0, client=nav, goal_factory=_message,
        spin_action_name='fixture_spin', spin_server_timeout_sec=0,
        spin_client=spin, spin_goal_factory=_spin_message, **kwargs)


@pytest.mark.parametrize('status,expected', [
    (4, NavStatus.SUCCEEDED), (5, NavStatus.CANCELED), (6, NavStatus.FAILED)])
def test_spin_one_attempt_fields_feedback_and_outcome(status, expected):
    """Keep signed yaw and duration precision, forward angular feedback, and never retry."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        client = _Client(_done(handle))
        backend = _backend(spin=client)
        feedback = asyncio.Queue()
        running = asyncio.create_task(backend.spin(SpinGoal(-0.75, 30.125), feedback.put_nowait))
        await client.sent.wait()
        message = client.goals[0]
        assert message.target_yaw == -0.75
        assert vars(message.time_allowance) == {'sec': 30, 'nanosec': 125000000}
        client.feedback(NS(feedback=NS(angular_distance_traveled=-0.3)))
        assert await feedback.get() == -0.3
        handle.result_future.set_result(NS(status=status))
        result = await running
        assert result.status == expected
        if status == 6:
            assert result.detail == 'Spin 终态 status=6(ABORTED)'
        assert len(client.goals) == 1 and handle.cancel_calls == 0
        await backend.close()

    asyncio.run(scenario())


def test_spin_admission_failures_release_shared_slot():
    """Unavailable and rejected Spin do not prevent the next ordinary navigation."""
    async def scenario():
        spin = _Client(ready=False)
        backend = _backend(spin=spin)
        unavailable = await backend.spin(SpinGoal(1, 30), lambda value: None)
        assert unavailable.status == NavStatus.FAILED
        assert unavailable.detail == "Spin action server 'fixture_spin' 在 0.0s 内不可用"
        assert not spin.goals
        spin.ready = True
        spin.send = _done(_Handle(accepted=False))
        rejected = await backend.spin(SpinGoal(1, 30), lambda value: None)
        assert rejected.detail == 'Spin 目标被拒绝'
        assert (await backend.navigate(_goal(), lambda value: None)).succeeded
        await backend.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('first', ['navigate', 'spin'])
def test_navigation_and_spin_cannot_overlap(first):
    """The second action type is refused until the first has a confirmed terminal result."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        nav, spin = _Client(_done(handle)), _Client(_done(handle))
        backend = _backend(nav, spin)

        async def navigate():
            return await backend.navigate(_goal(), lambda value: None)

        async def rotate():
            return await backend.spin(SpinGoal(0.5, 10), lambda value: None)

        active, next_one = (navigate, rotate) if first == 'navigate' else (rotate, navigate)
        running = asyncio.create_task(active())
        await (nav.sent if first == 'navigate' else spin.sent).wait()
        with pytest.raises(BusyError):
            await next_one()
        assert len(nav.goals) + len(spin.goals) == 1
        handle.result_future.set_result(NS(status=4))
        assert (await running).succeeded
        assert (await next_one()).succeeded
        assert len(nav.goals) == len(spin.goals) == 1
        await backend.close()

    asyncio.run(scenario())


def test_late_spin_handle_cancel_retains_both_barriers_and_fences_feedback():
    """A timed-out cancel waits for late acceptance, cancellation acknowledgement and result."""
    async def scenario():
        send = Future()
        spin = _Client(send)
        backend = _backend(spin=spin, cancel_timeout_sec=0.001)
        feedback = []
        running = asyncio.create_task(backend.spin(SpinGoal(0.5, 10), feedback.append))
        await spin.sent.wait()
        with pytest.raises(asyncio.TimeoutError):
            await backend.cancel()
        with pytest.raises(BusyError):
            await backend.navigate(_goal(), lambda value: None)
        handle = _Handle(cancel_status=None)
        handle.cancel_future = Future()
        send.set_result(handle)
        await handle.cancel_started.wait()
        spin.feedback(NS(feedback=NS(angular_distance_traveled=1)))
        handle.result_future.set_result(NS(status=5))
        await _settle()
        assert backend._session.busy and feedback == []
        handle.cancel_future.set_result(None)
        try:
            await running
        except (asyncio.CancelledError, asyncio.TimeoutError):
            pass
        await backend.cancel()
        assert handle.cancel_calls == 1
        assert (await backend.navigate(_goal(), lambda value: None)).succeeded
        await backend.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('yaw,seconds', [
    (float('nan'), 1), (float('inf'), 1), (1e39, 1),
    (0, float('nan')), (0, float('inf')), (0, -1), (0, 2**31),
    (True, 1), (0, True)])
def test_invalid_spin_goal_never_claims_or_sends(yaw, seconds):
    """Reject unrepresentable ROS fields before touching motion ownership."""
    async def scenario():
        spin = _Client()
        backend = _backend(spin=spin)
        with pytest.raises(ConfigError):
            await backend.spin(SpinGoal(yaw, seconds), lambda value: None)
        assert spin.goals == [] and not backend._session.busy
        assert (await backend.navigate(_goal(), lambda value: None)).succeeded
        await backend.close()

    asyncio.run(scenario())


def _ros_factories(monkeypatch):
    state = NS(created=[], fail_create=False, fail_destroy=set())

    def action_client(node, action_type, name):
        if state.fail_create and name == 'fixture_spin':
            raise OSError('spin construction failed')
        client = _Client()
        client.name = name
        client.destroyed = 0

        def destroy():
            if name in state.fail_destroy:
                raise OSError(f'{name} destroy failed')
            client.destroyed += 1

        client.destroy = destroy
        state.created.append(client)
        return client

    ros, actions = ModuleType('rclpy'), ModuleType('rclpy.action')
    actions.ActionClient = action_client
    ros.action = actions
    messages, nav_actions = ModuleType('nav2_msgs'), ModuleType('nav2_msgs.action')
    nav_actions.NavigateToPose = NS(Goal=_message)
    nav_actions.Spin = NS(Goal=_spin_message)
    messages.action = nav_actions
    for name, module in (('rclpy', ros), ('rclpy.action', actions),
                         ('nav2_msgs', messages), ('nav2_msgs.action', nav_actions)):
        monkeypatch.setitem(sys.modules, name, module)
    return state


def _owned_backend(**kwargs):
    return Nav2Client(object(), 'fixture_nav', 0, spin_action_name='fixture_spin', **kwargs)


def test_close_waits_for_spin_before_destroying_either_owned_client(monkeypatch):
    """Neither action client is destroyed when rotation cleanup still owns the slot."""
    state = _ros_factories(monkeypatch)

    async def scenario():
        backend = _owned_backend(cancel_timeout_sec=0.001)
        nav, spin = state.created
        handle = _Handle(cancel_status=None)
        admitted = asyncio.Event()

        def get_result():
            admitted.set()
            return handle.result_future

        handle.get_result_async = get_result
        spin.send = _done(handle)
        running = asyncio.create_task(backend.spin(SpinGoal(0.5, 10), lambda value: None))
        await spin.sent.wait()
        await admitted.wait()
        with pytest.raises(asyncio.TimeoutError):
            await backend.close()
        assert nav.destroyed == spin.destroyed == 0
        assert handle.cancel_calls == 1
        with pytest.raises(ClosedError):
            await backend.spin(SpinGoal(0.5, 10), lambda value: None)
        handle.result_future.set_result(NS(status=5))
        outcome = await running
        assert outcome.status == NavStatus.CANCELED
        await backend.close()
        await backend.close()
        assert nav.destroyed == spin.destroyed == 1

    asyncio.run(scenario())


@pytest.mark.parametrize('failed', ['fixture_nav', 'fixture_spin'])
def test_close_retries_only_failed_client_after_trying_both(monkeypatch, failed):
    """One destructor failure cannot skip other clients or double-destroy a completed stage."""
    state = _ros_factories(monkeypatch)

    async def scenario():
        backend = _owned_backend()
        state.fail_destroy.add(failed)
        with pytest.raises(OSError, match='destroy failed'):
            await backend.close()
        assert [client.destroyed for client in state.created] == (
            [0, 1] if failed == 'fixture_nav' else [1, 0])
        state.fail_destroy.clear()
        await backend.close()
        await backend.close()
        assert [client.destroyed for client in state.created] == [1, 1]

    asyncio.run(scenario())


@pytest.mark.parametrize('retry_cleanup', [False, True])
def test_second_constructor_failure_retains_rollback_without_sealing_session(
    monkeypatch, retry_cleanup,
):
    """Partial construction never loses its first client or closes a caller's shared session."""
    state = _ros_factories(monkeypatch)

    async def scenario():
        session = ActionSession()
        state.fail_create = True
        if retry_cleanup:
            state.fail_destroy.add('fixture_nav')
            with pytest.raises(RuntimeAssemblyError) as captured:
                _owned_backend(session=session)
            error = captured.value
            assert isinstance(error.__cause__, OSError)
            assert state.created[0].destroyed == 0
            state.fail_destroy.clear()
            await error.cleanup()
            await error.cleanup()
        else:
            with pytest.raises(OSError, match='spin construction failed'):
                _owned_backend(session=session)
        assert state.created[0].destroyed == 1
        assert not session.busy
        # Construction rollback is not shutdown authority over the shared slot.
        result = await session.execute(_Client(), object, lambda raw: None, server_timeout_sec=0)
        assert result.response.status == 4
        await session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('inject_nav,inject_spin', [(True, True), (True, False), (False, True)])
def test_injected_clients_are_never_destroyed_by_backend(monkeypatch, inject_nav, inject_spin):
    """Mixed injected/owned clients preserve separate destruction responsibility."""
    state = _ros_factories(monkeypatch)

    async def scenario():
        injected, kwargs = [], {}
        for enabled, key, factory_key, factory in (
                (inject_nav, 'client', 'goal_factory', _message),
                (inject_spin, 'spin_client', 'spin_goal_factory', _spin_message)):
            if enabled:
                client = _Client()
                client.destroy = lambda: pytest.fail('destroyed injected client')
                injected.append(client)
                kwargs.update({key: client, factory_key: factory})
        backend = _owned_backend(**kwargs)
        await backend.close()
        assert all(client.destroyed == 1 for client in state.created)
        assert len(injected) + len(state.created) == 2

    asyncio.run(scenario())


def test_disabled_spin_and_invalid_wiring_construct_no_extra_clients(monkeypatch):
    """Ordinary mode never imports or constructs Spin, and invalid injection fails before I/O."""
    state = _ros_factories(monkeypatch)

    async def scenario():
        backend = Nav2Client(object(), 'fixture_nav', 0)
        assert len(state.created) == 1
        with pytest.raises(ConfigError, match='not configured'):
            await backend.spin(SpinGoal(0, 1), lambda value: None)
        await backend.close()
        for kwargs in (
                {'spin_client': _Client()},
                {'spin_goal_factory': _spin_message},
                {'spin_client': _Client(), 'spin_goal_factory': _spin_message},
                {'spin_action_name': ''}, {'spin_server_timeout_sec': float('nan')},
                {'spin_server_timeout_sec': -1}):
            with pytest.raises(ConfigError):
                Nav2Client(object(), 'fixture_nav', 0, **kwargs)
        assert len(state.created) == 1

    asyncio.run(scenario())


def test_spin_readiness_uses_its_own_timeout_and_creates_fresh_messages():
    """Spin preserves its independent five-second default and doesn't reuse goal objects."""
    async def scenario():
        clock, spin = _Clock(), _Client(ready=False)
        backend = Nav2Client(
            object(), 'fixture_nav', 0, client=_Client(), goal_factory=_message,
            spin_action_name='fixture_spin', spin_client=spin,
            spin_goal_factory=_spin_message, clock=clock)
        result = await backend.spin(SpinGoal(0, 0), lambda value: None)
        assert '5.0s' in result.detail and clock.value == 15
        spin.ready = True
        assert (await backend.spin(SpinGoal(4, 0), lambda value: None)).succeeded
        assert (await backend.spin(SpinGoal(-4, 0.1234567899), lambda value: None)).succeeded
        assert spin.goals[0] is not spin.goals[1]
        assert [message.target_yaw for message in spin.goals] == [4, -4]
        assert vars(spin.goals[1].time_allowance) == {'sec': 0, 'nanosec': 123456789}
        await backend.close()

    asyncio.run(scenario())


def test_task_manager_preemption_from_spin_to_poi_waits_for_shared_cleanup():
    """The real manager and POI skill cannot start navigation ahead of rotation cancellation."""
    from nav_tasks.api import CommandRegistry, EventKind, PreemptPolicy, TaskManager, TaskRequest
    from nav_tasks.core.goal_resolution import DirectGoalResolver
    from nav_tasks.core.handlers.navigate_to import NavigateToPoi

    async def scenario():
        handle = _Handle(cancel_status=None)
        handle.cancel_future = Future()
        spin, nav = _Client(_done(handle)), _Client()
        backend = _backend(nav, spin)
        poi = NavigateToPoi(backend, NS(get=lambda key: _goal().poi), DirectGoalResolver())
        registry = CommandRegistry()

        async def rotate(context, goal):
            context.check_cancelled()
            return await backend.spin(goal, lambda angle: None)

        registry.register('rotate_fixture', lambda payload: SpinGoal(0.5, 10), rotate)
        registry.register('navigate_to', poi.decode, poi.run)
        manager = TaskManager(registry, backend.cancel)
        events = manager.subscribe_events()
        await manager.submit(TaskRequest('rotate', 'rotate_fixture', {}))
        await spin.sent.wait()
        replacement = asyncio.create_task(manager.submit(TaskRequest(
            'navigate', 'navigate_to', {'target_id': 'POI_1'}, preempt=PreemptPolicy.ALLOW)))
        await handle.cancel_started.wait()
        assert not nav.goals
        handle.result_future.set_result(NS(status=5))
        await _settle()
        assert not replacement.done() and not nav.goals
        handle.cancel_future.set_result(None)
        await replacement
        arrived = await asyncio.wait_for(events.get(), 1)
        assert (arrived.task_id, arrived.kind) == ('navigate', EventKind.SUCCEEDED)
        assert events.empty()  # Preemption is silent; only explicit cancel emits paused.
        assert handle.cancel_calls == 1 and len(nav.goals) == 1
        await manager.close()
        await backend.close()

    asyncio.run(scenario())
