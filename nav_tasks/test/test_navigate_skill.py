"""Pure scripted-backend tests of sequential POI navigation and cancellation."""

import asyncio
from dataclasses import replace
from types import SimpleNamespace

import pytest

from nav_tasks.core.goal_resolution import DirectGoalResolver
from nav_tasks.core.handlers.navigate_to import NavigateToPoi
from nav_tasks.core.registry import CommandRegistry, TaskContext
from nav_tasks.core.retry import RetryPolicy
from nav_tasks.core.state import TaskManager
from nav_tasks.domain.errors import ConfigError, PoiNotFoundError
from nav_tasks.domain.tasks import (
    EventKind, NavOutcome, NavStatus, ResolvedGoal, TaskRequest, TaskState,
)
from nav_tasks.domain.types import PoseRecord, StoredPoi


def _poi():
    pose = PoseRecord('world', 1, 2, 0, 0.25, 10)
    return StoredPoi('POI_1', '前台', pose, pose, 'session')


class _Clock:
    def __init__(self):
        self.value = 10.0
        self.delays = []
        self.sleep_entered = asyncio.Event()
        self.sleep_release = None

    def now(self):
        return self.value

    async def sleep(self, seconds):
        self.delays.append(seconds)
        self.value += seconds
        self.sleep_entered.set()
        if self.sleep_release is not None:
            await self.sleep_release.wait()


class _Backend:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.calls = []
        self.starts = asyncio.Queue()
        self.cancel_count = 0

    async def navigate(self, goal, feedback):
        self.calls.append((goal, feedback))
        self.starts.put_nowait(len(self.calls))
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, asyncio.Future):
            return await outcome
        return outcome

    async def cancel(self):
        self.cancel_count += 1


class _Resolver:
    def __init__(self, anchor, admissible):
        self.snapped = replace(anchor, map_pose=replace(anchor.map_pose, x=0.5))
        self.admissible = admissible
        self.rechecks = 0

    def resolve(self, poi):
        return ResolvedGoal(self.snapped, True)

    def is_admissible(self, poi):
        self.rechecks += 1
        return self.admissible


def _skill(backend, resolver=None, clock=None, retry=RetryPolicy()):
    return NavigateToPoi(
        backend, SimpleNamespace(get=lambda key: _poi() if key == 'POI_1' else None),
        resolver or DirectGoalResolver(), retry=retry, clock=clock)


def _context(feedback=None):
    return TaskContext('req_1', 2, 1, feedback or (lambda distance: None))


def _manager(skill, backend):
    registry = CommandRegistry()
    registry.register('navigate_to', skill.decode, skill.run)
    return TaskManager(registry, backend.cancel)


def _request():
    return TaskRequest('req_1', 'navigate_to', {'target_id': 'POI_1'}, sub_id=2)


@pytest.mark.parametrize('admissible,attempts', [(False, 1), (True, 2)])
def test_snap_success_only_revisits_newly_admissible_anchor(admissible, attempts):
    """An inaccessible anchor after snap is success, not an error or retry."""
    async def scenario():
        backend = _Backend([NavOutcome(NavStatus.SUCCEEDED)] * attempts)
        resolver = _Resolver(_poi(), admissible)
        skill = _skill(backend, resolver)
        manager = _manager(skill, backend)
        events = manager.subscribe_events()
        await manager.submit(_request())
        event = await events.get()
        assert event.kind == EventKind.SUCCEEDED
        assert len(backend.calls) == attempts
        assert backend.calls[0][0].poi == resolver.snapped
        if admissible:
            assert backend.calls[1][0].poi == _poi()
        assert resolver.rechecks == 1
        assert all((goal.request_id, goal.sub_id) == ('req_1', 2) for goal, _ in backend.calls)
        assert manager.status().state == TaskState.IDLE
        assert events.empty()
        await manager.close()

    asyncio.run(scenario())


def test_each_segment_gets_its_own_retry_budget_and_only_final_failure_escapes():
    """A successful retry of snap does not consume the anchor retry budget."""
    async def scenario():
        failure = NavOutcome(NavStatus.FAILED, 'Nav2 failed')
        backend = _Backend([failure, NavOutcome(NavStatus.SUCCEEDED), failure, failure])
        clock = _Clock()
        resolver = _Resolver(_poi(), True)
        skill = _skill(backend, resolver, clock)
        manager = _manager(skill, backend)
        events = manager.subscribe_events()
        await manager.submit(_request())
        event = await events.get()
        assert event.kind == EventKind.FAILED
        assert event.detail == '导航失败(共尝试2次): Nav2 failed'
        assert [goal.poi.map_pose.x for goal, _ in backend.calls] == [0.5, 0.5, 1, 1]
        assert clock.delays == [1.0, 1.0]
        assert events.empty()
        await manager.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('status', [NavStatus.FAILED, NavStatus.CANCELED])
def test_unsuccessful_snap_never_starts_anchor(status):
    """Only successful snap permits an anchor recheck or second leg."""
    async def scenario():
        backend = _Backend([NavOutcome(status, 'detail')])
        resolver = _Resolver(_poi(), True)
        skill = _skill(backend, resolver, retry=RetryPolicy(0))
        outcome = await skill.run(_context(), skill.decode({'target_id': 'POI_1'}))
        assert outcome.status == status
        assert len(backend.calls) == 1
        assert resolver.rechecks == 0

    asyncio.run(scenario())


def test_missing_and_unreachable_poi_never_call_backend():
    """Preserve exact missing/unreachable reasons without retry decoration."""
    async def scenario():
        backend = _Backend([])
        skill = _skill(backend, SimpleNamespace(resolve=lambda poi: None))
        with pytest.raises(PoiNotFoundError, match='目标点不存在: POI_404'):
            skill.decode({'target_id': 'POI_404'})
        for invalid in ({}, {'target_id': ''}, {'target_id': 1}):
            with pytest.raises(ConfigError):
                skill.decode(invalid)
        result = await skill.run(_context(), skill.decode({'target_id': 'POI_1'}))
        assert result == NavOutcome(NavStatus.FAILED, '目标点不可达，且未找到可行扩圈点')
        assert backend.calls == []

    asyncio.run(scenario())


def test_abort_during_retry_delay_cannot_resubmit_or_start_anchor():
    """The manager cancels the complete sequential skill, including retry waits."""
    async def scenario():
        backend = _Backend([NavOutcome(NavStatus.FAILED, 'first')])
        clock = _Clock()
        clock.sleep_release = asyncio.Event()
        resolver = _Resolver(_poi(), True)
        skill = _skill(backend, resolver, clock)
        manager = _manager(skill, backend)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await clock.sleep_entered.wait()
        await manager.cancel()
        clock.sleep_release.set()
        assert (await events.get()).status == 'paused'
        assert len(backend.calls) == 1
        assert backend.cancel_count == 1
        assert resolver.rechecks == 0
        assert events.empty()
        await manager.close()

    asyncio.run(scenario())


def test_feedback_throttle_resets_per_attempt_and_discards_old_callbacks():
    """Callbacks from an earlier attempt cannot masquerade as current feedback."""
    async def scenario():
        loop = asyncio.get_running_loop()
        first, second = loop.create_future(), loop.create_future()
        backend = _Backend([first, second])
        clock = _Clock()
        skill = _skill(backend, clock=clock)
        reported = []
        task = asyncio.create_task(skill.run(
            _context(reported.append), skill.decode({'target_id': 'POI_1'})))
        await backend.starts.get()
        feedback1 = backend.calls[0][1]
        feedback1(5)
        clock.value = 10.5
        feedback1(4)
        clock.value = 11
        feedback1(3)
        assert reported == [5.0, 3.0]
        first.set_result(NavOutcome(NavStatus.FAILED, 'retry'))
        await backend.starts.get()
        feedback1(99)
        feedback2 = backend.calls[1][1]
        feedback2(2)
        assert reported == [5.0, 3.0, 2.0]
        second.set_result(NavOutcome(NavStatus.SUCCEEDED))
        assert (await task).succeeded
        clock.value += 10
        feedback1(99)
        feedback2(99)
        assert reported == [5.0, 3.0, 2.0]

    asyncio.run(scenario())


def test_second_segment_reports_starting_and_does_not_arrive_early():
    """The snap-to-anchor transition retains the old STARTING status boundary."""
    async def scenario():
        loop = asyncio.get_running_loop()
        first, second = loop.create_future(), loop.create_future()
        backend = _Backend([first, second])
        skill = _skill(backend, _Resolver(_poi(), True), _Clock())
        manager = _manager(skill, backend)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await backend.starts.get()
        backend.calls[0][1](2)
        await events.get()
        assert manager.status().state == TaskState.NAVIGATING
        first.set_result(NavOutcome(NavStatus.SUCCEEDED))
        await backend.starts.get()
        assert manager.status().state == TaskState.STARTING
        assert events.empty()
        backend.calls[0][1](99)
        assert events.empty()
        backend.calls[1][1](1)
        assert (await events.get()).remaining_distance == 1
        assert manager.status().state == TaskState.NAVIGATING
        second.set_result(NavOutcome(NavStatus.SUCCEEDED))
        assert (await events.get()).kind == EventKind.SUCCEEDED
        assert events.empty()
        await manager.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('late_status', [NavStatus.SUCCEEDED, NavStatus.FAILED])
def test_cancel_suppressing_backend_cannot_trigger_anchor_or_retry(late_status):
    """Explicit task cancellation fences continuation even with a faulty backend."""
    async def scenario():
        started = asyncio.Event()
        calls = []

        class SwallowsCancellation:
            async def navigate(self, goal, feedback):
                calls.append(goal)
                started.set()
                if len(calls) == 1:
                    try:
                        await asyncio.Event().wait()
                    except asyncio.CancelledError:
                        return NavOutcome(late_status)
                return NavOutcome(NavStatus.SUCCEEDED)

            async def cancel(self):
                await asyncio.sleep(0)

        backend = SwallowsCancellation()
        skill = _skill(backend, _Resolver(_poi(), True))
        manager = _manager(skill, backend)
        events = manager.subscribe_events()
        await manager.submit(_request())
        await started.wait()
        await manager.cancel()
        assert len(calls) == 1
        assert (await events.get()).status == 'paused'
        assert events.empty()
        await manager.close()

    asyncio.run(scenario())
