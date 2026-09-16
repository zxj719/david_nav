"""Pure Dock command, failure text, throttling and post-cancel verdict fences."""

import asyncio
from types import SimpleNamespace as NS

import pytest

from nav_tasks.core.handlers.dock import Dock, DockRequest
from nav_tasks.core.registry import TaskContext
from nav_tasks.domain.errors import ConfigError
from nav_tasks.domain.tasks import NavOutcome, NavStatus
from nav_tasks.domain.types import PoseRecord, StoredPoi


def _pose(x=1):
    return PoseRecord('world', x, 2, 0, 0.5, 0)


@pytest.mark.parametrize('pose', [None, {}, _pose(float('nan')), _pose(True)])
def test_direct_pose_is_validated_before_admission(pose):
    """Malformed coordinates cannot preempt valid motion or fall back to navigation."""
    skill = Dock(NS())
    with pytest.raises(ConfigError):
        skill.decode({'pose': pose})


def test_poi_failure_is_one_attempt_with_exact_legacy_diagnostics_and_audit():
    """Dock POI failure preserves the old outer action's ABORTED wire description."""
    async def scenario():
        clock = NS(value=10.0)
        clock.now = lambda: clock.value
        reports, failures, goals, callbacks = [], [], [], []

        class Backend:
            async def dock(self, goal, on_feedback):
                goals.append(goal)
                callbacks.append(on_feedback)
                on_feedback(3.456)
                clock.value = 10.2
                on_feedback(2.125)  # Throttled progress, but latest failure distance.
                clock.value = 15.0
                return NavOutcome(NavStatus.FAILED)

        context = TaskContext('request', 2, 7, reports.append)
        skill = Dock(Backend(), clock=clock, on_failure=lambda ctx, failure: failures.append(
            (ctx, failure)))
        poi = StoredPoi('dock', '入位', _pose(), _pose(), 'session')
        result = await skill.run(context, DockRequest(poi.map_pose, poi))
        assert result.detail == (
            '导航失败(共尝试1次): Nav2 终态 status=6(ABORTED), target=dock(入位) '
            'goal=world(x=1.00, y=2.00, yaw=0.50rad), 剩余距离=2.12m, 本次耗时=5.0s')
        assert len(goals) == len(failures) == 1
        assert (goals[0].request_id, goals[0].sub_id, goals[0].generation) == ('request', 2, 7)
        assert failures[0][0] is context and failures[0][1].total_attempts == 1
        assert reports == [3.456]
        callbacks[0](99)
        assert reports == [3.456]

    asyncio.run(scenario())


def test_canceled_backend_success_is_not_a_task_success():
    """The pure owner context independently fences a worker's late successful return."""
    async def scenario():
        canceled = [False]
        reports = []

        class Backend:
            async def dock(self, goal, on_feedback):
                canceled[0] = True
                on_feedback(99)
                return NavOutcome(NavStatus.SUCCEEDED)

        context = TaskContext('r', 0, 1, reports.append, lambda: canceled[0])
        skill = Dock(Backend())
        with pytest.raises(asyncio.CancelledError):
            await skill.run(context, skill.decode({'pose': _pose()}))
        assert not reports

    asyncio.run(scenario())


def test_cleanup_exception_is_not_repackaged_as_an_ordinary_failed_attempt():
    """Unsettled cleanup must retain manager ownership rather than reaching retry wrapping."""
    async def scenario():
        error = TimeoutError('cleanup still running')

        class Backend:
            async def dock(self, goal, on_feedback):
                raise error

        skill = Dock(Backend())
        poi = StoredPoi('p', 'name', _pose(), _pose(), 'session')
        with pytest.raises(TimeoutError) as caught:
            await skill.run(TaskContext('r', 0, 1, lambda value: None),
                            DockRequest(poi.map_pose, poi))
        assert caught.value is error

    asyncio.run(scenario())
