"""Additive Dock event hooks preserve workflow behavior and worker ownership."""

import asyncio
import threading

import pytest

from nav_tasks.domain.footprint import FootprintCheck
from nav_tasks.domain.tasks import DockGoal, NavStatus
from nav_tasks.domain.types import PoseRecord
from nav_tasks.skills.docking.workflow import DockWorkflow
from test_dock_workflow import _Clock, _config, _Log, _Motion, _workflow
from test_dock_worker import _worker


@pytest.mark.parametrize('broken', [False, True])
def test_additive_events_do_not_change_logs_actions_or_measured_verdict(broken):
    """Even a failing observation sink leaves every existing log line unchanged."""
    pose = (2.0123456789, 3, 0.00123456789)
    reference, original_motion, original_log, _ = _workflow(pose=pose)
    expected = reference.execute((2, 3, 0))
    clock, log, motion, events = _Clock(), _Log(), _Motion(), []

    def observe(name, data):
        if broken:
            raise ValueError('observer unavailable')
        events.append((name, dict(data)))
        data['exy'] = 99  # The sink does not own the algorithm's measured values.

    workflow = DockWorkflow(
        _config(), motion, lambda: pose, lambda goal: FootprintCheck(True, 'clear'), log,
        now_fn=clock.now, sleep_fn=clock.sleep, on_event=observe)
    assert workflow.execute((2, 3, 0)) == expected
    assert motion.calls == original_motion.calls and log.lines == original_log.lines
    if not broken:
        assert [name for name, data in events] == [
            'started', 'approach_checked', 'stage_started', 'stage_started', 'measured']
        measured = events[-1][1]
        assert measured['exy'] == pytest.approx(0.0123456789)
        assert measured['eyaw'] == pytest.approx(0.00123456789)
        assert measured['settled'] is True


def test_failed_footprint_is_observed_without_any_stage_event():
    """Static-map refusal stays before motion and never turns into an arrival observation."""
    motion, events = _Motion(), []
    workflow = DockWorkflow(
        _config(), motion, lambda: (2, 3, 0),
        lambda goal: FootprintCheck(False, 'occupied_static_cell', (2, 3, 100)), _Log(),
        on_event=lambda name, data: events.append((name, data)))
    assert workflow.execute((2, 3, 0)) == 'failed'
    assert motion.calls == []
    assert [name for name, data in events] == ['started', 'approach_checked']
    assert events[-1][1]['clear'] is False
    assert events[-1][1]['occupied_cell'] == (2, 3, 100)


@pytest.mark.parametrize('cancel_second', [False, True])
def test_worker_observations_are_correlated_and_fenced_after_cancel_or_replacement(
        monkeypatch, cancel_second):
    """Existing worker threads publish their own identity; stale callbacks cannot leak."""
    async def scenario():
        callbacks, observed = [], []
        entered, release, canceled = threading.Event(), threading.Event(), asyncio.Event()

        class Backend:
            async def cancel(self):
                canceled.set()

        class Workflow:
            def __init__(self, *args, on_event, **kwargs):
                self.callback = on_event
                callbacks.append(on_event)

            def execute(self, goal):
                if len(callbacks) == 2:
                    entered.set()
                    assert release.wait(2), 'test failed to release its private worker'
                self.callback('measured', {'settled': True})
                return 'succeeded'

        monkeypatch.setattr('nav_tasks.adapters.ros.dock_worker.DockWorkflow', Workflow)
        loop_thread = threading.get_ident()
        worker = _worker(Backend(), on_event=lambda goal, name, data: observed.append(
            (goal, name, data, threading.get_ident())))
        pose = PoseRecord('world', 1, 2, 0, 0, 1)
        first, second = DockGoal(pose, 'same', 1, 1), DockGoal(pose, 'same', 1, 2)
        assert (await worker.dock(first, lambda value: None)).succeeded
        assert observed[0][0] == first and observed[0][3] != loop_thread
        callbacks[0]('late', {})
        assert len(observed) == 1
        operation = asyncio.create_task(worker.dock(second, lambda value: None))
        try:
            assert await asyncio.to_thread(entered.wait, 1)
            callbacks[0]('stale_after_replacement', {})
            assert len(observed) == 1
            cleanup = None
            if cancel_second:
                cleanup = asyncio.create_task(worker.cancel())
                await canceled.wait()
            release.set()
            result = await asyncio.wait_for(operation, 2)
            if cleanup is not None:
                await cleanup
            assert result.status == (NavStatus.CANCELED if cancel_second else NavStatus.SUCCEEDED)
            assert len(observed) == (1 if cancel_second else 2)
            if not cancel_second:
                assert observed[-1][0] == second
        finally:
            release.set()
            await worker.close()
        before = list(observed)
        callbacks[-1]('after_close', {})
        assert observed == before

    asyncio.run(scenario())
