"""Exercise the evidence oracle against real domain skills with memory-only motion ports."""

import asyncio
import logging

import pytest

from nav_executor.protocol.task_adapter import event_to_frame
from nav_sim.selftest_evidence import check_evidence, ExpectedCase
from nav_tasks.api import CommandRegistry, EventKind, TaskManager, TaskRequest
from nav_tasks.core.handlers.dock import Dock, DockAwareNavigation
from nav_tasks.core.handlers.navigate_to import NavigateToPoi
from nav_tasks.domain.footprint import FootprintCheck, OccupancyGridView
from nav_tasks.domain.tasks import NavOutcome, NavStatus, ResolvedGoal
from nav_tasks.domain.types import PoseRecord, StoredPoi
from nav_tasks.skills.docking.approach import check_approach_footprint
from nav_tasks.skills.docking.finish import DockTolerance
from nav_tasks.skills.docking.ports import DockActionResult
from nav_tasks.skills.docking.settle import SettleSpec
from nav_tasks.skills.docking.workflow import DockConfig, DockWorkflow


@pytest.mark.parametrize('blocked', [False, True])
def test_real_manager_poi_routing_and_dock_workflow_produce_required_evidence(blocked):
    """Actual domain events, not hand-written success rows, satisfy or reject the oracle."""
    async def scenario():
        cases = (ExpectedCase('private-poi', 7, 'ordinary', 'poi'),
                 ExpectedCase('private-dock', 8, 'precise', 'dock', (2.0, 0.0, 0.0)))
        nav, ws, calls = [], [], []
        clock = [0.0]
        pose = [0.0, 0.0, 0.0]

        def observe(source, name, data):
            nav.append(dict(event_id='a' * 32 + ':' + str(len(nav) + 1),
                            source=source, event_type=name, data=dict(data),
                            stamp=1.0, severity='info'))

        def sleep(duration):
            clock[0] += duration

        class Repository:
            def get(self, identity):
                point = PoseRecord('world', 2.0 if identity == 'precise' else 1.0, 0, 0, 0, 1)
                return StoredPoi(identity, identity, point, point, 'selftest')

        class Resolver:
            def resolve(self, point):
                return ResolvedGoal(point, False)

        class Motion:
            def navigate(self, point, label, timeout, extra):
                calls.append(('dock', point))
                pose[:] = point
                return DockActionResult(True)

            def spin(self, *args):
                pytest.fail('the zero-spin selftest should not need a corrective rotation')

        class Backend:
            async def cancel(self):
                pass

            async def navigate(self, goal, feedback):
                calls.append(('poi', goal.poi.poi_id))
                feedback(0.0)
                return NavOutcome(NavStatus.SUCCEEDED)

            async def dock(self, goal, feedback):
                config = DockConfig(0.5, DockTolerance(0.1, 0.4, 0),
                                    SettleSpec(max_rate_rad_s=0.05), 120, 30, 8)
                grid = OccupancyGridView('world', 100, 100, 0.1, -5, -5, 0, (0,) * 10000)
                footprint = ((-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2))

                def approach(point):
                    if blocked:
                        return FootprintCheck(False, 'occupied_static_cell', (65, 50, 100))
                    return check_approach_footprint(grid, 'world', footprint, point, 50)

                def dock_event(name, data):
                    observe('dock', 'dock_' + name, dict(
                        data, task_id=goal.request_id, sub_id=goal.sub_id,
                        generation=goal.generation))

                flow = DockWorkflow(config, Motion(), lambda: tuple(pose), approach,
                                    logging.getLogger('selftest_fixture'),
                                    now_fn=lambda: clock[0], sleep_fn=sleep, on_event=dock_event)
                target = goal.pose
                result = flow.execute((target.x, target.y, target.yaw))
                return NavOutcome(NavStatus(result))

        backend, registry = Backend(), CommandRegistry()
        route = DockAwareNavigation(NavigateToPoi(backend, Repository(), Resolver()),
                                    Dock(backend), ('precise',))
        registry.register('navigate_to', route.decode, route.run)
        manager = TaskManager(registry, backend.cancel,
                              on_event=lambda name, data: observe('tasks', name, data))
        queue = manager.subscribe_events()
        try:
            for case in cases:
                await manager.submit(TaskRequest(case.task_id, 'navigate_to',
                                                 {'target_id': case.target_id},
                                                 sub_id=case.sub_id, source='websocket'))
                while True:
                    event = await asyncio.wait_for(queue.get(), 1)
                    ws.append(event_to_frame(event))
                    if event.kind in (EventKind.SUCCEEDED, EventKind.FAILED):
                        break
            result = check_evidence(cases, nav, ws)
            assert result.complete is not blocked, result.errors
            assert calls[0] == ('poi', 'ordinary')
            if blocked:
                assert len(calls) == 1 and ws[-1]['event_type'] == 'on_error'
            else:
                assert calls[1:] == [('dock', (1.5, 0.0, 0)), ('dock', (2.0, 0.0, 0))]
                assert clock[0] == pytest.approx(0.4)
        finally:
            await manager.close()

    asyncio.run(scenario())
