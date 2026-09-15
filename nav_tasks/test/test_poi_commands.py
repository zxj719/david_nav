"""Administrative POI handlers with in-memory repositories and pose sources."""

import asyncio
from types import SimpleNamespace

import pytest

from nav_tasks.core.handlers.poi_commands import PoiCommands
from nav_tasks.domain.types import CapturedPose, PoseRecord


def _capture(stamp=10):
    pose = PoseRecord('world', 1, 2, 0, 0.5, stamp)
    return CapturedPose(pose, pose, 'session')


@pytest.mark.parametrize('now,accepted', [(10.49, True), (10.5, True), (10.51, False), (9, True)])
def test_mark_freshness_boundary_and_future_stamp_preserved(now, accepted):
    """Retain strict greater-than freshness and existing future-stamp acceptance."""
    async def scenario():
        writes = []

        async def capture():
            return _capture()

        def upsert(poi):
            writes.append(poi)
            return poi

        commands = PoiCommands(
            SimpleNamespace(capture_for_poi=capture), SimpleNamespace(upsert=upsert), lambda: now)
        result = await commands.mark('POI_NEW', '大厅')
        assert len(writes) == int(accepted)
        if accepted:
            assert result.error is None
            assert (result.poi.poi_id, result.poi.name) == ('POI_NEW', '大厅')
            assert result.poi.map_pose == _capture().map_pose
            assert result.poi.slam_session_id == 'session'
        else:
            assert result.error == '当前位置位姿数据过期'
            assert result.poi is None

    asyncio.run(scenario())


def test_capture_error_becomes_readable_failure_but_cancellation_propagates():
    """Capture errors are protocol-visible; cancellation must not become success."""
    async def scenario():
        async def unavailable():
            raise RuntimeError('TF unavailable')

        commands = PoiCommands(SimpleNamespace(capture_for_poi=unavailable), None, lambda: 10)
        assert (await commands.mark('poi', 'name')).error == 'TF unavailable'

        async def canceled():
            raise asyncio.CancelledError

        commands = PoiCommands(SimpleNamespace(capture_for_poi=canceled), None, lambda: 10)
        with pytest.raises(asyncio.CancelledError):
            await commands.mark('poi', 'name')

    asyncio.run(scenario())


def test_persistence_failure_not_reported_as_success():
    """A successful capture does not hide a failed atomic storage operation."""
    async def scenario():
        async def capture():
            return _capture()

        def fail(poi):
            raise OSError('disk full')

        commands = PoiCommands(
            SimpleNamespace(capture_for_poi=capture), SimpleNamespace(upsert=fail), lambda: 10)
        with pytest.raises(OSError, match='disk full'):
            await commands.mark('poi', 'name')

    asyncio.run(scenario())


def test_directory_delegates_ordered_names_without_capture_or_motion():
    """Directory updates require neither a pose source nor a navigation backend."""
    updates = []
    repository = SimpleNamespace(rename_many=lambda pairs: updates.extend(pairs))
    commands = PoiCommands(None, repository, lambda: 10)
    commands.update_directory(iter([('poi', '旧名'), ('poi', '新名')]))
    assert updates == [('poi', '旧名'), ('poi', '新名')]
