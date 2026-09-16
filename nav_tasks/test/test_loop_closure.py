"""Loop-closure polling policy through real storage and scripted localization."""

import asyncio
from types import SimpleNamespace as NS

import pytest

from nav_tasks.app.loop_closure import watch_loop_closure
from nav_tasks.domain.types import PoseRecord, StoredPoi, Transform2D
from nav_tasks.nav_site_data.yaml_poi_repository import YamlPoiRepository


def test_missing_tf_and_unchanged_transform_skip_reprojection(tmp_path):
    """Valid changes reach the real repository, while polling never commands motion."""
    async def scenario():
        store = YamlPoiRepository(tmp_path / 'store.yaml', map_frame='world')
        pose = PoseRecord('local', 1, 2, 3, 0, 10)
        store.upsert(StoredPoi('p', 'target', pose, pose, 's'))
        transforms = [None, Transform2D(2, 0, 0), Transform2D(2, 0, 0), Transform2D(3, 0, 0)]
        tick = -1
        calls = []
        intervals = []

        async def sleep(seconds):
            nonlocal tick
            intervals.append(seconds)
            tick += 1
            if tick == len(transforms):
                raise asyncio.CancelledError

        def read():
            if transforms[tick] is None:
                raise LookupError('not ready')
            return transforms[tick]

        def reproject(*args):
            calls.append(args)
            return store.reproject_session(*args)

        with pytest.raises(asyncio.CancelledError):
            await watch_loop_closure(
                NS(current_map_from_odom=read, current_slam_session_id=lambda: 's'),
                NS(reproject_session=reproject), 1.0, sleep=sleep)
        assert calls == [('s', transforms[1], 0.05, 0.03), ('s', transforms[3], 0.05, 0.03)]
        assert intervals == [1.0] * 5
        assert store.get('p').map_pose == PoseRecord('world', 4, 2, 3, 0, 10)

    asyncio.run(scenario())


@pytest.mark.parametrize('failure', ['repository', 'session'])
def test_non_tf_errors_escape_to_runtime_supervisor(failure):
    """Persistence and session failures are not swallowed as missing TF."""
    async def scenario():
        async def sleep(seconds):
            pass

        def fail(*args):
            raise OSError('injected')

        poses = NS(current_map_from_odom=lambda: Transform2D(0, 0, 0),
                   current_slam_session_id=fail if failure == 'session' else lambda: 's')
        with pytest.raises(OSError, match='injected'):
            await watch_loop_closure(poses, NS(reproject_session=fail), 1, sleep=sleep)

    asyncio.run(scenario())


def test_session_only_change_retains_legacy_skip_behavior():
    """The watcher does not introduce an unrequested new session/reset policy."""
    async def scenario():
        tick = -1
        calls = []

        async def sleep(seconds):
            nonlocal tick
            tick += 1
            if tick == 2:
                raise asyncio.CancelledError

        poses = NS(current_map_from_odom=lambda: Transform2D(0, 0, 0),
                   current_slam_session_id=lambda: f's{tick}')
        with pytest.raises(asyncio.CancelledError):
            await watch_loop_closure(poses, NS(reproject_session=lambda *args: calls.append(args)),
                                     1, sleep=sleep)
        assert len(calls) == 1
        assert calls[0][0] == 's0'

    asyncio.run(scenario())
