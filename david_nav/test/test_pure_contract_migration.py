"""Legacy import paths resolve to the owning pure implementation, not duplicate classes."""

import importlib

import pytest


@pytest.mark.parametrize('old,new,names', [
    ('navigation_types', 'nav_tasks.domain.types',
     ('PoseRecord', 'StoredPoi', 'Transform2D', 'CapturedPose')),
    ('grid_map_view', 'nav_tasks.domain.grid', ('GridMapView',)),
    ('dock_settle', 'nav_tasks.skills.docking.settle',
     ('SettleSpec', 'is_settled', 'wait_until_settled')),
    ('dock_finish', 'nav_tasks.skills.docking.finish',
     ('DockTolerance', 'DockOutcome', 'finish_docking')),
    ('navigation_protocol', 'nav_executor.protocol.messages',
     ('CommandParseError', 'NavigateToCommand', 'MarkCurrentPoiCommand',
      'AbortNavigationCommand', 'UpdatePoiListCommand', 'Command')),
    ('navigation_protocol', 'nav_executor.protocol.codec',
     ('parse_command', 'build_arrived_event', 'build_progress_event', 'build_error_event',
      'build_mark_poi_ack', 'build_mark_poi_error', 'build_mark_poi_success_event')),
])
def test_old_and_new_imports_have_identical_objects(old, new, names):
    """Existing callers and isinstance checks continue using the exact same definitions."""
    legacy, owner = importlib.import_module('david_nav.' + old), importlib.import_module(new)
    for name in names:
        assert getattr(legacy, name) is getattr(owner, name)
