"""Read-only startup planning through real configuration and injected host views."""

from dataclasses import replace
import json
from pathlib import Path

import pytest

from nav_bringup.navcfg import schema
from nav_bringup.navrun.preflight import ProcessInfo
from nav_bringup.navrun import up_planning


SOURCE = Path(__file__).resolve().parents[2]
SELF = ProcessInfo(100, 1, 1.0, ('navrun',), 1000)


def _request(tmp_path, **changes):
    return replace(up_planning.UpRequest(
        mode='tour', robot='example_diffdrive', site='offline',
        run_dir=tmp_path / 'not-created', no_executor=True), **changes)


def _plan(request, processes=(SELF,), resolve=lambda package, name: '/fake/' + name):
    inputs = up_planning.discover_inputs(request, lambda package: SOURCE / package)
    catalog = schema.load_catalog(schema.find_source_schemas(SOURCE))
    return up_planning.plan_up(request, inputs, catalog, processes, SELF.pid, resolve)


@pytest.mark.parametrize('mode', ['tour', 'precise', 'chassis', 'teleop'])
def test_plan_real_inputs_without_creating_run(tmp_path, mode):
    """Offline profile rendering is reachable without any process or file mutation."""
    request = _request(tmp_path, mode=mode)
    plan = _plan(request)
    report = up_planning.report_plan(plan)
    assert report['robot'] == request.robot
    assert report['mode'] == mode
    assert report['errors'] == []
    assert report['launch'][0:4] == ['ros2', 'launch', 'nav_bringup', 'nav.launch.py']
    assert not request.run_dir.exists()
    assert report['processes'][0]['executable'] == 'fake_chassis'
    json.dumps(report, allow_nan=False)


def test_external_chassis_is_not_owned_or_resolved(tmp_path):
    """Auto preserves an existing driver and omits its executable requirement."""
    driver = ProcessInfo(101, 1, 2.0, ('fake_chassis',), 1000)
    plan = _plan(_request(tmp_path), (SELF, driver))
    assert not plan.bundle.options.start_chassis
    assert all(process.name != 'wheeltec_robot' for process in plan.graph.processes)
    assert not plan.preflight.targets
    assert 101 in plan.preflight.protected_pids


def test_teleop_refuses_legacy_navigation_instead_of_preflight_killing_it(tmp_path):
    """A legacy stack without the new lease cannot be silently cleared to start teleop."""
    old_nav = ProcessInfo(102, 1, 2.0, ('/opt/ros/lib/nav2_controller/controller_server',), 1000)
    plan = _plan(_request(tmp_path, mode='teleop', robot='david'), (SELF, old_nav))
    assert plan.preflight.targets
    assert any('teleop refuses' in error for error in plan.errors)


def test_missing_executables_are_aggregated_before_startup(tmp_path):
    """Every selected entry point is checked even after an earlier missing dependency."""
    checked = []

    def resolve(package, name):
        checked.append((package, name))
        raise LookupError('not installed')

    plan = _plan(_request(tmp_path), resolve=resolve)
    assert len(checked) == len(plan.graph.processes)
    assert len(plan.errors) == len(checked)
    assert any('chassis_adapter' in error for error in plan.errors)
    assert not (tmp_path / 'not-created').exists()


def test_executor_binding_uses_site_and_resolves_readiness_gate(tmp_path):
    """A complete argv is frozen in the same bundle consumed by the launch adapter."""
    plan = _plan(_request(tmp_path, no_executor=False))
    assert plan.errors == ()
    assert plan.bundle.options.start_executor
    assert '--pose-file' in plan.bundle.options.executor_arguments
    assert 'ws://127.0.0.1:18101' in plan.bundle.options.executor_arguments
    assert plan.bundle.options.ready_timeout_sec == 120.0
    assert 'ready_gate' in dict(plan.executables)
    document = json.loads(dict(plan.bundle.artifacts)['launch.json'])
    assert document['options']['executor_arguments'] == list(
        plan.bundle.options.executor_arguments)


def test_report_does_not_disclose_other_process_arguments(tmp_path):
    """A diagnostic report contains cleanup identities, never unrelated command lines."""
    process = ProcessInfo(102, 1, 3.0, ('unrelated', '--token', 'DO_NOT_PRINT'), 1000)
    report = up_planning.report_plan(_plan(_request(tmp_path), (SELF, process)))
    assert 'DO_NOT_PRINT' not in json.dumps(report)


@pytest.mark.parametrize('field,value', [
    ('robot', '../robot'), ('site', '/tmp/site'), ('mode', 'unknown'),
    ('record', 'mcap'), ('chassis', 'yes'), ('no_executor', 'true'),
    ('run_dir', Path('relative')), ('robot', ''), ('site', '..'),
])
def test_invalid_selectors_fail_before_resource_lookup(tmp_path, field, value):
    """Names cannot traverse resource directories or disguise unresolved switches."""
    request = _request(tmp_path, **{field: value})
    with pytest.raises(ValueError):
        up_planning.discover_inputs(request, lambda _: pytest.fail('must validate first'))


def test_chassis_does_not_require_site_or_nav_assets(tmp_path):
    """Site-free tools resolve only their robot profile package."""
    requested = []

    def share(package):
        requested.append(package)
        return SOURCE / package

    inputs = up_planning.discover_inputs(_request(tmp_path, mode='chassis'), share)
    assert requested == ['example_diffdrive_config']
    assert inputs.site is None
    assert inputs.urdf is None


def test_robot_identity_mismatch_fails(tmp_path):
    """Selecting one package cannot silently render a different robot's profile."""
    request = _request(tmp_path, robot='other')
    inputs = up_planning.discover_inputs(
        _request(tmp_path), lambda package: SOURCE / package)
    catalog = schema.load_catalog(schema.find_source_schemas(SOURCE))
    with pytest.raises(ValueError, match='identity'):
        up_planning.plan_up(request, inputs, catalog, (SELF,), SELF.pid, lambda *_: '/fake')
