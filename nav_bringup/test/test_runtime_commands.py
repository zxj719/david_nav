"""Runtime binding is pure and consumes frozen robot/mode selections."""

from dataclasses import replace
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from nav_bringup.navcfg import schema
from nav_bringup.navrun.preflight import ProcessInfo
from nav_bringup.navrun.runtime_commands import bind_commands
from nav_bringup.navrun.up_planning import discover_inputs, plan_up, UpRequest


SOURCE = Path(__file__).resolve().parents[2]


def _plan(tmp_path, robot='example_diffdrive', site='offline', mode='tour', record='off',
          no_executor=True):
    request = UpRequest(mode, robot, site, tmp_path / 'run', record=record,
                        no_executor=no_executor)
    own = ProcessInfo(100, 1, 1.0, ('navrun',), 1000)
    inputs = discover_inputs(request, lambda package: SOURCE / package)
    if not inputs.mode.exists():
        overlay = {'chassis': {'adapter': {'input_topic': '/demo/safe'}}} if (
            mode == 'teleop') else {}
        mode_path = tmp_path / 'mode.yaml'
        mode_path.write_text(yaml.safe_dump({'schema_version': 1, 'mode': mode,
                                            'profile': overlay}), encoding='utf-8')
        inputs = replace(inputs, mode=mode_path)
    if robot == 'david' and inputs.site is not None:
        # Preserve the actual David profile/URDF while keeping tests independent
        # of the localization workspace and manually maintained POI files.
        site_data = yaml.safe_load(inputs.site.read_text())
        example = SOURCE / 'example_diffdrive_config/sites/offline'
        site_data['map']['nav_yaml'] = str(example / 'maps/room.yaml')
        site_data['pois']['named_poses_file'] = str(example / 'pois/named_poses.json')
        site_path = tmp_path / 'site.yaml'
        site_path.write_text(yaml.safe_dump(site_data), encoding='utf-8')
        inputs = replace(inputs, site=site_path)
    return plan_up(request, inputs, schema.load_catalog(schema.find_source_schemas(SOURCE)),
                   (own,), own.pid, lambda package, name: '/fixture/' + name)


def _bind(plan, **kwargs):
    return bind_commands(plan, resolve=lambda package, name: '/fixture/' + name,
                         which=lambda name: '/fixture/' + name, python='/fixture/python', **kwargs)


@pytest.mark.parametrize('mode', ['tour', 'precise', 'bench_slamgt'])
def test_navigation_uses_installed_check_cli_and_independent_dump(tmp_path, mode, monkeypatch):
    """Binding cannot initialize ROS or execute any process, even for strict checks."""
    plan = _plan(tmp_path, mode=mode)

    def forbidden(*args, **kwargs):
        raise AssertionError('binding must not execute')

    monkeypatch.setattr(subprocess, 'Popen', forbidden)
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    selected = _bind(plan, strict=True)
    assert selected.launch[:4] == ('/fixture/ros2', 'launch', 'nav_bringup', 'nav.launch.py')
    assert len(selected.prelaunch_gates) == 1
    localization = selected.prelaunch_gates[0]
    assert localization.name == 'localization' and localization.timeout_sec is None
    assert localization.argv[:3] == ('/fixture/navcfg', 'check', '--live')
    assert '--wait-localization-only' in localization.argv
    assert [gate.name for gate in selected.gates] == ['ready', 'live_check']
    assert selected.gates[0].argv[0] == '/fixture/nav_ready_exec'
    assert selected.gates[1].argv[:3] == ('/fixture/navcfg', 'check', '--live')
    assert selected.gates[1].argv[-1] == '--strict'
    assert selected.gates[1].timeout_sec is None
    assert '--dump-params-on-ready' not in selected.monitor
    assert selected.parameter_dump[:2] == ('/fixture/navrun', 'dump-params')
    assert selected.zero[-2:] == ('--topic', '/demo/command')
    assert '--map-frame' in selected.monitor and 'demo_world' in selected.monitor
    assert selected.record == ()
    assert selected.launch[-1] == 'phase:=core'
    assert selected.executor_launch == ()
    assert selected.event_flush == (
        '/fixture/python', '-m', 'nav_bringup.recorder.event_flush')


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_executor_is_separate_launch_using_identical_frozen_bundle(tmp_path, mode):
    """WS stays launch-managed but is absent from the initial core command."""
    plan = _plan(tmp_path, robot='david', site='daolan', mode=mode, no_executor=False)
    selected = _bind(plan, strict=True)
    assert plan.bundle.options.start_executor
    assert selected.launch[-1] == 'phase:=core'
    assert selected.executor_launch == selected.launch[:-1] + ('phase:=executor',)
    assert selected.gates[-1].argv[-1] == '--strict'


def test_chassis_skips_nav2_and_localization_gates(tmp_path):
    """Chassis recording needs neither Nav2 readiness nor localization availability."""
    selected = _bind(_plan(tmp_path, mode='chassis'))
    assert not selected.gates
    assert not selected.prelaunch_gates
    assert '--no-bt' in selected.monitor
    assert selected.parameter_dump


def test_david_no_executor_still_waits_the_profile_120_seconds(tmp_path):
    """Suppressing WS cannot accidentally restore the generic ninety-second timeout."""
    plan = _plan(tmp_path, robot='david', site='daolan', mode='precise', record='full')
    assert plan.bundle.options.ready_timeout_sec == 120.0
    selected = _bind(plan)
    assert selected.gates[0].argv[1:3] == ('--timeout', '120')
    assert selected.gates[0].timeout_sec == 125.0
    assert selected.record[0] == '/fixture/ros2'
    assert selected.record[1:] == plan.recording.argv[1:]
    assert selected.zero[-1] == '/cmd_vel'


def test_unresolved_ros2_is_rejected_before_launch(tmp_path):
    """Missing CLI dependencies fail while the operation is still read-only."""
    with pytest.raises(LookupError):
        bind_commands(_plan(tmp_path), resolve=lambda *args: '/fixture/tool',
                      which=lambda name: None)


def test_teleop_binds_a_direct_managed_keyboard_and_no_navigation_gates(tmp_path):
    """The inherited capabilities will go directly to keyboard, never through launch."""
    plan = _plan(tmp_path, mode='teleop')
    selected = _bind(plan)
    assert selected.keyboard[0] == '/fixture/chassis_teleop_managed'
    assert selected.keyboard[-1].endswith('/params/rendered/teleop.yaml')
    assert selected.tty_device == '/dev/tty'
    assert selected.gates == ()
    assert selected.prelaunch_gates == ()
    assert plan.graph.supervisor_keyboard
