"""The --OA selector arms front obstacle avoidance without changing a run that omits it."""

from pathlib import Path

import pytest
import yaml

from nav_bringup.navcfg import render, schema
from nav_bringup.navrun.cli import build_parser
from nav_bringup.navrun.preflight import ProcessInfo
from nav_bringup.navrun.runtime_commands import bind_commands
from nav_bringup.navrun.up_planning import discover_inputs, plan_up, UpRequest

SOURCE = Path(__file__).resolve().parents[2]
OWN = ProcessInfo(100, 1, 1.0, ('navrun',), 1000)
OVERLAY_ARTIFACT = 'params/inputs/overlay.yaml'
PLUGINS = 'local_costmap/local_costmap/ros__parameters/plugins'


def _request(tmp_path, oa, mode='tour'):
    return UpRequest(mode, 'example_diffdrive', 'offline', tmp_path / 'run',
                     record='off', no_executor=True, oa=oa)


def _plan(tmp_path, oa, mode='tour'):
    request = _request(tmp_path, oa, mode)
    inputs = discover_inputs(request, lambda package: SOURCE / package)
    return plan_up(request, inputs, schema.load_catalog(schema.find_source_schemas(SOURCE)),
                   (OWN,), OWN.pid, lambda package, name: '/fixture/' + name)


def _bind(plan):
    return bind_commands(plan, resolve=lambda package, name: '/fixture/' + name,
                         which=lambda name: '/fixture/' + name, python='/fixture/python')


def _parse(*arguments):
    return build_parser().parse_args(['up', '--mode', 'tour', *arguments])


def test_the_flag_stays_off_unless_it_is_asked_for():
    """Every existing entry point keeps its behavior because the default is off."""
    assert _parse().oa is False
    assert _parse('--OA').oa is True
    assert _parse('--oa').oa is True


@pytest.mark.parametrize('mode', ['chassis', 'teleop'])
def test_modes_without_a_costmap_reject_the_flag(tmp_path, mode):
    """Asking for obstacle avoidance where no costmap exists is a mistake, not a no-op."""
    with pytest.raises(ValueError):
        discover_inputs(_request(tmp_path, True, mode), lambda package: SOURCE / package)


def test_the_overlay_file_is_only_looked_up_when_the_flag_is_given(tmp_path):
    """Without the flag the run never reads the overlay, so it cannot be affected by it."""
    def share(package):
        return SOURCE / package

    assert discover_inputs(_request(tmp_path, False), share).overlay is None
    assert discover_inputs(_request(tmp_path, True), share).overlay.name == 'oa.yaml'


def test_the_run_freezes_the_overlay_it_actually_used(tmp_path):
    """A run record that omits the overlay could not explain the parameters it ran with."""
    artifacts = dict(_plan(tmp_path, True).bundle.artifacts)

    assert yaml.safe_load(artifacts[OVERLAY_ARTIFACT])['mode'] == 'oa'
    assert OVERLAY_ARTIFACT not in dict(_plan(tmp_path, False).bundle.artifacts)


def test_the_flag_uses_the_direct_cloud_in_the_only_obstacle_pipeline(tmp_path):
    """The standard obstacle layer consumes PointCloud2 without another process."""
    plan = _plan(tmp_path, True)
    rendered = yaml.safe_load(dict(plan.bundle.artifacts)['params/rendered/nav2.yaml'])

    assert 'obstacle_layer' in render.get_path(rendered, PLUGINS)


def test_a_run_without_the_flag_renders_what_it_renders_today(tmp_path):
    """The flag is additive: dropping it must leave the prepared run untouched."""
    plan = _plan(tmp_path, False)
    rendered = yaml.safe_load(dict(plan.bundle.artifacts)['params/rendered/nav2.yaml'])

    assert 'obstacle_layer' not in render.get_path(rendered, PLUGINS)


def test_oa_does_not_change_the_monitor_or_graph_option_schema(tmp_path):
    """OA is a parameter overlay, not a second monitoring or process-graph mode."""
    enabled = _plan(tmp_path, True)
    disabled = _plan(tmp_path, False)

    assert _bind(enabled).monitor == _bind(disabled).monitor
    assert 'oa' not in enabled.bundle.options.__dataclass_fields__
