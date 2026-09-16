"""Consume actual prepared manager argv without constructing ROS or storage owners."""

from pathlib import Path

import pytest
import yaml

from nav_bringup.launching.graph import build_graph, GraphOptions
from nav_bringup.navcfg import render, schema
from nav_bringup.navrun.preparation import BundleInputs, prepare_bundle, write_bundle

from nav_executor.app.config import load_configuration


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('robot', ['david', 'example_diffdrive'])
@pytest.mark.parametrize('mode,websocket', [
    ('tour', True), ('precise', True), ('precise', False)])
def test_prepared_graph_binds_effective_robot_site_and_dock(tmp_path, robot, mode, websocket):
    """The executable's real parser receives complete robot/mode/site data, not defaults."""
    package = ROOT / (robot + '_config')
    offline = ROOT / 'example_diffdrive_config/sites/offline'
    site = render.load_yaml(offline / 'site.yaml')
    site['map']['nav_yaml'] = str(offline / site['map']['nav_yaml'])
    for key in ('named_poses_file', 'store_file'):
        site['pois'][key] = str(offline / site['pois'][key])
    site['pois']['dock_poi_ids'] = ['dock-test']
    site_path = tmp_path / 'site.yaml'
    site_path.write_text(yaml.safe_dump(site))
    overlay = render.load_yaml(package / 'modes' / (mode + '.yaml'))
    overlay.setdefault('profile', {}).setdefault('executor', {})['nav_retry_count'] = 3
    mode_path = tmp_path / 'mode.yaml'
    mode_path.write_text(yaml.safe_dump(overlay))
    profile_path = package / 'robot' / (robot + '.yaml')
    profile = render.deep_merge(render.load_yaml(profile_path), overlay['profile'])
    inputs = BundleInputs(
        profile_path, mode_path, ROOT / 'nav_bringup/config/nav2_base.yaml',
        ROOT / 'nav_bringup/config/nav2_bindings.yaml', ROOT / 'nav_plugins/behavior_trees',
        ROOT / 'nav_plugins/config/bt_bindings.yaml', site=site_path,
        urdf=package / 'urdf' / (robot + '.urdf'))
    run = tmp_path / 'run with spaces'
    bundle = prepare_bundle(GraphOptions(mode, str(run), start_executor=websocket), inputs,
                            schema.load_catalog(schema.find_source_schemas(ROOT)))
    run.mkdir()
    write_bundle(bundle)
    manager = next(spec for spec in build_graph(bundle.options).processes
                   if spec.package == 'nav_executor')
    resolved = load_configuration(manager.arguments, profile_path=tmp_path / 'nonexistent',
                                  environ={})
    config, tasks = resolved.config, resolved.task_config()
    assert config.websocket_enabled == websocket and config.nav_retry_count == 3
    assert config.action_name == bundle.options.ready_action
    assert tasks.map_frame == profile['frames']['map']
    assert tasks.robot_base_frame == profile['frames']['poi_capture']
    assert tasks.odom_frame == profile['frames']['odom']
    assert config.pose_file == (site['pois']['named_poses_file'] if websocket else None)
    if websocket:
        assert config.poi_store_file == site['pois']['store_file']
        assert config.server_uri == site['agent_server']['uri']
    assert config.dock_poi_ids == (('dock-test',) if websocket and mode == 'precise' else ())
    if mode == 'tour':
        assert tasks.dock is None
    else:
        assert tasks.dock.robot_base_frame == profile['frames']['nav_base']
        assert tasks.dock.robot_base_frame != tasks.robot_base_frame
        assert tasks.dock.footprint == tuple(tuple(point) for point in profile['footprint'])
        assert tasks.dock.yaw_ok == profile['goal']['yaw_tolerance']
        assert tasks.dock.max_yaw_retries == 0
        assert tasks.dock.settle_max_rate_rad_s == 0.05
