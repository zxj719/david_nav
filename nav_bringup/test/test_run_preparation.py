"""Prepare run-local launch inputs without ROS, hardware or host mutations."""

from dataclasses import replace
import json
import os
from pathlib import Path

import pytest
import yaml

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import GraphOptions
from nav_bringup.navcfg import render, schema
from nav_bringup.navrun.preparation import BundleInputs, prepare_bundle, write_bundle


SOURCE = Path(__file__).resolve().parents[2]
CONFIG = SOURCE / 'nav_bringup' / 'config'
EXAMPLE = SOURCE / 'example_diffdrive_config'
PLUGINS = SOURCE / 'nav_plugins'


def _inputs(tmp_path: Path, mode: str = 'tour') -> BundleInputs:
    profile = render.load_yaml(EXAMPLE / 'robot/example_diffdrive.yaml')
    profile['chassis']['driver']['executable'] = 'fake_chassis'
    if mode == 'teleop':
        overlay = {'chassis': {'adapter': {'input_topic': profile['teleop']['cmd_topic']}}}
    else:
        overlay = {}
    mode_data = {'schema_version': 1, 'mode': mode, 'profile': overlay}
    if mode == 'precise':
        mode_data = render.load_yaml(EXAMPLE / 'modes/precise.yaml')
    profile_path = tmp_path / 'robot.yaml'
    mode_path = tmp_path / 'mode.yaml'
    profile_path.write_text(yaml.safe_dump(profile))
    mode_path.write_text(yaml.safe_dump(mode_data))
    return BundleInputs(
        profile_path, mode_path, CONFIG / 'nav2_base.yaml', CONFIG / 'nav2_bindings.yaml',
        PLUGINS / 'behavior_trees', PLUGINS / 'config/bt_bindings.yaml',
        site=EXAMPLE / 'sites/offline/site.yaml', urdf=EXAMPLE / 'urdf/example_diffdrive.urdf')


def _prepare(tmp_path: Path, mode: str = 'tour', **switches):
    inputs = _inputs(tmp_path, mode)
    options = GraphOptions(mode, str(tmp_path / 'run'), **switches)
    catalog = schema.load_catalog(schema.find_source_schemas(SOURCE))
    return prepare_bundle(options, inputs, catalog)


@pytest.mark.parametrize('limit', ['max_linear_x', 'max_angular_z'])
def test_teleop_limits_checked_without_navigation_inputs(tmp_path, limit):
    """Site-free teleop still enforces keyboard ceilings against the adapter hard limits."""
    inputs = _inputs(tmp_path, 'teleop')
    profile = yaml.safe_load(inputs.profile.read_text())
    profile['teleop'][limit] = 9.0
    inputs.profile.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='teleop'):
        prepare_bundle(GraphOptions('teleop', str(tmp_path / 'run')),
                       replace(inputs, site=None, urdf=None),
                       schema.load_catalog(schema.find_source_schemas(SOURCE)))


@pytest.mark.parametrize('mode', ['tour', 'precise', 'bench_slamgt', 'chassis', 'teleop'])
def test_prepare_is_read_only_and_written_bundle_roundtrips(tmp_path: Path, mode: str) -> None:
    """All five mode bundles satisfy the installed launch reader's contract."""
    prepared = _prepare(tmp_path, mode)
    assert not (tmp_path / 'run').exists()
    assert prepared.robot == 'example_diffdrive'
    assert prepared.site == 'offline'
    assert prepared.options.driver_executable == 'fake_chassis'
    assert not [issue for issue in prepared.issues if issue.severity == 'error']
    (tmp_path / 'run').mkdir()
    write_bundle(prepared)
    assert load_options(str(tmp_path / 'run'), {}) == prepared.options
    assert (tmp_path / 'run/params/inputs/robot.yaml').read_bytes() == (
        tmp_path / 'robot.yaml').read_bytes()


def test_nav2_and_adapter_topics_are_bound_to_the_same_profile(tmp_path: Path) -> None:
    """No David topic/frame or hardware default leaks into another robot's bundle."""
    prepared = _prepare(tmp_path, 'precise', scan_bridge=True)
    artifacts = dict(prepared.artifacts)
    adapter = yaml.safe_load(artifacts['params/rendered/adapter.yaml'])
    params = adapter['chassis_adapter']['ros__parameters']
    assert params['input_cmd_vel_topic'] == '/demo/smoothed'
    assert params['output_cmd_vel_topic'] == '/demo/command'
    assert params['cmd_vel_timeout'] == 1.0
    assert prepared.options.smooth_command_topic == '/demo/smoothed'
    dock = yaml.safe_load(artifacts['params/rendered/dock.yaml'])
    dock_params = dock['dock_navigator']['ros__parameters']
    assert dock_params['robot_base_frame'] == 'demo_axle'
    assert dock_params['global_frame'] == 'demo_world'
    assert dock_params['static_map_topic'] == '/demo/map'
    assert yaml.safe_load(dock_params['footprint']) == (
        render.load_yaml(tmp_path / 'robot.yaml')['footprint'])
    assert dock_params['settle_max_rate_rad_s'] == 0.05
    scan = yaml.safe_load(artifacts['params/rendered/scan_bridge.yaml'])
    assert scan['pointcloud_to_laserscan']['ros__parameters']['target_frame'] == 'demo_axle'
    map_params = yaml.safe_load(artifacts['params/rendered/map_server.yaml'])
    assert map_params['map_server']['ros__parameters']['topic_name'] == '/demo/map'
    for name, content in prepared.artifacts:
        if name.startswith('params/rendered/'):
            for forbidden in (b'/lightning/', b'/dev/wheeltec', b'base_footprint'):
                assert forbidden not in content


def test_bt_paths_and_map_metadata_are_run_local(tmp_path: Path) -> None:
    """Mutable parameter sources are snapshotted and BT paths refer to this run."""
    prepared = _prepare(tmp_path)
    artifacts = dict(prepared.artifacts)
    nav2 = yaml.safe_load(artifacts['params/rendered/nav2.yaml'])
    for key in ('default_bt_xml_filename', 'default_nav_to_pose_bt_xml',
                'default_nav_through_poses_bt_xml'):
        path = Path(nav2['bt_navigator']['ros__parameters'][key])
        assert str(path.relative_to(tmp_path / 'run')) in artifacts
    map_data = yaml.safe_load(artifacts['params/rendered/map.yaml'])
    assert Path(map_data['image']).is_absolute()
    fingerprints = json.loads(artifacts['params/inputs/fingerprints.json'])
    assert len(fingerprints['map_image']['sha256']) == 64
    assert fingerprints['named_poses']['path'].endswith('/pois/named_poses.json')
    assert not any('named_poses.json' in name for name in artifacts)


def test_teleop_binds_layout_terminal_and_explicit_robot_deadzone(tmp_path: Path) -> None:
    """Alternate robots do not inherit David's old implicit keyboard floor."""
    prepared = _prepare(tmp_path, 'teleop')
    tree = yaml.safe_load(dict(prepared.artifacts)['params/rendered/teleop.yaml'])
    values = tree['chassis_teleop']['ros__parameters']
    assert values['layout'] == 'wasd'
    assert values['tty_device'] == '/dev/tty'
    assert values['linear_deadzone'] == values['angular_deadzone'] == 0.0
    assert values['linear_speed'] == 0.1
    assert values['guard_topics'] == [
        prepared.options.chassis_command_topic, prepared.options.nav_command_topic]
    assert values['guard_topics'][0] == '/demo/command'
    assert values['discovery_sec'] == 1.0


@pytest.mark.parametrize('mode', ['chassis', 'teleop'])
def test_non_navigation_does_not_require_map_or_urdf(tmp_path: Path, mode: str) -> None:
    """Manual chassis tools work with no localization or site map assets."""
    inputs = replace(_inputs(tmp_path, mode), site=None, urdf=None,
                     nav2_base=Path('/missing'), bt_templates=Path('/missing'))
    prepared = prepare_bundle(GraphOptions(mode, str(tmp_path / 'run')), inputs,
                              schema.ParameterCatalog([]))
    assert not any(name.endswith(('map.yaml', 'nav2.yaml')) for name, _ in prepared.artifacts)
    assert prepared.site == 'none'


def test_bench_does_not_start_a_second_map_server(tmp_path: Path) -> None:
    """The bench provider owns the map publisher."""
    prepared = _prepare(tmp_path, 'bench_slamgt')
    assert not prepared.options.map_server
    assert 'params/rendered/map_server.yaml' not in dict(prepared.artifacts)


@pytest.mark.parametrize('mutation', ['mode', 'frame', 'limit', 'driver', 'map'])
def test_invalid_inputs_leave_no_output(tmp_path: Path, mutation: str) -> None:
    """Validation fails before even creating a run directory."""
    inputs = _inputs(tmp_path)
    profile = render.load_yaml(inputs.profile)
    if mutation == 'mode':
        inputs.mode.write_text('schema_version: 1\nmode: precise\n')
    elif mutation == 'frame':
        profile['frames']['nav_base'] = 'missing_link'
    elif mutation == 'limit':
        profile['limits']['max_vel_x'] = 20.0
    elif mutation == 'driver':
        del profile['chassis']['driver']['executable']
    elif mutation == 'map':
        inputs = replace(inputs, site=tmp_path / 'missing_site')
    inputs.profile.write_text(yaml.safe_dump(profile))
    with pytest.raises((ValueError, OSError)):
        prepare_bundle(GraphOptions('tour', str(tmp_path / 'run')), inputs,
                       schema.load_catalog(schema.find_source_schemas(SOURCE)))
    assert not (tmp_path / 'run').exists()


def test_teleop_requires_explicit_matching_adapter_overlay(tmp_path: Path) -> None:
    """A wrong keyboard input route fails instead of silently dropping commands."""
    inputs = _inputs(tmp_path, 'teleop')
    inputs.mode.write_text('schema_version: 1\nmode: teleop\n')
    with pytest.raises(ValueError, match='teleop.*adapter'):
        prepare_bundle(GraphOptions('teleop', str(tmp_path / 'run')), inputs,
                       schema.ParameterCatalog([]))


def test_writer_refuses_existing_artifacts_before_writing_anything(tmp_path: Path) -> None:
    """Retrying preparation cannot overwrite a user's existing run record."""
    prepared = _prepare(tmp_path)
    root = tmp_path / 'run'
    (root / 'params/rendered').mkdir(parents=True)
    existing = root / 'params/rendered/nav2.yaml'
    existing.write_bytes(b'user content')
    with pytest.raises(FileExistsError):
        write_bundle(prepared)
    assert existing.read_bytes() == b'user content'
    assert not (root / 'launch.json').exists()
    assert not (root / 'params/inputs').exists()


def test_writer_rejects_symlinked_destination(tmp_path: Path) -> None:
    """Preparation must never follow a run directory link into unrelated files."""
    prepared = _prepare(tmp_path)
    root = tmp_path / 'run'
    root.mkdir()
    other = tmp_path / 'other'
    other.mkdir()
    (root / 'params').symlink_to(other, target_is_directory=True)
    with pytest.raises(ValueError, match='symlink'):
        write_bundle(prepared)
    assert not list(other.iterdir())


def test_write_failure_never_publishes_a_launchable_bundle(tmp_path: Path, monkeypatch) -> None:
    """launch.json is the final commit marker, including when a disk write fails."""
    prepared = _prepare(tmp_path)
    (tmp_path / 'run').mkdir()
    original = Path.open

    def fail(path, *args, **kwargs):
        if path.name == 'nav2.yaml' and args and args[0] == 'xb':
            raise OSError('simulated disk full')
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, 'open', fail)
    with pytest.raises(OSError, match='disk full'):
        write_bundle(prepared)
    assert not (tmp_path / 'run/launch.json').exists()


@pytest.mark.parametrize('mode', ['tour', 'precise', 'bench_slamgt'])
def test_david_nav2_render_is_unchanged_except_run_local_bt_paths(
    tmp_path: Path, mode: str,
) -> None:
    """New bundle preparation reuses the renderer already checked against Nav2 goldens."""
    david = SOURCE / 'david_config'
    inputs = replace(_inputs(tmp_path, mode), profile=david / 'robot/david.yaml',
                     mode=david / 'modes' / (mode + '.yaml'), urdf=david / 'urdf/david.urdf')
    prepared = prepare_bundle(GraphOptions(mode, str(tmp_path / 'run')), inputs,
                              schema.load_catalog(schema.find_source_schemas(SOURCE)))
    expected = render.render_files(inputs.nav2_base, inputs.nav2_bindings,
                                   inputs.profile, inputs.mode)
    actual = yaml.safe_load(dict(prepared.artifacts)['params/rendered/nav2.yaml'])
    for name in ('default_bt_xml_filename', 'default_nav_to_pose_bt_xml',
                 'default_nav_through_poses_bt_xml'):
        assert Path(actual['bt_navigator']['ros__parameters'][name]).name == (
            Path(expected['bt_navigator']['ros__parameters'][name]).name)
        actual['bt_navigator']['ros__parameters'][name] = (
            expected['bt_navigator']['ros__parameters'][name])
    assert actual == expected
    driver = yaml.safe_load(dict(prepared.artifacts)['params/rendered/driver.yaml'])
    assert driver['wheeltec_robot']['ros__parameters']['publish_imu'] is False
    assert driver['wheeltec_robot']['ros__parameters']['cmd_timeout_sec'] == 0.0


def test_disabled_owned_driver_emits_no_driver_parameters(tmp_path: Path) -> None:
    """Using an external chassis never includes an owned hardware parameter artifact."""
    prepared = _prepare(tmp_path, start_chassis=False)
    assert 'params/rendered/driver.yaml' not in dict(prepared.artifacts)


def test_rviz_requires_an_explicit_asset_and_preserves_its_bytes(tmp_path: Path) -> None:
    """Optional display setup is snapshotted, not evaluated as a shell command."""
    inputs = _inputs(tmp_path)
    options = GraphOptions('tour', str(tmp_path / 'run'), rviz=True)
    with pytest.raises(ValueError, match='rviz selected'):
        prepare_bundle(options, inputs, schema.ParameterCatalog([]))
    rviz = tmp_path / 'layout.rviz'
    rviz.write_bytes(b'Panels: []\n')
    prepared = prepare_bundle(options, replace(inputs, rviz=rviz), schema.ParameterCatalog([]))
    assert dict(prepared.artifacts)['inputs/navigation.rviz'] == rviz.read_bytes()


@pytest.mark.parametrize('path', ['../escape', '/outside', 'params/../escape', './escape'])
def test_writer_rejects_untrusted_artifact_paths(tmp_path: Path, path: str) -> None:
    """Even a manually constructed bundle cannot address files outside its run."""
    prepared = _prepare(tmp_path)
    (tmp_path / 'run').mkdir()
    poisoned = replace(prepared, artifacts=prepared.artifacts + ((path, b'no'),))
    with pytest.raises(ValueError, match='normalized and relative'):
        write_bundle(poisoned)
    assert not list((tmp_path / 'run').iterdir())


def test_commit_marker_flush_failure_leaves_no_launch_file(tmp_path: Path, monkeypatch) -> None:
    """Even a final-file write failure cannot expose incomplete launch.json bytes."""
    prepared = _prepare(tmp_path)
    root = tmp_path / 'run'
    root.mkdir()

    def fail(_descriptor):
        raise OSError('marker disk failure')

    monkeypatch.setattr(os, 'fsync', fail)
    with pytest.raises(OSError, match='marker disk failure'):
        write_bundle(prepared)
    assert not (root / 'launch.json').exists()
    assert not list(root.glob('.launch-*'))


def test_existing_marker_wins_if_created_during_writes(tmp_path: Path, monkeypatch) -> None:
    """Atomic publication fails instead of replacing a concurrently created marker."""
    prepared = _prepare(tmp_path)
    root = tmp_path / 'run'
    root.mkdir()
    original = os.link

    def competing_writer(source, destination):
        Path(destination).write_bytes(b'user marker')
        return original(source, destination)

    monkeypatch.setattr(os, 'link', competing_writer)
    with pytest.raises(FileExistsError):
        write_bundle(prepared)
    assert (root / 'launch.json').read_bytes() == b'user marker'
    assert not list(root.glob('.launch-*'))


@pytest.mark.parametrize('version', [True, 2, None])
def test_unknown_robot_schema_versions_are_not_silently_loaded(
    tmp_path: Path, version,
) -> None:
    """Versioned profile contracts reject booleans, missing and future versions."""
    inputs = _inputs(tmp_path)
    profile = render.load_yaml(inputs.profile)
    profile['schema_version'] = version
    inputs.profile.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='robot: requires schema_version'):
        prepare_bundle(GraphOptions('tour', str(tmp_path / 'run')), inputs,
                       schema.ParameterCatalog([]))
    assert not (tmp_path / 'run').exists()
