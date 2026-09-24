"""Configuration-only second-robot acceptance against unchanged shared code."""

import hashlib
from pathlib import Path

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

from nav_bringup.navcfg import behavior_trees, cli, render, schema, validation

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent
BRINGUP = SOURCE / 'nav_bringup'
PLUGINS = SOURCE / 'nav_plugins'
PROFILE = ROOT / 'robot' / 'example_diffdrive.yaml'


def _fingerprint():
    return {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for directory in (BRINGUP / 'config', BRINGUP / 'nav_bringup',
                              PLUGINS / 'config', PLUGINS / 'behavior_trees')
            for path in directory.rglob('*')
            if path.is_file() and path.suffix in ('.py', '.yaml', '.tmpl')}


@pytest.mark.parametrize('name', ['tour', 'precise'])
def test_config_only_render_validation_and_bindings(name):
    """Different robot values reach every consumer with no shared code edits."""
    before = _fingerprint()
    profile = render.load_yaml(PROFILE)
    mode_file = ROOT / 'modes' / (name + '.yaml')
    mode = render.parse_mode(render.load_yaml(mode_file), mode_file)
    tree = render.render_files(BRINGUP / 'config' / 'nav2_base.yaml',
                               BRINGUP / 'config' / 'nav2_bindings.yaml', PROFILE, mode_file)
    effective = render.deep_merge(profile, mode.profile)
    bindings_path = BRINGUP / 'config' / 'nav2_bindings.yaml'
    bindings = render.parse_bindings(render.load_yaml(bindings_path), bindings_path)
    for key, binding in bindings.items():
        expected = render.VALUE_FORMATS[binding.value_format](render.profile_value(effective, key))
        assert all(render.get_path(tree, path) == expected for path in binding.paths)
    bundle = behavior_trees.render_bundle(
        PLUGINS / 'behavior_trees', PLUGINS / 'config' / 'bt_bindings.yaml', effective)
    catalog = schema.load_catalog(schema.find_source_schemas(SOURCE))
    urdf = (ROOT / 'urdf' / 'example_diffdrive.urdf').read_text()
    map_data = render.load_yaml(ROOT / 'sites' / 'offline' / 'maps' / 'room.yaml')
    issues = validation.validate_configuration(
        catalog, profile, mode, tree, map_data, bundle, urdf)
    assert not [issue for issue in issues if issue.severity == 'error'], issues
    assert not [issue for issue in issues if issue.rule == 'diff_drive_bt_lateral']
    assert validation.frame_offset(urdf, 'demo_body', 'demo_axle').xyz == (0.04, 0.0, -0.15)
    flattened = yaml.safe_dump(tree) + '\n'.join(bundle.values())
    for forbidden in ('base_footprint', 'base_link', '/lightning/', '/home/agx/', '/david001/'):
        assert forbidden not in flattened
    assert _fingerprint() == before


@pytest.mark.parametrize('name', ['tour', 'precise'])
def test_installed_data_and_cli_render_validate(name, tmp_path):
    """The installed package works without source-only path assumptions."""
    installed = Path(get_package_share_directory('example_diffdrive_config'))
    profile = installed / 'robot' / 'example_diffdrive.yaml'
    mode = installed / 'modes' / (name + '.yaml')
    output = tmp_path / 'nav2.yaml'
    bt_dir = tmp_path / 'bt'
    assert cli.main(['render', '--profile', str(profile), '--mode', str(mode),
                     '--output', str(output), '--bt-output-dir', str(bt_dir)]) == 0
    assert cli.main(['validate', '--profile', str(profile), '--mode', str(mode),
                     '--site', str(installed / 'sites' / 'offline' / 'site.yaml'),
                     '--urdf', str(installed / 'urdf' / 'example_diffdrive.urdf'),
                     '--bt-dir', str(bt_dir)]) == 0
    assert len(list(bt_dir.glob('*.xml'))) == 3
    assert render.load_yaml(output)['bt_navigator']['ros__parameters']['robot_base_frame'] == (
        'demo_axle')


def test_site_is_self_contained_with_an_ascii_map():
    """No referenced fixture depends on a production home or external map."""
    site_dir = ROOT / 'sites' / 'offline'
    site = render.load_yaml(site_dir / 'site.yaml')
    for raw in (site['map']['nav_yaml'], site['pois']['named_poses_file'],
                site['pois']['store_file']):
        path = site_dir / raw
        assert path.is_file()
        assert path.resolve().is_relative_to(ROOT)
    map_path = site_dir / site['map']['nav_yaml']
    metadata = render.load_yaml(map_path)
    image = map_path.parent / metadata['image']
    tokens = ' '.join(line for line in image.read_text().splitlines()
                      if not line.startswith('#')).split()
    assert tokens[:4] == ['P2', '12', '12', '255']
    assert len(tokens[4:]) == 144
    assert metadata['mode'] == 'trinary'


def test_package_contains_no_runtime_code():
    """Porting adds data and tests, not a forked navigation implementation."""
    assert not (ROOT / 'example_diffdrive_config').exists()
    assert not (ROOT / 'launch').exists()
