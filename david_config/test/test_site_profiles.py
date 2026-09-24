"""Site profiles own their map and point-file references."""

from pathlib import Path

import pytest
import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SITES_DIR = PACKAGE_DIR / 'sites'
DAOLAN = SITES_DIR / 'daolan'


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


@pytest.mark.parametrize(
    'site_file', sorted(SITES_DIR.glob('*/site.yaml')), ids=lambda path: path.parent.name)
def test_site_profile_is_complete(site_file):
    """Every site resolves its map and declares its server endpoint."""
    site = _load(site_file)
    nav_yaml = site_file.parent / site['map']['nav_yaml']
    assert site['schema_version'] == 1
    assert site['site']['name'] == site_file.parent.name
    assert nav_yaml.is_file()
    assert _load(nav_yaml)['mode'] == 'trinary'
    assert site['agent_server']['uri'].startswith('ws://')


def test_daolan_map_image_matches_the_lightning_site():
    """The Nav2 map YAML points at the map exported by localization."""
    site = _load(DAOLAN / 'site.yaml')
    image = _load(DAOLAN / site['map']['nav_yaml'])['image']
    assert Path(image).parent == Path(site['map']['lightning_map_dir'])


def test_daolan_point_files_and_server_are_explicit():
    """Daolan keeps POI files and its agent endpoint in site data."""
    site = _load(DAOLAN / 'site.yaml')
    assert site['pois']['store_file']
    assert site['pois']['named_poses_file']
    assert site['agent_server']['uri'].startswith('ws://')
