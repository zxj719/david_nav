"""Site profiles hold the map and point files production uses today (PROGRESS P2.2b)."""

from pathlib import Path

import pytest
import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_DIR.parent
SITES_DIR = PACKAGE_DIR / 'sites'
DAOLAN = SITES_DIR / 'daolan'


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


@pytest.mark.parametrize(
    'site_file', sorted(SITES_DIR.glob('*/site.yaml')), ids=lambda path: path.parent.name)
def test_site_profile_is_complete(site_file):
    """A site is named after its directory and references an existing trinary map yaml."""
    site = _load(site_file)
    nav_yaml = site_file.parent / site['map']['nav_yaml']

    assert site['schema_version'] == 1
    assert site['site']['name'] == site_file.parent.name
    assert nav_yaml.is_file()
    assert _load(nav_yaml)['mode'] == 'trinary'
    assert site['agent_server']['uri'].startswith('ws://')


def test_daolan_map_yaml_equals_the_production_copy():
    """The site map yaml is byte-identical to the one the bringup launch loads today."""
    site_copy = DAOLAN / 'maps' / 'daolan_map_nav.yaml'
    production = SOURCE_DIR / 'david_nav' / 'config' / 'maps' / 'daolan_map_nav.yaml'

    assert site_copy.read_bytes() == production.read_bytes()


def test_daolan_map_image_is_in_the_lightning_map_directory():
    """The map yaml points at the image of the lightning map directory."""
    site = _load(DAOLAN / 'site.yaml')
    image = _load(DAOLAN / site['map']['nav_yaml'])['image']

    assert Path(image).parent == Path(site['map']['lightning_map_dir'])


def test_daolan_points_and_server_match_the_executor():
    """Point files equal the executor defaults; the server equals the headless configuration."""
    site = _load(DAOLAN / 'site.yaml')
    executor = (SOURCE_DIR / 'david_nav' / 'david_nav' / 'navigation_executor.py').read_text(
        encoding='utf-8')
    headless = _load(SOURCE_DIR / 'david_nav' / 'config' / 'navigation_headless.yaml')

    assert f'os.path.expanduser("{site["pois"]["store_file"]}")' in executor
    assert f'os.path.expanduser("{site["pois"]["named_poses_file"]}")' in executor
    assert headless['executor']['server_uri'] == site['agent_server']['uri']
