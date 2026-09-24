"""The David profile is self-contained and valid for the single pipeline."""

from pathlib import Path

import pytest
import yaml

from nav_bringup.navcfg import schema

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_DIR.parent
PROFILE = PACKAGE_DIR / 'robot/david.yaml'


@pytest.fixture(scope='module')
def profile():
    """Load the shared David profile for module-scoped assertions."""
    return yaml.safe_load(PROFILE.read_text(encoding='utf-8'))


def test_frames_and_speed_limits_are_explicit(profile):
    """The profile owns frame choices and bounded command limits."""
    assert (profile['frames']['nav_base'], profile['frames']['body']) == (
        'base_footprint', 'base_link')
    adapter = profile['chassis']['adapter']
    assert profile['limits']['max_vel_x'] <= adapter['hard_max_linear_x']
    assert profile['limits']['max_vel_theta'] <= adapter['hard_max_angular_z']
    assert profile['teleop']['linear_speed'] <= adapter['hard_max_linear_x']
    assert profile['teleop']['angular_speed'] <= adapter['hard_max_angular_z']


def test_dock_policy_is_separate_from_nav2_goal_tolerance(profile):
    """Dock acceptance remains an explicit task policy, not a Nav2 side effect."""
    dock = profile['dock']
    precise_goal = yaml.safe_load((PACKAGE_DIR / 'modes/precise.yaml').read_text())[
        'profile']['goal']
    assert (dock['xy_ok'], dock['yaw_ok']) == (0.25, 0.4363)
    assert dock['xy_ok'] > precise_goal['xy_tolerance']


def test_urdf_meshes_are_owned_by_david_config():
    """The surviving robot package owns every chassis mesh used by its URDF."""
    urdf = (PACKAGE_DIR / 'urdf/david.urdf').read_text(encoding='utf-8')
    for name in ('base_link.STL', 'left_wheel_link.STL', 'right_wheel_link.STL'):
        assert f'package://david_config/meshes/zongzhuang/{name}' in urdf
        assert (PACKAGE_DIR / 'meshes/zongzhuang' / name).is_file()


def test_profile_passes_workspace_schemas(profile):
    """The single robot profile satisfies every registered schema."""
    catalog = schema.load_catalog(schema.find_source_schemas(SOURCE_DIR))
    errors = [issue.message for issue in catalog.check_profile(profile)
              if issue.severity == 'error']
    assert not errors
