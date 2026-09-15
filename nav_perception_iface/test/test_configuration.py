"""旧预处理参数金样与配置错误回归，不启动 ROS 节点."""

import copy
from pathlib import Path

import pytest
import yaml

from nav_perception_iface.configuration import load_scan_configuration, scan_configuration

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'david_config/robot/david.yaml'
GOLDEN = Path(__file__).parent / 'golden/pointcloud_to_scan.yaml'


@pytest.fixture
def profile():
    """Provide an independent copy of the current robot profile."""
    return yaml.safe_load(PROFILE.read_text(encoding='utf-8'))


def test_all_scan_parameters_and_remappings_equal_the_legacy_launch(profile):
    """The extraction preserves every original node parameter and topic."""
    golden = yaml.safe_load(GOLDEN.read_text(encoding='utf-8'))
    original = copy.deepcopy(profile)
    config = scan_configuration(profile)
    assert config.enabled is golden['enabled'] is False
    assert config.parameters == golden['parameters']
    assert [['cloud_in', config.cloud_topic], ['scan', config.scan_topic]] == golden['remappings']
    assert profile == original


@pytest.mark.parametrize('override, expected', [(None, False), (True, True), (False, False)])
def test_explicit_legacy_enabled_override(override, expected):
    """The old enable_scan_bridge flag overrides only the enabled field."""
    config = load_scan_configuration(PROFILE, override)
    assert config.enabled is expected
    assert config.parameters == load_scan_configuration(PROFILE).parameters


def test_missing_enabled_defaults_to_disabled(profile):
    """The optional enable key is conservative when omitted."""
    del profile['sensors']['scan_from_cloud']['enabled']
    assert not scan_configuration(profile).enabled


def test_other_robot_frames_and_topics_require_only_profile_changes(profile):
    """A different sensor and reference frame need no implementation edits."""
    profile['frames']['scan_target'] = 'other_floor'
    profile['sensors']['scan_from_cloud'].update(
        cloud_topic='/other/cloud', scan_topic='/other/scan')
    config = scan_configuration(profile)
    assert config.parameters['target_frame'] == 'other_floor'
    assert (config.cloud_topic, config.scan_topic) == ('/other/cloud', '/other/scan')


@pytest.mark.parametrize('key,value', [
    ('enabled', 'false'), ('use_inf', 1), ('transform_tolerance', -0.1),
    ('angle_increment', 0.0), ('scan_time', 0.0), ('range_min', -1.0),
    ('min_height', float('nan')), ('max_height', float('inf')),
    ('range_max', 0.1), ('angle_max', -2.0), ('cloud_topic', ''),
])
def test_invalid_profiles_are_rejected(profile, key, value):
    """Wrong types, bad intervals and nonfinite numbers cannot enter the node."""
    profile['sensors']['scan_from_cloud'][key] = value
    with pytest.raises(ValueError):
        scan_configuration(profile)


def test_missing_frame_and_parameter_are_errors(profile):
    """A missing coordinate convention must not get a robot-specific fallback."""
    del profile['frames']['scan_target']
    with pytest.raises(ValueError, match='frames.scan_target'):
        scan_configuration(profile)


def test_invalid_yaml_is_a_configuration_error(tmp_path):
    """A malformed input file reports an actionable configuration failure."""
    path = tmp_path / 'invalid.yaml'
    path.write_text('broken: [', encoding='utf-8')
    with pytest.raises(ValueError, match='YAML'):
        load_scan_configuration(path)
