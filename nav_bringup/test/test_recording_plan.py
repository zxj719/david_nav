"""Plan explicit SQLite recording commands without starting ROS or touching storage."""

from pathlib import Path

import pytest
import yaml

from nav_bringup.navrun.recording import plan_recording


def test_light_recording_has_explicit_topics_and_new_run_local_output(tmp_path):
    """No all-topics flag or compression accidentally expands a light recording."""
    result = plan_recording({'record': {'light_topics': ['/drive', '/tf']}}, 'light', tmp_path)
    assert result.argv == ('ros2', 'bag', 'record', '--storage', 'sqlite3',
                           '--max-cache-size', '104857600', '--output',
                           str(tmp_path / 'bag/data'), '/drive', '/tf')
    assert result.enabled
    assert not (tmp_path / 'bag').exists()


@pytest.mark.parametrize('mode', ['off', 'light'])
def test_disabled_or_empty_light_list_is_explicitly_not_recording(tmp_path, mode):
    """An empty selection is not reported as a running rosbag recorder."""
    result = plan_recording({}, mode, tmp_path)
    assert not result.enabled
    assert result.argv == ()
    assert result.reason


@pytest.mark.parametrize('settings', [
    {'storage': 'mcap'}, {'max_cache_size_bytes': -1}, {'light_topics': ['/good', '--all']},
    {'light_topics': 'all'}, {'light_topics': ['/bad name']},
    {'full_regex': '['}, {'full_include_hidden': 'yes'}, {'full_compression': 'zip'}])
def test_bad_recording_configuration_fails_without_starting_anything(tmp_path, settings):
    """Only supported storage/options and explicit ROS topic names reach argv."""
    profile = {'record': {'light_topics': ['/one'], 'full_topics': ['/one'], **settings}}
    with pytest.raises(ValueError):
        plan_recording(profile, 'full' if any(key.startswith('full_') for key in settings)
                       else 'light', tmp_path)


def test_full_recording_requires_explicit_topics_or_regex(tmp_path):
    """Full does not silently become --all when the robot profile lacks a selection."""
    with pytest.raises(ValueError, match='selection'):
        plan_recording({}, 'full', tmp_path)


def test_david_selections_are_explicit_and_safe(tmp_path):
    """Both lists and the full regex produce bounded explicit recording commands."""
    source = Path(__file__).resolve().parents[2]
    profile = yaml.safe_load((source / 'david_config/robot/david.yaml').read_text())
    for mode in ('light', 'full'):
        plan = plan_recording(profile, mode, tmp_path)
        assert list(plan.topics) == profile['record'][mode + '_topics']
        assert plan.enabled
        assert '--all' not in plan.argv
        assert plan.argv[plan.argv.index('--storage') + 1] == 'sqlite3'
    full = plan_recording(profile, 'full', tmp_path)
    assert full.regex == profile['record']['full_regex']
    assert '--include-hidden-topics' in full.argv
    assert '--include-unpublished-topics' in full.argv
    assert full.argv[full.argv.index('--compression-format') + 1] == 'zstd'
