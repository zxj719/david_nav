"""The --OA overlay adds one direct PointCloud2 obstacle source."""

from pathlib import Path

import pytest

from nav_bringup.navcfg import render

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_DIR.parent
PROFILE = PACKAGE_DIR / 'robot' / 'david.yaml'
MODES_DIR = PACKAGE_DIR / 'modes'
OVERLAY = MODES_DIR / 'overlays' / 'oa.yaml'
NAV2_CONFIG_DIR = SOURCE_DIR / 'nav_bringup' / 'config'
LOCAL_COSTMAP = 'local_costmap/local_costmap/ros__parameters'
PLUGINS = LOCAL_COSTMAP + '/plugins'
OBSTACLE_LAYER = LOCAL_COSTMAP + '/obstacle_layer'
SOURCE = OBSTACLE_LAYER + '/aligned_cloud'


def _overlay():
    return render.parse_mode(render.load_yaml(OVERLAY), OVERLAY)


def _effective(mode='tour', with_oa=True):
    overlay = render.parse_mode(render.load_yaml(MODES_DIR / f'{mode}.yaml'), mode)
    if with_oa:
        overlay = render.merge_overlays(overlay, _overlay())
    return render.deep_merge(render.load_yaml(PROFILE), overlay.profile)


def _render(mode='tour', with_oa=True):
    selected = render.parse_mode(render.load_yaml(MODES_DIR / f'{mode}.yaml'), mode)
    if with_oa:
        selected = render.merge_overlays(selected, _overlay())
    return render.render(
        render.load_yaml(NAV2_CONFIG_DIR / 'nav2_base.yaml'),
        render.parse_bindings(
            render.load_yaml(NAV2_CONFIG_DIR / 'nav2_bindings.yaml'), 'nav2_bindings.yaml'),
        render.load_yaml(PROFILE), selected)


def test_the_overlay_keeps_the_identity_of_the_mode_it_extends():
    """Adding --OA to a mode does not turn the run into a different mode."""
    merged = render.merge_overlays(
        render.parse_mode(render.load_yaml(MODES_DIR / 'tour.yaml'), 'tour'), _overlay())

    assert merged.name == 'tour'


def test_the_overlay_does_not_start_the_scan_bridge():
    """PointCloud2 goes straight to Nav2 instead of taking a LaserScan detour."""
    assert _effective(with_oa=False)['sensors']['scan_from_cloud']['enabled'] is False
    assert _effective()['sensors']['scan_from_cloud']['enabled'] is False


def test_the_local_costmap_marks_obstacles_before_it_inflates_them():
    """Layer order decides what gets inflated; marking after inflating would inflate nothing."""
    plugins = render.get_path(_render(), PLUGINS)

    assert 'obstacle_layer' in plugins
    assert plugins.index('obstacle_layer') < plugins.index('inflation_layer')


def test_the_overlay_uses_one_standard_obstacle_layer():
    """The OA path has no second custom layer competing for the same observations."""
    plugins = render.get_path(_render(), PLUGINS)

    assert plugins == ['static_layer', 'obstacle_layer', 'inflation_layer']


def test_the_obstacle_layer_reads_the_aligned_cloud_directly():
    """The active source is PointCloud2 and matches the localization profile topic."""
    tree = _render()
    profile = _effective()

    assert render.get_path(tree, OBSTACLE_LAYER + '/observation_sources') == 'aligned_cloud'
    assert render.get_path(tree, SOURCE + '/topic') == profile['localization'][
        'aligned_cloud_topic']
    assert render.get_path(tree, SOURCE + '/data_type') == 'PointCloud2'
    assert render.get_path(tree, SOURCE + '/sensor_frame') == profile['frames']['nav_base']


def test_the_direct_source_keeps_the_validated_height_and_range_window():
    """The direct source mirrors the field-tested alias 0 obstacle window."""
    source = render.get_path(_render(), SOURCE)

    assert (source['min_obstacle_height'], source['max_obstacle_height']) == (-0.55, 0.35)
    assert (source['obstacle_min_range'], source['obstacle_max_range']) == (0.2, 2.5)
    assert (source['raytrace_min_range'], source['raytrace_max_range']) == (0.2, 3.0)
    assert source['marking'] is True
    assert source['clearing'] is True


def test_the_overlay_only_touches_the_local_costmap_obstacle_wiring():
    """Everything outside the plugin list and direct source is unchanged."""
    with_oa = _render()
    without = _render(with_oa=False)
    render.set_path(with_oa, PLUGINS, render.get_path(without, PLUGINS))
    render.set_path(
        with_oa, OBSTACLE_LAYER + '/observation_sources',
        render.get_path(without, OBSTACLE_LAYER + '/observation_sources'))
    del render.get_path(with_oa, OBSTACLE_LAYER)['aligned_cloud']

    assert with_oa == without


@pytest.mark.parametrize('mode', ['tour', 'precise', 'bench_slamgt'])
def test_every_navigation_mode_accepts_the_overlay(mode):
    """The flag is orthogonal to the mode, so no mode may fail to render with it."""
    assert 'obstacle_layer' in render.get_path(_render(mode), PLUGINS)
