"""Real offline rendering of private selftest inputs, without ROS or operator sites."""

from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from nav_bringup.navcfg import schema
from nav_bringup.navcfg import render
from nav_bringup.navrun.up_planning import discover_inputs, UpRequest
from nav_sim.selftest_scene import prepare_scene
from nav_sim.selftest_plan import load_scene


ROOT = Path(__file__).resolve().parents[2]


def _inputs(robot, mode, root):
    request = UpRequest(mode, robot, 'not_an_operator_site', root)
    return discover_inputs(request, lambda name: ROOT / name)


@pytest.mark.parametrize('robot', ['david', 'example_diffdrive'])
@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_real_bundle_is_private_and_contains_both_cases(tmp_path, robot, mode):
    """Both robots retain mode tuning while all writable and ROS endpoints are private."""
    root = tmp_path / 'run'
    inputs = _inputs(robot, mode, root)
    originals = {path: path.read_bytes() for path in (inputs.profile, inputs.mode, inputs.urdf)}
    catalog = schema.load_catalog(schema.find_source_schemas(ROOT))
    plan = prepare_scene(root, robot, mode, 'selftest_' + 'a' * 32, inputs, catalog,
                         'ws://127.0.0.1:18123')
    assert root.is_dir() and (root.stat().st_mode & 0o777) == 0o700
    assert [case.kind for case in plan.cases] == ['poi', 'dock']
    assert [case.goal for case in plan.cases] == [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    assert all(path.read_bytes() == value for path, value in originals.items())
    profile = yaml.safe_load((root / 'params/rendered/executor_profile.yaml').read_text())
    overlay = render.parse_mode(yaml.safe_load(originals[inputs.mode]), inputs.mode)
    selected = render.deep_merge(yaml.safe_load(originals[inputs.profile]), overlay.profile)
    for key in ('goal', 'limits', 'footprint', 'frames', 'reactive'):
        assert profile[key] == selected[key]
    prefix = '/' + plan.namespace + '/'
    assert profile['nav2']['odom_topic'] == prefix + 'odom'
    assert profile['localization']['odom_topic'] == prefix + 'odom'
    assert profile['chassis']['driver']['package'] == 'nav_sim'
    assert profile['chassis']['driver']['parameters'] == {}
    assert profile['executor']['poi_store_file'] == str(root / 'poi_store.yaml')
    assert not (root / 'poi_store.yaml').exists()
    assert not inputs.site.exists()  # An operator site was never required or read.
    processes = {item.name: item for item in plan.graph.processes}
    assert processes['fake_chassis'].package == 'nav_sim'
    assert processes['fake_localization'].package == 'nav_sim'
    assert processes['controller_server'].package == 'nav2_controller'
    assert not {'wheeltec_robot', 'rviz2', 'pointcloud_to_laserscan'} & processes.keys()
    assert not any(item.package == 'wheeltec_driver' for item in processes.values())
    executor = processes['navigation_executor']
    assert '--dock-poi-id' in executor.arguments
    assert '--dock-config-file' in executor.arguments
    assert prefix + 'navigate_to_pose' == executor.ready_action
    assert '--ros-args' in executor.arguments
    assert '__ns:=' + prefix[:-1] in executor.arguments
    dock = yaml.safe_load((root / 'params/rendered/dock.yaml').read_text())
    values = dock['dock_navigator']['ros__parameters']
    assert values['static_map_topic'] == prefix + 'map'
    assert values['goal_topic'] == prefix + 'dock_goal'
    assert values['settle_max_rate_rad_s'] == 0.05
    assert values['xy_ok'] == profile['goal']['xy_tolerance']
    for item in processes.values():
        if item.ros_node:
            assert ('/tf', prefix + 'tf') in item.remappings
            assert ('/tf_static', prefix + 'tf_static') in item.remappings
            tree = yaml.safe_load(Path(item.parameter_file).read_text())
            assert set(tree) == {plan.namespace}
    record = json.loads((root / 'selftest.json').read_text())
    assert record['backend'] == 'real_nav2'
    assert record['executed'] is False
    assert record['mode'] == mode and record['robot'] == robot
    assert load_scene(root, plan.digest) == plan


@pytest.mark.parametrize('bad_namespace', ['', '/', 'selftest_../x', 'ordinary', 'selftest_x'])
def test_bad_namespace_has_no_writes(tmp_path, bad_namespace):
    """Non-unique namespaces never create a directory."""
    root = tmp_path / 'run'
    with pytest.raises(ValueError):
        prepare_scene(root, 'david', 'tour', bad_namespace,
                      _inputs('david', 'tour', root), schema.ParameterCatalog([]),
                      'ws://127.0.0.1:18123')
    assert not root.exists()


@pytest.mark.parametrize('uri', [
    'ws://remote:123', 'ws://127.0.0.1:0', 'wss://127.0.0.1:123', 'ws://127.0.0.1:123/path'])
def test_nonprivate_server_refused_before_writes(tmp_path, uri):
    """Only a plain owned loopback WebSocket listener is admissible."""
    root = tmp_path / 'run'
    with pytest.raises(ValueError):
        prepare_scene(root, 'david', 'tour', 'selftest_' + 'b' * 32,
                      _inputs('david', 'tour', root), schema.ParameterCatalog([]), uri)
    assert not root.exists()


def test_existing_output_and_symlink_are_not_overwritten(tmp_path):
    """Neither an existing empty directory nor a symlink is a new experiment."""
    root = tmp_path / 'run'
    root.mkdir()
    inputs = _inputs('david', 'tour', root)
    with pytest.raises(FileExistsError):
        prepare_scene(root, 'david', 'tour', 'selftest_' + 'b' * 32,
                      inputs, schema.ParameterCatalog([]), 'ws://127.0.0.1:18123')
    link = tmp_path / 'link'
    link.symlink_to(root, target_is_directory=True)
    with pytest.raises((FileExistsError, ValueError)):
        prepare_scene(link, 'david', 'tour', 'selftest_' + 'b' * 32,
                      inputs, schema.ParameterCatalog([]), 'ws://127.0.0.1:18123')
    assert list(root.iterdir()) == []


def test_mode_mismatch_and_invalid_robot_fail_before_rendering(tmp_path):
    """Selector identity is checked before creating private assets."""
    root = tmp_path / 'run'
    inputs = _inputs('david', 'tour', root)
    wrong = replace(inputs, mode=ROOT / 'david_config/modes/precise.yaml')
    with pytest.raises(ValueError):
        prepare_scene(root, 'david', 'tour', 'selftest_' + 'b' * 32,
                      wrong, schema.ParameterCatalog([]), 'ws://127.0.0.1:18123')
    assert not root.exists()


@pytest.fixture
def scene(tmp_path):
    """Create one actual offline fixture for integrity rejection tests."""
    root = tmp_path / 'run'
    return prepare_scene(
        root, 'example_diffdrive', 'precise', 'selftest_' + 'c' * 32,
        _inputs('example_diffdrive', 'precise', root),
        schema.load_catalog(schema.find_source_schemas(ROOT)), 'ws://127.0.0.1:18123')


@pytest.mark.parametrize('name', [
    'scene/poses.json', 'scene/map.pgm', 'params/rendered/dock.yaml', 'params/selftest/nav2.yaml'])
def test_changed_input_or_rendered_bytes_cannot_launch(scene, name):
    """The launcher verifies every frozen input and emitted parameter file."""
    (scene.root / name).write_bytes(b'changed')
    with pytest.raises(ValueError, match='artifact changed'):
        load_scene(scene.root, scene.digest)


def test_plan_change_is_not_authorized_by_recomputing_its_own_hash(scene):
    """The expected digest comes from the parent, not from mutable plan metadata."""
    path = scene.root / 'selftest.json'
    path.write_bytes(path.read_bytes() + b' ')
    with pytest.raises(ValueError, match='plan changed'):
        load_scene(scene.root, scene.digest)


def test_even_authenticated_plan_cannot_select_hardware(scene):
    """An explicit launcher whitelist independently rejects hardware executables."""
    path = scene.root / 'selftest.json'
    record = json.loads(path.read_text())
    record['graph']['processes'][0]['package'] = 'wheeltec_driver'
    payload = json.dumps(record).encode()
    path.write_bytes(payload)
    with pytest.raises(ValueError, match='hardware is forbidden'):
        load_scene(scene.root, hashlib.sha256(payload).hexdigest())


def test_symlinked_artifact_is_not_read(scene):
    """Even byte-identical targets must not bypass the private artifact boundary."""
    path = scene.root / 'params/rendered/dock.yaml'
    saved = scene.root / 'saved.yaml'
    path.rename(saved)
    path.symlink_to(saved)
    with pytest.raises(ValueError, match='symlinked'):
        load_scene(scene.root, scene.digest)
