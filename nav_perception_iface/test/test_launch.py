"""只构造 launch 图的预处理迁移验证，不执行 Node 动作."""

import importlib.util
from pathlib import Path

from launch import LaunchContext


def _module():
    path = Path(__file__).resolve().parents[1] / 'launch/pointcloud_to_scan.launch.py'
    spec = importlib.util.spec_from_file_location('perception_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(enabled):
    context = LaunchContext()
    context.launch_configurations.update({
        'enabled': enabled,
        'profile_file': str(Path(__file__).resolve().parents[2] / 'david_config/robot/david.yaml'),
    })
    return context


def test_launch_default_profile_creates_no_nodes(monkeypatch):
    """The default is truly empty, not a launched-but-idle process."""
    module = _module()
    calls = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: calls.append(kwargs))
    assert module._configure(_context('profile')) == []
    assert module._configure(_context('false')) == []
    assert calls == []


def test_explicit_enable_constructs_the_original_single_node(monkeypatch):
    """Enabling only constructs one original preprocessing action with profile parameters."""
    module = _module()
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    nodes = module._configure(_context('true'))
    assert len(nodes) == 1
    assert nodes[0]['package'] == 'pointcloud_to_laserscan'
    assert nodes[0]['executable'] == 'pointcloud_to_laserscan_node'
    assert nodes[0]['name'] == 'pointcloud_to_laserscan'
    assert nodes[0]['parameters'][0]['min_height'] == 0.10
    assert len(nodes[0]['parameters'][0]) == 11


def test_legacy_bringup_delegates_instead_of_defining_a_second_node():
    """The existing entry point uses the extracted launch and retains its old enable switch."""
    source = (Path(__file__).resolve().parents[2]
              / 'david_nav/launch/2d_david_nav2_bringup.launch.py')
    text = source.read_text(encoding='utf-8')
    assert "package='pointcloud_to_laserscan'" not in text
    assert "'launch', 'pointcloud_to_scan.launch.py'" in text
    assert "'enabled': enable_scan_bridge" in text


def test_launch_description_can_be_constructed_without_launching():
    """Description creation only declares inputs and an opaque configuration action."""
    assert len(_module().generate_launch_description().entities) == 3
