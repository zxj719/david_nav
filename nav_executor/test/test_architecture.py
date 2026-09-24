"""Enforce package and pure-layer dependencies without installing import-linter."""

import ast
import inspect
from pathlib import Path

from nav_bringup.lint import architecture, literals, node_attributes

PACKAGE = Path(__file__).resolve().parents[1] / 'nav_executor'


def test_transitive_architecture():
    """Enforce logical layers and independent adapters including relative imports."""
    violations = architecture.check_package(PACKAGE)
    assert not violations, '\n'.join(violations)


def test_import_boundaries():
    """Forbid reverse task/transport dependencies and I/O in pure layers."""
    violations = []
    for path in PACKAGE.rglob('*.py'):
        relative = path.relative_to(PACKAGE)
        pure = relative.parts[0] in ('domain', 'protocol', 'core', 'skills')
        for node in ast.walk(ast.parse(path.read_text())):
            targets = []
            if isinstance(node, ast.Import):
                targets = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                targets = [node.module or '']
            for target in targets:
                root = target.split('.')[0]
                if pure and (root in ('rclpy', 'websockets', 'yaml') or root.endswith('_msgs')):
                    violations.append(f'{relative}: I/O dependency in pure layer {target}')
                if root == 'nav_tasks' and target != 'nav_tasks.api':
                    violations.append(f'{relative}: bypasses public TaskApi values: {target}')
    assert not violations, '\n'.join(violations)


def test_robot_specific_literals_stay_in_profiles():
    """Pure contracts must not reintroduce robot-specific frames, IDs or topics."""
    violations = literals.check_paths([PACKAGE])
    assert not violations, literals.format_violations(violations)


def _node_sources():
    workspace = PACKAGE.parents[1]
    sources, packages = {}, set()
    for package in ('nav_bringup', 'nav_executor', 'nav_tasks', 'nav_sim',
                    'nav_perception_iface', 'chassis_adapter', 'chassis_teleop'):
        root = workspace / package / package
        assert root.is_dir(), f'missing navigation source package: {root}'
        for path in sorted(root.rglob('*.py')):
            parts = path.relative_to(root).with_suffix('').parts
            initializer = parts[-1] == '__init__'
            module = '.'.join((package,) + (parts[:-1] if initializer else parts))
            sources[module] = path.read_text(encoding='utf-8')
            if initializer:
                packages.add(module)
    return sources, packages


def test_all_navigation_node_subclasses_preserve_parent_attributes():
    """Use installed Humble source without initializing ROS or constructing nodes."""
    from rclpy.node import Node
    sources, packages = _node_sources()
    result = node_attributes.audit_sources(
        sources, inspect.getsource(Node), packages=packages,
        # Reviewed lifecycle extension seals callbacks/unregisters TF before super.
        # Existing health-node tests cover cleanup; no property/field writes are exempt.
        allowed_methods={('nav_perception_iface.health_node.LocalizationMonitor',
                          'destroy_node')})
    expected = {
        'chassis_adapter.node.AdapterNode', 'chassis_teleop.node.KeyboardNode',
        'nav_sim.fake_chassis.FakeChassis', 'nav_sim.fake_localization.FakeLocalization',
        'nav_perception_iface.health_node.LocalizationMonitor',
    }
    assert expected <= set(result.checked_classes)
    assert not result.violations, '\n'.join(result.violations)


def test_installed_node_snapshot_detects_method_and_instance_collisions():
    """The real parent snapshot must catch both handle and publisher-list damage."""
    from rclpy.node import Node
    result = node_attributes.audit_sources({'probe': '''
from rclpy.node import Node
class Invalid(Node):
    def handle(self):
        self._publishers = []
'''}, inspect.getsource(Node))
    assert result.checked_classes == ('probe.Invalid',)
    assert any('handle: method' in item for item in result.violations)
    assert any('_publishers: instance write' in item for item in result.violations)
