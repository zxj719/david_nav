"""Enforce package and pure-layer dependencies without installing import-linter."""

import ast
from pathlib import Path

from nav_bringup.lint import architecture, literals

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
                if root == 'david_nav':
                    violations.append(f'{relative}: reverse legacy dependency {target}')
                if pure and (root in ('rclpy', 'websockets', 'yaml') or root.endswith('_msgs')):
                    violations.append(f'{relative}: I/O dependency in pure layer {target}')
                if root == 'nav_tasks' and target != 'nav_tasks.api':
                    violations.append(f'{relative}: bypasses public TaskApi values: {target}')
    assert not violations, '\n'.join(violations)


def test_robot_specific_literals_stay_in_profiles():
    """Pure contracts must not reintroduce robot-specific frames, IDs or topics."""
    violations = literals.check_paths([PACKAGE])
    assert not violations, literals.format_violations(violations)
