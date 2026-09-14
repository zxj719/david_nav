"""Tests for the forbidden literal policy of refactor plan §5.4."""

from pathlib import Path

from nav_bringup.lint import literals


def test_check_source_reports_forbidden_literals():
    """Frame names and robot topic prefixes are reported; docstrings are not."""
    source = (
        '"""Module doc mentioning base_footprint."""\n'
        'FRAME = "base_footprint"\n'
        'TOPIC = "/lightning/loc/odometry"\n'
        'OK = "map"\n'
        'def f():\n'
        '    """Mention base_link in a docstring."""\n'
        '    return "base"\n'
    )

    violations = literals.check_source(source, 'demo.py')

    assert [(v.line, v.value) for v in violations] == [
        (2, 'base_footprint'),
        (3, '/lightning/loc/odometry'),
        (7, 'base'),
    ]
    assert 'demo.py:2: forbidden literal' in literals.format_violations(violations)


def test_check_paths_scans_directories(tmp_path):
    """Directories are searched recursively."""
    (tmp_path / 'pkg').mkdir()
    (tmp_path / 'pkg' / 'clean.py').write_text('X = "odom"\n', encoding='utf-8')
    (tmp_path / 'pkg' / 'dirty.py').write_text('X = "base_link"\n', encoding='utf-8')
    (tmp_path / 'pkg' / 'policy.py').write_text('X = "base"\n', encoding='utf-8')

    violations = literals.check_paths([tmp_path], exclude=[tmp_path / 'pkg' / 'policy.py'])

    assert [(v.path.name, v.value) for v in violations] == [('dirty.py', 'base_link')]


def test_nav_bringup_has_no_robot_literals():
    """The nav_bringup package code names no robot frames or robot topics."""
    package_dir = Path(__file__).resolve().parents[1] / 'nav_bringup'

    violations = literals.check_paths(
        [package_dir], exclude=[package_dir / 'lint' / 'literals.py'])

    assert not violations, literals.format_violations(violations)
