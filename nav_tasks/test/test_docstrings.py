"""Enforce public argument, return and exception documentation."""

from pathlib import Path

from nav_bringup.lint import docstrings


def test_public_contracts_are_documented():
    """Scan source APIs without importing implementations."""
    violations = docstrings.check_paths([Path(__file__).resolve().parents[1] / 'nav_tasks'])
    assert not violations, docstrings.format_violations(violations)
