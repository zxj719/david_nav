"""Enforce public argument, return and exception documentation."""

from pathlib import Path

from nav_bringup.lint import docstrings


def test_public_contracts_are_documented():
    """Scan source APIs without importing implementations."""
    violations = docstrings.check_paths([Path(__file__).resolve().parents[1] / 'nav_tasks'])
    assert not violations, docstrings.format_violations(violations)


def test_long_private_functions_are_documented():
    """Require nonblank documentation for private definitions over ten lines."""
    violations = docstrings.check_private_paths([
        Path(__file__).resolve().parents[1] / 'nav_tasks'])
    assert not violations, docstrings.format_violations(violations)


def test_all_long_functions_are_documented():
    """所有作用域及所有命名的长函数均需有非空说明."""
    violations = docstrings.check_long_paths([
        Path(__file__).resolve().parents[1] / 'nav_tasks'])
    assert not violations, docstrings.format_violations(violations)
