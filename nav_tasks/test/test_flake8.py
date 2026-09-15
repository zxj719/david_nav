"""Enforce formatting across the complete package."""

from ament_flake8.main import main_with_errors


def test_flake8():
    """Require zero formatting errors."""
    code, errors = main_with_errors(argv=[])
    assert code == 0, '\n'.join(errors)
