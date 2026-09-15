"""Enforce Google-style documentation across the complete package."""

from ament_pep257.main import main


def test_pep257():
    """Require Google conventions without legacy exemptions."""
    assert main(argv=['.', 'test', '--convention', 'google']) == 0
