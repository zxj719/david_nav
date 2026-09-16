"""Check the handwritten interface tests, not rosidl-generated implementation code."""

from ament_flake8.main import main_with_errors
from ament_pep257.main import main


def test_flake8():
    """Require clean Python formatting for the handwritten test suite."""
    code, errors = main_with_errors(argv=['test'])
    assert code == 0, '\n'.join(errors)


def test_pep257():
    """Require Google-style docstrings without legacy exemptions."""
    assert main(argv=['test', '--convention', 'google']) == 0
