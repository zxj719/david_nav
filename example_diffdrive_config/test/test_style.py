"""Data package format and Google documentation gates."""

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257


def test_flake8():
    """Package metadata and tests follow the ROS format."""
    code, errors = main_with_errors(argv=[])
    assert code == 0, '\n'.join(errors)


def test_pep257():
    """Package metadata and tests have Google-style documentation."""
    assert pep257(argv=['.', 'test', '--convention', 'google']) == 0
