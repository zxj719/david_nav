"""Formatting and robot-literal checks."""

from pathlib import Path

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257

from nav_bringup.lint import literals


def test_flake8():
    """Package and test code follow the ROS Python format."""
    code, errors = main_with_errors(argv=[])
    assert code == 0, '\n'.join(errors)


def test_pep257():
    """Documentation follows Google conventions."""
    assert pep257(argv=['.', 'test', '--convention', 'google']) == 0


def test_robot_literals():
    """Runtime code contains no robot-specific frame or topic literals."""
    assert not literals.check_paths([Path(__file__).resolve().parents[1] / 'chassis_adapter'])
