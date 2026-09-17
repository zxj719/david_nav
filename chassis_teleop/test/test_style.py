"""Google documentation, formatting and robot-literal gates."""

from pathlib import Path

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257

from nav_bringup.lint import docstrings, literals


def test_long_private_functions_are_documented():
    """Require nonblank documentation for private definitions over ten lines."""
    paths = [Path(__file__).resolve().parents[1] / 'chassis_teleop']
    violations = docstrings.check_private_paths(paths)
    assert not violations, docstrings.format_violations(violations)


def test_all_long_functions_are_documented():
    """所有作用域及所有命名的长函数均需有非空说明."""
    violations = docstrings.check_long_paths([
        Path(__file__).resolve().parents[1] / 'chassis_teleop'])
    assert not violations, docstrings.format_violations(violations)


def test_flake8():
    """Package and test code follow the ROS Python format."""
    code, errors = main_with_errors(argv=[])
    assert code == 0, '\n'.join(errors)


def test_pep257():
    """Documentation follows Google conventions."""
    assert pep257(argv=['.', 'test', '--convention', 'google']) == 0


def test_api_docs_and_robot_literals():
    """Runtime APIs are documented without robot-specific frame literals."""
    paths = [Path(__file__).resolve().parents[1] / 'chassis_teleop']
    violations = docstrings.check_paths(paths)
    assert not violations, docstrings.format_violations(violations)
    assert not literals.check_paths(paths)
