"""Keep every sandbox guard aligned with driver and component-container entry points."""

import ast
from pathlib import Path
import re

import pytest

SOURCE = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('relative', [
    'nav_bringup/sandbox_tests/run_sandbox_checks.sh',
    'nav_bringup/sandbox_tests/conftest.py',
    'wheeltec_driver/sandbox_tests/run_sandbox_checks.sh',
])
def test_guard_matches_driver_and_container_executables(relative: str) -> None:
    """Renaming or composition must not hide a live hardware/navigation stack."""
    source = (SOURCE / relative).read_text()
    if relative.endswith('.sh'):
        pattern = re.search(r"readonly LIVE_PATTERN='([^']+)'", source).group(1)
    else:
        assignment = next(node for node in ast.parse(source).body
                          if isinstance(node, ast.Assign)
                          and any(isinstance(target, ast.Name) and target.id == 'LIVE_PATTERN'
                                  for target in node.targets))
        pattern = ast.literal_eval(assignment.value)
    for executable in ('wheeltec_robot_node', 'wheeltec_driver_node',
                       'component_container', 'component_container_mt',
                       'component_container_isolated'):
        assert re.search(pattern, f'/install/wheeltec_driver/lib/wheeltec_driver/{executable}')
