"""Pin the SPEC R10 legacy warnings to the retained executable defaults."""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / 'david_nav'


def test_waypoint_startup_reports_v1_and_retains_its_default_action():
    """Inspect only the selected log/binding lines; never execute the launcher."""
    lines = (ROOT / 'start_waypoint_nav_headless.sh').read_text().splitlines()
    selected = [line for line in lines if '前台起 navigation_executor(' in line]
    assert selected == ["  log '前台起 navigation_executor(nav WS v1 协议)。Ctrl+C 全栈退出。'"]
    assert any('ACTION_NAME="${NAV_ACTION_NAME:-/waypoint_follower/navigate}"' == line
               for line in lines)


def test_dead_reckoning_is_documented_as_explicit_nondefault_backend():
    """The arbitrary incoming pose is ignored by this retained opt-in backend."""
    tree = ast.parse((ROOT / 'david_nav/dead_reckoning_executor.py').read_text())
    document = ast.get_docstring(tree)
    assert '默认 /waypoint_follower/navigate' in document
    assert '显式选择 /dead_reckoning/navigate' in document
    assert '忽略 target_pose' in document


def test_semantic_navigator_has_deprecation_and_actual_replacement_guidance():
    """Warn without silently changing historical robot-specific defaults."""
    tree = ast.parse((ROOT / 'david_nav/semantic_pose_navigator.py').read_text())
    document = ast.get_docstring(tree) or ''
    assert 'DEPRECATED' in document
    assert 'nav_tasks.api' in document
    values = {node.targets[0].id: ast.literal_eval(node.value)
              for node in tree.body if isinstance(node, ast.Assign)
              and isinstance(node.targets[0], ast.Name)
              and node.targets[0].id in ('DEFAULT_POSE_FILE', 'DEFAULT_ROBOT_BASE_FRAME')}
    assert values == {'DEFAULT_POSE_FILE': '/home/unitree/semantic_map/named_poses.json',
                      'DEFAULT_ROBOT_BASE_FRAME': 'base'}
