"""Keep the selected WebSocket distribution in installation metadata, not just prose."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('package', ['nav_executor', 'nav_sim'])
def test_python_metadata_declares_selected_websocket_release(package):
    """The pip-selected runtime must not silently depend on an unrelated apt version."""
    tree = ast.parse((ROOT.parent / package / 'setup.py').read_text())
    calls = [node for node in ast.walk(tree)
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
             and node.func.id == 'setup']
    assert len(calls) == 1
    requirements = next(item.value for item in calls[0].keywords
                        if item.arg == 'install_requires')
    assert 'websockets==16.1' in ast.literal_eval(requirements)


@pytest.mark.parametrize('package', ['nav_executor', 'nav_sim'])
def test_ros_metadata_does_not_reintroduce_the_old_apt_websocket_selection(package):
    """Leave the selected pip distribution to Python metadata, including tests."""
    document = ET.parse(ROOT.parent / package / 'package.xml').getroot()
    dependencies = [element.text for element in document if element.tag.endswith('depend')]
    assert 'python3-websockets' not in dependencies
