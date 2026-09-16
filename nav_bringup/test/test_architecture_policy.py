"""Exercise architecture checks against direct and concealed reverse imports."""

import pytest

from nav_bringup.lint.architecture import check_sources


@pytest.mark.parametrize('source', [
    'import nav_tasks.app.runtime',
    'from nav_tasks.app import runtime',
    'from ..app import runtime',
    'from .. import app',
    'def late():\n    from ..app.runtime import create',
    'if False:\n    import nav_tasks.app.runtime',
])
def test_reverse_layer_imports(source):
    """Absolute, relative, deferred and conditional imports all enforce layers."""
    result = check_sources('nav_tasks', {
        'nav_tasks.core.worker': source,
        'nav_tasks.app': '',
        'nav_tasks.app.runtime': '',
    })
    assert any('upward layer' in item for item in result), result


@pytest.mark.parametrize('external', ['rclpy', 'yaml', 'websockets', 'nav_msgs.msg'])
def test_indirect_io_through_unclassified_helper(external):
    """An unclassified helper cannot launder pure-layer I/O."""
    result = check_sources('nav_tasks', {
        'nav_tasks.domain.value': 'from .. import utility',
        'nav_tasks.utility': f'import {external}',
    })
    assert any('pure-layer I/O' in item for item in result), result


@pytest.mark.parametrize('source, target', [
    ('nav_tasks.adapters.ros.reader', 'nav_tasks.nav_site_data.repository'),
    ('nav_tasks.nav_site_data.repository', 'nav_tasks.nav2_client'),
    ('nav_tasks.nav2_client', 'nav_tasks.nav_site_data.repository'),
    ('nav_executor.adapters.transport.client', 'nav_executor.adapters.ros.host'),
    ('nav_executor.adapters.ros.host', 'nav_executor.adapters.transport.client'),
])
def test_indirect_adapter_independence(source, target):
    """Logical adapter groups stay independent through intermediate modules."""
    package = source.split('.')[0]
    result = check_sources(package, {
        source: f'import {package}.helper',
        f'{package}.helper': f'import {target}',
        target: '',
    })
    assert any('adapter independence' in item for item in result), result


def test_parent_initializers_are_dependencies():
    """Importing a child also executes its package initializer."""
    result = check_sources('nav_tasks', {
        'nav_tasks.domain.value': 'from ..helpers.leaf import value',
        'nav_tasks.helpers': 'import yaml',
        'nav_tasks.helpers.leaf': 'value = 1',
    })
    assert any('pure-layer I/O' in item for item in result), result


def test_relative_import_in_initializer():
    """A package initializer resolves relative imports from its own package."""
    result = check_sources('nav_tasks', {
        'nav_tasks.domain': 'from ..app import runtime',
        'nav_tasks.domain.value': '',
        'nav_tasks.app.runtime': '',
    }, packages={'nav_tasks.domain', 'nav_tasks.app'})
    assert any('upward layer' in item for item in result), result


@pytest.mark.parametrize('source', [
    'from nav_tasks.core import state',
    'import nav_tasks.nav2_client',
    'from nav_tasks import nav2_client',
    'import david_nav',
])
def test_executor_public_package_boundary(source):
    """The executor cannot import private task implementation or legacy code."""
    assert check_sources('nav_executor', {'nav_executor.app.runtime': source})


@pytest.mark.parametrize('name', ['create_runtime', 'create_dock_ingress',
                                  'declare_dock_parameters'])
def test_protocol_cannot_import_io_factories_from_facade(name):
    """Pure wire code must not reach ROS through the public factory facade."""
    result = check_sources('nav_executor', {
        'nav_executor.protocol.codec': f'from nav_tasks.api import {name}',
    })
    assert any('pure-layer factory' in item for item in result), result


def test_allowed_composition_and_same_adapter_group():
    """The outer facade composes; ROS backend peers may depend on each other."""
    assert not check_sources('nav_tasks', {
        'nav_tasks.api': 'from .app import runtime',
        'nav_tasks.app.runtime': 'from ..adapters.ros import motion\n'
                                 'from ..nav_site_data import repository',
        'nav_tasks.adapters.ros.motion': 'from ...nav2_client import Nav2Client',
        'nav_tasks.nav2_client': 'from .core import ports\nimport rclpy',
        'nav_tasks.nav_site_data.repository': 'import yaml',
        'nav_tasks.core.ports': 'from ..skills.docking import config',
        'nav_tasks.skills.docking.config': 'from ...domain import values',
        'nav_tasks.domain.values': '',
    })
    assert not check_sources('nav_executor', {
        'nav_executor.protocol.codec': 'from nav_tasks.api import StoredPoi',
        'nav_executor.app.runtime': 'from nav_tasks.api import create_runtime',
    })


def test_cycles_terminate_and_keep_witness():
    """A helper cycle cannot hide an upward edge or cause unbounded traversal."""
    result = check_sources('nav_tasks', {
        'nav_tasks.domain.value': 'from .. import first',
        'nav_tasks.first': 'from . import second',
        'nav_tasks.second': 'from . import first\nfrom .app import runtime',
        'nav_tasks.app.runtime': '',
    })
    assert any('nav_tasks.domain.value -> nav_tasks.first -> nav_tasks.second -> '
               'nav_tasks.app.runtime' in item for item in result), result


@pytest.mark.parametrize('left, right', [('protocol', 'skills'), ('skills', 'protocol')])
def test_protocol_and_skills_are_independent(left, right):
    """The protocol and skills share a layer but cannot import each other."""
    result = check_sources('nav_tasks', {
        f'nav_tasks.{left}.worker': f'from ..{right} import other',
        f'nav_tasks.{right}.other': '',
    })
    assert any('protocol/skills independence' in item for item in result), result


def test_policy_rejects_incorrect_package_scope():
    """A misconfigured caller cannot accidentally scan nothing relevant."""
    with pytest.raises(ValueError, match='covers'):
        check_sources('unrelated', {})
    with pytest.raises(ValueError, match='belong'):
        check_sources('nav_tasks', {'nav_executor.core': ''})
    with pytest.raises(ValueError, match='escapes'):
        check_sources('nav_tasks', {'nav_tasks.api': 'from ... import bad'})


def test_filesystem_scan_resolves_initializer_imports(tmp_path):
    """The directory entry point distinguishes packages and regular modules."""
    from nav_bringup.lint.architecture import check_package
    package = tmp_path / 'nav_tasks'
    (package / 'domain').mkdir(parents=True)
    (package / '__init__.py').write_text('')
    (package / 'domain' / '__init__.py').write_text('from ..app import runtime')
    (package / 'app').mkdir()
    (package / 'app' / 'runtime.py').write_text('')
    assert any('upward layer' in item for item in check_package(package))
