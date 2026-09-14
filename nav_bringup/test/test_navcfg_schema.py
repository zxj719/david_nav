"""Tests for package parameter schemas and their aggregation."""

import os
from pathlib import Path

import pytest

from nav_bringup.navcfg import schema

PACKAGE_DIR = Path(__file__).resolve().parents[1]


def _schema(parameters, package='demo_pkg'):
    return {'schema_version': 1, 'package': package, 'parameters': parameters}


def _spec(**fields):
    values = {'type': 'float', 'description': 'demo'}
    values.update(fields)
    return values


def _single(**fields):
    return schema.parse_schema(_schema({'a.b': _spec(**fields)}), 'demo.yaml').parameters[0]


def test_parse_schema_reads_every_field():
    """All declaration fields are parsed and keys are sorted."""
    parsed = schema.parse_schema(_schema({
        'robot.name': _spec(type='string', required=True),
        'limits.max_vel_x': _spec(
            unit='m/s', default=0.15, range=[0.0, None], tuning='docs/tuning.md#speed'),
        'record.storage': _spec(type='string', default='sqlite3', choices=['sqlite3']),
        'record.light_topics': _spec(type='list', items='string', default=[]),
    }), 'demo/config/schema.yaml')

    assert parsed.package == 'demo_pkg'
    assert parsed.path == Path('demo/config/schema.yaml')
    assert [spec.key for spec in parsed.parameters] == [
        'limits.max_vel_x', 'record.light_topics', 'record.storage', 'robot.name']
    speed, topics, storage, name = parsed.parameters
    assert (speed.value_type, speed.unit, speed.default, speed.has_default) == (
        'float', 'm/s', 0.15, True)
    assert (speed.value_range, speed.tuning) == ((0.0, None), 'docs/tuning.md#speed')
    assert (topics.items, topics.default) == ('string', [])
    assert storage.choices == ('sqlite3',)
    assert name.required and not name.has_default


@pytest.mark.parametrize('data, message', [
    ('not a mapping', 'must be a mapping'),
    ({'schema_version': 2, 'package': 'p', 'parameters': {}}, 'schema_version'),
    ({'schema_version': 1, 'package': 'Bad-Name', 'parameters': {}}, 'package must be'),
    ({'schema_version': 1, 'package': 'p', 'parameters': {}, 'extra': 1}, 'unknown top-level'),
    (_schema({'Limits.MaxVel': _spec()}), 'invalid key'),
    (_schema({'a.b': 'float'}), 'declaration must be a mapping'),
    (_schema({'a.b': _spec(type='double')}), 'type must be one of'),
    (_schema({'a.b': _spec(description=' ')}), 'description is required'),
    (_schema({'a.b': _spec(colour='red')}), 'unknown fields'),
    (_schema({'a.b': _spec(required='yes')}), 'required must be'),
    (_schema({'a.b': _spec(required=True, default=1.0)}), 'cannot have a default'),
    (_schema({'a.b': _spec(type='string', range=[0, 1])}), 'range is only allowed'),
    (_schema({'a.b': _spec(range=[2.0, 1.0])}), 'minimum exceeds maximum'),
    (_schema({'a.b': _spec(range=[0.0])}), 'range must be'),
    (_schema({'a.b': _spec(default=-1.0, range=[0.0, None])}), 'invalid default'),
    (_schema({'a.b': _spec(type='string', choices=[])}), 'non-empty list'),
    (_schema({'a.b': _spec(type='int', choices=[1, 'two'])}), 'is not a int'),
    (_schema({'a.b': _spec(items='string')}), 'needs type list'),
])
def test_malformed_schemas_are_rejected(data, message):
    """Every format violation raises SchemaError with a pointed message."""
    with pytest.raises(schema.SchemaError, match=message):
        schema.parse_schema(data, 'demo.yaml')


def test_null_default_means_unset():
    """A null default is accepted for any type."""
    assert _single(type='string', default=None).has_default


def test_check_value():
    """Types, ranges, choices and list items are checked."""
    int_spec = _single(type='int', range=[1, 10])
    assert schema.check_value(int_spec, 5) == []
    assert 'expected int' in schema.check_value(int_spec, True)[0]
    assert 'expected int' in schema.check_value(int_spec, 5.0)[0]
    assert schema.check_value(int_spec, 11) == ['a.b: 11 outside range [1, 10]']

    float_spec = _single(range=[0.0, None])
    assert schema.check_value(float_spec, 3) == []
    assert schema.check_value(float_spec, False) != []
    assert schema.check_value(float_spec, -0.5) == ['a.b: -0.5 outside range [0.0, inf]']

    list_spec = _single(type='list', items='string')
    assert schema.check_value(list_spec, ['/scan', 3]) == ['a.b[1]: expected string, got int 3']

    choice_spec = _single(type='string', choices=['sqlite3'])
    assert schema.check_value(choice_spec, 'mcap') == ["a.b: 'mcap' is not one of ['sqlite3']"]
    assert schema.check_value(_single(type='map'), {'x': 1}) == []


def test_catalog_merges_schemas():
    """Specs, packages and defaults of several packages are combined."""
    first = schema.parse_schema(
        _schema({'limits.max_vel_x': _spec(default=0.15)}, package='pkg_a'), 'a.yaml')
    second = schema.parse_schema(
        _schema({'chassis.cmd_topic': _spec(type='string', default='/cmd_vel'),
                 'robot.name': _spec(type='string', required=True)}, package='pkg_b'),
        'b.yaml')

    catalog = schema.ParameterCatalog([second, first])

    assert catalog.packages == ('pkg_a', 'pkg_b')
    assert [spec.key for spec in catalog.specs] == [
        'chassis.cmd_topic', 'limits.max_vel_x', 'robot.name']
    assert catalog.get('limits.max_vel_x').package == 'pkg_a'
    assert catalog.get('limits.max_vel_y') is None
    assert catalog.defaults() == {'chassis.cmd_topic': '/cmd_vel', 'limits.max_vel_x': 0.15}


def test_catalog_rejects_conflicts():
    """Duplicate packages, duplicate keys and nested keys are rejected."""
    first = schema.parse_schema(
        _schema({'limits.max_vel_x': _spec()}, package='pkg_a'), 'a.yaml')
    duplicate = schema.parse_schema(
        _schema({'limits.max_vel_x': _spec()}, package='pkg_c'), 'c.yaml')
    nested = schema.parse_schema(
        _schema({'limits.max_vel_x.hard': _spec()}, package='pkg_c'), 'c.yaml')

    with pytest.raises(schema.SchemaError, match='declared by both pkg_a and pkg_c'):
        schema.ParameterCatalog([first, duplicate])
    with pytest.raises(schema.SchemaError, match='nested inside limits.max_vel_x'):
        schema.ParameterCatalog([first, nested])
    with pytest.raises(schema.SchemaError, match='two schemas'):
        schema.ParameterCatalog([first, first])


def test_check_profile():
    """Invalid values and missing required keys are errors; unknown keys warn."""
    catalog = schema.ParameterCatalog([schema.parse_schema(_schema({
        'limits.max_vel_x': _spec(range=[0.0, 1.0]),
        'robot.name': _spec(type='string', required=True),
        'sensors.scan_from_cloud': _spec(type='map', default={}),
        'site.server_url': _spec(type='string', default=None),
    }), 'demo.yaml')])

    issues = catalog.check_profile({
        'schema_version': 1,
        'limits': {'max_vel_x': 1.5, 'max_vel_y': 0.0},
        'sensors': {'scan_from_cloud': {'enabled': False}},
        'site': {'server_url': None},
        'teleop': {'layout': 'wasd'},
    })

    assert [(issue.key, issue.severity) for issue in issues] == [
        ('limits.max_vel_x', 'error'),
        ('limits.max_vel_y', 'warn'),
        ('teleop', 'warn'),
        ('robot.name', 'error'),
    ]
    assert issues[-1].message == 'robot.name: required by demo_pkg but not set'
    assert catalog.check_profile({'robot': {'name': 'demo'}, 'limits': {'max_vel_x': 0.15}}) == []


def test_find_installed_schemas_prefers_overlay(tmp_path):
    """The first prefix that installs a package wins."""
    overlay = tmp_path / 'overlay'
    underlay = tmp_path / 'underlay'
    for prefix, package in ((overlay, 'pkg_a'), (underlay, 'pkg_a'), (underlay, 'pkg_b')):
        path = prefix / 'share' / package / 'config' / 'schema.yaml'
        path.parent.mkdir(parents=True)
        path.write_text('schema_version: 1\n', encoding='utf-8')
    environ = {'AMENT_PREFIX_PATH': os.pathsep.join(
        [str(overlay), str(underlay), str(tmp_path / 'missing')])}

    prefixes = schema.ament_prefixes(environ)

    assert prefixes == [overlay, underlay, tmp_path / 'missing']
    assert schema.find_installed_schemas(prefixes) == [
        overlay / 'share' / 'pkg_a' / 'config' / 'schema.yaml',
        underlay / 'share' / 'pkg_b' / 'config' / 'schema.yaml',
    ]


def test_find_source_schemas_skips_colcon_ignore(tmp_path):
    """Packages with a schema are found; ignored trees and schema-less packages are not."""
    src = tmp_path / 'src'
    for relative in ('pkg_a', 'group/pkg_b', 'third_party/old_pkg'):
        package = src / relative
        (package / 'config').mkdir(parents=True)
        (package / 'package.xml').write_text('<package/>', encoding='utf-8')
        (package / 'config' / 'schema.yaml').write_text('schema_version: 1\n', encoding='utf-8')
    (src / 'third_party' / 'COLCON_IGNORE').write_text('', encoding='utf-8')
    (src / 'no_schema').mkdir()
    (src / 'no_schema' / 'package.xml').write_text('<package/>', encoding='utf-8')

    assert schema.find_source_schemas(src) == [
        src / 'group' / 'pkg_b' / 'config' / 'schema.yaml',
        src / 'pkg_a' / 'config' / 'schema.yaml',
    ]


def test_load_schema_reports_unreadable_files(tmp_path):
    """Broken YAML and missing files raise SchemaError."""
    broken = tmp_path / 'schema.yaml'
    broken.write_text('parameters: [unclosed\n', encoding='utf-8')

    with pytest.raises(schema.SchemaError, match='cannot read schema'):
        schema.load_schema(broken)
    with pytest.raises(schema.SchemaError, match='cannot read schema'):
        schema.load_schema(tmp_path / 'missing.yaml')


def test_nav_bringup_schema_is_valid():
    """The schema shipped with this package loads."""
    catalog = schema.load_catalog([PACKAGE_DIR / 'config' / 'schema.yaml'])

    assert catalog.packages == ('nav_bringup',)
    assert catalog.get('record.storage').default == 'sqlite3'


def test_workspace_schemas_merge_without_conflicts():
    """All schemas in the source tree of this workspace merge into one catalog."""
    paths = schema.find_source_schemas(PACKAGE_DIR.parent)

    assert PACKAGE_DIR / 'config' / 'schema.yaml' in paths
    assert 'nav_bringup' in schema.load_catalog(paths).packages
