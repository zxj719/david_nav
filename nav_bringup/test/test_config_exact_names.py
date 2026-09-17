"""配置声明名与绑定标记必须完整匹配，不接收尾部换行."""

import pytest

from nav_bringup.navcfg import render, schema


@pytest.mark.parametrize('suffix', ['\n', '\r\n'])
def test_schema_package_rejects_trailing_newline(suffix):
    """包名不能包含正则结束锚点可能略过的换行."""
    data = {'schema_version': 1, 'package': 'demo' + suffix, 'parameters': {}}
    with pytest.raises(schema.SchemaError, match='package must be'):
        schema.parse_schema(data, 'fixture')


@pytest.mark.parametrize('suffix', ['\n', '\r\n'])
def test_schema_key_rejects_trailing_newline(suffix):
    """点分声明键的末尾必须确实结束，不能创建不可正常查找的键."""
    data = {'schema_version': 1, 'package': 'demo', 'parameters': {
        'limits.speed' + suffix: {'type': 'float', 'description': 'fixture'}}}
    with pytest.raises(schema.SchemaError, match='invalid key'):
        schema.parse_schema(data, 'fixture')


@pytest.mark.parametrize('suffix', ['\n', '\r\n'])
def test_binding_key_rejects_trailing_newline(suffix):
    """绑定表与 schema 使用同样严格的完整键语法."""
    data = {'schema_version': 1, 'bindings': {'limits.speed' + suffix: ['node/speed']}}
    with pytest.raises(render.RenderError, match='invalid profile key'):
        render.parse_bindings(data, 'fixture')


@pytest.mark.parametrize('suffix', ['\n', '\r\n'])
def test_binding_marker_rejects_trailing_newline(suffix):
    """标记识别不能把带额外换行的值当成规范绑定标记."""
    with pytest.raises(render.RenderError, match='malformed binding marker'):
        render.find_markers({'node': '<bind:limits.speed>' + suffix})
