"""验证模式覆盖不能通过子路径或列表下标别名绕开绑定保护."""

import copy

import pytest

from nav_bringup.navcfg import render


@pytest.mark.parametrize('value, suffix', [
    ({'limit': 1}, '/limit'),
    ({'limit': 1}, '/new_key'),
    ([1, 2], '/0'),
    ([1, 2], '/00'),
])
def test_cannot_override_descendant_of_bound_value(value, suffix):
    """整体绑定的值必须整体保持一致，包括新增字典子键."""
    base = {'first': '<bind:settings>', 'second': '<bind:settings>'}
    bindings = render.parse_bindings(
        {'schema_version': 1, 'bindings': {'settings': ['first', 'second']}}, 'fixture')
    profile = {'settings': value}
    snapshot = copy.deepcopy(profile)
    mode = render.ModeOverlay('fixture', {}, {'first' + suffix: 99})
    with pytest.raises(render.RenderError, match='covers bound parameters'):
        render.render(base, bindings, profile, mode)
    assert profile == snapshot
    assert base == {'first': '<bind:settings>', 'second': '<bind:settings>'}


@pytest.mark.parametrize('index', ['00', '000', '\u0660'])
def test_list_index_alias_cannot_overwrite_bound_scalar(index):
    """列表路径按实际下标而非文本判断，前导零与 Unicode 数字均不可绕过."""
    base = {'limits': ['<bind:limit>', 2], 'second': '<bind:limit>'}
    bindings = render.parse_bindings(
        {'schema_version': 1, 'bindings': {'limit': ['limits/0', 'second']}}, 'fixture')
    mode = render.ModeOverlay('fixture', {}, {'limits/' + index: 99})
    with pytest.raises(render.RenderError, match='covers bound parameters'):
        render.render(base, bindings, {'limit': 1}, mode)


def test_ancestor_through_index_alias_cannot_overwrite_binding():
    """列表下标别名下的父容器同样受保护."""
    base = {'nodes': [{'limit': '<bind:limit>'}]}
    bindings = render.parse_bindings(
        {'schema_version': 1, 'bindings': {'limit': ['nodes/0/limit']}}, 'fixture')
    mode = render.ModeOverlay('fixture', {}, {'nodes/00': {'limit': 99}})
    with pytest.raises(render.RenderError, match='covers bound parameters'):
        render.render(base, bindings, {'limit': 1}, mode)


def test_numeric_dictionary_keys_are_not_list_aliases():
    """字典的 0 与 00 是不同键，不能错误归一化或误拒绝."""
    base = {'limits': {'0': '<bind:limit>', '00': 2}}
    bindings = render.parse_bindings(
        {'schema_version': 1, 'bindings': {'limit': ['limits/0']}}, 'fixture')
    mode = render.ModeOverlay('fixture', {}, {'limits/00': 99})
    assert render.render(base, bindings, {'limit': 1}, mode) == {
        'limits': {'0': 1, '00': 99}}


def test_unbound_list_alias_and_similar_key_prefix_remain_allowed():
    """未绑定列表项和只有相似文本前缀的键仍可覆盖."""
    base = {'limits': ['<bind:limit>', 2], 'limits_extra': {'value': 3}}
    bindings = render.parse_bindings(
        {'schema_version': 1, 'bindings': {'limit': ['limits/0']}}, 'fixture')
    mode = render.ModeOverlay('fixture', {}, {'limits/01': 99, 'limits_extra/value': 88})
    assert render.render(base, bindings, {'limit': 1}, mode) == {
        'limits': [1, 99], 'limits_extra': {'value': 88}}
