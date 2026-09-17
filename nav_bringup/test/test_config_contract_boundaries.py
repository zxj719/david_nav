"""配置和诊断说明的纯对象/临时文件边界回归."""

from nav_bringup.doctor import checks
from nav_bringup.navcfg import render, schema


def test_catalog_defaults_keep_nested_reference():
    """默认值汇总只创建外层字典，不声称深度隔离."""
    parsed = schema.parse_schema({'schema_version': 1, 'package': 'fixture', 'parameters': {
        'settings': {'type': 'map', 'description': 'fixture', 'default': {'enabled': True}},
    }}, 'fixture')
    catalog = schema.ParameterCatalog([parsed])
    defaults = catalog.defaults()
    defaults['settings']['enabled'] = False
    assert catalog.get('settings').default == {'enabled': False}


def test_mode_overlay_keeps_source_mapping_reference():
    """冻结数据类不意味着嵌套映射深度不可变."""
    profile = {'limit': 1}
    mode = render.parse_mode(
        {'schema_version': 1, 'mode': 'fixture', 'profile': profile}, 'fixture')
    profile['limit'] = 2
    assert mode.profile == {'limit': 2}


def test_render_does_not_expand_markers_in_injected_values():
    """仅替换基础树标记，不把注入的字符串当作第二轮模板."""
    binding = render.parse_bindings(
        {'schema_version': 1, 'bindings': {'label': ['node']}}, 'fixture')
    result = render.render({'node': '<bind:label>'}, binding, {'label': '<bind:other>'})
    assert result == {'node': '<bind:other>'}


def test_environment_check_does_not_parse_profile_xml(tmp_path):
    """配置文件存在可得到局部 OK，但不证明 XML 有效或 DDS 可见."""
    profile = tmp_path / 'bad.xml'
    profile.write_text('not XML', encoding='utf-8')
    context = checks.DoctorContext(
        repo_dir=tmp_path, install_dir=tmp_path, environ={
            'ROS_DISTRO': 'humble', 'ROS_DOMAIN_ID': '99',
            'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile)})
    assert checks.check_ros_environment(context).status is checks.Status.OK
