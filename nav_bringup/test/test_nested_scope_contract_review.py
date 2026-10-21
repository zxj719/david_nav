"""保护非平凡嵌套函数说明，不执行诊断、信号、启动或删除."""

from nav_bringup.navcfg.live_inputs import parameter_sections


def test_parameter_flattening_preserves_leaf_references():
    """展开只重建字典和参数键，列表等叶子仍是调用方原对象."""
    values = [1, 2]
    tree = {'scope': {'node': {'ros__parameters': {'nested': {'values': values}}}}}
    result = parameter_sections(tree)
    assert result == {'/scope/node': {'nested.values': [1, 2]}}
    assert result['/scope/node']['nested.values'] is values
    values.append(3)
    assert result['/scope/node']['nested.values'] == [1, 2, 3]
