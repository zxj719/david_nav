"""检查已审查的中文摘要，并冻结兼容的接口检查帮助输出."""

import ast
from pathlib import Path
import re

import pytest

from nav_perception_iface import contract_cli


@pytest.mark.parametrize('path, qualified_name', [
    ('configuration.py', 'load_contract_profile'),
    ('contract_cli.py', '<module>'),
    ('contract_cli.py', 'main'),
    ('contract_probe.py', '<module>'),
    ('contract_probe.py', 'ContractProbe'),
    ('contract_probe.py', 'ContractProbe.__init__'),
    ('contract_probe.py', 'ContractProbe.__enter__'),
    ('contract_probe.py', 'ContractProbe.__exit__'),
    ('contract_probe.py', 'ContractProbe.close'),
    ('contract_probe.py', 'ContractProbe.spin'),
    ('contract_probe.py', 'ContractProbe.localized'),
    ('contract_probe.py', 'ContractProbe.sample'),
    ('health_core.py', '<module>'),
    ('health_core.py', 'HealthPolicy'),
    ('health_core.py', 'HealthPolicy.__post_init__'),
    ('health_core.py', 'PoseSample'),
    ('health_core.py', 'PoseSample.__post_init__'),
    ('health_core.py', 'HealthReport'),
    ('health_core.py', 'HealthMonitor'),
    ('health_core.py', 'HealthMonitor.__init__'),
    ('health_core.py', 'HealthMonitor.history_size'),
    ('health_core.py', 'HealthMonitor.sample'),
    ('health_node.py', '<module>'),
    ('health_node.py', 'LocalizationMonitor'),
    ('health_node.py', 'LocalizationMonitor.__init__'),
    ('health_node.py', 'LocalizationMonitor.destroy_node'),
    ('health_node.py', 'main'),
])
def test_reviewed_documentation_has_chinese_summary(path, qualified_name):
    """逐名保护中文摘要，分节和语义仍需代码审查.

    Args:
        path: 包内源码文件名。
        qualified_name: 模块标记或类与函数的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_perception_iface'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    if qualified_name != '<module>':
        for name in qualified_name.split('.'):
            matches = [child for child in node.body
                       if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                       and child.name == name]
            assert len(matches) == 1, qualified_name
            node = matches[0]
    doc = ast.get_docstring(node) or ''
    assert doc.strip(), qualified_name
    assert re.search('[\u4e00-\u9fff]', doc.splitlines()[0]), qualified_name


def test_help_preserves_pretranslation_output_without_observation(monkeypatch, capsys):
    """中文模块说明不能改变既有帮助输出，帮助路径也不能开始加载档案.

    Args:
        monkeypatch: pytest 替换进程名称、显示宽度与禁止进入的读取边界。
        capsys: pytest 标准输出捕获器。
    """
    monkeypatch.setenv('COLUMNS', '80')
    monkeypatch.setattr(contract_cli.sys, 'argv', ['nav_interface_check'])
    monkeypatch.setattr(contract_cli, 'load_contract_profile',
                        lambda *_: pytest.fail('help must not read a profile'))
    with pytest.raises(SystemExit) as result:
        contract_cli.main(['--help'])
    assert result.value.code == 0
    output = capsys.readouterr()
    expected = (Path(__file__).parent / 'golden' / 'interface_check_help.txt').read_text(
        encoding='utf-8')
    assert output.out == expected
    assert output.err == ''
