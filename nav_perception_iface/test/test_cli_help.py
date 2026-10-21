"""冻结接口检查器的兼容帮助输出，帮助路径不开始观察."""

from pathlib import Path

import pytest

from nav_perception_iface import contract_cli


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
