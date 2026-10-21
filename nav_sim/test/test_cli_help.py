"""保护仿真工具的原有帮助输出，帮助路径不启动运行时."""

import importlib
from pathlib import Path
import sys

import pytest


@pytest.mark.parametrize('name, program', [
    ('fake_agent_server', 'fake_agent_server'),
    ('selftest_launch', 'selftest_launch.py'),
])
def test_help_keeps_original_output_without_runtime(name, program, monkeypatch, capsys):
    """帮助必须逐字不变，且在域检查前退出而不进入服务或启动流程.

    Args:
        name: 可单独显示帮助的模块名。
        program: 与安装入口或模块运行对应的稳定程序名。
        monkeypatch: pytest 环境和属性替换器。
        capsys: pytest 标准输出捕获器。
    """
    module = importlib.import_module('nav_sim.' + name)
    monkeypatch.setenv('COLUMNS', '80')
    monkeypatch.setattr(sys, 'argv', [program])
    monkeypatch.setattr(module, 'require_sandbox',
                        lambda *_: pytest.fail('help must exit before runtime guards'))
    with pytest.raises(SystemExit) as result:
        module.main(['--help'])
    assert result.value.code == 0
    output = capsys.readouterr()
    expected = (Path(__file__).parent / 'golden' / (name + '_help.txt')).read_text(
        encoding='utf-8')
    assert output.out == expected
    assert output.err == ''
