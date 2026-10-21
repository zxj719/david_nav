"""冻结性能工具的原有帮助输出，显示帮助时不开始采样."""

from pathlib import Path
import sys

import pytest

from nav_executor.app import idle_profile


def test_profile_help_is_unchanged_and_never_starts_sampling(monkeypatch, capsys):
    """显示帮助应在系统计数器读取、文件创建和采样之前退出.

    Args:
        monkeypatch: pytest 进程参数、环境和系统查询替换器。
        capsys: pytest 标准输出捕获器。
    """
    monkeypatch.setenv('COLUMNS', '80')
    monkeypatch.setattr(sys, 'argv', ['nav_executor_profile'])
    monkeypatch.setattr(idle_profile.os, 'sysconf',
                        lambda *_: pytest.fail('help must not query sampling configuration'))
    with pytest.raises(SystemExit) as result:
        idle_profile.main(['--help'])
    assert result.value.code == 0
    output = capsys.readouterr()
    expected = (Path(__file__).parent / 'golden' / 'profile_help.txt').read_text(
        encoding='utf-8')
    assert output.out == expected
    assert output.err == ''
