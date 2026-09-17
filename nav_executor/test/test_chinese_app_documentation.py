"""检查执行器装配说明的中文摘要，并冻结性能工具的原有帮助输出."""

import ast
from pathlib import Path
import re
import sys

import pytest

from nav_executor.app import idle_profile


@pytest.mark.parametrize('path, qualified_name', [
    ('adapters/ros/__init__.py', '<module>'),
    ('adapters/ros/host.py', '<module>'),
    ('adapters/ros/host.py', 'RosHost'),
    ('adapters/ros/host.py', 'RosHost.__init__'),
    ('adapters/ros/host.py', 'RosHost.open'),
    ('adapters/ros/host.py', 'RosHost.spin_once'),
    ('adapters/ros/host.py', 'RosHost.quiesce'),
    ('adapters/ros/host.py', 'RosHost.close'),
    ('app/__init__.py', '<module>'),
    ('app/cli.py', '<module>'),
    ('app/cli.py', 'build_help_text'),
    ('app/cli.py', 'parse_arguments'),
    ('app/cli.py', 'main'),
    ('app/config.py', '<module>'),
    ('app/config.py', 'ExecutorConfig'),
    ('app/config.py', 'ExecutorConfig.legacy_values'),
    ('app/config.py', 'ExecutorConfig.task_config'),
    ('app/config.py', 'ExecutorConfig.ws_options'),
    ('app/config.py', 'ResolvedConfig'),
    ('app/config.py', 'ResolvedConfig.task_config'),
    ('app/config.py', 'ResolvedConfig.log_line'),
    ('app/config.py', 'load_headless_executor_section'),
    ('app/config.py', 'load_configuration'),
    ('app/dock_cli.py', '<module>'),
    ('app/dock_cli.py', 'main'),
    ('app/idle_profile.py', '<module>'),
    ('app/idle_profile.py', 'main'),
    ('app/logging.py', '<module>'),
    ('app/logging.py', 'OutboundAudit'),
    ('app/logging.py', 'OutboundAudit.__init__'),
    ('app/logging.py', 'OutboundAudit.on_sending'),
    ('app/logging.py', 'OutboundAudit.on_sent'),
    ('app/runtime.py', '<module>'),
    ('app/runtime.py', 'run'),
    ('app/signals.py', '<module>'),
    ('app/signals.py', 'run_with_signals'),
    ('app/status.py', '<module>'),
    ('app/status.py', 'StatusPoseSource'),
    ('app/status.py', 'StatusPoseSource.current_map_pose'),
    ('app/status.py', 'format_status_line'),
    ('app/status.py', 'print_status_loop'),
])
def test_reviewed_app_documentation_has_chinese_summary(path, qualified_name):
    """摘要检查不替代代码语义审查或异常路径证明.

    Args:
        path: 包内相对文件路径。
        qualified_name: 模块标记或定义的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_executor'
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
