"""本批已审说明的门禁与纯参数边界：不启动子进程、不创建运行目录、不碰 ROS."""

import ast
from pathlib import Path
import re

import pytest

import nav_bringup
from nav_bringup.navrun import event_tail, teleop_options

MODULES = ('navrun/cli.py', 'navrun/teleop_options.py', 'navrun/up.py',
           'navrun/check.py', 'navrun/event_tail.py')


@pytest.mark.parametrize('name', MODULES)
def test_reviewed_summaries_are_chinese(name):
    """检查本批模块的现有摘要，不声称语言存在性可以证明说明语义."""
    path = Path(nav_bringup.__file__).parent / name
    tree = ast.parse(path.read_text(encoding='utf-8'))
    missing = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            doc = ast.get_docstring(node)
            if doc and re.search(r'[一-鿿]', doc.splitlines()[0]) is None:
                missing.append((getattr(node, 'lineno', 1), getattr(node, 'name', '<module>')))
    assert not missing, missing


def test_overrides_only_carry_what_was_given():
    """没写的字段不会被补默认值，冻结出来的就是操作者显式给的那几项."""
    assert teleop_options.TeleopOptions().values('teleop') == {}
    assert teleop_options.TeleopOptions(layout='wasd').values('teleop') == {'layout': 'wasd'}


@pytest.mark.parametrize('mode', ['tour', 'precise', 'chassis'])
def test_overrides_outside_teleop_are_refused(mode):
    """键盘覆盖放错模式时在资源发现之前就拒绝."""
    with pytest.raises(ValueError):
        teleop_options.TeleopOptions(layout='wasd').values(mode)


@pytest.mark.parametrize('python', ['python3', '/usr/bin/pyt\0hon3'])
def test_helper_command_requires_an_absolute_clean_interpreter(python):
    """解释器必须是绝对路径且不含 NUL；拒绝发生在拼命令阶段."""
    with pytest.raises(ValueError):
        event_tail.command(python)


def test_helper_command_keeps_run_identity_out_of_the_argument_vector():
    """运行身份走环境变量，不出现在命令行上."""
    command = event_tail.command('/usr/bin/python3')
    assert command[0] == '/usr/bin/python3'
    assert not any('nav_runs' in part for part in command)
