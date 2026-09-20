"""本批已审说明的门禁与静态检查边界：只解析源码文本，从不导入或执行被检查的代码."""

import ast
from pathlib import Path
import re
import textwrap

import pytest

import nav_bringup
from nav_bringup.lint import docstrings, literals

MODULES = ('lint/docstrings.py', 'lint/literals.py', 'lint/architecture.py',
           'lint/node_attributes.py')


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


def test_checking_a_module_never_executes_it(tmp_path):
    """被检查的源码只解析不执行：其导入与顶层副作用都不会发生."""
    marker = tmp_path / 'executed.txt'
    source = textwrap.dedent('''
        """Module summary."""

        import pathlib

        pathlib.Path({marker!r}).write_text("executed")
    ''').format(marker=str(marker))
    assert docstrings.check_source(source) == []
    assert not marker.exists()


def test_sections_are_checked_for_presence_not_truthfulness():
    """分节齐全即通过：说明与实现是否相符不在本检查范围内."""
    source = textwrap.dedent('''
        """Module summary."""


        def scale(value):
            """Return the value unchanged.

            Args:
                value: Anything at all.

            Returns:
                The doubled value.
            """
            return value
    ''')
    assert docstrings.check_source(source) == []


def test_missing_sections_are_reported_with_their_location():
    """缺哪几节就报哪几节，且都带上定义所在行，便于直接定位."""
    source = textwrap.dedent('''
        """Module summary."""


        def scale(value):
            """Scale one value."""
            return value
    ''')
    violations = docstrings.check_source(source, 'fixture.py')
    assert [item.message for item in violations] == [
        'missing Args: section', 'missing Returns: section']
    assert all(item.line == 5 for item in violations)
    assert 'fixture.py' in str(violations[0])


def test_only_static_literals_are_found():
    """只看静态字符串常量：拼接与 f-string 的变量部分发现不了."""
    source = textwrap.dedent('''
        """Module summary."""

        direct = "base_link"
        joined = "base" + "_link"
        prefix = "base"
        formatted = f"{prefix}_link"
    ''')
    found = literals.check_source(source, 'fixture.py', ('base_link',), ())
    assert [item.value for item in found] == ['base_link']


def test_docstrings_are_not_scanned_for_forbidden_literals():
    """说明文本里出现禁用名字不算违例，只有代码里的常量才算."""
    source = '"""这段说明提到 base_link，不应当被判为违例."""\n'
    assert literals.check_source(source, 'fixture.py', ('base_link',), ()) == []
