"""静态常量检查的边界：只解析源码文本，从不导入或执行被检查的代码."""

import textwrap

from nav_bringup.lint import literals


def test_checking_a_module_never_executes_it(tmp_path):
    """被检查的源码只解析不执行：其导入与顶层副作用都不会发生."""
    marker = tmp_path / 'executed.txt'
    source = textwrap.dedent('''
        """Module summary."""

        import pathlib

        pathlib.Path({marker!r}).write_text("executed")
    ''').format(marker=str(marker))
    assert literals.check_source(source, 'fixture.py', ('base_link',), ()) == []
    assert not marker.exists()


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
