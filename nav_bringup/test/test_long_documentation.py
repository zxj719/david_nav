"""检查所有作用域的长函数说明门禁，不执行被扫描源码."""

import textwrap

import pytest

from nav_bringup.lint import docstrings


def _function(name='work', lines=11, *, asynchronous=False, doc=None):
    prefix = 'async ' if asynchronous else ''
    body = [] if doc is None else [f'    {doc!r}']
    body.extend('    pass' for _ in range(lines - 1 - len(body)))
    return f'{prefix}def {name}():\n' + '\n'.join(body) + '\n'


@pytest.mark.parametrize('name', ['work', '_work', '__init__', '__private'])
@pytest.mark.parametrize('asynchronous', [False, True])
def test_all_names_and_sync_async_are_checked(name, asynchronous):
    """所有命名及同步/异步定义均纳入存在性检查."""
    issues = docstrings.check_long_source(_function(name, asynchronous=asynchronous), 'x.py')
    assert len(issues) == 1
    assert (issues[0].path, issues[0].line, issues[0].name) == ('x.py', 1, name)
    assert issues[0].message == 'missing docstring for function over 10 lines'


@pytest.mark.parametrize('lines, expected', [(10, 0), (11, 1)])
def test_definition_span_boundary(lines, expected):
    """十行允许没有说明，十一行必须有说明."""
    assert len(docstrings.check_long_source(_function(lines=lines))) == expected


@pytest.mark.parametrize('doc, expected', [(None, 1), ('', 1), ('   ', 1), ('说明.', 0)])
def test_nonblank_documentation_is_required(doc, expected):
    """空白字面量不能替代说明，非空说明仍需另行语义审查."""
    assert len(docstrings.check_long_source(_function(doc=doc))) == expected


def test_nested_private_class_control_flow_and_source_order():
    """跨类、普通名嵌套及条件分支保留限定名称和源码顺序."""
    source = ('class _Owner:\n    def method(self):\n        """外层方法."""\n'
              + textwrap.indent(_function('inner', asynchronous=True), '        ')
              + 'if True:\n' + textwrap.indent(_function('later'), '    '))
    issues = docstrings.check_long_source(source)
    assert [(item.name, item.line) for item in issues] == [
        ('_Owner.method.inner', 4), ('later', 16)]


def test_short_functions_are_not_promoted_to_public_contracts():
    """独立存在性门禁不要求短私有或短嵌套函数分节."""
    source = 'class _Owner:\n    def method(self):\n        def inner():\n            pass\n'
    assert not docstrings.check_long_source(source)


def test_decorators_excluded_comments_and_multiline_signature_included():
    """沿用物理定义跨度，不按语句数或装饰器行数计算."""
    source = '@decorate\ndef work(\n    value,\n):\n' + '    # note\n\n' * 3 + '    pass\n'
    assert not docstrings.check_long_source(source)
    assert len(docstrings.check_long_source(source.replace('    pass', '\n    pass'))) == 1


def test_source_is_parsed_without_execution():
    """模块顶层危险调用与函数体都只解析，不导入执行."""
    source = 'raise RuntimeError("must not run")\n' + _function()
    assert len(docstrings.check_long_source(source)) == 1


def test_paths_are_sorted_and_directory_tests_excluded(tmp_path):
    """目录扫描跳过测试，显式测试路径仍被检查."""
    for name in ('z.py', 'a.py', 'test_skip.py'):
        (tmp_path / name).write_text(_function(), encoding='utf-8')
    issues = docstrings.check_long_paths([tmp_path])
    assert [item.path for item in issues] == [str(tmp_path / name) for name in ('a.py', 'z.py')]
    assert len(docstrings.check_long_paths([tmp_path / 'test_skip.py'])) == 1


def test_syntax_errors_keep_filename():
    """不能解析的源码不能被当成合规."""
    with pytest.raises(SyntaxError) as error:
        docstrings.check_long_source('def broken(', 'broken.py')
    assert error.value.filename == 'broken.py'


def test_missing_file_is_not_silently_ignored(tmp_path):
    """显式文件不可读时传播读取异常."""
    with pytest.raises(OSError):
        docstrings.check_long_paths([tmp_path / 'missing.py'])


def test_existing_private_and_public_rules_remain_independent():
    """不扩大原私有门禁，也不让存在性检查替代公共接口分节."""
    source = _function('public', doc='说明.')
    assert not docstrings.check_long_source(source)
    assert not docstrings.check_private_source(_function('public'))
    assert not docstrings.check_private_source(_function('__init__'))
    assert docstrings.check_source(source)
