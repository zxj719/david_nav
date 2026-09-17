"""Exercise the opt-in SPEC 5.3 long-private-function documentation gate."""

import pytest

from nav_bringup.lint import docstrings


def _function(name='_work', lines=11, *, asynchronous=False, doc=None):
    prefix = 'async ' if asynchronous else ''
    body = [] if doc is None else [f'    {doc!r}']
    body.extend('    pass' for _ in range(lines - 1 - len(body)))
    return f'{prefix}def {name}():\n' + '\n'.join(body) + '\n'


@pytest.mark.parametrize('lines, expected', [(10, 0), (11, 1)])
def test_ten_line_boundary(lines, expected):
    """Count the definition through its last statement, excluding decorators."""
    assert len(docstrings.check_private_source(_function(lines=lines))) == expected


@pytest.mark.parametrize('doc', [None, '', '   '])
def test_missing_or_blank_documentation(doc):
    """Reject empty text rather than treating any string literal as documentation."""
    violations = docstrings.check_private_source(_function(doc=doc), 'sample.py')
    assert len(violations) == 1
    assert violations[0].path == 'sample.py'
    assert violations[0].line == 1
    assert violations[0].name == '_work'
    assert 'docstring' in violations[0].message


@pytest.mark.parametrize('asynchronous', [False, True])
def test_documented_sync_and_async_functions(asynchronous):
    """Accept nonblank documentation without claiming its prose is correct."""
    source = _function(asynchronous=asynchronous, doc='Explain the invariant.')
    assert not docstrings.check_private_source(source)


@pytest.mark.parametrize('name', ['public', '__init__', '__private'])
def test_public_and_dunder_names_are_outside_this_gate(name):
    """Leave public API checks unchanged and exclude double-underscore methods."""
    assert not docstrings.check_private_source(_function(name))


def test_nested_functions_private_classes_and_source_order():
    """Find private definitions inside control flow and retain qualified names."""
    inner = '\n'.join('        ' + line for line in _function().splitlines())
    later = _function('_later', asynchronous=True).splitlines()
    source = ('class _Owner:\n    def public(self):\n' + inner + '\n'
              'if True:\n' + '\n'.join('    ' + line for line in later))
    violations = docstrings.check_private_source(source)
    assert [item.name for item in violations] == ['_Owner.public._work', '_later']
    assert [item.line for item in violations] == [3, 15]


def test_comments_blank_lines_and_multiline_signatures_count():
    """Use physical definition spans, not AST statement counts or decorators."""
    source = '@decorate\ndef _work(\n    value,\n):\n' + '    # detail\n\n' * 3 + '    pass\n'
    assert not docstrings.check_private_source(source)
    assert len(docstrings.check_private_source(source.replace('    pass', '\n    pass'))) == 1


def test_path_order_and_directory_test_exclusion(tmp_path):
    """Share public path discovery; explicitly requested test files remain checked."""
    for name in ('z.py', 'a.py', 'test_skip.py'):
        (tmp_path / name).write_text(_function(), encoding='utf-8')
    violations = docstrings.check_private_paths([tmp_path])
    expected = [str(tmp_path / name) for name in ('a.py', 'z.py')]
    assert [item.path for item in violations] == expected
    assert len(docstrings.check_private_paths([tmp_path / 'test_skip.py'])) == 1


def test_invalid_source_is_not_silently_ignored():
    """Propagate syntax errors with the supplied filename."""
    with pytest.raises(SyntaxError) as error:
        docstrings.check_private_source('def broken(', 'broken.py')
    assert error.value.filename == 'broken.py'


def test_private_gate_does_not_replace_public_contract_checks():
    """An opt-in private scan cannot make undocumented public APIs pass."""
    source = _function('public')
    assert not docstrings.check_private_source(source)
    assert docstrings.check_source(source)
