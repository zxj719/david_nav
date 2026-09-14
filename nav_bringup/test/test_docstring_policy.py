"""Tests for the Google-style docstring section checks."""

from pathlib import Path
import textwrap

from nav_bringup.lint import docstrings

PACKAGE_DIR = Path(__file__).resolve().parents[1] / 'nav_bringup'

COMPLIANT_MODULE = '''
"""Module summary."""


class Scaler:
    """Scales values.

    Attributes:
        factor: Multiplier applied to every value.
    """

    def __init__(self, factor):
        self.factor = factor

    @property
    def doubled(self):
        """float: Twice the factor."""
        return self.factor * 2

    def apply(self, value: float) -> float:
        """Scale one value.

        Args:
            value: Input value.

        Returns:
            The scaled value.

        Raises:
            ValueError: If the factor is negative.
        """
        if self.factor < 0:
            raise ValueError('negative factor')
        return value * self.factor


def iterate(*values, **options):
    """Iterate over values.

    Args:
        *values: Values to yield.
        **options: Ignored options.

    Yields:
        Each value in order.
    """
    yield from values


def _private(value):
    return value


def handler(event) -> None:
    """Handle an event without returning anything.

    Args:
        event: The event.
    """
    def nested(inner):
        return inner
    try:
        nested(event)
    except KeyError:
        raise
'''


def _messages(source):
    return [violation.message for violation in docstrings.check_source(textwrap.dedent(source))]


def test_compliant_module_has_no_violations():
    """A module following every rule reports nothing."""
    assert docstrings.check_source(COMPLIANT_MODULE) == []


def test_missing_module_docstring_is_reported():
    """The module itself must have a docstring."""
    assert _messages('X = 1\n') == ['missing module docstring']


def test_missing_function_and_class_docstrings_are_reported():
    """Public functions and classes need docstrings."""
    source = '''
        """Module."""

        def run():
            pass

        class Thing:
            pass
    '''
    assert _messages(source) == ['missing function docstring', 'missing class docstring']


def test_missing_args_section_is_reported():
    """Parameters other than self and cls require an Args section."""
    source = '''
        """Module."""

        def run(speed):
            """Run."""
    '''
    assert _messages(source) == ['missing Args: section']


def test_undocumented_argument_is_reported():
    """Every parameter must be named in the Args section."""
    source = '''
        """Module."""

        def run(speed, heading):
            """Run.

            Args:
                speed: Linear speed.
            """
    '''
    assert _messages(source) == ["argument 'heading' not documented in Args:"]


def test_returned_value_requires_returns_section():
    """Returning a value requires a Returns section."""
    source = '''
        """Module."""

        def answer():
            """Answer."""
            return 42
    '''
    assert _messages(source) == ['missing Returns: section']


def test_non_none_return_annotation_requires_returns_section():
    """A declared return type requires a Returns section even without a body."""
    source = '''
        """Module."""

        def answer() -> int:
            """Answer."""
            ...
    '''
    assert _messages(source) == ['missing Returns: section']


def test_generator_requires_yields_section():
    """Generators document what they yield."""
    source = '''
        """Module."""

        def numbers():
            """Numbers."""
            yield 1
    '''
    assert _messages(source) == ['missing Yields: section']


def test_explicit_raise_requires_raises_section():
    """Raising an exception requires a Raises section; a bare re-raise does not."""
    source = '''
        """Module."""

        def fail():
            """Fail."""
            raise RuntimeError('boom')

        def reraise():
            """Re-raise."""
            try:
                pass
            except KeyError:
                raise
    '''
    assert _messages(source) == ['missing Raises: section']


def test_private_members_and_nested_scopes_are_ignored():
    """Private definitions and nested functions are out of scope."""
    source = '''
        """Module."""

        def _helper(value):
            return value

        class _Internal:
            def method(self, value):
                return value

        def outer():
            """Outer."""
            def inner(value):
                return value
    '''
    assert _messages(source) == []


def test_violation_string_and_report_format():
    """Violations render as path:line name: message, one per line."""
    violations = docstrings.check_source('def run():\n    pass\n', path='pkg/mod.py')

    assert docstrings.format_violations(violations) == (
        'pkg/mod.py:1 <module>: missing module docstring\n'
        'pkg/mod.py:1 run: missing function docstring'
    )


def test_check_paths_skips_test_modules_in_directories(tmp_path):
    """Directory scans ignore test_*.py files but check everything else."""
    (tmp_path / 'good.py').write_text('"""Good."""\n', encoding='utf-8')
    (tmp_path / 'bad.py').write_text('X = 1\n', encoding='utf-8')
    (tmp_path / 'test_ignored.py').write_text('X = 1\n', encoding='utf-8')

    violations = docstrings.check_paths([tmp_path])

    assert [Path(v.path).name for v in violations] == ['bad.py']


def test_nav_bringup_package_complies():
    """The nav_bringup package itself follows the docstring policy."""
    violations = docstrings.check_paths([PACKAGE_DIR])
    assert not violations, docstrings.format_violations(violations)
