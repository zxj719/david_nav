"""Exercise direct public data-attribute documentation without importing code."""

import textwrap

import pytest

from nav_bringup.lint import docstrings


def _attributes(source):
    return [item.message for item in docstrings.check_source(textwrap.dedent(source))
            if 'Attributes:' in item.message]


@pytest.mark.parametrize('declaration', [
    'value: int', 'value = 1', 'value, _hidden = (1, 2)',
    'if True:\n        value = 1',
    'def __init__(self):\n        self.value = 1',
    'def update(self):\n        self.value: int = 2',
    'async def update(this):\n        this.value = 2',
    '@classmethod\n    def create(cls):\n        cls.value = 2',
    'def update(self):\n        setattr(self, "value", 2)',
    '__slots__ = ("value", "_hidden")',
    '__slots__ = {"value": "The value", "_hidden": "Private"}',
])
def test_public_data_attribute_requires_named_documentation(declaration):
    """Each directly declared public field requires its own Attributes entry."""
    source = '\n'.join([
        '"""Module."""', 'class Record:', '    """Record."""',
        '    ' + declaration,
    ])
    assert _attributes(source) == ["attribute 'value' not documented in Attributes:"]


def test_attribute_name_in_prose_or_another_section_is_not_documentation():
    """Mentions and longer names cannot satisfy a missing exact field entry."""
    source = '''
        """Module."""
        class Record:
            """Record with value.

            Attributes:
                value_extra: Something else.

            Example:
                value: This is not an attribute description.
            """
            value: int
    '''
    assert _attributes(source) == ["attribute 'value' not documented in Attributes:"]


def test_typed_entry_and_property_docstring_are_sufficient():
    """Properties keep their getter documentation without duplicate class entries."""
    source = '''
        """Module."""
        class Record:
            """Record.

            Attributes:
                value (int): Stored value.
            """
            value: int
            @property
            def doubled(self):
                """Twice the stored value."""
                return self.value * 2
    '''
    assert docstrings.check_source(textwrap.dedent(source)) == []


def test_private_nested_and_foreign_receivers_do_not_become_class_fields():
    """Do not count nested function state or attributes of unrelated objects."""
    source = '''
        """Module."""
        class Record:
            """Record."""
            _hidden: int
            def __init__(self):
                self._private = 1
                other.value = 2
                def nested():
                    self.nested = 3
                class Local:
                    field = 4
            @staticmethod
            def _factory(self):
                self.not_an_instance = 5
        class _Private:
            public = 1
    '''
    assert _attributes(source) == []


def test_nested_public_class_reports_qualified_name():
    """Keep the existing public nested-class traversal and diagnostic location."""
    source = '''
        """Module."""
        class Outer:
            """Outer."""
            class Inner:
                """Inner."""
                value: int
    '''
    violations = docstrings.check_source(textwrap.dedent(source), 'fixture.py')
    assert [(item.path, item.name, item.message) for item in violations] == [
        ('fixture.py', 'Outer.Inner', "attribute 'value' not documented in Attributes:")]
