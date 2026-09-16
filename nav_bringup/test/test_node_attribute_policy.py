"""Characterize inherited ROS attribute collisions without importing ROS."""

import pytest

from nav_bringup.lint.node_attributes import audit_sources

NODE = '''
class Node:
    def __init__(self):
        self._publishers = []
        self._parameters = {}
    @property
    def handle(self):
        return object()
    def destroy_node(self):
        pass
'''


@pytest.mark.parametrize('binding, base', [
    ('from rclpy.node import Node', 'Node'),
    ('from rclpy.node import Node as RosNode', 'RosNode'),
    ('import rclpy.node', 'rclpy.node.Node'),
    ('import rclpy.node as nodes', 'nodes.Node'),
    ('from rclpy import node as nodes', 'nodes.Node'),
])
def test_import_aliases_do_not_hide_handle_method(binding, base):
    """The historical handle method collision is detected for normal import forms."""
    result = audit_sources({'pkg.worker': binding + f'\nclass Worker({base}):\n'
                            '    def handle(self):\n        pass\n'}, NODE)
    assert result.checked_classes == ('pkg.worker.Worker',)
    assert any('handle' in item for item in result.violations)


@pytest.mark.parametrize('body', [
    'handle = None',
    'handle: object = None',
    'async def handle(self):\n    pass',
    'def run(self):\n    self.handle = None',
    'def run(me):\n    me._publishers = []',
    'def run(self):\n    self._parameters, self.own = {}, 1',
    'def run(self):\n    self._publishers += []',
    'def run(self):\n    del self.handle',
    'def run(self):\n    setattr(self, "handle", None)',
    'def run(self):\n    delattr(self, "_publishers")',
])
def test_class_members_and_instance_writes_collide(body):
    """Methods, fields and literal attribute writes cannot overwrite Node internals."""
    source = 'from rclpy.node import Node\nclass Worker(Node):\n' + '\n'.join(
        '    ' + line for line in body.splitlines())
    assert audit_sources({'pkg.worker': source}, NODE).violations


def test_indirect_inheritance_and_mixin_collisions():
    """Relative imports and mixins are checked at every Node descendant."""
    result = audit_sources({
        'pkg.base': 'from rclpy.node import Node\nclass Parent(Node):\n    pass',
        'pkg.mixed': 'class BadMixin:\n    def handle(self):\n        pass',
        'pkg.worker': 'from .base import Parent as Base\nfrom .mixed import BadMixin\n'
                      'class Worker(BadMixin, Base):\n    pass',
    }, NODE)
    assert result.checked_classes == ('pkg.base.Parent', 'pkg.worker.Worker')
    assert any('pkg.worker.Worker' in item and 'pkg.mixed.BadMixin' in item
               for item in result.violations)


def test_legitimate_composition_and_nested_foreign_objects():
    """Non-Node helpers, private mangling, reads and foreign assignments stay legal."""
    result = audit_sources({'pkg.worker': '''
from rclpy.node import Node
class Worker(Node):
    def __init__(self):
        super().__init__('worker')
        self.own = self.handle
        self.__private = 1
    def run(self, other):
        other.handle = 1
        class Other:
            def run(self):
                self.handle = 1
class Composed:
    def handle(self):
        self._publishers = []
'''}, NODE.replace('self._publishers', 'self.__private'))
    assert result.checked_classes == ('pkg.worker.Worker',)
    assert not result.violations


def test_explicit_lifecycle_override_requires_super_delegation():
    """Only an exact approved method may override, and it must call the parent."""
    source = ('from rclpy.node import Node\nclass Worker(Node):\n'
              '    def destroy_node(self):\n        super().destroy_node()')
    allowed = {('pkg.worker.Worker', 'destroy_node')}
    assert not audit_sources({'pkg.worker': source}, NODE,
                             allowed_methods=allowed).violations
    assert audit_sources({'pkg.worker': source.replace('super().destroy_node()', 'pass')},
                         NODE, allowed_methods=allowed).violations
    assert audit_sources({'pkg.worker': source.replace('destroy_node', 'handle')},
                         NODE, allowed_methods=allowed).violations


def test_allowance_does_not_allow_instance_overwrite():
    """Approved lifecycle overrides do not grant writes to the same attribute."""
    result = audit_sources({'pkg.worker': '''
from rclpy.node import Node
class Worker(Node):
    def destroy_node(self):
        super().destroy_node()
        self.destroy_node = None
'''}, NODE, allowed_methods={('pkg.worker.Worker', 'destroy_node')})
    assert any('instance' in item for item in result.violations)


def test_stale_allowance_is_an_error():
    """Exceptions cannot silently become a growing exemption list."""
    result = audit_sources({'pkg.worker': ''}, NODE,
                           allowed_methods={('pkg.worker.Worker', 'destroy_node')})
    assert any('unused allowance' in item for item in result.violations)


def test_nested_node_class_is_checked():
    """A Node subclass defined inside a factory is still checked statically."""
    result = audit_sources({'pkg.worker': '''
from rclpy.node import Node
def build():
    class Worker(Node):
        def handle(self):
            pass
    return Worker
'''}, NODE)
    assert result.checked_classes
    assert result.violations


def test_unknown_parent_snapshot_is_rejected():
    """A missing Node class cannot make a package scan vacuously green."""
    with pytest.raises(ValueError, match='Node'):
        audit_sources({}, 'class SomethingElse:\n    pass')


def test_reexported_node_and_conditional_class_field():
    """Import reexports and conditional class writes do not bypass the guard."""
    result = audit_sources({
        'pkg.base': 'from rclpy.node import Node as PublicNode',
        'pkg.worker': 'from .base import PublicNode\nclass Worker(PublicNode):\n'
                      '    if True:\n        handle = None',
    }, NODE)
    assert result.checked_classes == ('pkg.worker.Worker',)
    assert any('handle' in item for item in result.violations)


def test_slot_declaration_cannot_shadow_node_property():
    """A slot descriptor also shadows an inherited attribute."""
    result = audit_sources({'pkg.worker': 'from rclpy.node import Node\n'
                            'class Worker(Node):\n    __slots__ = ("handle",)'}, NODE)
    assert any('handle' in item for item in result.violations)


def test_ambiguous_class_names_are_not_silently_dropped():
    """A source layout beyond the static resolver must produce an explicit error."""
    with pytest.raises(ValueError, match='ambiguous'):
        audit_sources({'pkg.worker': 'class One:\n    pass\nclass One:\n    pass'}, NODE)


def test_alias_cycles_terminate_without_hiding_known_node():
    """Reexport cycles cannot hang the scan of an otherwise known Node subclass."""
    result = audit_sources({
        'pkg': 'import pkg.x.x as x',
        'pkg.worker': 'from rclpy.node import Node\nimport pkg.x as x\n'
                      'class Worker(x.Mixin, Node):\n    handle = None',
    }, NODE, packages={'pkg'})
    assert result.checked_classes == ('pkg.worker.Worker',)
    assert result.violations
