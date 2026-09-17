"""Inspect ROS Node inheritance and attribute collisions without executing code.

Static imports, relative imports, local inheritance and mixins are resolved from
the supplied source set. Computed bases, runtime rebinding and receiver aliases
are not resolved. Duplicate class names within a module are rejected rather
than silently selecting one. Parent attributes come from installed Node source,
never a constructed ROS node or a hand-maintained list of private attributes.
"""

import ast
from dataclasses import dataclass
from typing import Iterable, Mapping


@dataclass(frozen=True)
class NodeAudit:
    """Static subclass inventory and dependency-free collision diagnostics.

    Attributes:
        checked_classes: Sorted fully qualified Node subclass names.
        violations: Sorted collisions and unused or invalid override allowances.
    """

    checked_classes: tuple[str, ...]
    violations: tuple[str, ...]


def audit_sources(
    sources: Mapping[str, str], node_source: str, *,
    allowed_methods: Iterable[tuple[str, str]] = (),
    packages: Iterable[str] = (),
) -> NodeAudit:
    """Compare static Node descendants with the installed parent's declared members.

    Args:
        sources: Module names mapped to Python source, including shared base classes.
        node_source: Source containing the installed rclpy Node class definition.
        allowed_methods: Exact owner/method pairs for reviewed lifecycle overrides.
            Each must exist, collide and contain a same-method super delegation.
            Instance writes remain forbidden even for an approved method name.
        packages: Module names that represent package initializers.

    Returns:
        Subclass inventory and line-numbered violations; no source is imported.

    Raises:
        SyntaxError: Any supplied Python source is invalid.
        ValueError: Node is missing, class names are ambiguous or imports escape scope.
    """
    parents = [node for node in ast.parse(node_source).body
               if isinstance(node, ast.ClassDef) and node.name == 'Node']
    if len(parents) != 1:
        raise ValueError('expected exactly one installed Node definition')
    reserved = {name for name, _, _, _ in _members(parents[0]) if _ordinary(name)}
    classes, bases, aliases = {}, {}, {}
    initializers = set(packages)
    for module, source in sorted(sources.items()):
        tree = ast.parse(source, filename=module)
        bindings = _bindings(module, tree, module in initializers)
        aliases.update((f'{module}.{key}', value) for key, value in bindings.items())
        for node in ast.walk(tree):
            if not isinstance(node, ast.ClassDef):
                continue
            name = f'{module}.{node.name}'
            if name in classes:
                raise ValueError(f'ambiguous class name: {name}')
            classes[name] = node
            bases[name] = {_qualified(base, module, bindings) for base in node.bases}
    aliases = {key: value for key, value in aliases.items() if key not in classes}
    bases = {name: {_canonical(base, aliases) for base in parents}
             for name, parents in bases.items()}
    descendants = {'rclpy.node.Node'}
    while True:
        added = {name for name, parents in bases.items() if parents & descendants}
        if added <= descendants:
            break
        descendants.update(added)
    descendants.discard('rclpy.node.Node')
    allowances = set(allowed_methods)
    used = set()
    violations = set()
    for child in sorted(descendants):
        pending, visited = [child], set()
        while pending:
            owner = pending.pop()
            if owner in visited or owner not in classes:
                continue
            visited.add(owner)
            pending.extend(bases[owner])
            for name, kind, line, member in _members(classes[owner]):
                if name not in reserved or not _ordinary(name):
                    continue
                allowance = (owner, name)
                if kind == 'method' and allowance in allowances:
                    used.add(allowance)
                    if _delegates(member, name):
                        continue
                    reason = 'approved override lacks super delegation'
                else:
                    reason = f'{kind} shadows Node attribute'
                violations.add(f'{child}: {owner}:{line}: {name}: {reason}')
    violations.update(f'{owner}.{name}: unused allowance'
                      for owner, name in allowances - used)
    return NodeAudit(tuple(sorted(descendants)), tuple(sorted(violations)))


def _ordinary(name):
    # Special lifecycle methods are intentional; double-private names are mangled.
    return not name.startswith('__')


def _bindings(module, tree, is_package):
    """收集语法树中的导入别名，作为静态名称解析的绑定表.

    Args:
        module: 当前模块限定名。
        tree: 已解析语法树；会扫描嵌套导入，不模拟 Python 动态作用域。
        is_package: 是否按包初始化模块解析相对导入。

    Returns:
        局部导入名到目标限定名的映射，重复键保留最后扫描到的绑定。

    Raises:
        ValueError: 相对导入超出当前包层级。
    """
    result = {}
    context = module.split('.') if is_package else module.split('.')[:-1]
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                result[alias.asname or alias.name.split('.')[0]] = (
                    alias.name if alias.asname else alias.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom):
            if node.level > len(context):
                raise ValueError(f'{module}: relative import escapes package')
            prefix = context[:len(context) - node.level + 1] if node.level else []
            base = '.'.join(prefix + ([node.module] if node.module else []))
            for alias in node.names:
                result[alias.asname or alias.name] = f'{base}.{alias.name}'
    return result


def _qualified(node, module, bindings):
    if isinstance(node, ast.Name):
        return bindings.get(node.id, f'{module}.{node.id}')
    if isinstance(node, ast.Attribute):
        return f'{_qualified(node.value, module, bindings)}.{node.attr}'
    if isinstance(node, ast.Subscript):
        return _qualified(node.value, module, bindings)
    return ''


def _members(node):
    """枚举类中可静态识别的方法、字段、slots 和方法接收者写入.

    Args:
        node: 待检查的类 AST 节点。

    Yields:
        名称、成员类别、行号和可选方法节点四元组；静态方法不扫描接收者写入，
        不解析计算型字段名、任意接收者别名或动态生成成员。
    """
    for statement in _class_statements(node):
        if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef)):
            yield statement.name, 'method', statement.lineno, statement
            positional = statement.args.posonlyargs + statement.args.args
            if positional and not any(isinstance(item, ast.Name) and item.id == 'staticmethod'
                                      for item in statement.decorator_list):
                visitor = _Writes(positional[0].arg)
                for part in statement.body:
                    visitor.visit(part)
                for name, line in visitor.names:
                    yield name, 'instance write', line, None
        elif isinstance(statement, (ast.Assign, ast.AnnAssign)):
            targets = (statement.targets if isinstance(statement, ast.Assign)
                       else [statement.target])
            for target in targets:
                for part in ast.walk(target):
                    if isinstance(part, ast.Name):
                        yield part.id, 'class field', part.lineno, None
                        if part.id == '__slots__' and statement.value is not None:
                            for value in ast.walk(statement.value):
                                if (isinstance(value, ast.Constant)
                                        and isinstance(value.value, str)):
                                    yield value.value, 'slot', part.lineno, None


def _class_statements(node):
    pending = list(reversed(node.body))
    while pending:
        statement = pending.pop()
        if isinstance(statement, ast.ClassDef):
            continue
        yield statement
        if not isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef)):
            pending.extend(reversed(list(ast.iter_child_nodes(statement))))


def _canonical(name, aliases):
    """沿最长匹配导入别名展开限定名，以循环标记保证有界终止.

    Args:
        name: 待归一化的点分名称。
        aliases: 局部限定名前缀到目标限定名的映射。

    Returns:
        无可展开绑定时的名称；循环或反复扩展同一前缀时保留当前名称，
        不执行模块来尝试证明动态身份。
    """
    visited = set()
    expanded = set()
    while name not in visited:
        visited.add(name)
        parts = name.split('.')
        for length in range(len(parts), 0, -1):
            prefix = '.'.join(parts[:length])
            if prefix in aliases:
                if prefix in expanded:
                    return name
                expanded.add(prefix)
                name = '.'.join([aliases[prefix], *parts[length:]])
                break
        else:
            return name
    return name


def _delegates(method, name):
    return any(
        isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and node.func.attr == name and isinstance(node.func.value, ast.Call)
        and isinstance(node.func.value.func, ast.Name) and node.func.value.func.id == 'super'
        for node in ast.walk(method))


class _Writes(ast.NodeVisitor):

    def __init__(self, receiver):
        self.receiver = receiver
        self.names = []

    def visit_Attribute(self, node):
        if (isinstance(node.value, ast.Name) and node.value.id == self.receiver
                and isinstance(node.ctx, (ast.Store, ast.Del))):
            self.names.append((node.attr, node.lineno))
        self.generic_visit(node)

    def visit_Call(self, node):
        if (isinstance(node.func, ast.Name) and node.func.id in ('setattr', 'delattr')
                and len(node.args) >= 2 and isinstance(node.args[0], ast.Name)
                and node.args[0].id == self.receiver and isinstance(node.args[1], ast.Constant)
                and isinstance(node.args[1].value, str)):
            self.names.append((node.args[1].value, node.lineno))
        self.generic_visit(node)

    def visit_ClassDef(self, node):
        pass

    def visit_FunctionDef(self, node):
        pass

    def visit_AsyncFunctionDef(self, node):
        pass

    def visit_Lambda(self, node):
        pass
