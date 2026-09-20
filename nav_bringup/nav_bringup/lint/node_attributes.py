"""检查 ROS Node 的继承与属性冲突，全程不执行被检查的代码.

静态导入、相对导入、本地继承与 mixin 都从给定的源码集合里解析；运行期计算出来的基类、
运行期重新绑定以及接收者别名解析不了。同一模块内重名的类会被拒绝，
而不是静默地挑一个用。父类属性取自已安装的 Node 源码，
既不是构造出来的 ROS 节点，也不是手工维护的私有属性清单——
因此升级 rclpy 后结论会随之变化，这是有意为之。
"""

import ast
from dataclasses import dataclass
from typing import Iterable, Mapping


@dataclass(frozen=True)
class NodeAudit:
    """静态得到的子类清单，以及不依赖运行时的冲突诊断.

    Attributes:
        checked_classes: 排序后的 Node 子类完全限定名。
        violations: 排序后的冲突，以及未被用到或无效的覆盖豁免。
    """

    checked_classes: tuple[str, ...]
    violations: tuple[str, ...]


def audit_sources(
    sources: Mapping[str, str], node_source: str, *,
    allowed_methods: Iterable[tuple[str, str]] = (),
    packages: Iterable[str] = (),
) -> NodeAudit:
    """把静态求得的 Node 后代与已安装父类的声明成员逐项比对.

    Args:
        sources: 模块名到 Python 源码的映射，包含共享基类。
        node_source: 含已安装 rclpy Node 类定义的源码。
        allowed_methods: 经审查的生命周期覆盖，按"属主/方法"精确配对。
            每一项都必须真实存在、确实冲突，并且内部有对同名父方法的 super 委托；
            即使方法名已获批准，实例写入仍然禁止。
        packages: 代表包初始化文件的模块名。

    Returns:
        子类清单与带行号的违例；过程中不导入任何源码。

    Raises:
        SyntaxError: 给定的某份 Python 源码非法。
        ValueError: 缺少 Node、类名有歧义，或导入越出了给定范围。
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
    """把有限的名称表达式转换为限定名，供静态父类识别使用.

    裸名称先查导入绑定，否则加当前模块前缀；属性递归拼接，
    泛型下标只保留外层类型。不执行表达式或证明其运行时身份。

    Args:
        node: 名称、属性或下标形式的 AST 表达式。
        module: 当前模块的点分名称。
        bindings: 已收集的裸名称到导入限定名的映射。

    Returns:
        推测的限定名；不支持的表达式返回空串。
    """
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
    """按源码遍历次序展开类体 AST，保留方法声明但不进入方法体.

    嵌套类整体跳过；条件块等继续展开，因此产出不只限于 ast.stmt 节点。

    Args:
        node: 待扫描字段声明的类 AST。

    Yields:
        类作用域内可供字段规则检查的节点，不执行类体。
    """
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
    """查找形如 super(...).指定方法(...) 的语法调用，不证明运行委托.

    使用 ast.walk，包含嵌套节点和不可达分支；不解析 super 重绑定或调用参数，
    因此结果仅作现有属性规则中的语法证据，不能当作完整控制流保证。

    Args:
        method: 待扫描的方法 AST。
        name: 期望委托的父类方法名称。

    Returns:
        是否至少存在一个符合语法形状的调用。
    """
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
