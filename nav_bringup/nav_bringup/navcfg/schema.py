"""解析各包声明的参数 schema，并汇总为档案校验目录.

各消费包在 config/schema.yaml 声明自己的参数，ParameterCatalog 合并后
供 navcfg validate/doc 使用。这里实现自有 schema_version 1 格式，
不是 JSON Schema 通用实现，也不应用或写回默认参数值。

格式示例::

    schema_version: 1
    package: nav_bringup
    parameters:
      record.storage:
        type: string            # bool | int | float | string | list | map
        description: 存储插件
        default: sqlite3        # 可选；null 表示未设置；required 键不能有默认
        choices: [sqlite3]      # 可选，仅标量类型
        unit: s                 # 可选，自由文本
        range: [0.0, null]      # 可选，仅数值；闭区间，null 为开放端
        items: string           # 可选，list 的标量元素类型
        required: false
        tuning: docs/tuning.md  # 可选，调参说明链接

点分键和包名须完整匹配小写命名规则；目录拒绝重复声明及键的祖先/后代冲突。
数据类不自行验证，校验入口仍保留版本数值相等及空值处理等既有语义。
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

import yaml

SCHEMA_VERSION = 1
SCHEMA_RELATIVE_PATH = Path('config') / 'schema.yaml'
TYPES = ('bool', 'int', 'float', 'string', 'list', 'map')
SCALAR_TYPES = ('bool', 'int', 'float', 'string')
PROFILE_META_KEYS = ('schema_version',)

_KEY_RE = re.compile(r'^[a-z][a-z0-9_]*(\.[a-z][a-z0-9_]*)*$')
_PACKAGE_RE = re.compile(r'^[a-z][a-z0-9_]*$')
_TOP_LEVEL_FIELDS = frozenset({'schema_version', 'package', 'parameters'})
_SPEC_FIELDS = frozenset({
    'type', 'description', 'default', 'unit', 'range', 'choices', 'items', 'required', 'tuning',
})


class SchemaError(ValueError):
    """表示 schema 读取、格式或声明合并冲突，不包含所有 Python 异常."""


@dataclass(frozen=True)
class ParameterSpec:
    """保存一个档案键的声明，冻结字段但不深度冻结默认值.

    Attributes:
        key: 点分档案路径，例如 limits.max_vel_x。
        package: 消费并声明该键的包名。
        value_type: 文件 type 字段，对应 TYPES 中的类型名。
        description: 参数用途说明。
        has_default: 是否显式声明 default 字段，区别于默认值为 None。
        default: 声明的默认值；None 表示未设置，不会由目录自动写回档案。
        unit: 可选单位文本。
        value_range: 可选闭区间 (下界, 上界)，None 端表示开放边界。
        choices: 可选允许值元组。
        items: list 元素的可选标量类型约束。
        required: 是否要求档案显式包含该键。
        tuning: 可选调参说明链接。
    """

    key: str
    package: str
    value_type: str
    description: str
    has_default: bool = False
    default: Any = None
    unit: Optional[str] = None
    value_range: Optional[Tuple[Optional[float], Optional[float]]] = None
    choices: Optional[Tuple[Any, ...]] = None
    items: Optional[str] = None
    required: bool = False
    tuning: Optional[str] = None


@dataclass(frozen=True)
class PackageSchema:
    """保存一个包的 schema 解析结果，不自行验证内容.

    Attributes:
        package: 包名。
        path: schema 来源路径，用于诊断。
        parameters: 声明元组，parse_schema 返回时按键排序。
    """

    package: str
    path: Path
    parameters: Tuple[ParameterSpec, ...]


@dataclass(frozen=True)
class ProfileIssue:
    """表示一条档案检查发现，不自行执行拒绝或告警动作.

    Attributes:
        key: 涉及的点分档案路径。
        severity: error 表示校验错误，warn 表示未声明等提示。
        message: 通常含键名的说明文本。
    """

    key: str
    severity: str
    message: str


def parse_schema(data: Any, path: Union[str, Path]) -> PackageSchema:
    """按自有版本格式检查已解析的 schema 数据并构造声明.

    版本沿用 Python 相等比较；parameters 的假值先视为空映射，不作严格类型拒绝。
    包名和键完整匹配格式，单个声明检查字段组合及非 None 默认值。
    此处不检查跨声明键嵌套或重复包，交由 ParameterCatalog 处理。

    Args:
        data: YAML 解析结果，顶层要求 dict。
        path: 来源文件名，用于错误信息和结果记录。

    Returns:
        参数按键排序的包声明；部分嵌套默认值仍引用原数据。

    Raises:
        SchemaError: 已处理的格式、声明约束或默认值错误。
    """
    source = str(path)
    if not isinstance(data, dict):
        raise SchemaError(f'{source}: a schema must be a mapping')
    version = data.get('schema_version')
    if version != SCHEMA_VERSION:
        raise SchemaError(f'{source}: schema_version must be {SCHEMA_VERSION}, got {version!r}')
    package = data.get('package')
    if not isinstance(package, str) or not _PACKAGE_RE.fullmatch(package):
        raise SchemaError(f'{source}: package must be a ROS package name, got {package!r}')
    unknown = sorted(str(name) for name in set(data) - _TOP_LEVEL_FIELDS)
    if unknown:
        raise SchemaError(f'{source}: unknown top-level fields {unknown}')
    parameters = data.get('parameters') or {}
    if not isinstance(parameters, dict):
        raise SchemaError(f'{source}: parameters must be a mapping')
    specs = [_parse_spec(key, fields, package, source) for key, fields in parameters.items()]
    return PackageSchema(package, Path(path), tuple(sorted(specs, key=lambda spec: spec.key)))


def load_schema(path: Union[str, Path]) -> PackageSchema:
    """读取 UTF-8 YAML 并调用自有 schema 解析器.

    Args:
        path: config/schema.yaml 的路径。

    Returns:
        解析的包声明。

    Raises:
        SchemaError: 文件 IO、YAML 解析或已处理的格式错误。
        UnicodeError: UTF-8 解码失败，不统一包装为 SchemaError。
    """
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise SchemaError(f'{path}: cannot read schema: {error}') from error
    return parse_schema(data, path)


def check_value(spec: ParameterSpec, value: Any) -> List[str]:
    """按已解析声明检查类型、列表元素、范围及枚举.

    不检查数值有限性，也不递归验证 map 的内部字段。调用方需传入有效声明，
    手工构造的非法约束组合可能直接触发 Python 异常。

    Args:
        spec: 预期已通过解析器校验的声明。
        value: 档案值或默认值，不在此特殊放行 None。

    Returns:
        问题说明列表；类型不符时立即返回一项，否则累计其它约束问题。
        空列表只表示通过这里实现的检查，不证明全部业务约束成立。
    """
    if not _has_type(value, spec.value_type):
        return [f'{spec.key}: expected {spec.value_type}, got {type(value).__name__} {value!r}']
    problems = []
    if spec.items is not None:
        for index, item in enumerate(value):
            if not _has_type(item, spec.items):
                problems.append(
                    f'{spec.key}[{index}]: expected {spec.items}, '
                    f'got {type(item).__name__} {item!r}')
    if spec.value_range is not None:
        low, high = spec.value_range
        if (low is not None and value < low) or (high is not None and value > high):
            problems.append(
                f"{spec.key}: {value!r} outside range "
                f"[{'-inf' if low is None else low}, {'inf' if high is None else high}]")
    if spec.choices is not None and value not in spec.choices:
        problems.append(f'{spec.key}: {value!r} is not one of {list(spec.choices)}')
    return problems


class ParameterCatalog:
    """合并多个包的参数声明，并按目录检查档案.

    只检测重复包、重复键及声明键嵌套，不重新解析手工构造的数据类，
    也不填入默认值、加载 ROS 参数或修改档案。
    """

    def __init__(self, schemas: Iterable[PackageSchema]) -> None:
        """索引声明并拒绝重复所有者或祖先/后代键冲突.

        Args:
            schemas: 包声明迭代器，每个包最多出现一次，声明对象不深复制。

        Raises:
            SchemaError: 包重复、键重复，或某个声明键位于另一个声明键之下。
        """
        self._schemas: Dict[str, PackageSchema] = {}
        self._specs: Dict[str, ParameterSpec] = {}
        for schema in schemas:
            previous = self._schemas.get(schema.package)
            if previous is not None:
                raise SchemaError(
                    f'package {schema.package} has two schemas: {previous.path} and {schema.path}')
            self._schemas[schema.package] = schema
            for spec in schema.parameters:
                other = self._specs.get(spec.key)
                if other is not None:
                    raise SchemaError(
                        f'{spec.key} is declared by both {other.package} and {spec.package}')
                self._specs[spec.key] = spec
        for key, spec in self._specs.items():
            parts = key.split('.')
            for length in range(1, len(parts)):
                outer = self._specs.get('.'.join(parts[:length]))
                if outer is not None:
                    raise SchemaError(
                        f'{key} ({spec.package}) is nested inside {outer.key} ({outer.package})')

    @property
    def packages(self) -> Tuple[str, ...]:
        """Tuple[str, ...]: 按名称排序的包名元组."""
        return tuple(sorted(self._schemas))

    @property
    def specs(self) -> Tuple[ParameterSpec, ...]:
        """Tuple[ParameterSpec, ...]: 按键排序的声明元组，元素为已保存的对象."""
        return tuple(self._specs[key] for key in sorted(self._specs))

    def get(self, key: str) -> Optional[ParameterSpec]:
        """按完整点分键查找已保存的声明.

        Args:
            key: 要精确匹配的点分路径。

        Returns:
            声明对象本身；不存在时为 None，不复制嵌套值。
        """
        return self._specs.get(key)

    def defaults(self) -> Dict[str, Any]:
        """收集显式默认值，不应用到档案或深复制值.

        Returns:
            点分键到默认值的新字典，包含显式 None；嵌套列表/映射仍共享引用。
        """
        return {spec.key: spec.default for spec in self.specs if spec.has_default}

    def check_profile(
        self,
        profile: Mapping[str, Any],
        meta_keys: Sequence[str] = PROFILE_META_KEYS,
    ) -> List[ProfileIssue]:
        """对嵌套档案做声明检查，返回问题而不修改或补全输入.

        元数据仅按顶层名称跳过，不在此检查其版本。声明为 map 的键作为整体值，
        不再逐个警告其子键。显式 None 仅在声明默认值也显式为 None 时放行。
        必需键只按是否出现判断；出现但值错误时报告值错误而非再报缺失。

        Args:
            profile: 预期映射形状的档案。
            meta_keys: 忽略的顶层元数据键名。

        Returns:
            按档案遍历顺序的值错误/未声明警告，再附按键排序的缺失必需项。
            不做全部机器人业务约束、递归深度或循环引用检查。
        """
        issues: List[ProfileIssue] = []
        seen: List[str] = []
        top_level = {name: value for name, value in profile.items() if name not in meta_keys}
        self._check_mapping(top_level, '', issues, seen)
        for spec in self.specs:
            if spec.required and spec.key not in seen:
                issues.append(ProfileIssue(
                    spec.key, 'error', f'{spec.key}: required by {spec.package} but not set'))
        return issues

    def _check_mapping(
        self,
        mapping: Mapping[str, Any],
        prefix: str,
        issues: List[ProfileIssue],
        seen: List[str],
    ) -> None:
        """递归比对档案映射与已声明参数，把检查结果累积到调用方列表.

        Args:
            mapping: 当前档案层级。
            prefix: 当前点分键前缀。
            issues: 原地追加类型/约束错误和未声明字段警告的列表。
            seen: 原地记录匹配声明键，供上层检查缺失必需值。

        显式默认 None 的声明允许该值；只有声明过下层键时才继续递归。
        """
        for name, value in mapping.items():
            key = f'{prefix}.{name}' if prefix else str(name)
            spec = self._specs.get(key)
            if spec is not None:
                seen.append(key)
                if value is None and spec.has_default and spec.default is None:
                    continue
                issues.extend(
                    ProfileIssue(key, 'error', message) for message in check_value(spec, value))
            elif isinstance(value, Mapping) and any(
                    declared.startswith(key + '.') for declared in self._specs):
                self._check_mapping(value, key, issues, seen)
            else:
                issues.append(
                    ProfileIssue(key, 'warn', f'{key}: not declared by any package schema'))


def ament_prefixes(environ: Mapping[str, str]) -> List[Path]:
    """拆分环境提供的 AMENT_PREFIX_PATH，保留前缀顺序.

    Args:
        environ: 要读取的环境映射，不修改真实环境。

    Returns:
        排除空段后的 Path 列表；不展开用户目录、去重或检查存在性。
        第一个前缀由调用方约定为优先 overlay。
    """
    entries = environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    return [Path(entry) for entry in entries if entry]


def find_installed_schemas(prefixes: Sequence[Union[str, Path]]) -> List[Path]:
    """在各安装前缀的 share 下发现固定相对位置的 schema.

    Args:
        prefixes: 按优先级排序的安装前缀。

    Returns:
        按包目录名排序的候选路径列表，同名包首个前缀优先。
        不解析 package.xml、校验包归属或确认候选是常规可读文件。
    """
    found: Dict[str, Path] = {}
    for prefix in prefixes:
        share = Path(prefix) / 'share'
        if not share.is_dir():
            continue
        for candidate in sorted(share.glob('*/' + SCHEMA_RELATIVE_PATH.as_posix())):
            found.setdefault(candidate.parent.parent.name, candidate)
    return [found[package] for package in sorted(found)]


def find_source_schemas(source_root: Union[str, Path]) -> List[Path]:
    """按 package.xml 和固定 schema 路径发现源码包声明.

    Args:
        source_root: 工作空间 src 扫描根。

    Returns:
        按 package.xml 路径顺序收集的 config/schema.yaml 文件。
        包到扫描根之间存在 COLCON_IGNORE 时排除；不解析清单、其它
        colcon 忽略规则或包依赖，也不保证遍历期间文件稳定。
    """
    root = Path(source_root)
    schemas = []
    for manifest in sorted(root.glob('**/package.xml')):
        package_dir = manifest.parent
        if _ignored_by_colcon(package_dir, root):
            continue
        candidate = package_dir / SCHEMA_RELATIVE_PATH
        if candidate.is_file():
            schemas.append(candidate)
    return schemas


def load_catalog(paths: Iterable[Union[str, Path]]) -> ParameterCatalog:
    """逐个读取 schema，再合并声明目录.

    Args:
        paths: schema 路径迭代器。

    Returns:
        已完成合并冲突检查的目录，不应用到运行节点。

    Raises:
        SchemaError: 文件解析或合并出现已处理错误。
        UnicodeError: 输入文件 UTF-8 解码失败。
    """
    return ParameterCatalog(load_schema(path) for path in paths)


def _parse_spec(key: Any, fields: Any, package: str, source: str) -> ParameterSpec:
    """严格解析单个参数声明，并验证非 None 默认值符合自身约束.

    Args:
        key: 小写点分参数键。
        fields: 原始声明字段映射。
        package: 提供声明的包名。
        source: 用于错误定位的来源名称。

    Returns:
        保留默认值存在性、类型、范围、枚举和单位的 ParameterSpec。

    Raises:
        SchemaError: 键、未知字段、类型/约束组合、说明或默认值不合法。
    """
    if not isinstance(key, str) or not _KEY_RE.fullmatch(key):
        raise SchemaError(f'{source}: invalid key {key!r}; use dotted lowercase words')
    where = f'{source}: {key}'
    if not isinstance(fields, dict):
        raise SchemaError(f'{where}: declaration must be a mapping')
    unknown = sorted(str(name) for name in set(fields) - _SPEC_FIELDS)
    if unknown:
        raise SchemaError(f'{where}: unknown fields {unknown}')
    value_type = fields.get('type')
    if value_type not in TYPES:
        raise SchemaError(f'{where}: type must be one of {list(TYPES)}, got {value_type!r}')
    description = fields.get('description')
    if not isinstance(description, str) or not description.strip():
        raise SchemaError(f'{where}: description is required')
    for name in ('unit', 'tuning'):
        if fields.get(name) is not None and not isinstance(fields[name], str):
            raise SchemaError(f'{where}: {name} must be a string')
    required = fields.get('required', False)
    if not isinstance(required, bool):
        raise SchemaError(f'{where}: required must be true or false')
    has_default = 'default' in fields
    if required and has_default:
        raise SchemaError(f'{where}: a required key cannot have a default')
    value_range = None
    if fields.get('range') is not None:
        if value_type not in ('int', 'float'):
            raise SchemaError(f'{where}: range is only allowed for int and float')
        value_range = _parse_range(fields['range'], where)
    choices = None
    if fields.get('choices') is not None:
        raw_choices = fields['choices']
        if value_type not in SCALAR_TYPES or not isinstance(raw_choices, list) or not raw_choices:
            raise SchemaError(f'{where}: choices must be a non-empty list for a scalar type')
        for choice in raw_choices:
            if not _has_type(choice, value_type):
                raise SchemaError(f'{where}: choice {choice!r} is not a {value_type}')
        choices = tuple(raw_choices)
    items = fields.get('items')
    if items is not None and (value_type != 'list' or items not in SCALAR_TYPES):
        raise SchemaError(f'{where}: items must be a scalar type and needs type list')
    spec = ParameterSpec(
        key=key,
        package=package,
        value_type=value_type,
        description=description.strip(),
        has_default=has_default,
        default=fields.get('default'),
        unit=fields.get('unit'),
        value_range=value_range,
        choices=choices,
        items=items,
        required=required,
        tuning=fields.get('tuning'),
    )
    if has_default and spec.default is not None:
        problems = check_value(spec, spec.default)
        if problems:
            raise SchemaError(f'{source}: invalid default: ' + '; '.join(problems))
    return spec


def _parse_range(value: Any, where: str) -> Tuple[Optional[float], Optional[float]]:
    """解析上下界二元列表，允许 null 表示开放边界并拒绝布尔值.

    两端都有值时检查下界不大于上界；不在此另行检查非有限浮点值。

    Args:
        value: 待解析的 [minimum, maximum] 列表。
        where: schema 错误的来源描述。

    Returns:
        原样保留数值类型的上下界元组，不强制转换为 float。

    Raises:
        SchemaError: 结构、边界类型或上下界顺序不合要求。
    """
    if not isinstance(value, list) or len(value) != 2:
        raise SchemaError(f'{where}: range must be [minimum, maximum]')
    low, high = value
    for bound in (low, high):
        if bound is not None and (isinstance(bound, bool) or not isinstance(bound, (int, float))):
            raise SchemaError(f'{where}: range bounds must be numbers or null')
    if low is not None and high is not None and low > high:
        raise SchemaError(f'{where}: range minimum exceeds maximum')
    return low, high


def _has_type(value: Any, value_type: str) -> bool:
    """按 schema 已验证的类型名检查 Python 值，不在此检查有限性或范围.

    Args:
        value: 待检查值。
        value_type: 已确认受支持的声明类型名；末分支对应 dict。

    Returns:
        类型是否匹配；bool 不作为 int/float，float 声明允许整数值。
    """
    if value_type == 'bool':
        return isinstance(value, bool)
    if value_type == 'int':
        return isinstance(value, int) and not isinstance(value, bool)
    if value_type == 'float':
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if value_type == 'string':
        return isinstance(value, str)
    if value_type == 'list':
        return isinstance(value, list)
    return isinstance(value, dict)


def _ignored_by_colcon(package_dir: Path, root: Path) -> bool:
    """从包目录向上查找 COLCON_IGNORE，避免重新纳入被忽略的包.

    给定根本身也参与检查；到给定根或文件系统根后停止。
    不解析 colcon 的其它忽略机制，也不证明 package_dir 确实位于 root 之下。

    Args:
        package_dir: 待检查的包目录。
        root: 此次源码发现的预期扫描根。

    Returns:
        访问的目录链上是否存在 COLCON_IGNORE。
    """
    directory = package_dir
    while True:
        if (directory / 'COLCON_IGNORE').exists():
            return True
        if directory == root or directory.parent == directory:
            return False
        directory = directory.parent
