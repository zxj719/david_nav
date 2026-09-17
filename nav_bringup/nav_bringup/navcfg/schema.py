"""Parameter schemas declared by each package, and their aggregation.

Every package that consumes robot profile keys declares them in its own
``config/schema.yaml`` (refactor plan §12.1); there is no separate parameter
module. This module parses those files, merges them into one
:class:`ParameterCatalog` and checks profile values against it. ``navcfg
validate`` and ``navcfg doc`` (PROGRESS P2.5, P2.6) build on the catalog.

Schema file format, version 1::

    schema_version: 1
    package: nav_bringup
    parameters:
      record.storage:
        type: string            # bool | int | float | string | list | map
        description: rosbag2 storage plugin
        default: sqlite3        # optional; null means "unset"; required keys have none
        choices: [sqlite3]      # optional, scalar types only
        unit: s                 # optional, free text
        range: [0.0, null]      # optional, int and float only; inclusive; null is open
        items: string           # optional, list only: type of every element
        required: false         # optional
        tuning: docs/tuning.md  # optional link to tuning notes

Keys are dotted profile paths of lowercase words, for example
``limits.max_vel_x``. A key may not be declared by two packages, and no key may
be nested inside another declared key.
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
    """A schema file is malformed or conflicts with another schema."""


@dataclass(frozen=True)
class ParameterSpec:
    """Declaration of one profile key.

    Attributes:
        key: Dotted profile path, for example ``limits.max_vel_x``.
        package: Package that consumes and documents the key.
        value_type: One of :data:`TYPES`; the ``type`` field in the file.
        description: What the key controls.
        has_default: Whether the declaration has a ``default`` field.
        default: Value used when a profile omits the key; ``None`` means unset.
        unit: Unit of the value, if any.
        value_range: Inclusive ``(minimum, maximum)``; a ``None`` bound is open.
        choices: Allowed values, if restricted.
        items: Element type of a ``list`` value, if restricted.
        required: Whether every profile must set the key.
        tuning: Link to tuning notes, if any.
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
    """The parameters one package declares.

    Attributes:
        package: Package name.
        path: File the schema was read from.
        parameters: Declarations ordered by key.
    """

    package: str
    path: Path
    parameters: Tuple[ParameterSpec, ...]


@dataclass(frozen=True)
class ProfileIssue:
    """A problem found in a robot profile.

    Attributes:
        key: Dotted profile path.
        severity: ``error`` rejects the profile; ``warn`` is reported only.
        message: Description including the key.
    """

    key: str
    severity: str
    message: str


def parse_schema(data: Any, path: Union[str, Path]) -> PackageSchema:
    """Validate the parsed content of a schema file.

    Args:
        data: Content of a schema file as loaded from YAML.
        path: File the data came from, used in messages.

    Returns:
        The package schema.

    Raises:
        SchemaError: If the data does not follow the schema format.
    """
    source = str(path)
    if not isinstance(data, dict):
        raise SchemaError(f'{source}: a schema must be a mapping')
    version = data.get('schema_version')
    if version != SCHEMA_VERSION:
        raise SchemaError(f'{source}: schema_version must be {SCHEMA_VERSION}, got {version!r}')
    package = data.get('package')
    if not isinstance(package, str) or not _PACKAGE_RE.match(package):
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
    """Read and validate a schema file.

    Args:
        path: The ``config/schema.yaml`` file.

    Returns:
        The package schema.

    Raises:
        SchemaError: If the file cannot be read or does not follow the format.
    """
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise SchemaError(f'{path}: cannot read schema: {error}') from error
    return parse_schema(data, path)


def check_value(spec: ParameterSpec, value: Any) -> List[str]:
    """Check a value against a declaration.

    Args:
        spec: The declaration.
        value: The value from a profile or a default.

    Returns:
        Problem descriptions; empty if the value is valid.
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
    """All parameter declarations of a set of packages."""

    def __init__(self, schemas: Iterable[PackageSchema]) -> None:
        """Merge package schemas.

        Args:
            schemas: Schemas to merge, at most one per package.

        Raises:
            SchemaError: If a package has two schemas, a key is declared by two
                packages, or a key is nested inside another declared key.
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
        """Tuple[str, ...]: Names of the merged packages, sorted."""
        return tuple(sorted(self._schemas))

    @property
    def specs(self) -> Tuple[ParameterSpec, ...]:
        """Tuple[ParameterSpec, ...]: All declarations, sorted by key."""
        return tuple(self._specs[key] for key in sorted(self._specs))

    def get(self, key: str) -> Optional[ParameterSpec]:
        """Look up a declaration.

        Args:
            key: Dotted profile path.

        Returns:
            The declaration, or ``None`` if no package declares the key.
        """
        return self._specs.get(key)

    def defaults(self) -> Dict[str, Any]:
        """Collect declared defaults.

        Returns:
            Mapping of each key that has a default to that default.
        """
        return {spec.key: spec.default for spec in self.specs if spec.has_default}

    def check_profile(
        self,
        profile: Mapping[str, Any],
        meta_keys: Sequence[str] = PROFILE_META_KEYS,
    ) -> List[ProfileIssue]:
        """Check a nested robot profile against the declarations.

        Args:
            profile: Profile as loaded from YAML.
            meta_keys: Top-level keys that describe the file itself.

        Returns:
            Errors for invalid values and missing required keys, and warnings
            for keys no package declares, in profile order followed by the
            missing keys.
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
    """List the install prefixes of ``AMENT_PREFIX_PATH``.

    Args:
        environ: Environment, typically ``os.environ``.

    Returns:
        Prefixes in overlay order, the most specific first.
    """
    entries = environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    return [Path(entry) for entry in entries if entry]


def find_installed_schemas(prefixes: Sequence[Union[str, Path]]) -> List[Path]:
    """Find the schema files of installed packages.

    Args:
        prefixes: Install prefixes in overlay order.

    Returns:
        ``<prefix>/share/<package>/config/schema.yaml`` files sorted by
        package; for a package in several prefixes the first prefix wins.
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
    """Find the schema files of packages in a source tree.

    Args:
        source_root: Workspace ``src`` directory.

    Returns:
        ``config/schema.yaml`` files of directories that contain a
        ``package.xml``, sorted by path; packages under a ``COLCON_IGNORE``
        are skipped.
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
    """Load schema files and merge them.

    Args:
        paths: Schema files.

    Returns:
        The merged catalog.

    Raises:
        SchemaError: If a file is malformed or the schemas conflict.
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
    if not isinstance(key, str) or not _KEY_RE.match(key):
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
