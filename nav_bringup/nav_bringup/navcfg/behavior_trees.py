"""从机器人档案渲染行为树，不导入任何运行时插件实现."""

from __future__ import annotations

import math
from pathlib import Path
import re
from string import Template
from typing import Any, Dict, Mapping
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

from nav_bringup.navcfg import render

_TOKEN = re.compile(r'^[a-z][a-z0-9_]*$')
_FORMATS = frozenset({'text', '.1f', '.2f', '.3f', '.4f', '.6f'})
BT_PARAMETERS = (
    'default_bt_xml_filename', 'default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml')


def parse_bindings(data: Any) -> Dict[str, Dict[str, str]]:
    """校验模板变量到档案键的映射表.

    Args:
        data: YAML 加载后的对象。

    Returns:
        模板变量名到 key/format 的映射。

    Raises:
        render.RenderError: 格式版本、变量名或数值格式不合法。
    """
    if (not isinstance(data, dict) or data.get('schema_version') != 1
            or set(data) != {'schema_version', 'bindings'}
            or not isinstance(data['bindings'], dict)):
        raise render.RenderError('BT bindings require schema_version: 1 and a bindings mapping')
    bindings = {}
    for token, entry in data['bindings'].items():
        if not isinstance(token, str) or not _TOKEN.fullmatch(token):
            raise render.RenderError(f'invalid BT token: {token!r}')
        if (not isinstance(entry, dict) or set(entry) - {'key', 'format'}
                or not isinstance(entry.get('key'), str) or not entry['key']
                or entry.get('format', 'text') not in _FORMATS):
            raise render.RenderError(f'invalid BT binding: {token}')
        bindings[token] = {'key': entry['key'], 'format': entry.get('format', 'text')}
    return bindings


def _scalar(value: Any, value_format: str, key: str) -> str:
    """格式化单个属性，并拒绝非有限数和空值."""
    if isinstance(value, bool) or not isinstance(value, (str, int, float)):
        raise render.RenderError(f'{key}: BT attribute must be a string or finite number')
    if isinstance(value, (int, float)) and not math.isfinite(value):
        raise render.RenderError(f'{key}: BT attribute must be finite')
    if value_format != 'text':
        if not isinstance(value, (int, float)):
            raise render.RenderError(f'{key}: numeric BT format requires a number')
        formatted = format(value, value_format)
        # Match old trailing zeros without rounding a new robot's tuning values.
        value = formatted if float(formatted) == value else str(value)
    result = str(value)
    if not result:
        raise render.RenderError(f'{key}: BT attribute must not be empty')
    return escape(result, {'"': '&quot;', "'": '&apos;'})


def render_template(
    text: str, profile: Mapping[str, Any], bindings: Mapping[str, Mapping[str, str]],
) -> str:
    """渲染一棵树，保留黑板花括号与数值属性的原有文本格式.

    Args:
        text: 使用 string.Template 占位符的 XML。
        profile: 合并模式叠加后的机器人档案。
        bindings: 已校验的模板变量绑定表。

    Returns:
        解析校验通过的 XML 文本。

    Raises:
        render.RenderError: 变量、档案值或输出 XML 无效。
    """
    template = Template(text)
    tokens = set()
    for match in template.pattern.finditer(text):
        if match.group('invalid') is not None:
            raise render.RenderError('invalid BT template placeholder')
        token = match.group('named') or match.group('braced')
        if token:
            tokens.add(token)
    values = {}
    for token in tokens:
        if token not in bindings:
            raise render.RenderError(f'unbound BT placeholder: {token}')
        entry = bindings[token]
        values[token] = _scalar(
            render.profile_value(profile, entry['key']), entry['format'], entry['key'])
    output = template.substitute(values)
    try:
        root = ET.fromstring(output)
    except ET.ParseError as error:
        raise render.RenderError(f'invalid rendered BT XML: {error}') from error
    if root.tag != 'root' or not root.findall('BehaviorTree'):
        raise render.RenderError('BT XML needs a root containing BehaviorTree')
    return output


def render_bundle(
    template_dir: Path, binding_file: Path, profile: Mapping[str, Any],
) -> Dict[str, str]:
    """在内存中渲染目录内全部模板，失败时不产生部分输出.

    Args:
        template_dir: 含 .xml.tmpl 的目录。
        binding_file: YAML 绑定表路径。
        profile: 已合并的机器人档案。

    Returns:
        XML 文件名到渲染文本的映射。

    Raises:
        render.RenderError: 模板不存在或任一模板渲染失败。
    """
    bindings = parse_bindings(render.load_yaml(binding_file))
    paths = sorted(template_dir.glob('*.xml.tmpl'))
    if not paths:
        raise render.RenderError(f'no BT templates in {template_dir}')
    return {
        path.name.removesuffix('.tmpl'): render_template(
            path.read_text(encoding='utf-8'), profile, bindings)
        for path in paths
    }


def bind_output_paths(tree: Dict[str, Any], bundle: Mapping[str, str], directory: Path) -> None:
    """将 Nav2 的三处 BT 路径改为本次运行的渲染产物.

    Args:
        tree: 原地更新的 Nav2 参数树。
        bundle: 已渲染的 XML 文件名与内容。
        directory: 行为树输出目录。

    Raises:
        render.RenderError: Nav2 选择的文件没有对应模板。
    """
    for parameter in BT_PARAMETERS:
        path = f'bt_navigator/ros__parameters/{parameter}'
        name = Path(render.get_path(tree, path)).name
        if name not in bundle:
            raise render.RenderError(f'no BT template for selected tree: {name}')
        render.set_path(tree, path, str(directory.resolve() / name))
