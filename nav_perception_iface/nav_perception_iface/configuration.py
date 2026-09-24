"""只读接口检查器共享的机器人档案加载与字段读取."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import yaml


def profile_value(profile: Mapping[str, Any], key: str) -> Any:
    """读取点分隔档案键，不引入机器人专用默认值.

    Args:
        profile: 机器人档案映射。
        key: 点分隔的必需字段路径。

    Returns:
        档案内保存的原始值。

    Raises:
        ValueError: 路径不存在或中间节点不是映射。
    """
    value = profile
    for part in key.split('.'):
        if not isinstance(value, Mapping) or part not in value:
            raise ValueError(f'missing profile key: {key}')
        value = value[part]
    return value


def string_value(profile: Mapping[str, Any], key: str) -> str:
    """读取必需的非空话题或坐标系名称.

    Args:
        profile: 机器人档案映射。
        key: 点分隔的必需字符串字段路径。

    Returns:
        未剥除空白的原字符串。

    Raises:
        ValueError: 路径不存在或值不是非空字符串。
    """
    value = profile_value(profile, key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{key}: expected a non-empty string')
    return value


def load_contract_profile(
    profile_path: Path, mode_path: Optional[Path] = None,
) -> Dict[str, Any]:
    """读取版本一机器人档案，并深合并可选模式叠加.

    Args:
        profile_path: 基础机器人档案路径。
        mode_path: 可选模式叠加路径。

    Returns:
        不与 YAML 解析结果共享容器的有效档案。

    Raises:
        OSError: 文件无法读取。
        ValueError: YAML、版本、模式名或 profile 叠加不合法。
    """
    def read(path):
        try:
            data = yaml.safe_load(path.read_bytes())
        except yaml.YAMLError as error:
            raise ValueError(f'invalid contract configuration: {path}') from error
        if (not isinstance(data, dict) or type(data.get('schema_version')) is not int
                or data['schema_version'] != 1):
            raise ValueError(f'contract configuration requires schema_version: 1: {path}')
        return data

    def merge(original, overlay):
        result = deepcopy(original)
        for key, value in overlay.items():
            result[key] = (merge(result[key], value) if isinstance(value, dict)
                           and isinstance(result.get(key), dict) else deepcopy(value))
        return result

    profile = read(profile_path)
    if mode_path is None:
        return profile
    mode = read(mode_path)
    overlay = {} if mode.get('profile') is None else mode['profile']
    if (not isinstance(mode.get('mode'), str) or not mode['mode']
            or not isinstance(overlay, dict)):
        raise ValueError('contract mode requires a mode name and profile mapping')
    return merge(profile, overlay)
