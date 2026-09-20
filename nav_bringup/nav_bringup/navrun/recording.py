"""规划由档案决定的 SQLite 包录制，过程中不启动进程也不写文件.

纯规划：返回的只是一条待执行的命令，录不录得成要看实际执行时的磁盘与话题情况。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from typing import Any, Mapping, Tuple


@dataclass(frozen=True)
class RecordingPlan:
    """确切的录制命令及其显式选择.

    Attributes:
        mode: 请求的策略：light、full 或 off。
        enabled: 就绪检查之后是否真的要启动录制器。
        argv: 程序与参数；未启用时为空。
        output: rosbag2 将要创建的本次运行输出目录。
        topics: 显式选中的话题，已去重。
        regex: full 模式下附加的话题选择器，或为空。
        reason: 没有安排录制时的原因说明。
    """

    mode: str
    enabled: bool
    argv: Tuple[str, ...]
    output: str
    topics: Tuple[str, ...]
    regex: str = ''
    reason: str = ''


def plan_recording(
    profile: Mapping[str, Any], mode: str, run_dir: Path, ros2: str = 'ros2',
) -> RecordingPlan:
    """校验一次录制选择，并拼出不经 shell 的调用命令.

    Args:
        profile: 生效的机器人/模式档案，record 段可缺省。
        mode: light、full 或 off；full 必须显式给出话题列表或正则。
        run_dir: 自有的绝对运行记录目录；其 bag 父目录已归录制器所有。
        ros2: 监督方解析出的 ROS CLI 可执行文件；预览时为 ros2。

    Returns:
        纯粹的录制计划。light 列表为空即显式关闭——不会回退成"录所有话题"，
        也不会自动装插件或隐式改用 MCAP。

    Raises:
        ValueError: 模式、路径、存储插件、选择器或选项非法。
    """
    if mode not in ('light', 'full', 'off'):
        raise ValueError('record mode must be light, full or off')
    if not run_dir.is_absolute() or '..' in run_dir.parts or run_dir == Path('/'):
        raise ValueError('recording requires a normalized absolute run directory')
    if not isinstance(ros2, str) or not ros2:
        raise ValueError('ROS CLI executable must be nonempty')
    output = str(run_dir / 'bag/data')
    if mode == 'off':
        return RecordingPlan(mode, False, (), output, (), reason='recording explicitly disabled')
    values = profile.get('record', {})
    if not isinstance(values, Mapping) or values.get('storage', 'sqlite3') != 'sqlite3':
        raise ValueError('record settings require sqlite3 storage')
    cache = values.get('max_cache_size_bytes', 104857600)
    if type(cache) is not int or cache < 0:
        raise ValueError('record.max_cache_size_bytes must be a nonnegative integer')
    selected = values.get(mode + '_topics', [])
    if not isinstance(selected, list) or not all(
            isinstance(topic, str) and re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', topic)
            and '//' not in topic and not topic.endswith('/') for topic in selected):
        raise ValueError('record topic selection must be a list of absolute ROS topic names')
    topics = tuple(dict.fromkeys(selected))
    regex = ''
    extra = []
    if mode == 'full':
        regex = values.get('full_regex', '')
        if not isinstance(regex, str):
            raise ValueError('record.full_regex must be a string')
        try:
            re.compile(regex)
        except re.error as error:
            raise ValueError('record.full_regex is invalid') from error
        for key, flag in (('full_include_unpublished', '--include-unpublished-topics'),
                          ('full_include_hidden', '--include-hidden-topics')):
            enabled = values.get(key, False)
            if type(enabled) is not bool:
                raise ValueError(f'record.{key} must be boolean')
            if enabled:
                extra.append(flag)
        compression = values.get('full_compression', 'none')
        if compression not in ('none', 'zstd'):
            raise ValueError('full compression must be none or zstd')
        if compression == 'zstd':
            extra += ['--compression-mode', 'file', '--compression-format', 'zstd']
        if not topics and not regex:
            raise ValueError('full recording requires an explicit topic or regex selection')
    elif not topics:
        return RecordingPlan(mode, False, (), output, (), reason='empty light topic selection')
    argv = [ros2, 'bag', 'record', '--storage', 'sqlite3',
            '--max-cache-size', str(cache), '--output', output, *extra, *topics]
    if regex:
        argv += ['-e', regex]
    return RecordingPlan(mode, True, tuple(argv), output, topics, regex)
