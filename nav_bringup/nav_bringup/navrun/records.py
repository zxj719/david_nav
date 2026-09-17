"""Read-only run reports and frozen tuning exports for the navrun CLI.

Reports intentionally exclude raw manifest commands, linked processes and the
environment. Explicit parameter/event exports can still contain sensitive data.
They are evidence from a run, never a replacement configuration source of truth.
"""

from __future__ import annotations

import argparse
import difflib
import json
from pathlib import Path
import sys
from typing import Any, Dict

import yaml

from nav_bringup.navcfg import render
from nav_bringup.navrun import record_reader
from nav_bringup.recorder.run_dir import default_runs_root

_FIELDS = ('run_id', 'mode', 'robot', 'site', 'started_at', 'ended_at',
           'duration_sec', 'exit_code', 'exit_reason')


def _mapping(text: str) -> Dict[str, Any]:
    # Reject aliases before loading: expansion/recursive merges are not needed
    # for recorder-generated artifacts, and must not become unbounded work.
    """在加载 YAML 前拒绝别名，再要求记录制品的顶层为字典.

    记录检查不需要别名展开或递归合并，先扫描可避免这些结构造成额外展开工作。
    这不是全部 YAML 资源限制；字节预算由外层读取器执行。

    Args:
        text: 已由有界读取器取得的 YAML 制品文本。

    Returns:
        无 YAML 别名且顶层为字典的解析结果。

    Raises:
        ValueError: 检测到别名或顶层不是字典。
        yaml.YAMLError: YAML 扫描或安全加载失败。
    """
    for token in yaml.scan(text):
        if isinstance(token, yaml.tokens.AliasToken):
            raise ValueError('aliases are not supported in inspected mappings')
    value = yaml.safe_load(text)
    if not isinstance(value, dict):
        raise ValueError('expected an artifact mapping')
    return value


def _brief(reader: record_reader.RecordReader) -> Dict[str, Any]:
    """读取并验证运行摘要身份，只返回有界标量字段.

    Args:
        reader: 绑定特定 run_id 的受限记录读取器。

    Returns:
        固定摘要字段及依据 ended_at 推导的记录状态，不等于在线进程状态。

    Raises:
        ValueError: 清单缺失、schema/身份不符、字段非标量或超过长度限制。
        OSError: 记录读取失败。
    """
    contents = reader.read('manifest.yaml')
    if contents is None:
        raise ValueError('manifest is missing')
    manifest = _mapping(contents)
    if (type(manifest.get('schema_version')) is not int
            or manifest['schema_version'] != 1 or manifest.get('run_id') != reader.run_id):
        raise ValueError('manifest identity or schema mismatch')
    result = {key: manifest.get(key) for key in _FIELDS}
    if any(isinstance(value, (dict, list)) for value in result.values()):
        raise ValueError('manifest summary fields must be scalar')
    if any(len(str(value)) > 4096 for value in result.values()):
        raise ValueError('manifest summary field exceeds inspection limit')
    result['record_state'] = 'finished' if manifest.get('ended_at') else 'unfinished'
    return result


def _diff(reader: record_reader.RecordReader) -> Dict[str, Any]:
    """比较冻结机器人档案与模式叠加，明确不冒充生效参数比较.

    Args:
        reader: 只读的运行记录访问器。

    Returns:
        档案文本差异、独立 Nav2 覆盖及可选遥控选项；缺源快照时
        available=False，runtime_comparison 始终标记为未计算。

    Raises:
        ValueError: 冻结映射或模式格式无效。
        OSError: 读取冻结输入失败。
    """
    robot_text = reader.read('params/inputs/robot.yaml')
    mode_text = reader.read('params/inputs/mode.yaml')
    if robot_text is None or mode_text is None:
        return {'available': False, 'reason': 'frozen robot or mode input is missing'}
    robot = _mapping(robot_text)
    mode = render.parse_mode(_mapping(mode_text), Path('params/inputs/mode.yaml'))
    merged = render.deep_merge(robot, mode.profile)
    before = yaml.safe_dump(robot, sort_keys=True, allow_unicode=True).splitlines(keepends=True)
    after = yaml.safe_dump(merged, sort_keys=True, allow_unicode=True).splitlines(keepends=True)
    teleop = reader.read('params/inputs/teleop_options.yaml')
    return {
        'available': True,
        'profile_unified': ''.join(difflib.unified_diff(
            before, after, fromfile='frozen_robot', tofile='robot_plus_mode')),
        'nav2_overrides': dict(mode.nav2_overrides),
        'teleop_options': _mapping(teleop) if teleop is not None else {},
        'runtime_comparison': 'not_computed',
        'scope': 'mode profile delta; Nav2 and CLI overrides reported separately; '
                 'not a rendered-to-effective comparison',
    }


def report(root: Path, selector: str, *, params: bool = False,
           diff: bool = False, events: bool = False) -> Dict[str, Any]:
    """Inspect one record without calling ROS, signaling processes or writing data.

    Args:
        root: Runs root selected by the operator.
        selector: Run ID or latest.
        params: Include exact input, rendered and effective artifact texts.
        diff: Include the frozen mode overlay delta and separate CLI overrides.
        events: Include complete event lines and flag an incomplete final line.

    Returns:
        JSON-ready evidence; unfinished records make no process-liveness claim.

    Raises:
        ValueError: Required data is missing, malformed or exceeds read limits.
        OSError: Record files cannot be safely opened.
    """
    with record_reader.open_record(root, selector) as reader:
        result = _brief(reader)
        result['snapshot_atomic'] = False
        result['summary'] = reader.read('summary.md')
        if params:
            result['params'] = {name: reader.tree('params/' + name)
                                for name in ('inputs', 'rendered', 'effective')}
        if diff:
            result['diff'] = _diff(reader)
        if events:
            contents = reader.read('events.jsonl')
            complete = '' if contents is None else contents[:contents.rfind('\n') + 1]
            result['events'] = {'jsonl': complete, 'available': contents is not None,
                                'pending_tail': contents is not None and contents != complete}
        return result


def run(args: argparse.Namespace) -> int:
    """Print a JSON record listing or report, or a redacted diagnostic on stderr.

    Args:
        args: Parsed ls/show command options.

    Returns:
        Zero on success, two when a report cannot be read safely.
    """
    root = args.runs_root if args.runs_root is not None else default_runs_root()
    errors = (OSError, ValueError, yaml.YAMLError, RecursionError, TypeError)
    try:
        if args.subcommand == 'ls':
            result = []
            for run_id in record_reader.record_ids(root):
                try:
                    with record_reader.open_record(root, run_id) as reader:
                        result.append(_brief(reader))
                except errors:
                    result.append({'run_id': run_id, 'record_state': 'unreadable'})
        else:
            result = report(root, args.selector, params=args.params,
                            diff=args.diff, events=args.events)
        output = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False, default=str)
    except errors as error:
        # YAML/parser exceptions may include entire lines containing credentials.
        print('navrun: record inspection failed (' + type(error).__name__ +
              '); check selector, artifact integrity and read limits', file=sys.stderr)
        return 2
    print(output)
    return 0
