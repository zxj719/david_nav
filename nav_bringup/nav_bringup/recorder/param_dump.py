"""通过无 daemon 的 ROS CLI 记录当前发现节点的参数文本.

按 PLAN §13.2 写入 params/effective，沿用金样采集的跳过模式、
每命令超时及节点重试规则。非空且退出码为零的文本才写入，
不解析 YAML 或验证它覆盖全部参数，也不保证跨节点一致快照。

不同非跳过节点映射到同名文件，或命中报告保留文件名时，
整组冲突节点失败并跳过请求，不覆盖节点文件。合法无冲突名字仍使用
既有斜杠转双下划线的金样格式，报告数据结构不变。
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import fnmatch
import functools
from pathlib import Path
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

import yaml

from nav_bringup.recorder import manifest

# Humble ComponentManager explicitly disables its own parameter services. Only
# our exact helper name is excluded; every loaded component remains in the dump.
DEFAULT_SKIP_PATTERNS = (
    '*_rclcpp_node', '*/transform_listener_impl_*', '*/_ros2cli_*', '/nav2_container')
NODES_FILE_NAME = 'nodes.txt'
REPORT_FILE_NAME = 'dump_report.yaml'


@dataclass(frozen=True)
class DumpSettings:
    """保存图发现与参数采集选项，不在构造时验证数值.

    Attributes:
        spin_time_sec: 传给 --spin-time 的发现等待秒数。
        timeout_sec: 默认 runner 的每条命令超时秒数，不是整图采集上限。
        attempts: 每个节点最多尝试次数；非正数会使该节点不尝试而失败。
        retry_delay_sec: 相邻失败尝试之间的等待秒数。
        skip_patterns: 不采集的节点名称模式，使用大小写敏感 fnmatch。
    """

    spin_time_sec: float = 5.0
    timeout_sec: float = 30.0
    attempts: int = 3
    retry_delay_sec: float = 2.0
    skip_patterns: Tuple[str, ...] = DEFAULT_SKIP_PATTERNS


@dataclass(frozen=True)
class DumpReport:
    """保存本次图发现和参数写入的分类结果.

    Attributes:
        nodes: 去重且排序后的节点名。
        dumped: 本次返回成功写入的节点，不证明参数文本语义有效。
        skipped: 匹配跳过模式的节点。
        failed: 转储失败、输出为空或输出文件名冲突的节点。
        duplicates: 图列表里重复出现的名称，仅提示重名，不证明有外来进程。
        list_failed: 节点列表命令是否返回非零。
    """

    nodes: Tuple[str, ...]
    dumped: Tuple[str, ...]
    skipped: Tuple[str, ...]
    failed: Tuple[str, ...]
    duplicates: Tuple[str, ...] = ()
    list_failed: bool = False

    @property
    def ok(self) -> bool:
        """bool: 列表未失败且 failed 为空；空图、全部跳过或有重名仍可为真."""
        return not self.list_failed and not self.failed

    def to_dict(self) -> Dict[str, Any]:
        """复制分类元组为可写入 YAML/JSON 的列表.

        Returns:
            含各节点分类列表与 list_failed 布尔值的映射，不重新校验报告。
        """
        return {
            'nodes': list(self.nodes),
            'dumped': list(self.dumped),
            'skipped': list(self.skipped),
            'failed': list(self.failed),
            'duplicates': list(self.duplicates),
            'list_failed': self.list_failed,
        }


def node_file_name(node: str) -> str:
    """将节点名映射为金样兼容的参数文件名，不保证映射唯一.

    Args:
        node: 预期为完整 ROS 节点名；本函数不验证完整名称语法。

    Returns:
        去掉一个开头斜杠、其它斜杠改为双下划线，再添加 .yaml。
        /a/b 与 /a__b 会得到相同结果，整图写入前由 dump_graph 排除冲突。

    Raises:
        ValueError: 输入为空或只有一个斜杠。
    """
    stripped = node[1:] if node.startswith('/') else node
    if not stripped:
        raise ValueError(f'invalid node name {node!r}')
    return stripped.replace('/', '__') + '.yaml'


def should_skip(node: str, patterns: Sequence[str]) -> bool:
    """判断节点名是否匹配任一大小写敏感跳过模式.

    Args:
        node: 要匹配的完整节点名。
        patterns: fnmatchcase 支持的模式序列。

    Returns:
        至少一个模式匹配整名时为 True。
    """
    return any(fnmatch.fnmatchcase(node, pattern) for pattern in patterns)


def parse_node_list(output: str) -> Tuple[List[str], List[str]]:
    """从命令输出提取以斜杠开头的非空名称行.

    Args:
        output: 节点列表命令返回的文本，可包含告警行。

    Returns:
        (唯一排序名称, 重复出现的排序名称)。去掉每行两侧空白并忽略
        不以斜杠开头的行，不执行完整 ROS 名称语法验证。
    """
    counts = Counter(line.strip() for line in output.splitlines() if line.strip().startswith('/'))
    names = sorted(counts)
    return names, [name for name in names if counts[name] > 1]


def default_runner(settings: DumpSettings) -> manifest.CommandRunner:
    """绑定每命令超时选项，构造默认执行器但不启动命令.

    Args:
        settings: 参数采集选项。

    Returns:
        绑定 timeout_sec 的 manifest.run_command；注入的其它 runner
        不自动获得此包装，也不能据此承诺整图采集总时限。
    """
    return functools.partial(manifest.run_command, timeout_sec=settings.timeout_sec)


def dump_graph(
    output_dir: Union[str, Path],
    runner: Optional[manifest.CommandRunner] = None,
    settings: DumpSettings = DumpSettings(),
    sleep: Callable[[float], None] = time.sleep,
) -> DumpReport:
    """采集已发现且不冲突的节点参数，写入节点列表和分类报告.

    不同的非跳过节点映射为相同文件名，或命中报告保留文件名时，
    将所有冲突节点列入 failed，不请求其参数或写入该节点文件。
    普通冲突目标的旧文件保留，无冲突节点仍按原规则采集；
    保留的报告路径仍用于写入本次报告，不保留其旧内容。

    列表只请求一次，参数命令按节点重试；没有整图总时限。
    失败转储不清除已有节点文件，列表失败也不删除旧 nodes.txt，
    调用方须结合本次报告而非仅凭文件存在判断成功。文件写入非事务，
    未处理的文件或 runner 异常可能留下部分制品并阻止报告更新。

    Args:
        output_dir: 输出目录，不存在时创建。
        runner: 命令执行器；None 时使用带单次超时的默认实现。
        settings: 发现、超时、重试和跳过设置。
        sleep: 失败尝试之间的等待函数，可在测试中替换。

    Returns:
        本次写入、跳过、失败和重复名称的报告；非空参数文本不做 YAML 校验。
    """
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    run = runner if runner is not None else default_runner(settings)
    spin_time = f'{settings.spin_time_sec:g}'
    code, listing = run(['ros2', 'node', 'list', '--no-daemon', '--spin-time', spin_time])
    if code != 0:
        report = DumpReport(nodes=(), dumped=(), skipped=(), failed=(), list_failed=True)
    else:
        names, duplicates = parse_node_list(listing)
        (output / NODES_FILE_NAME).write_text(
            ''.join(f'{name}\n' for name in names), encoding='utf-8')
        dumped: List[str] = []
        skipped: List[str] = []
        failed: List[str] = []
        destinations = {
            name: node_file_name(name) for name in names
            if not should_skip(name, settings.skip_patterns)
        }
        destination_counts = Counter(destinations.values())
        for name in names:
            if name not in destinations:
                skipped.append(name)
            elif (destination_counts[destinations[name]] > 1
                  or destinations[name] in (NODES_FILE_NAME, REPORT_FILE_NAME)):
                failed.append(name)
            elif _dump_node(name, output / destinations[name], run, settings, sleep):
                dumped.append(name)
            else:
                failed.append(name)
        report = DumpReport(
            nodes=tuple(names),
            dumped=tuple(dumped),
            skipped=tuple(skipped),
            failed=tuple(failed),
            duplicates=tuple(duplicates),
        )
    with (output / REPORT_FILE_NAME).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(report.to_dict(), handle, sort_keys=False)
    return report


def _dump_node(
    node: str,
    destination: Path,
    run: manifest.CommandRunner,
    settings: DumpSettings,
    sleep: Callable[[float], None],
) -> bool:
    """按固定尝试次数采集节点参数，仅成功且非空的输出才覆盖写入.

    不解析 YAML；失败或空白输出不清除既有目标。命令超时依赖传入 runner，
    不限制 runner 或 sleep 自身的阻塞时间，写入失败可能部分覆盖文件。

    Args:
        node: ROS 节点完整名称。
        destination: 已选定的输出文件，成功时覆盖写入。
        run: 返回退出码和文本的执行器，默认调用方会配置单命令超时。
        settings: 发现等待、重试次数和间隔，时间单位秒。
        sleep: 可注入的秒制等待函数。

    Returns:
        至少一次成功写入时为 True；所有命令失败或空输出时为 False。

    Raises:
        OSError: 参数文件写入失败。
        Exception: runner 或等待函数抛出未转换异常。
    """
    command = ['ros2', 'param', 'dump', '--no-daemon', '--spin-time',
               f'{settings.spin_time_sec:g}', node]
    for attempt in range(1, settings.attempts + 1):
        code, output = run(command)
        if code == 0 and output.strip():
            destination.write_text(output, encoding='utf-8')
            return True
        if attempt < settings.attempts:
            sleep(settings.retry_delay_sec)
    return False
