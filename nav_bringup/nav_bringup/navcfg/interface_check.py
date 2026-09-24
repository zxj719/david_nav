"""调用独立打包的接口观察器，不导入它的运行时.

以子进程方式调用：本模块不 import 观察器代码，也不在本进程里初始化 ROS，
因此观察器崩溃不会带塌调用方。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
from typing import Any, Callable, Mapping, Optional, Tuple

from nav_bringup.launching.executables import resolve_executable
from nav_bringup.navcfg.live_check import Finding


@dataclass(frozen=True)
class InterfaceResult:
    """已校验的第一版观察器响应.

    Attributes:
        status: 0 表示约定满足，1 表示有错误，3 表示定位尚未就绪。
        findings: 结构化的约定判定结果，原样来自观察器模块，本模块不改写。
        evidence: 窗口内的观察数据；还在等待、尚无采样时为 None。
    """

    status: int
    findings: Tuple[Finding, ...]
    evidence: Optional[Mapping[str, Any]]


def _evidence_valid(value):
    """检查接口探针返回 JSON 的结构和端点计数类型，不判断接口是否满足要求.

    Args:
        value: 候选证据对象，必须具有精确字段集合。

    Returns:
        结构合法时为 True；计数必须为非负真整数，变换对为两个字符串，
        空证据可合法但并不表示实际定位或话题就绪。
    """
    if not isinstance(value, dict) or set(value) != {
            'transforms', 'publishers', 'subscribers', 'samples', 'types'}:
        return False

    def strings(items):
        return isinstance(items, list) and all(isinstance(i, str) for i in items)

    return (isinstance(value['transforms'], list)
            and all(strings(pair) and len(pair) == 2 for pair in value['transforms'])
            and strings(value['samples'])
            and isinstance(value['types'], dict)
            and all(isinstance(k, str) and strings(v) for k, v in value['types'].items())
            and all(isinstance(value[key], dict) and all(
                isinstance(topic, str) and type(count) is int and count >= 0
                for topic, count in value[key].items()) for key in ('publishers', 'subscribers')))


def parse_report(output: str, exit_code: int) -> InterfaceResult:
    """校验唯一一份最终 JSON 报告，并核对它与进程退出码是否自洽.

    报告与退出码不一致时按错误处理，不偏信其中任何一个。

    Args:
        output: 有界的合并输出；报告之前可能混有非 JSON 的 ROS 日志行。
        exit_code: 清理之后观察到的子进程状态。

    Returns:
        已校验的判定结果与原始窗口证据，供导航检查使用。

    Raises:
        ValueError: 输出格式错误、版本不支持、缺失或与退出码不一致。
    """
    documents = [json.loads(line) for line in output.splitlines() if line.startswith('{')]
    if type(exit_code) is not int or len(documents) != 1 or not isinstance(documents[0], dict):
        raise ValueError('interface checker must emit exactly one JSON result')
    data = documents[0]
    if (type(data.get('schema_version')) is not int or data['schema_version'] != 1
            or not isinstance(data.get('findings'), list)):
        raise ValueError('unsupported interface checker report')
    findings = []
    for item in data['findings']:
        if (not isinstance(item, dict) or set(item) != {'severity', 'code', 'message'}
                or item['severity'] not in ('info', 'warn', 'error')
                or not isinstance(item['code'], str) or not item['code']
                or not isinstance(item['message'], str)):
            raise ValueError('malformed interface finding')
        findings.append(Finding(**item))
    evidence = data.get('evidence')
    if evidence is not None and not _evidence_valid(evidence):
        raise ValueError('malformed interface evidence')
    errors = any(item.severity == 'error' for item in findings)
    if not ((exit_code in (0, 1) and data.get('status') == 'ready'
             and evidence is not None and bool(exit_code) == errors)
            or (exit_code == 3 and data.get('status') == 'waiting')):
        raise ValueError(f'interface checker failed or report disagrees with exit {exit_code}')
    return InterfaceResult(exit_code, tuple(findings), evidence)


def check_interface(
    run_dir: Path, sample_sec: float, stopped: Callable[[], bool],
) -> Optional[InterfaceResult]:
    """持有一个有界的只读接口检查子进程，并在返回前把它释放掉.

    子进程留在调用方的进程组里：即使本 navcfg 进程被杀，外层 navrun 仍能把它收干净，
    它永远不是脱管的后台帮手。临时输出有上限，并且只在子进程清理完成后才删除。

    Args:
        run_dir: 已校验的运行目录，内含冻结的机器人/模式输入。
        sample_sec: 子进程发现 TF 之后的正采样窗口。
        stopped: 调用方持有的信号标志；被取消时清理完成后返回 None。

    Returns:
        已校验的结果；调用方请求中断时返回 None。返回结果只反映采样窗口内的观察，
        不构成对窗口之外时间的任何结论。

    Raises:
        ValueError: 时间参数非法、可执行文件缺失，或观察器输出非法。
        LookupError: 单独安装的观察器包不存在。
        OSError: 子进程或其私有输出无法打开。
        RuntimeError: 观察器超出时间或输出预算。
    """
    if type(sample_sec) not in (int, float) or not math.isfinite(sample_sec) or sample_sec <= 0:
        raise ValueError('interface sample window must be finite and positive')
    if stopped():
        return None
    executable = resolve_executable('nav_perception_iface', 'nav_interface_check')
    command = [executable, '--profile', str(run_dir / 'params/inputs/robot.yaml'),
               '--mode', str(run_dir / 'params/inputs/mode.yaml'),
               '--sample-sec', str(sample_sec), '--wait-timeout', '5']
    with tempfile.TemporaryDirectory(prefix='navcfg-interface-') as directory:
        path = Path(directory) / 'observer.log'
        with path.open('xb') as output:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=output,
                                       stderr=subprocess.STDOUT, close_fds=True,
                                       start_new_session=False)
        try:
            deadline = time.monotonic() + sample_sec + 10.0
            while process.poll() is None:
                if stopped():
                    return None
                if time.monotonic() >= deadline or path.stat().st_size > 1024 * 1024:
                    raise RuntimeError('interface checker exceeded its time/output budget')
                time.sleep(0.05)
            if stopped():
                return None
            if path.stat().st_size > 1024 * 1024:
                raise RuntimeError('interface checker exceeded its output budget')
            return parse_report(path.read_text(encoding='utf-8'), process.returncode)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
