"""用于 navcfg 子进程集成的独立只读接口检查入口.

只观察定位与底盘接口，不选择或启动提供方。
命令行帮助沿用独立的兼容文案，不以本模块的中文说明改变既有 CLI 输出。
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import sys
import time
from typing import Optional, Sequence

from nav_perception_iface.configuration import load_contract_profile
from nav_perception_iface.contracts import check_contracts


# 保留已发布的帮助文案，避免翻译源码说明时改变 CLI 输出。
_CLI_DESCRIPTION = (
    'Independent read-only interface probe, suitable for navcfg subprocess integration.')


def main(argv: Optional[Sequence[str]] = None) -> int:
    """等待人工定位，再把当前接口证据与检查结论报告为 JSON.

    只在纯输入检查后加载 ROS 探针，定位未就绪时不开始数据订阅。
    有候选报告时在探针退出和信号处理器恢复后输出；清理失败可覆盖候选，
    缺依赖等未处理异常路径不保证一定输出 JSON。等待提示写入标准错误。

    Args:
        argv: 命令行参数；省略时使用进程参数。profile 和可选 mode 为显式文件，
            不选择或启动任何接口提供方。

    Returns:
        0 表示契约满足，1 表示契约错误，2 表示已捕获的操作错误，
        3 表示有界等待后定位仍未就绪；中断为 128 加信号编号。
        报告 status 为 ready 仍可能含错误并返回 1，不能只看 status 判成功。

    Raises:
        ImportError: 所需 ROS 运行依赖不可用。
        SystemExit: 参数解析处理帮助或拒绝非法命令行。
        Exception: 未被捕获的运行异常、信号恢复或最终输出失败。
    """
    parser = argparse.ArgumentParser(description=_CLI_DESCRIPTION)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--mode', type=Path)
    parser.add_argument('--sample-sec', type=float, default=2.0)
    parser.add_argument('--wait-timeout', type=float, default=0.0)
    parser.add_argument('--no-chassis', action='store_true',
                        help='explicit standalone diagnostic without a driver contract')
    args = parser.parse_args(argv)
    received = []
    previous = {}
    document = []

    def stop(signum, _frame):
        received.append(signum)

    def report(status, findings=(), snapshot=None):
        evidence = None if snapshot is None else {
            'transforms': sorted(snapshot.transforms), 'publishers': dict(snapshot.publishers),
            'subscribers': dict(snapshot.subscribers), 'samples': sorted(snapshot.samples),
            'types': {key: sorted(value) for key, value in snapshot.types.items()}}
        # A close failure can replace this candidate before anything is emitted.
        document[:] = [{'schema_version': 1, 'status': status,
                        'findings': [asdict(item) for item in findings], 'evidence': evidence}]

    try:
        if (not math.isfinite(args.sample_sec) or args.sample_sec <= 0
                or not math.isfinite(args.wait_timeout) or args.wait_timeout < 0):
            raise ValueError('sample window must be positive; wait timeout must be nonnegative')
        profile = load_contract_profile(args.profile, args.mode)
        from nav_perception_iface.contract_probe import ContractProbe

        # Validate the contract before creating a context or changing signal handlers.
        probe = ContractProbe(profile, chassis_required=not args.no_chassis)
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[signum] = signal.signal(signum, stop)
        with probe:
            started = next_hint = time.monotonic()
            while not received and not probe.localized():
                now = time.monotonic()
                if args.wait_timeout and now - started >= args.wait_timeout:
                    report('waiting')
                    return 3
                if now >= next_hint:
                    print('等待在 RViz 手动给初始位姿', file=sys.stderr, flush=True)
                    next_hint = now + 5.0
                remaining = args.wait_timeout - (now - started) if args.wait_timeout else 0.1
                probe.spin(max(0.0, min(0.1, remaining)))
            if received:
                report('interrupted')
                return 128 + received[0]
            snapshot = probe.sample(args.sample_sec, lambda: bool(received))
            if received:
                report('interrupted')
                return 128 + received[0]
            findings = check_contracts(profile, snapshot, chassis_required=not args.no_chassis)
            if any(item.code == 'localization_waiting' for item in findings):
                report('waiting', findings, snapshot)
                return 3
            report('ready', findings, snapshot)
            return int(any(item.severity == 'error' for item in findings))
    except (ValueError, OSError, RuntimeError) as error:
        print(f'nav_interface_check: {error}', file=sys.stderr)
        report('error')
        return 128 + received[0] if received else 2
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        if document:
            print(json.dumps(document[0], ensure_ascii=False, allow_nan=False), flush=True)
