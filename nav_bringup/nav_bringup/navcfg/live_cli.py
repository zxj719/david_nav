"""执行需显式开启的只读在线检查，同时保持调用方对关停的所有权.

检查只观察不干预：不启动定位、不下发动作、不产生运动，也不起 ROS CLI 守护进程。
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from typing import Any

from nav_bringup.navcfg.live_check import assess, Finding, wait_localization
from nav_bringup.navcfg.live_inputs import load_live_spec
from nav_bringup.navcfg.interface_check import check_interface
from nav_bringup.recorder.events import EventLog


def run(args: argparse.Namespace) -> int:
    """检查一次已准备好的运行；绝不启动定位、动作、运动或 CLI 守护进程.

    Args:
        args: 显式的在线检查选择、run_dir、strict 标志、等待/采样/查询上限，
            以及持有方监督进程可选提供的事件文件路径。

    Returns:
        检查完成返回 0（告警与错误已打印）；strict 模式下有错误返回 1；
        输入非法或操作失败返回 2；给了诊断上限而定位仍未就绪返回 3；
        被中断返回 128＋信号号。只等待模式下返回 0 仅代表 TF 已就绪，
        不代表完整检查跑过。

    Raises:
        ImportError: 缺少必需的 ROS 运行时依赖。
    """
    if not args.live:
        print('navcfg check: select --live explicitly', file=sys.stderr)
        return 2
    stopped = threading.Event()
    received = []
    previous = {}

    def stop(signum: int, _frame: Any) -> None:
        received.append(signum)
        stopped.set()

    try:
        for value, minimum in ((args.wait_timeout, 0.0), (args.sample_sec, 0.0),
                               (args.parameter_timeout, 0.0)):
            if not math.isfinite(value) or value < minimum:
                raise ValueError('check durations must be finite and nonnegative')
        if args.sample_sec == 0.0 or args.parameter_timeout == 0.0:
            raise ValueError('sample and parameter query windows must be positive')
        spec, initial = load_live_spec(args.run_dir)
        events = EventLog(args.events) if args.events is not None else None

        def emit(finding: Finding) -> None:
            print(f'{finding.severity.upper()} [{finding.code}] {finding.message}', flush=True)
            if events is not None:
                events.append('navcfg', finding.code, finding.severity,
                              {'message': finding.message})

        # Import the ROS observer only after pure input validation succeeds.
        from nav_bringup.navcfg.live_ros import RosProbe

        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, stop)

        def _check_once() -> int:
            """执行一次有界只读观察，释放本次探针后汇总接口与配置发现.

            使用外层闭包中的配置、停止信号和事件写入器；不启动定位或发送运动目标。
            定位在采样中丢失时返回待重试状态，不能把 wait-only 就绪当作完整检查通过。

            Returns:
                0 表示非严格检查完成或 wait-only 定位就绪，1 表示严格模式发现错误，
                3 表示定位仍待就绪；停止时返回 128 加信号编号，缺少编号的接口中断
                回退为 130。操作异常交由外层 run 的异常边界处理。
            """
            with RosProbe(spec) as probe:
                ready = wait_localization(probe.localized, probe.spin, time.monotonic,
                                          stopped.is_set, emit, args.wait_timeout)
                if not ready:
                    return 128 + received[0] if received else 3
                if args.wait_localization_only:
                    return 0
                deadline = time.monotonic() + args.sample_sec
                while not stopped.is_set() and time.monotonic() < deadline:
                    probe.spin(max(0.0, min(0.1, deadline - time.monotonic())))
                if stopped.is_set():
                    return 128 + received[0]
                snapshot = probe.snapshot(stopped.is_set, args.parameter_timeout)
                if stopped.is_set():
                    return 128 + received[0]
            findings = initial + assess(spec, snapshot)
            if not snapshot.localized:
                for finding in findings:
                    emit(finding)
                return 3
            interface = check_interface(args.run_dir, args.sample_sec, stopped.is_set)
            if stopped.is_set() or interface is None:
                return 128 + received[0] if received else 130
            findings += interface.findings
            if events is not None:
                events.append('navcfg', 'interface_snapshot', 'info',
                              {'status': interface.status, 'evidence': interface.evidence})
            for finding in findings:
                emit(finding)
            if interface.status == 3:
                if not any(item.code == 'localization_waiting' for item in findings):
                    emit(Finding('info', 'localization_waiting', '等待在 RViz 手动给初始位姿'))
                return 3
            if interface.status == 0:
                emit(Finding('info', 'interface_contracts_ready',
                             'localization/chassis/perception contracts satisfied '
                             'in sample window'))
            errors = sum(item.severity == 'error' for item in findings)
            warnings = sum(item.severity == 'warn' for item in findings)
            print(f'navcfg check: {errors} error(s), {warnings} warning(s)', flush=True)
            return 1 if args.strict and errors else 0

        while True:
            result = _check_once()
            if result != 3 or args.wait_timeout > 0.0:
                return result
            emit(Finding('info', 'localization_retry',
                         'localization pending; waiting before a fresh full check'))
            if stopped.wait(0.25):
                return 128 + received[0] if received else 130
    except KeyboardInterrupt:
        return 130
    except (ValueError, LookupError, OSError, RuntimeError) as error:
        print(f'navcfg check: {error}', file=sys.stderr)
        return 128 + received[0] if received else 2
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
