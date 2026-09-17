"""只启动通过摘要校验的私有图，本进程不提交目标.

外层自测运行器持有整个进程组，并须另行获得目标发送授权、实现观察及结果检查。
单独启动本模块不代表导航实验已完成；它也不授予外层运动权限。
"""

import argparse
import asyncio
import json
import math
import os
from pathlib import Path
import signal
import time
from typing import Optional, Sequence

from nav_sim.model import require_sandbox
from nav_sim.selftest_plan import load_scene
from nav_bringup.navrun.owned_process import OwnedProcess


_CLI_DESCRIPTION = """Launch only a digest-verified private graph; this process never submits goals.

The outer selftest runner owns this entire process group. It must explicitly
authorize and implement its own goal dispatch, observation and result checks.
Starting this module alone is not a completed navigation experiment.
"""


async def close_fixture(child: OwnedProcess, timeout_sec: float = 15.0) -> int:
    """中断并回收调用方仍持有的启动器进程组，等待期间让出事件循环.

    向仍保留所有权的整个组发送 SIGINT；只中断组长不足以让 Humble
    交互式启动器转发给子进程。超时后尝试 OwnedProcess 组清理，并报告超时。
    不能仅凭组长返回码宣称全部子进程正常退出，调用方还须核对逐进程审计。

    Args:
        child: 调用方在本事件循环线程创建、尚未放弃所有权的启动器。
        timeout_sec: 组清理前等待自然退出的有限正秒数。

    Returns:
        成功完成组清理和回收后的启动器退出码。

    Raises:
        ValueError: 宽限期不是有限正数。
        TimeoutError: 组长未在宽限期内退出；已经尝试组清理。
        OSError: 发信号、所有权核验或组清理失败；不保证失败后所有资源均已释放。
    """
    if type(timeout_sec) not in (int, float) or not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError('fixture shutdown timeout must be positive and finite')
    if child.poll() is None:
        os.killpg(child.pid, signal.SIGINT)
    deadline = time.monotonic() + timeout_sec
    while child.poll() is None and time.monotonic() < deadline:
        await asyncio.sleep(0.02)
    graceful = child.poll() is not None
    status = child.close(timeout_sec=0)
    if not graceful:
        raise TimeoutError('fixture required forced group cleanup')
    return status


def main(args: Optional[Sequence[str]] = None) -> int:
    """核验私有输入后运行共享启动适配器，帮助路径提前退出.

    显式启动开关不同于发目标许可。本进程不创建 Nav2 目标客户端、速度发布器
    或 WS 服务器；域和摘要检查先于 ROS 运行时导入和子进程创建。

    Args:
        args: 内部启动参数，None 时读取进程命令行。

    Returns:
        启动服务退出码；捕获键盘中断时为 130。

    Raises:
        RuntimeError: 未显式选择域 99。
        ValueError: 场景或冻结资产未通过完整性检查。
        OSError: 私有资产或排他创建的进程审计文件无法打开。
        SystemExit: 显示帮助或命令行参数解析失败。
    """
    parser = argparse.ArgumentParser(description=_CLI_DESCRIPTION)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--plan-sha256', required=True)
    parser.add_argument('--start-verified-fixture', required=True, action='store_true')
    options = parser.parse_args(args)
    require_sandbox(os.environ)
    scene = load_scene(options.run_dir, options.plan_sha256)

    from launch import LaunchDescription, LaunchService
    from launch.actions import GroupAction, RegisterEventHandler
    from launch.event_handlers import OnProcessExit, OnProcessStart
    from launch_ros.actions import PushRosNamespace

    from nav_bringup.launching.runtime import launch_actions

    # Every child ROS Node inherits the namespace. The plain readiness-gated
    # executor already has explicit ROS remappings and a private action name.
    actions = launch_actions(scene.graph)
    with (scene.root / 'process_events.jsonl').open('x', encoding='utf-8') as audit:
        def observe(event, _context):
            record = {'name': event.process_name, 'pid': event.pid,
                      'node': getattr(event.action, 'node_name', None),
                      'stamp': time.monotonic(),
                      'event': 'exit' if hasattr(event, 'returncode') else 'start'}
            if record['event'] == 'exit':
                record['returncode'] = event.returncode
            audit.write(json.dumps(record, allow_nan=False) + '\n')
            audit.flush()

        description = LaunchDescription([
            RegisterEventHandler(OnProcessStart(on_start=observe)),
            RegisterEventHandler(OnProcessExit(on_exit=observe)),
            GroupAction([PushRosNamespace(scene.namespace), *actions]),
        ])
        service = LaunchService(argv=[])
        service.include_launch_description(description)
        try:
            return service.run()
        except KeyboardInterrupt:
            return 130


if __name__ == '__main__':
    raise SystemExit(main())
