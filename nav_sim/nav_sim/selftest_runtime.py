"""管理真实 Nav2 私有自测的资源所有权，由显式入口检查后按需导入."""

import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Callable, Mapping

import websockets

from nav_bringup.navrun.owned_process import OwnedProcess
from nav_sim.model import finite_number, require_sandbox
from nav_sim.selftest_coordinator import drive, Peer
from nav_sim.selftest_evidence import check_evidence
from nav_sim.selftest_launch import close_fixture
from nav_sim.selftest_plan import load_scene
from nav_sim.selftest_scene import Scene
from nav_sim.selftest_trace import audit_processes, EvidenceTrace


async def _cleanup(scene, child, observer, server, peer, result):
    """停止自有实验资源并在退出期间继续排空观察，不把尾部清理错误漏判.

    Args:
        scene: 私有实验场景，用于核对所有计划子进程的退出证据。
        child: 本次创建的启动器所有者，或尚未创建时的 None。
        observer: 只读 ROS 观察器，或 None；关闭前保留十次 0.02 秒排空机会。
        server: 本次持有的 WS 服务端，子进程结束后关闭并等待。
        peer: 服务端会话，先标记 stopping，避免把预期断联认作运行故障。
        result: 原地追加错误并记录 cleanup_complete 的结果字典。

    观察失败记入 errors，但 cleanup_complete 只表示相应资源清理状态；
    最终成功必须另行要求 errors 为空。排空窗口不保证远端 DDS 持久交付。

    Raises:
        asyncio.CancelledError: 清理任务自身被取消；外层用 shield 保留它的所有权。
    """
    peer.stopping = True
    clean = child is not None
    observing = observer is not None

    def observe():
        nonlocal observing
        if observing:
            try:
                observer.step()
            except Exception as error:
                observing = False
                result['errors'].append('observer_' + type(error).__name__)

    try:
        if child is not None:
            ending = asyncio.create_task(close_fixture(child))
            while not ending.done():
                observe()
                await asyncio.wait({ending}, timeout=0.02)
            code = ending.result()
            if code not in (0, 130, -2):
                raise RuntimeError('abnormal launcher exit')
            audit_processes(scene, final=True)
    except Exception as error:
        clean = False
        result['errors'].append('cleanup_' + type(error).__name__)
    finally:
        try:
            server.close()
            await server.wait_closed()
        except Exception as error:
            clean = False
            result['errors'].append('server_close_' + type(error).__name__)
        finally:
            if observer is not None:
                # Continue draining after child/WS closure. One spin budget can
                # otherwise be exhausted by TF before the final task event runs.
                for _ in range(10):
                    observe()
                    await asyncio.sleep(0.02)
                try:
                    observer.close()
                except Exception as error:
                    clean = False
                    result['errors'].append('observer_close_' + type(error).__name__)
            result['cleanup_complete'] = clean


def require_idle_host() -> None:
    """只读扫描受保护栈，拒绝在用户定位或导航进程旁启动自测.

    排除当前进程及祖先；检查只是当时快照，不冻结后续主机状态，也不发信号。

    Raises:
        RuntimeError: 域不符、发现非祖先受保护进程，或 pgrep 返回码不可靠。
        OSError: 无法读取进程祖先信息或执行只读扫描。
        subprocess.TimeoutExpired: pgrep 超过 5 秒未结束。
        ValueError: 进程记录或扫描结果中的 PID 无法解析。
    """
    require_sandbox(os.environ)
    ancestors, pid = set(), os.getpid()
    while pid > 1 and pid not in ancestors:
        ancestors.add(pid)
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        pid = int(fields[1])
    scan = subprocess.run([
        'pgrep', '-f', 'run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|'
        'wheeltec_driver_node|navigation_executor|dock_navigator|rviz2|component_container',
    ], capture_output=True, text=True, check=False, timeout=5)
    if scan.returncode not in (0, 1) or {int(value) for value in scan.stdout.split()} - ancestors:
        raise RuntimeError('selftest requires an idle host; no processes were signalled')


async def execute(prepare: Callable[[str], Scene], timeout: float,
                  startup_only: bool, allow_goals: bool) -> Mapping[str, object]:
    """持有回环服务器、只读观察器和真实私有图，按证据与清理结果判定.

    常规运行异常记入结果；准备、完整性或证据记录器创建失败发生在运行
    try 块之前，可能没有 result.json。进入运行块后以 shield 保护自有清理，
    但清理或结果写入本身抛异常仍可能覆盖先前异常，不能保证结果总能落盘。

    Args:
        prepare: 使用实际监听回环 URI 生成场景的回调。
        timeout: 交给 drive 的有限正秒数，不覆盖此前准备与后续清理的总墙钟时间。
        startup_only: 只验证启动和清理，不发目标；此模式 complete 始终为 False。
        allow_goals: 完整实验要求的显式入口标志，不能替代外部运动授权。

    Returns:
        已排他写入的结果映射；仅两用例执行、事件/测量与清理全部通过且无错误，
        complete 才为 True。这里只验证私有自测，不等同实机验收。

    Raises:
        RuntimeError: 进入运行块前域、授权或主机空闲检查失败。
        ValueError: 参数、准备或完整性检查失败。
        OSError: 私有资产创建或结果写入失败。
        KeyboardInterrupt: 中断向外传播；已进入运行块时先尝试自有清理。
        asyncio.CancelledError: 取消向外传播；已进入运行块时先等待受保护的自有清理。
    """
    require_sandbox(os.environ)
    if finite_number(timeout, 'timeout') <= 0:
        raise ValueError('timeout must be positive')
    if type(startup_only) is not bool or type(allow_goals) is not bool:
        raise ValueError('execution switches must be booleans')
    if not startup_only and not allow_goals:
        raise RuntimeError('complete selftest requires explicit --allow-goals')
    require_idle_host()
    from nav_sim.selftest_ros import Observer

    peer = Peer()
    async with websockets.serve(peer.handle, '127.0.0.1', 0, max_size=65536, max_queue=16,
                                close_timeout=2) as server:
        scene = prepare(f'ws://127.0.0.1:{server.sockets[0].getsockname()[1]}')
        load_scene(scene.root, scene.digest)
        result = dict(schema_version=1, backend='real_nav2', executed=False, complete=False,
                      passed=False, kind='startup_only' if startup_only else 'closed_loop',
                      plan_sha256=scene.digest, measurements=[], errors=[], cleanup_complete=False)
        with EvidenceTrace(scene.root) as trace:
            peer.trace = trace
            observer = child = None
            try:
                # Recheck after preparation and before creating the first ROS participant.
                require_idle_host()
                observer = Observer(scene, trace)
                child = OwnedProcess([
                    sys.executable, '-m', 'nav_sim.selftest_launch', '--run-dir', str(scene.root),
                    '--plan-sha256', scene.digest, '--start-verified-fixture',
                ], dict(os.environ, ROS_LOG_DIR=str(scene.root / 'ros_logs')),
                    scene.root / 'launch.log')
                result['executed'] = True
                result['measurements'] = await drive(
                    scene, trace, peer, observer, child, timeout, startup_only)
            except Exception as error:
                result['errors'].append(type(error).__name__ + ': ' + str(error)[:240])
            except BaseException as error:
                result['errors'].append(type(error).__name__)
                raise
            finally:
                # Protect the entire cleanup, including WS and ROS resource closure.
                # Keep ownership until it finishes even after repeated cancellation.
                cleanup = asyncio.create_task(_cleanup(
                    scene, child, observer, server, peer, result))
                cancelled = False
                while not cleanup.done():
                    try:
                        await asyncio.shield(cleanup)
                    except asyncio.CancelledError:
                        cancelled = True
                        if 'CancelledError' not in result['errors']:
                            result['errors'].append('CancelledError')
                cleanup.result()
                if peer.error:
                    result['errors'].append(peer.error)
                if not startup_only:
                    evidence = check_evidence(scene.cases, trace.nav, trace.ws)
                    result['errors'].extend(evidence.errors)
                result['passed'] = (result['executed'] and result['cleanup_complete']
                                    and not result['errors'])
                result['complete'] = result['passed'] and not startup_only
                with (scene.root / 'result.json').open('x', encoding='utf-8') as stream:
                    json.dump(result, stream, indent=2, allow_nan=False)
                    stream.write('\n')
                if cancelled:
                    raise asyncio.CancelledError()
        return result
