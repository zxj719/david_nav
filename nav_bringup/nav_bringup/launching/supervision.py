"""由 launch 持有的有界进程重启，不做进程发现，也不向外部进程发信号.

只管本次 launch 自己起的进程：不按名字搜别人的进程，不 kill 任何不属于自己的东西。
"""

from __future__ import annotations

from dataclasses import replace
from typing import Callable, List

from launch import LaunchContext
from launch.action import Action
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited

from nav_bringup.launching.graph import exit_decision
from nav_bringup.launching.graph import LaunchGraph
from nav_bringup.launching.graph import ProcessSpec


ProcessFactory = Callable[[ProcessSpec], ExecuteProcess]


def managed_actions(graph: LaunchGraph, factory: ProcessFactory) -> List[Action]:
    """为每个自有进程构建带有限重启预算的 launch 动作.

    所有初始进程对象在返回任何动作之前就全部构造完毕：生产用的工厂必须立即解析
    可执行文件与参数，这样缺依赖会在底盘驱动启动之前就失败。每次重启都新建一个
    动作，因此不存在能活过预算的无限 respawn 协程。预算用尽后转入整栈关停，
    不是静默放弃。

    Args:
        graph: 已校验的进程图，其中不含由外部持有的进程。
        factory: 构造未执行进程动作的工厂；测试可注入。

    Returns:
        在对应进程或启动定时器之前注册的处理器。

    Raises:
        ValueError: 进程身份重复。
    """
    names = [process.name for process in graph.processes]
    if len(names) != len(set(names)):
        raise ValueError('owned process identities must be unique')
    actions = []
    for spec in graph.processes:
        actions.extend(_owned_actions(spec, factory, 0))
    return actions


def _owned_actions(spec: ProcessSpec, factory: ProcessFactory, used: int) -> List[Action]:
    """为一个自有进程注册退出策略，再按需延迟启动.

    Args:
        spec: 包含启动延迟和重启预算的进程描述。
        factory: 创建该次受管进程 action 的工厂。
        used: 已消耗的重启次数，递归重启时递增。

    Returns:
        先注册一次性退出处理器、再启动的 action 列表；重启取消额外启动延迟，
        预算耗尽时请求所属 launch 关闭，不按名称搜杀其它进程。
    """
    process = factory(spec)

    def on_exit(event: ProcessExited, context: LaunchContext) -> List[Action]:
        decision = exit_decision(spec, used, context.is_shutdown)
        if decision == 'restart':
            return _owned_actions(replace(spec, delay_sec=0.0), factory, used + 1)
        if decision == 'shutdown':
            return [EmitEvent(event=Shutdown(reason=(
                f'{spec.name} exited ({event.returncode}); restart budget {used} exhausted')))]
        return []

    handler = RegisterEventHandler(OnProcessExit(
        target_action=process, on_exit=on_exit, handle_once=True))
    start = TimerAction(period=spec.delay_sec, actions=[process]) if spec.delay_sec else process
    return [handler, start]
