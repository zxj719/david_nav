# ADR-0001：导航后台执行，入站不等待导航终态

状态：已实现并以纯替身回归验证；真实任务验收另列。适用当前拆包布局。

## 背景原文（保留）

来源 `93676a5:david_nav/david_nav/navigation_executor_core.py`；行号可能漂移，
下面是原注释，不用新架构叙述替换历史事故记录。

```python
# nav WS v1.1(2026-07-27):bridge.start_navigation 会 await 到 nav2
# 终态,inline await 会把 WS 收包循环整个卡死 → abort_navigation 读
# 不进来,"随时可中断"不成立。导航后台 task 化,core 持有引用以便
# abort/preempt 时 cancel(否则任务还挂在旧 goal 的重试等待上,
# 会把旧目标重新发出去)。

# 后台 task 化(v1.1):handle_message 立刻返回,WS 循环继续收
# abort/ping。sleep(0) 让新任务先跑到第一个 await(真实 bridge
# 在此已把 goal 发给 nav2)。
```

## 决定与当前落点

`nav_tasks.core.TaskManager` 通过 `TaskSupervisor` 持有技能任务，
`nav_executor.protocol.TaskCommandAdapter.handle_message` 只等待 admission，
不等待导航终态。核心状态限于一个 asyncio loop，ROS 回调按宿主约定回到该 loop。
抢占不是抛弃旧 task：先取消/等待后端与技能清理，迟到 handle 也必须处置。
清理未证明完成时保留 ERROR/busy 和资源，不能假报 IDLE 或接受下一段运动。

“可中断”不表示所有命令处理零等待：abort/preempt 会等清理，mark 会等位姿捕获/存储。
无效新目标不清掉旧 owner；R2 崩溃和清理失败详见
[ADR-0008](../../nav_tasks/docs/adr-0008-crash-recovery.md)。
传输重连不会重建 manager，也不因掉线取消已接收导航。

## 证据与后果

`nav_executor/test/test_task_wire_golden.py` 的 G1/G9–G14 及
`nav_tasks/test/test_task_manager.py`、`test_task_completion.py` 固化 admission、
abort、抢占、迟到反馈和关闭所有权。使用内存假后端，不把它当实体停车时限证明。
不引入新线协议字段；后台任务必须有强引用和异常归属，不能 fire-and-forget。
