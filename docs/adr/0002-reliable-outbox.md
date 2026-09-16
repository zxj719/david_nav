# ADR-0002：断联期间保留在途业务帧

状态：当前 `ReliableOutbox` 已实现。标题里的“可靠”不表示持久化或 exactly-once。

## 背景原文（保留）

来源 [不可变旧入口](../../david_nav/test/golden/legacy_entries/navigation_executor.py.txt)
的 `_drain_outbound_queue` 文档和应答入队注释：

```text
发送排水，带送达保证（2026-07-27）。

旧实现先 get() 出队再 send —— 断联瞬间 send 抛异常时事件已不在队列，
永久丢失。实测后果：机器人到点，on_arrived 恰落在断联窗口 → 服务端等
不到到达事件 → 超时判导航失败（"明明到点却上报失败"）。
新实现：payload 挪进 pending_holder（跨重连存活，由 main_async 持有），
send 成功才移除；重连后新 drain 任务优先重发 holder 里的滞留事件。
```

```python
# 2026-07-27: 应答帧也走送达保证队列（原直发在断联
# 瞬间同样会丢，且异常会拆连接）
```

## 决定与当前落点

[ReliableOutbox](../../nav_executor/nav_executor/adapters/transport/outbox.py)
属于进程，不属于连接。put 时冻结 JSON；单 drain 持锁，发送中帧先保留为 pending，
send 成功才移除。失败/取消后下一个连接先重发 pending，再发新帧。
成功后的日志回调失败不重新入队。任务事件 pump 在取出事件后同步 put，期间没有 await。

业务应答和任务事件都入队；当前 JSON 心跳直接发送，不在此承诺范围内。
连接子任务强引用由连接 owner 持有并 join，任务生命周期由 runtime supervisor 持有。
断联不取消导航；最终关闭要先完成任务清理、停止 event consumer，再退订/释放资源。

## 证据与限制

`nav_executor/test/test_outbox.py` 覆盖冻结字节、取消/异常、单发送者及日志失败；
`test_task_wire_golden.py` 的 G15 覆盖真实任务链与断联窗口，`test_ws_client.py`
覆盖发送失败唤醒重连。原历史“送达保证”仅指已入队帧在本进程跨连接保留：
无服务端处理 ACK、无进程崩溃持久化，模糊 send 结果可能导致重复。
无界队列保留旧策略；长断联/慢消费者可能累积内存，不能以此证明 exactly-once。
