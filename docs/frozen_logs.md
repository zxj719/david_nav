# 冻结日志接口与消费者

适用重构分支；日志不是新的协议版本。以下关键片段、前缀、标点和格式精度按
旧金样保留。改变需补 ADR、更新消费者并经人工审核，不能重造 golden 掩盖差异。

## 执行器与重试

| 冻结格式（花括号是变量） | 当前生产者 | 已知消费者/证明 |
|---|---|---|
| `[navigation_executor] NAV ERROR reported to server: request_id={id} sub_id={sub} error_message={text}` | `nav_executor/app/logging.py` OutboundAudit，发送尝试前 | SPEC 人工 grep 接口；旧 `test_executor_golden.py`、新 `test_task_wire_golden.py` G8/G15。不是服务端已处理 ACK。 |
| `[navigation_executor] OUT sent: {event} request_id={id} sub_id={sub}` | 同上，成功发送终态帧后 | 新旧 G15/outbox 测试、人工排错。 |
| `[navigation_executor] OUT RESENT(断联重发): {event} request_id={id} sub_id={sub}` | 同上，成功重发后 | 新旧 G15/outbox 测试、人工排错；不表示 exactly-once。 |
| `[navigation_executor] STATUS state={state} poi={poi} phase={phase} d2g={distance} x={x:.2f} y={y:.2f} yaw={yaw:.2f}rad` | `nav_executor/app/status.py` + 同一前缀 logger | 新旧 G16/status 测试，人工状态检查；未活动字段 `-`，活动距离 `{d:.2f}m`；无位姿分支为 `pose=n/a ({ExceptionType})`。 |
| `[nav2_action_bridge] NAV FAILURE attempt {i}/{n} request_id={id} sub_id={sub}: {detail}` | `nav_tasks/app/logging.py` RetryAudit | 新旧 G8/retry 测试、人工诊断。 |
| `[nav2_action_bridge] suppressing failure report, re-publishing goal in {delay:.1f}s (attempt {next}/{n})` | 同上，中间失败 | 新旧重试金样，不能改成中间 on_error。 |
| `[nav2_action_bridge] retry budget exhausted ({n} attempts) — reporting on_error to server: {detail}` | 同上，预算耗尽 | 新旧 G8 错误金样，长横线保留。 |

这些生产者相对路径省略外层包目录；源码链接：
[Outbox audit](../nav_executor/nav_executor/app/logging.py)、
[STATUS](../nav_executor/nav_executor/app/status.py)、
[RetryAudit](../nav_tasks/nav_tasks/app/logging.py)。前缀输出当前仍用显式 print/注入 sink，
不是已经迁成 Python logging；不为文档统一术语而改变输出/调用点行为。

2026-09-16 只读仓内匹配 `NAV ERROR|STATUS state=|OUT RESENT|NAV FAILURE`
（排除文档/测试）定位到上述生产者及保留旧 nav2_action_bridge；未找到另外的仓内
运行时解析器。该搜索不是外部消费者不存在的证明；P0 人工 grep 接口继续视为兼容边界。
同轮只读扫描生产 `~/ros2_ws/src` 与 `~/lightning_ws_new/src` 的 Python/shell/C++
源码，只有旧 navigation_executor.py 与 nav2_action_bridge.py 生产者命中；未发现新增
定位侧解析器。这里只报告该搜索范围，不修改定位或生产文件，不覆盖其它语言/外部工具。

## Dock：有实际正则消费者

历史消费者来自提交 `93676a5` 的 Dock bench，
的 `parse_dock_timeline` / `_RE_*`，由 dock bench 记录/指标流程使用。
它按关键片段切阶段，主要评测数值由独立 TF 复算，不能把被测对象自评当定位真值。
时间线/测量正则与判词规则也必须继续兼容。

| 冻结片段/格式 | 生产者 | 正则/判词消费者 |
|---|---|---|
| `[dock] ===== 收到目标 (%.3f, %.3f) yaw=%.1f°` | skills/docking/workflow | `_RE_GOAL_RECEIVED` |
| `[dock] 阶段1 预备点 (%.3f, %.3f) yaw=%.1f°（沿目标航向后退 %.2fm）` | workflow | `_RE_STAGE1_START` |
| `[dock] 阶段2 入位 -> (%.3f, %.3f) yaw=%.1f°` | workflow | `_RE_STAGE2_START` |
| `[dock] %s: %s`，label 为 `阶段1(预备点)` / `阶段2(入位)` | adapters/ros/dock_motion | `_RE_STAGE_DONE`；保持 `阶段[12]` 与括号布局 |
| `[dock] 入位实测 exy=%.3fm eyaw=%.1f° (停稳=%s)` | workflow 的 finish observer | `_RE_MEASURED`，前三段数值/单位/符号不能改 |
| `[dock] 第%d次纯旋转修正: Spin(%.1f°)` | workflow | `_RE_SPIN` |
| `[dock] 旋转后 exy=%.3fm eyaw=%.1f° (停稳=%s)` | workflow | 既有日志金样/人工判读 |
| `[dock] ===== 成功: %s` / `[dock] ===== 失败: %s` | workflow._report | `_RE_VERDICT`，内部 `_VERDICT_RULES` 还匹配“一次入位达标”“旋转后达标”“阶段1 未到预备点”“xy 超差”“仍未收敛”“取不到位姿”“车未停稳”等判词 |

完整成功/失败尾部可能带 `  (实测 exy=%.3fm eyaw=%.1f°)`。
新加预备点静态 footprint 检查和结构化事件不删除旧行、不把 recovery 成功变成到达；
取消行是 `===== 已取消:`，不能改写成成功。
生产者：[workflow.py](../nav_tasks/nav_tasks/skills/docking/workflow.py)、
[dock_motion.py](../nav_tasks/nav_tasks/adapters/ros/dock_motion.py)。
证明：[Dock 工作流测试](../nav_tasks/test/test_dock_workflow.py)、
[结构化事件测试](../nav_tasks/test/test_dock_event_observations.py)、
历史 bench 时间线样例已随旧台架退役。

结构化 `/nav/events` 是附加观察面，见 [事件文档](events.md)；bench 的正则消费者
尚未整体迁成事件订阅，因此不能废弃这些日志接口。
