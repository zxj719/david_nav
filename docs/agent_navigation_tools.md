# Agent 导航工具与任务编排

本文面向调用 David 导航能力的上层 Agent、Agent Server 和任务编排器，回答两个问题：

1. 导航模块向 Agent 提供哪些工具，参数和返回事件是什么；
2. Agent 能用这些原子工具完成哪些任务，哪些能力目前并未提供。

本文描述重构工作区 `~/ros2_ws_refactor/src` 当前实现。线上消息的最终契约仍以
[WebSocket v1/v1.1](protocol/nav_ws_v1.md) 和
[AsyncAPI](protocol/nav_ws_v1.asyncapi.yaml) 为准；本文不新增协议动作。

## 1. 接入关系

```text
上层 Agent / Agent Server（WebSocket 服务端）
  │  下发 4 类 JSON 命令
  ▼
nav_executor（David 上的 WebSocket 客户端）
  │
  ▼
nav_tasks.api → 唯一 TaskManager → POI 导航 / 精准入位技能
  │
  ▼
Nav2 → chassis_adapter → wheeltec_driver
```

执行器主动连接场地配置中的 Agent Server。David 当前配置为
`ws://192.168.123.186:8101`。Agent Server 不需要反向登录机器人，也不直接调用 Nav2
Action。任务所有权、抢占、取消和 Dock 都由 David 上的唯一 `TaskManager` 管理。

启动导航并不等于 Agent 接口已经可用。接口可用还要求：

- Lightning 定位、地图和 TF 已就绪；
- Nav2 已通过启动检查；
- 启动模式启用了 executor；
- David 能连接到配置的 Agent Server；
- 目标 POI 已存在于手工点位源或运行时 POI 仓库。

当前快捷入口与 Agent 能力如下：

| 入口 | 导航模式 | Agent WS | 对 `POI_017` 的处理 |
|---|---|---|---|
| `20` | precise 精准入位 | 启用 | 路由到两段式 Dock |
| `24` | tour 普通导览 | 启用 | 按普通 POI 导航 |
| `21` | tour + 动态避障 | 当前快捷入口带 `--no-executor`，不提供 Agent WS | 不接收 Agent 请求 |

模式改变运动策略，不改变 4 个 Agent 命令的报文形状。

## 2. Agent 工具总览

| Agent 工具 | 线协议 `action` | 是否占用运动槽 | 用途 | 完成信号 |
|---|---|---:|---|---|
| 导航到 POI | `navigate_to` | 是 | 前往已存在的点位；在 precise 模式下，配置的 Dock POI 自动执行精准入位 | `on_arrived` 或 `on_error` |
| 取消导航 | `abort_navigation` | 操作当前运动槽 | 按 `request_id` 取消匹配的活动任务并等待清理 | 匹配任务通常产生 `on_progress/status=paused` |
| 标记当前位置 | `mark_current_poi` | 否 | 捕获当前定位位姿，创建或更新本地 POI | `on_mark_poi_success` 或 `on_mark_poi_error` |
| 更新 POI 名称 | `update_poi_list` | 否 | 批量重命名已经存在的 POI | 当前无业务 ACK |

`ping` / `pong` 是应用层心跳，不是业务工具，也不能作为任务成功或机器人存活的充分证明。

### 2.1 导航到 POI：`navigate_to`

规范请求：

```json
{
  "action": "navigate_to",
  "request_id": "tour-20260929-001",
  "sub_id": 1,
  "target_id": "POI_001"
}
```

参数：

| 字段 | 必填 | 说明 |
|---|---:|---|
| `request_id` | 是 | 非空任务标识；用于进度、终态和取消关联 |
| `sub_id` | 否 | 整数子任务号，省略时为 `0` |
| `target_id` | 是 | 已存在的 POI ID，只接受字符串 |

调用被接纳只表示任务进入后台执行，不表示机器人已经到达。执行器随后异步返回：

```json
{
  "event_type": "on_progress",
  "request_id": "tour-20260929-001",
  "sub_id": 1,
  "remaining_distance": 2.346,
  "status": "running"
}
```

成功终态：

```json
{
  "event_type": "on_arrived",
  "request_id": "tour-20260929-001",
  "sub_id": 1
}
```

失败终态：

```json
{
  "event_type": "on_error",
  "request_id": "tour-20260929-001",
  "sub_id": 1,
  "error_message": "目标点不存在: POI_404"
}
```

行为要点：

- v1 只认 `target_id`。`target_pose` 和 `label` 即使随消息发送也不会变成坐标直达；
- WebSocket 新目标采用允许抢占策略。已有运动任务时，系统先清理旧 owner，再接纳新任务；
- 普通 POI 会执行地图/代价图准入、必要时扩圈吸附以及配置的失败重试；
- 到达扩圈可行点可能被判为成功。它表示当前策略下到达最近可行位置，不等于到达原始锚点坐标；
- precise 模式下，`site.yaml` 中 `dock_poi_ids` 指定的 POI 绕过普通吸附和普通重试，进入两段式精准入位；
- 当前 David 的 Dock POI 是 `POI_017`。系统先检查预备点的完整车身 footprint，再到预备点、进入目标位姿并完成停稳/位姿判定；只有完整流程成功才发 `on_arrived`；
- tour 模式不会把 `POI_017` 当作 Dock，仍走普通点到点导航。

### 2.2 取消导航：`abort_navigation`

```json
{
  "action": "abort_navigation",
  "request_id": "tour-20260929-001"
}
```

取消只匹配活动任务的 `request_id`。它会等待 Nav2、Dock worker 和任务 owner 完成软件清理，
不会因为收到命令就立即假报空闲。成功清理后通常会产生：

```json
{
  "event_type": "on_progress",
  "request_id": "tour-20260929-001",
  "sub_id": 1,
  "remaining_distance": 0.0,
  "status": "paused"
}
```

空闲、重复取消或已经过期的 `request_id` 当前不返回新的 ACK。`paused` 是协议兼容状态，
不是可用 `resume` 继续的挂起句柄；若要重新开始，Agent 应提交新的 `navigate_to`。

需要区分两种情况：Agent 主动取消并成功清理后会释放任务；Nav2 自身返回 CANCELED 的历史兼容
路径也可能发 `paused`，但仍保留任务 owner。上层不能仅凭 `paused` 推断系统必然已经 IDLE，
更不能把软件清理结果当成实体机器人已经停稳的传感器证明。

### 2.3 标记当前位置：`mark_current_poi`

```json
{
  "action": "mark_current_poi",
  "request_id": "teach-20260929-001",
  "sub_id": 1,
  "poi": {
    "id": "POI_021",
    "name": "展厅入口"
  }
}
```

该工具捕获当前地图位姿和 odom 位姿，并写入运行时 POI 仓库。它不抢占正在进行的导航。
地图位姿过期或不可用时标记失败。

当前会按顺序返回逻辑 ACK 和结果：

```json
{"event_type":"on_mark_poi_ack","request_id":"teach-20260929-001","sub_id":1,"status":"accepted"}
{"event_type":"on_mark_poi_success","request_id":"teach-20260929-001","sub_id":1,"poi":{"id":"POI_021","name":"展厅入口"}}
```

失败时第二帧为：

```json
{
  "event_type": "on_mark_poi_error",
  "request_id": "teach-20260929-001",
  "sub_id": 1,
  "error_message": "当前位置位姿数据过期"
}
```

`on_mark_poi_ack` 当前不是收到命令后立即发送，而是和标记结果一起进入发送队列。Agent 应以
success/error 作为本次标记的终态，不应把 ACK 当成已经持久化。

### 2.4 更新已有 POI 名称：`update_poi_list`

```json
{
  "action": "update_poi_list",
  "version": 3,
  "poi_list": [
    {"id": "POI_001", "name": "前台"},
    {"id": "POI_021", "name": "展厅入口"}
  ]
}
```

该工具只更新已存在 POI 的显示名称：

- 不创建未知 ID；
- 不删除 POI；
- 不修改坐标；
- 不占用或抢占运动任务；
- 当前没有业务 ACK；
- `version` 会被解析，但当前不执行版本冲突检查；
- 整批字段形状会先校验，但后续存储写入并不提供事务回滚保证。

因此，Agent 不能把“消息已发送”解释为“目录已确认落盘”。需要强确认时，应由上层业务增加
自己的目录读取/确认机制，不能假设现有协议已经提供。

## 3. Agent 可以完成的任务

### 3.1 单点导航

Agent 根据用户意图选择已知 `target_id`，发送一次 `navigate_to`，持续消费同一
`request_id/sub_id` 的事件，直到 `on_arrived` 或 `on_error`。适合“去前台”“去展厅入口”
等已建点任务。

### 3.2 多点导览或巡检

当前没有原子 `navigate_through_poses` Agent 工具。Agent 可在应用层维护点位序列，按顺序调用：

```text
navigate_to(A) ─on_arrived→ navigate_to(B) ─on_arrived→ navigate_to(C)
                         └─on_error→ 按业务策略停止、跳过或回退
```

每一段使用新的 `request_id`，`sub_id` 可表示整条业务任务中的段号。Agent 必须等上一段终态，
否则新请求会触发抢占，而不是自动追加到队列。

### 3.3 中断当前任务并改去新目标

Agent 可以先对当前 `request_id` 发送 `abort_navigation`，等待清理信号后再提交新目标；也可以
直接发送新的 `navigate_to`，由 ALLOW 抢占策略先清理旧任务。显式先取消更容易形成清晰的业务
审计；无论哪种方式，都不能并发占用两个运动槽。

### 3.4 现场教学并复用点位

机器人由操作人员移动到合适位置后，Agent 调用 `mark_current_poi` 保存点位。收到
`on_mark_poi_success` 后，该 ID 可以用于后续 `navigate_to`，也可以通过 `update_poi_list`
改变显示名称。

该流程只记录位姿，不负责建图、启动定位或判断这个点是否适合作为安全导航目标。首次投入任务前
仍应由现场人员检查通行空间和朝向。

### 3.5 精准入位

在 precise 模式中，Agent 对配置在 `dock_poi_ids` 的目标仍调用普通 `navigate_to`。David 当前
以 `POI_017` 启用该路由。导航模块在内部完成预备点计算、完整 footprint 静态地图检查、两段运动、
停稳等待和最终位姿判定。Agent 不需要也不能通过 WS 分别控制这些内部步骤。

### 3.6 断线期间继续任务并在重连后接收结果

WebSocket 断开不会取消运动。任务事件进入进程内 `ReliableOutbox`，重连后优先重发未确认发送
完成的帧。因此 Agent Server 可以在网络短暂中断后继续接收进度或终态。

该机制是进程内、至少可能重复的交付：模糊发送失败可能造成重复帧，执行器重启会丢失内存队列，
也没有 server ACK 或 exactly-once 保证。Agent 应按业务任务状态做幂等处理，不要把
`request_id/sub_id` 当作永久全局去重键。

## 4. 当前没有提供的 Agent 能力

以下能力不能从现有 4 个工具推断出来：

- 任意坐标导航、直接传 Pose、路径点数组或路径轨迹；
- 独立的 `dock_to` WebSocket 命令；ROS 侧存在直接 Dock 入站，但不是 Agent WS 工具；
- 查询 POI 列表、读取 POI 坐标、删除 POI或确认批量重命名结果；
- 查询 TaskManager 是否 IDLE、查询当前模式或恢复已暂停任务；
- 原子多点路线、排队多个运动任务或并行执行两个导航目标；
- 建图、重定位、启动/停止 Lightning、修改地图或定位参数；
- 机械臂、抓取、递送物品等操作任务。Agent 可在更高层把导航与其它模块组合，但导航模块本身
  只对底盘运动和 POI 数据负责；
- 因 Agent Server 断线而自动停车；现有行为是导航继续执行；
- 仅凭 `on_arrived`、`paused` 或软件 ownership 事件证明实体位置精度或物理停稳。

## 5. 上层 Agent 的编排规则

1. 每个新运动任务生成清晰且当前唯一的 `request_id`；用 `sub_id` 表示复合任务步骤。
2. 发送 `navigate_to` 后等待异步终态，不把发送成功、socket 可写或第一条进度当成到达。
3. 只把匹配 `request_id/sub_id` 的 `on_arrived` / `on_error` 归入当前步骤；容忍重连重放。
4. 连续路线一次只提交一段。想替换当前目标时，明确选择“先取消”或“直接抢占”。
5. 取消后不要立即假设底盘已停；需要物理安全确认的任务还应结合现场传感器或操作流程。
6. 不发送坐标字段绕过 POI 管理。未知目标应先完成点位教学或由配置维护者加入点位源。
7. 对 Dock POI 只发送一次 `navigate_to` 并等待整体终态，不把预备点或中途 Nav2 success 当到达。
8. 把 `update_poi_list` 当作无回执管理命令；上层如需强一致，需另建确认接口。
9. 连接丢失时根据业务风险决定是否在重连后显式取消；现有导航不会因断线自动结束。

## 6. Function Calling 包装建议

如果上层使用 LLM Function Calling，可把现有协议一对一包装为下列工具。这里的工具名属于
Agent 应用层，David 线上 `action` 仍保持第 2 节中的固定值。

| 建议工具名 | 参数 | 转换后的 WS action |
|---|---|---|
| `navigation_go_to_poi` | `request_id:string, target_id:string, sub_id?:integer` | `navigate_to` |
| `navigation_cancel` | `request_id:string` | `abort_navigation` |
| `navigation_mark_current_poi` | `request_id:string, sub_id:integer, poi_id:string, name:string` | `mark_current_poi` |
| `navigation_rename_pois` | `version?:integer, pois:[{id,name}]` | `update_poi_list` |

工具处理器应只做字段映射和事件关联，把成功条件留给导航终态事件。特别是
`navigation_go_to_poi` 的函数调用返回值应表达“已提交”，随后由事件流完成任务，而不是同步返回
“已到达”。

## 7. 相关真源

- 线协议与交付边界：[docs/protocol/nav_ws_v1.md](protocol/nav_ws_v1.md)
- 可机读 schema：[docs/protocol/nav_ws_v1.asyncapi.yaml](protocol/nav_ws_v1.asyncapi.yaml)
- 任务与所有权：[docs/architecture.md](architecture.md)
- 行为和安全边界：[docs/behavior_policy.md](behavior_policy.md)
- 执行器入口：[nav_executor/README.md](../nav_executor/README.md)
- POI / Dock 任务实现：[nav_tasks/README.md](../nav_tasks/README.md)
- 当前模式和快捷入口：[docs/alias_mapping.md](alias_mapping.md)
