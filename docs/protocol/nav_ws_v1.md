# 导航 WebSocket v1 / v1.1

适用于重构分支当前 `nav_executor`，不表示生产已部署。对应
[AsyncAPI 描述](nav_ws_v1.asyncapi.yaml)、
[codec](../../nav_executor/nav_executor/protocol/codec.py)、
[消息类型](../../nav_executor/nav_executor/protocol/messages.py)和
[入站适配器](../../nav_executor/nav_executor/protocol/task_adapter.py)。
本次反写文档，不引入命令、握手、鉴权或新版本字段；发现不一致先保留运行行为、
补回归并评审，不能为了通过文档测试改金样。

## 连接、方向与版本

执行器作为 WebSocket **客户端**连接配置的 `server_uri`；本描述中的 application
是执行器。按 [AsyncAPI 2.6 官方定义](https://github.com/asyncapi/spec/blob/v2.6.0/spec/asyncapi.md#channelItemObject)，
`publish` 描述应用接收的消息，`subscribe` 描述应用发出的消息。
`agent.invalid:8101` 和 `/` 只是示例地址/路径，真实 URI 由配置决定，测试不连接它。
机器描述版本 `asyncapi: 2.6.0` 与导航业务版本 `info.version: 1.1.0` 是不同概念。

| 版本 | 保持/改变的内容 |
|---|---|
| v1（2026-07-22 定稿） | `navigate_to` 仅认 `target_id`；旧 v2 `target_pose`、`label` 忽略，不走坐标直达。下表字段及事件不变。 |
| v1.1（2026-07-27） | 相同消息形状；导航后台执行，收包不等导航终态，运行期间可处理匹配请求的 abort；终态/应答加入跨重连内存队列。不是新握手或协商流程。 |
| 重构 R1/R2 | 畸形消息隔离、任务崩溃清理；仍沿用现有错误帧。不是 v2，也没有 `dock_to` WS 命令。 |

## Agent → executor

AsyncAPI schemas 定义**规范生产者格式**，不是对兼容解码器所有宽松输入的穷举。

| action | 规范字段 | 当前结果 |
|---|---|---|
| `navigate_to` | `request_id` 非空字符串，`target_id` 非空字符串，`sub_id` 整数（省略为 0） | 提交后立即返回入站循环；进度/最终事件随后发送。WS 新目标使用 ALLOW 抢占策略，先清理旧 owner。 |
| `mark_current_poi` | `request_id`、`sub_id`、`poi:{id,name}` 必填；id 非空字符串，name 可为空 | 捕获/写入后按顺序返回 `on_mark_poi_ack` 和 success/error。当前并非 ACK 即时发送：两帧同批入队；若底层抛异常而非返回 mark error，ACK 也可能尚未入队。 |
| `abort_navigation` | `request_id` 必填非空字符串 | 只取消匹配任务；等待后端/技能清理，成功清理后 paused，空闲、重复、过期请求不发新增 ACK。清理失败不能假报已停车/IDLE。 |
| `update_poi_list` | `version` 整数（省略为 0），`poi_list` 数组（省略为空）；项为 `{id,name}` | 只重命名已存在点位，不创建未知点、不传坐标、无 ACK；version 被解码但不参与版本冲突裁决。完整项形状先校验，不等于存储事务回滚。 |

兼容细节（保留，不能从较严格的 schema 推断解码器已收紧）：

- `action` 和多数文本接受字符串/数字/bool 后 `str(...).strip()`；`target_id` 仍必须是字符串。
- navigate 的 request_id 尽力从标量恢复；缺失/空白无可关联身份。navigate 的 sub_id
  用 `int(...)`，缺失/非法/溢出回退为 0。mark 的 sub_id 和 update 的 version
  转换失败则报畸形；空白 name 允许。目录 id/name 验证后仍保留原输入，执行时再 `str()`。
- 额外命令字段保持忽略；`type: ping/pong` 优先于 action。规范生产者不要混发两种判别字段。
- JSON 非对象/无法解析/未知 action 记录并忽略；畸形且有可恢复 request_id 则 `on_error`，
  无身份只记录。单条应用消息异常不主动拆连接；真实 socket 失败仍重连。
- Python JSON 解码的非标准宽松行为并非承诺的生产者格式；本次不改变解析器。

## Executor → agent

事件由有序 codec 构造，JSON `ensure_ascii=False`，保留默认分隔符及键序。
schema 的 `x-wire-keys` 是本仓审计扩展，JSON Schema 本身不验证成员顺序。

| event_type | 按线序排列的字段 |
|---|---|
| `on_progress` | event_type, request_id, sub_id, remaining_distance, status |
| `on_arrived` | event_type, request_id, sub_id |
| `on_error` | event_type, request_id, sub_id, error_message |
| `on_mark_poi_ack` | event_type, request_id, sub_id, status（`accepted`） |
| `on_mark_poi_success` | event_type, request_id, sub_id, poi（id, name） |
| `on_mark_poi_error` | event_type, request_id, sub_id, error_message |

距离单位米，`round(float(value), 3)`，正常进度 status=`running`，取消 status=`paused`
且距离 0.0；保留原中文错误文字。普通导航按每次尝试的既有节流时间（默认 1 秒）报告，
不承诺精确 1Hz。仅最后有效阶段发到达：若扩圈后原点可行，还须完成原点段；Dock 由同一
`navigate_to` 按配置的 ID 路由，必须经过相应技能判定，不存在中途阶段成功即到达。

历史外部 Nav2 CANCELED 的 G12 行为仍是 paused/0.0 且保留 NAVIGATING owner；
不要把它与主动 abort 成功清理后的释放混同。无效新 POI 不丢弃旧 owner，是已记录的
安全差异。ROS Dock 入站产生的私有任务事件不转换成无来源的 WS 到达帧。

## 心跳、断联与交付边界

双向应用消息 `{"type":"ping"}` / `{"type":"pong"}`，不是 WS 控制帧。
默认每 30 秒发送 JSON ping；收到 ping 直接回 pong，pong 不触发任务。
本客户端关闭库自动 ping（`ping_interval=None`）。超过三个心跳周期未收到任何帧仅警告，
不是额外实现的主动断线计时器。

业务事件/应答经 `ReliableOutbox`；**心跳直接写当前 socket，不入可靠队列**。
一个进程拥有同一队列和任务适配器，连接只拥有收/发/心跳子任务。send 失败/取消时保留
在途帧在队首，重连优先重发；连接断开本身不取消导航。send 成功后移除，不等服务端业务 ACK。
模糊失败可能造成重复，重启会失去内存队列；无持久化、无 exactly-once、无 server ACK。
request_id/sub_id 可被复用，不能把这对字段当全局永久去重键。队列仍无界，长断联可增内存。
详见 [ADR-0001](../adr/0001-navigation-runs-as-background-task.md) 与
[ADR-0002](../adr/0002-reliable-outbox.md)。

## 验证与已知未验证项

`nav_executor/test/test_protocol_documentation.py` 必跑检查消息清单/方向、文档示例由真实
codec 逐字节生成、键序与两套既有 golden 常量逐项对应。golden 源文件保持原样，不重新生成。
原 G1–G16、outbox、畸形消息及取消回归继续运行；文档检查不替代它们，也不发 ROS 目标。

同一测试文件提供 `jsonschema.Draft7Validator` 的**完整 payload 校验**：
每个 schema 自检、全部消息示例及抽取的既有完整事件字面量验证，另断言缺字段/错误类型拒绝。
仅解析本地 `$ref`，无网络获取。

2026-09-18 起这两项校验**已经真正运行**（此前编号 TODO-ASYNCAPI-1 的 `pytest.skip` 已消除）：
经用户批准后在用户目录安装 `jsonschema` 4.26.0，并把官方 AsyncAPI 2.6.0 元 schema 归档到
`docs/protocol/schemas/asyncapi-2.6.0.json`（固定上游提交、记录 sha256，校验时只读本地副本，
不联网取任何 schema）。整份 `nav_ws_v1.asyncapi.yaml` 按该元 schema 校验的结果是 **0 错误**，
payload 校验与示例校验同样全绿。

仍然**没有**做的事，不要据此推断：未运行官方 `asyncapi` CLI（本地校验用的是归档元 schema
而不是官方命令行工具的完整规则集）；元 schema 与 payload 通过只说明报文结构合法，
既不能证明真实任务时序，也不能证明机器人实际到达任何位置。
