# G1 导航接口使用说明与 JSON 一致性报告

## 1. 概述

本文档用于说明 `g1_nav` 中导航执行方如何与导航管理模块通信，并重点明确当前实现所采用的 JSON 消息格式，避免联调时出现“连接正常但字段不匹配”的问题。

本文档同时覆盖两部分内容：

- 基础导航接口
- G1 当前实现的扩展接口

本次整理的核心目标不是重新设计一套协议，而是把代码、文档和服务端联调格式统一到同一份定义上。

## 1.1 系统架构

- 大脑：负责业务决策，下发导航请求或 POI 相关请求
- 导航管理模块（中枢）：负责任务调度、请求编号、状态汇总
- 导航执行方（小脑）：负责物理移动、POI 本地记录、执行状态上报

在当前 G1 实现中：

- 小脑对应本机 `navigation_executor.py`
- 中枢通过 WebSocket 与小脑通信
- 小脑再桥接到本机 Nav2、TF、SLAM 和本地 POI 存储

## 1.2 一致性原则

当前实现统一采用以下 JSON 约定：

1. 所有业务消息使用顶层平铺字段
2. 不使用 `data` 外层包裹
3. 基础导航消息沿用 `navigate_to / abort_navigation / on_progress / on_arrived / on_error`
4. G1 扩展消息继续沿用同样的顶层平铺风格
5. 服务端只维护语义信息，不以小脑上报结果作为几何真值源

其中第 2 点最关键。当前代码明确按 flat schema 解析，如果服务端发送：

```json
{
  "action": "navigate_to",
  "data": {
    "request_id": "req_001"
  }
}
```

则不会被视为正确协议格式。

## 2. 连接方式

### 2.1 WebSocket 连接地址

导航执行方连接到：

```text
ws://[服务器地址]:8100/ws/navigation/executor
```

当前 G1 默认配置为：

```text
ws://172.16.21.205:8100/ws/navigation/executor
```

### 2.2 本机启动方式

如果要启动 G1 本机导航执行链路，当前推荐入口是：

```bash
cd /home/unitree/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/g1_nav
./start_navigation_headless.sh
```

如果只启动导航，不启动 frontier exploration：

```bash
./start_navigation_headless.sh -e
```

### 2.3 连接状态日志

当前执行端会打印以下连接状态：

- `connecting to ...`
- `connected`
- `connection failed: ...`
- `connection closed: ...`

这些日志用于区分：

- 进程未启动
- 依赖缺失导致进程退出
- 服务端不可达
- 连接建立后又断开

## 3. JSON 消息格式总原则

### 3.1 基础规则

当前项目中，所有消息都遵循：

- 命令消息使用 `action`
- 事件消息使用 `event_type`
- 导航任务使用 `request_id`
- 导航子任务使用 `sub_id`
- 目标点位使用 `target_id`

### 3.2 统一风格

以下是推荐且已在代码中实现的统一风格：

```json
{
  "action": "navigate_to",
  "request_id": "req_2026_001",
  "sub_id": 1,
  "target_id": "POI_001"
}
```

而不是：

```json
{
  "action": "navigate_to",
  "payload": {
    "request_id": "req_2026_001"
  }
}
```

## 4. 基础导航接口

### 4.1 心跳消息

执行端定期发送：

```json
{
  "type": "ping"
}
```

收到心跳时返回：

```json
{
  "type": "pong"
}
```

### 4.2 导航指令 `navigate_to`

服务端下发：

```json
{
  "action": "navigate_to",
  "request_id": "req_2026_001",
  "sub_id": 1,
  "target_id": "POI_001"
}
```

字段说明：

- `action`：固定为 `navigate_to`
- `request_id`：导航请求 ID
- `sub_id`：子任务 ID
- `target_id`：目标点位 ID

### 4.3 中止导航 `abort_navigation`

服务端下发：

```json
{
  "action": "abort_navigation",
  "request_id": "req_2026_001"
}
```

字段说明：

- `action`：固定为 `abort_navigation`
- `request_id`：要取消的导航请求 ID

## 5. 基础事件上报

### 5.1 进度更新 `on_progress`

执行端上报：

```json
{
  "event_type": "on_progress",
  "request_id": "req_2026_001",
  "sub_id": 1,
  "remaining_distance": 1.5,
  "status": "running"
}
```

字段说明：

- `event_type`：固定为 `on_progress`
- `request_id`：导航请求 ID
- `sub_id`：子任务 ID
- `remaining_distance`：剩余距离，单位米
- `status`：当前状态，如 `running`、`paused`

### 5.2 到达事件 `on_arrived`

执行端上报：

```json
{
  "event_type": "on_arrived",
  "request_id": "req_2026_001",
  "sub_id": 1
}
```

### 5.3 错误事件 `on_error`

执行端上报：

```json
{
  "event_type": "on_error",
  "request_id": "req_2026_001",
  "sub_id": 1,
  "error_message": "路径被堵"
}
```

字段说明：

- `event_type`：固定为 `on_error`
- `request_id`：导航请求 ID
- `sub_id`：子任务 ID
- `error_message`：错误描述

## 6. G1 扩展接口

在保留基础导航接口 JSON 风格不变的前提下，当前 G1 实现增加了 POI 目录同步和当前位置标记能力。

### 6.1 POI 目录同步 `update_poi_list`

服务端下发：

```json
{
  "action": "update_poi_list",
  "version": 3,
  "poi_list": [
    {
      "id": "POI_001",
      "name": "前台"
    },
    {
      "id": "POI_002",
      "name": "会议室门口"
    }
  ]
}
```

字段说明：

- `action`：固定为 `update_poi_list`
- `version`：目录版本号
- `poi_list`：语义 POI 列表

当前实现语义：

- 该消息只用于同步语义目录
- 小脑本地保留几何信息
- 该消息不应用作地图坐标下发协议

### 6.2 当前位置标记 `mark_current_poi`

服务端下发：

```json
{
  "action": "mark_current_poi",
  "request_id": "req_mark_001",
  "sub_id": 2,
  "poi": {
    "id": "POI_006",
    "name": "会议室门口"
  }
}
```

字段说明：

- `action`：固定为 `mark_current_poi`
- `request_id`：本次标记请求 ID
- `sub_id`：子任务 ID
- `poi.id`：POI 唯一标识
- `poi.name`：POI 名称

### 6.3 标记请求已接收 `on_mark_poi_ack`

执行端上报：

```json
{
  "event_type": "on_mark_poi_ack",
  "request_id": "req_mark_001",
  "sub_id": 2,
  "status": "accepted"
}
```

### 6.4 标记成功 `on_mark_poi_success`

执行端上报：

```json
{
  "event_type": "on_mark_poi_success",
  "request_id": "req_mark_001",
  "sub_id": 2,
  "poi": {
    "id": "POI_006",
    "name": "会议室门口"
  }
}
```

这里特别强调：

- 当前协议已统一为语义返回
- 成功回包只返回 `poi.id` 和 `poi.name`
- 小脑不再通过该事件向中枢暴露 POI 几何字段

这样做的目的，是让中枢继续停留在抽象层，不把机器人侧地图几何信息重新耦合回服务端。

### 6.5 标记失败 `on_mark_poi_error`

执行端上报：

```json
{
  "event_type": "on_mark_poi_error",
  "request_id": "req_mark_001",
  "sub_id": 2,
  "error_message": "当前位置定位失败"
}
```

## 7. 执行行为约定

当前小脑实现有以下行为约定：

- 新的 `navigate_to` 会抢占当前导航任务
- `abort_navigation` 仅按 `request_id` 匹配取消
- `mark_current_poi` 可以在导航过程中执行
- `update_poi_list` 只更新语义目录，不应打断导航

这些行为不改变 JSON 结构，但会影响中枢的调度预期。

## 8. 小脑与中枢的边界

为了保证协议一致性和职责清晰，当前推荐边界如下：

- 中枢维护语义 POI
- 小脑维护几何 POI
- 中枢按 `target_id` 调度
- 小脑按本地几何信息执行导航

因此：

- `navigate_to` 只需要 `target_id`
- `on_mark_poi_success` 只需要 `id` 和 `name`
- 服务端不应把小脑上报的成功事件当作地图坐标真值同步源

## 9. 当前代码中的落地位置

当前和 JSON 协议直接相关的实现文件包括：

- [navigation_protocol.py](/home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_protocol.py)
- [navigation_executor_core.py](/home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor_core.py)
- [navigation_executor.py](/home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor.py)
- [test_navigation_executor_protocol.py](/home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_protocol.py)

其中：

- `navigation_protocol.py` 负责解析和构造 JSON
- `navigation_executor_core.py` 负责协议行为
- `navigation_executor.py` 负责 WebSocket 收发
- `test_navigation_executor_protocol.py` 负责锁定 JSON 格式

## 10. 联调时重点检查项

### 10.1 是否使用 flat schema

服务端必须确认：

- `request_id` 在顶层
- `sub_id` 在顶层
- `target_id` 在顶层
- `poi` 在顶层

### 10.2 是否误传几何字段

服务端如果想保持抽象层，应避免依赖以下字段：

- `frame_id`
- `position`
- `yaw`
- `orientation`

当前 `on_mark_poi_success` 已不再输出这些字段。

### 10.3 是否混用了旧版包装格式

联调时禁止混用：

- `data`
- `payload`
- 其他额外外层包装对象

## 11. 常见问题与故障排除

### 11.1 WebSocket 已连接，但导航无响应

优先检查：

- 是否仍使用顶层平铺字段
- `action` 拼写是否正确
- `target_id` 是否存在于小脑本地目录

### 11.2 标记成功，但服务端拿不到期望字段

优先检查：

- 服务端是否仍在读取旧版几何字段
- 是否已切换到 `poi.id + poi.name` 的语义返回格式

### 11.3 到达或错误事件关联不上任务

优先检查：

- `request_id` 是否透传
- `sub_id` 是否透传

## 12. 接口版本建议

建议将当前接口视为：

- 基础导航接口：`v1.0`
- G1 语义 POI 扩展：`v1.1`

其中 `v1.1` 的关键变化是：

- 保持 flat schema
- 增加 `update_poi_list`
- 增加 `mark_current_poi`
- `on_mark_poi_success` 只返回语义字段，不返回几何字段

## 13. 结论

这次整理后的重点不是新增更多消息，而是把现有消息的 JSON 形状固定下来。

当前推荐统一认定：

- 导航消息使用顶层平铺格式
- 基础导航接口保持不变
- POI 扩展接口沿用相同风格
- 中枢只处理语义字段
- 小脑保留几何真值

只要中枢和小脑都严格遵守这份 JSON 定义，联调时最容易出错的字段不一致问题会明显减少。
