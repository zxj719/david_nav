# G1 自主探索插队 POI 与大脑引导 Frontier 设计报告

## 1. 背景

当前 `g1_nav` 已经具备以下能力：

- `mark_current_poi` 可以在机器人运动过程中执行，且不会打断当前导航
- `navigation_executor` 可以通过 WebSocket 接收 `navigate_to / abort_navigation / update_poi_list / mark_current_poi`
- `frontier_explorer` 可以本地自主探索
- `frontier_explorer` 当前支持通过本地 ROS 话题 `explore/resume` 控制“是否继续探索”

但如果要满足下面这组目标，现有架构还差一层明确的状态机和协议：

- 机器人处于自主建图探索中
- 服务端可随时要求机器人前往某个 POI
- 机器人到达 POI 后必须停住并等待
- 只有收到显式动作后，机器人才能：
  - 继续探索
  - 前往下一个 POI
  - 保持 idle

另外，现有探索链路还存在一个重要边界问题：

- `explore/resume = false` 当前只会停止后续 frontier 规划
- 它不会主动取消当前已经发出的 frontier 导航 goal
- 如果执行器在探索过程中直接向 Nav2 发送新的 POI 导航 goal，`frontier_explorer` 会把这次外部抢占看成一次“导航被 Nav2 中断”
- 这有概率造成 frontier 被误 blacklist，或者让探索内部状态机进入非预期路径

因此，这个问题不能只靠“多加一个 WebSocket 命令”解决，还需要把本地探索控制面一起理顺。

## 2. 目标行为

建议把目标行为明确为以下规则：

### 2.1 标点规则

- `mark_current_poi` 是 side task
- 无论当前处于 `exploring`、`navigating_to_poi` 还是 `holding`
- 只要位姿可用，都允许立即执行
- 不应打断当前运动状态

### 2.2 探索插队去 POI 规则

当机器人正在自主探索时，服务端下发：

```json
{
  "action": "navigate_to",
  "request_id": "req_nav_001",
  "sub_id": 1,
  "target_id": "POI_006"
}
```

系统应执行以下流程：

1. 执行器先请求探索器进入暂停态
2. 如果探索器当前已有 frontier goal，则应以“受控取消”方式停止，而不是让它把这次取消当成失败
3. 探索暂停完成后，执行器接管 Nav2，开始前往目标 POI
4. 到达后，机器人停止
5. 状态切换为 `holding`
6. 不自动恢复探索

### 2.3 到达后等待规则

到达 POI 后：

- 默认状态不是 `exploring`
- 默认状态也不是“自动继续”
- 而是 `holding`

此时服务端可以显式下发：

- 恢复探索
- 去下一个 POI
- 切换到 idle

### 2.4 idle 规则

`idle` 的语义建议定义为：

- 不进行 frontier 自主探索
- 不存在当前 POI 导航任务
- 机器人保持停止
- 可以继续接受：
  - `mark_current_poi`
  - `navigate_to`
  - 显式恢复探索

## 3. 推荐状态机

建议在执行器侧引入比当前 `IDLE / STARTING / NAVIGATING` 更上层的一层运行模式。

### 3.1 推荐模式枚举

- `exploring`
- `navigating_to_poi`
- `holding`
- `idle`
- `error`

### 3.2 模式语义

#### `exploring`

- frontier_explorer 拥有目标选择权
- frontier_explorer 可以自行向 Nav2 下发 frontier 导航目标
- 执行器不应同时再发 POI goal

#### `navigating_to_poi`

- frontier_explorer 已暂停
- 执行器拥有 Nav2 goal 控制权
- 当前运动目标来自服务端指定 POI

#### `holding`

- 刚到达某个 POI
- 机器人停止
- frontier_explorer 保持暂停
- 必须等待服务端显式动作

#### `idle`

- 机器人停止
- frontier_explorer 不运行
- 没有当前 POI 导航任务

### 3.3 核心状态转移

- `exploring -> navigating_to_poi`
  - 触发条件：服务端发送 `navigate_to`
- `navigating_to_poi -> holding`
  - 触发条件：POI 到达
- `holding -> exploring`
  - 触发条件：服务端发送显式恢复探索动作
- `holding -> idle`
  - 触发条件：服务端发送显式 idle 动作
- `holding -> navigating_to_poi`
  - 触发条件：服务端发送新的 `navigate_to`
- `idle -> exploring`
  - 触发条件：服务端发送显式恢复探索动作
- `idle -> navigating_to_poi`
  - 触发条件：服务端发送 `navigate_to`

## 4. 协议方案比较

## 方案 A：只新增 `resume_exploration`

### 做法

- 保持当前 `navigate_to`
- 保持当前 `mark_current_poi`
- 新增一个 `resume_exploration`
- 到达 POI 后默认停住

### 优点

- 改动最小
- 服务端理解成本低

### 缺点

- `idle` 没有被协议显式表达
- 执行器的模式不清晰
- 后续扩展“半自主探索”或“大脑引导探索”时不够用

### 结论

不推荐作为长期协议，但可作为极短期临时补丁。

## 方案 B：新增统一模式控制 `set_run_mode`

### 做法

保留现有业务命令：

- `navigate_to`
- `abort_navigation`
- `mark_current_poi`
- `update_poi_list`

新增一个显式运行模式命令：

```json
{
  "action": "set_run_mode",
  "request_id": "req_mode_001",
  "mode": "exploring"
}
```

或：

```json
{
  "action": "set_run_mode",
  "request_id": "req_mode_002",
  "mode": "idle"
}
```

### 模式定义

- `exploring`
- `idle`

其中：

- `navigate_to` 自动把执行器带入 `navigating_to_poi`
- 到达 POI 后自动进入 `holding`
- `holding` 是执行器内部派生态，不需要服务端主动设置

### 优点

- 服务端协议仍然保持 flat JSON 风格
- 运行意图更清晰
- 后续可自然扩展到：
  - `brain_guided_exploring`
  - `teleop_hold`
  - `semantic_search`

### 缺点

- 执行器需要承担更多状态管理逻辑
- 需要补本地探索控制接口

### 结论

这是最推荐的方案。

## 方案 C：直接把探索控制也全部做成服务端编排

### 做法

- frontier_explorer 不再自主选点
- 服务端持续告诉机器人当前该去哪

### 优点

- 大脑控制力最大

### 缺点

- 网络抖动、图像上传延迟、服务端推理时延都会直接影响探索连续性
- 本地安全 fallback 变复杂
- 对当前系统改动过大

### 结论

不建议现在就走这条路。

## 5. 推荐的服务端协议

建议保持现有 flat JSON 风格，不引入 `data` 包裹层。

### 5.1 继续沿用的消息

服务端 -> 执行器：

- `navigate_to`
- `abort_navigation`
- `update_poi_list`
- `mark_current_poi`

执行器 -> 服务端：

- `on_progress`
- `on_arrived`
- `on_error`
- `on_mark_poi_ack`
- `on_mark_poi_success`
- `on_mark_poi_error`

### 5.2 新增消息：设置运行模式

服务端 -> 执行器：

```json
{
  "action": "set_run_mode",
  "request_id": "req_mode_001",
  "mode": "exploring"
}
```

```json
{
  "action": "set_run_mode",
  "request_id": "req_mode_002",
  "mode": "idle"
}
```

字段说明：

- `action`：固定为 `set_run_mode`
- `request_id`：本次模式切换请求 ID
- `mode`：目标模式，当前建议仅支持：
  - `exploring`
  - `idle`

### 5.3 新增事件：模式请求已接收

执行器 -> 服务端：

```json
{
  "event_type": "on_run_mode_ack",
  "request_id": "req_mode_001",
  "mode": "exploring",
  "status": "accepted"
}
```

### 5.4 新增事件：模式实际变更

执行器 -> 服务端：

```json
{
  "event_type": "on_run_mode_changed",
  "mode": "holding",
  "request_id": "req_nav_001",
  "source": "navigate_to"
}
```

或：

```json
{
  "event_type": "on_run_mode_changed",
  "mode": "exploring",
  "request_id": "req_mode_001",
  "source": "set_run_mode"
}
```

当前建议支持的 `mode` 回传值：

- `exploring`
- `navigating_to_poi`
- `holding`
- `idle`
- `error`

### 5.5 新增事件：探索控制失败

如果执行器无法完成模式切换，例如 frontier_explorer 没有正确暂停，应返回：

```json
{
  "event_type": "on_error",
  "request_id": "req_mode_001",
  "sub_id": 0,
  "error_message": "探索暂停失败，无法切换到 POI 导航模式"
}
```

## 6. 本地 ROS 控制面建议

只改 WebSocket 协议还不够，执行器与 `frontier_explorer` 之间还需要一个本地受控接口。

### 6.1 当前问题

当前只有：

- `explore/resume: Bool`

它的问题是：

- `false` 只会阻止未来 frontier 规划
- 不会取消当前已经发出的 frontier goal
- frontier_explorer 无法区分：
  - 是自己内部重规划
  - 还是执行器要接管 POI 导航

### 6.2 推荐升级

建议把本地探索控制升级为“命令 + 状态”两部分。

推荐内部接口语义：

- `pause_and_hold`
  - 停止探索
  - 若当前有 frontier goal，则受控取消
  - 不 blacklist 当前 frontier
- `resume_exploration`
  - 恢复探索
- `enter_idle`
  - 停止探索并保持空闲

对应地，frontier_explorer 需要回报本地状态：

- `exploring`
- `paused`
- `idle`

无论最终是 topic、service 还是 action，这层语义必须存在。

### 6.3 为什么必须有“受控暂停”

如果没有这层内部协议，执行器直接发 POI goal 时：

- frontier_explorer 的 Nav2 goal 会被外部抢占
- 它会把这个结果当成一次普通 `ABORTED`
- 从而触发本地 blacklist 或错误恢复路径

这会破坏探索器的内部状态一致性。

## 7. 立即建议的实现路径

建议按下面顺序落地：

### 第 1 步：先把探索与 POI 插队的控制面做对

- 执行器新增 `set_run_mode`
- frontier_explorer 增加本地“受控暂停/恢复”接口
- `navigate_to` 在探索中触发时，不再直接抢 Nav2，而是先暂停探索器
- 到达 POI 后进入 `holding`

### 第 2 步：再做服务端可见的模式事件

- `on_run_mode_ack`
- `on_run_mode_changed`

这样服务端可以稳定知道机器人现在是：

- 正在探索
- 正在去 POI
- 已到达并等待
- 空闲

### 第 3 步：最后再接“大脑参与 frontier 选点”

不要在这一步之前就把大脑直接塞进 frontier 决策链，否则问题会混在一起，难以定位。

## 8. 关于“大脑想让机器人去自己想去的地方”

你的想法本质上是在问：

> 能不能把 frontier 目标选择的一部分权力从本地探索器转交给大脑，让大脑利用图像理解和任务语义来影响探索方向？

答案是可以，但不建议让大脑直接替代本地 frontier 规划，而应采用：

> 大脑做“候选目标优先级决策”，本地仍负责“安全候选生成与最终可达导航”

这两层职责不要混掉。

## 9. 三种大脑引导探索思路

## 方案 1：只给大脑“左 / 前 / 右”三方向选择

### 做法

- 机器人在当前位姿附近生成三类方向候选
  - 左
  - 前
  - 右
- 每类方向绑定一个代表性 frontier 或 snapped goal
- 上传该方向对应的关键帧图像
- 大脑只需要选：
  - 左
  - 前
  - 右
  - 放弃，回退到本地自主选择

### 优点

- 结构最简单
- 通信成本最低
- 很适合快速验证“语义偏好是否真有价值”

### 缺点

- 太粗糙
- 一个方向里可能包含多个语义完全不同的空间
- 在复杂室内环境中，`左/前/右` 的信息量往往不够

### 结论

适合作为第一版实验，不适合作为长期方案。

## 方案 2：让大脑输出“粗糙任务链”

例如：

- 先往前走到头
- 再左拐
- 在关键帧重新判断

### 优点

- 更接近人类口头导航
- 更容易结合任务语义

### 缺点

- 对状态同步要求很高
- 回环、重定位、局部绕障后，原始“前、左、右”链很容易失真
- 服务端需要维护较长的时序上下文

### 结论

这是可研究方向，但不建议作为第一版。

## 方案 3：大脑在本地候选 frontier 之间做排名

### 做法

机器人本地继续完成：

- frontier 生成
- snapped goal 可达性检查
- 安全 clearance 检查
- 候选 frontier 聚类

然后把前 `K` 个候选交给大脑，例如：

```json
{
  "event_type": "on_frontier_candidates",
  "request_id": "exp_001",
  "candidates": [
    {
      "candidate_id": "F_001",
      "direction": "front_left",
      "anchor": {"x": 2.1, "y": 1.4},
      "goal": {"x": 1.8, "y": 1.2},
      "distance": 2.3,
      "image_ref": "kf_188"
    },
    {
      "candidate_id": "F_002",
      "direction": "front",
      "anchor": {"x": 3.4, "y": 0.9},
      "goal": {"x": 3.1, "y": 0.8},
      "distance": 3.0,
      "image_ref": "kf_189"
    }
  ]
}
```

服务端返回：

```json
{
  "action": "select_frontier_candidate",
  "request_id": "exp_001",
  "candidate_id": "F_002"
}
```

### 优点

- 大脑真正参与了探索意图
- 本地仍掌握安全与可达性
- 很适合引入“找书不去厕所方向”这类任务语义

### 缺点

- 需要新的一组探索候选协议
- 图像采样、候选去重、时延 fallback 都要设计

### 结论

这是最推荐的中期方案。

## 10. 推荐的总体路线

建议按三个阶段推进。

### 阶段 A：先完成“探索插队 POI + 到点 hold”

目标：

- 在自主探索时可随时去 POI
- 到达后不自动继续
- 必须显式恢复探索或切 idle

这一步主要解决控制面和状态机问题。

### 阶段 B：加入“大脑引导但不接管”的探索候选选择

目标：

- 本地生成安全候选 frontier
- 大脑根据图像和任务语义做选择
- 超时则回退到本地自主策略

这一步主要解决语义引导问题。

### 阶段 C：再研究“粗糙任务链”或“方向链”

目标：

- 支持更强的语义路线规划
- 如“沿走廊前进，到尽头左转后再观察”

这一步复杂度最高，建议放后。

## 11. 我的推荐结论

### 11.1 对当前需求

最推荐的立即方案是：

- 服务端新增 `set_run_mode`
- 执行器引入：
  - `exploring`
  - `navigating_to_poi`
  - `holding`
  - `idle`
- frontier_explorer 升级为支持“受控暂停/恢复”
- 到达 POI 后进入 `holding`
- 只有显式命令才恢复探索

### 11.2 对“大脑想去哪就去哪”

最推荐的中期方案是：

- 不让大脑直接发底层运动意图
- 不让大脑直接接管 Nav2 goal 生成
- 而是让大脑在“本地安全 frontier 候选集合”中做语义排序或选择

这能最大化保留：

- 本地安全性
- 本地可达性验证
- 网络抖动下的 fallback

同时又能真正把任务语义注入探索决策。

## 12. 下一步建议

建议下一轮实现直接按下面顺序进行：

1. 先补本地探索控制接口，替代当前单纯的 `explore/resume`
2. 在 WebSocket 协议中加入 `set_run_mode / on_run_mode_ack / on_run_mode_changed`
3. 把执行器状态机扩展到 `exploring / navigating_to_poi / holding / idle`
4. 验证：
   - 探索中 `mark_current_poi` 不打断动作
   - 探索中 `navigate_to` 会受控暂停探索并去 POI
   - 到达后进入 `holding`
   - `set_run_mode: exploring` 后恢复探索
   - `set_run_mode: idle` 后保持停止
5. 再单独设计 frontier 候选上送协议，不要和本轮控制面改动混在一起
