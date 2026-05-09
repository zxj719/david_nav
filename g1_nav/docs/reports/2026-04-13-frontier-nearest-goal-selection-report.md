# 2026-04-13 Frontier 最近目标选择问题复盘报告

## 问题描述

现场现象是：

- 当前轮探索里，机器人前方已经出现一个更近的 frontier
- 但系统仍然经常选择更远的 frontier
- 从 RViz 看，最终被选中的目标不像“当前最近可执行 goal”，而更像“上一轮已经盯上的方向”

这类问题会直接带来两个后果：

1. 探索效率下降，机器人绕远路
2. 现场调参会变得误导，因为 YAML 看起来在配“最近 frontier”，但运行时行为并不真的按这个语义执行

## 根因分析

这次问题不是单一参数大小不对，而是“选择语义在不同层被改写了”。

### 根因 1：搜索层和执行层的排序目标不一致

`FrontierSearch::search()` 本身已经会按距离优先排序，语义是：

- 距离越近越优先
- `frontier_distance_weight` 与 `frontier_size_weight` 控制排序

但是到了 `frontier_explorer.cpp` 的执行层，候选 frontier 又被重新打分了一次，而且那套打分不是简单的“最近 goal 优先”，而是混入了额外偏好：

- 靠近上一目标的偏好
- 靠近机器人朝向的偏好
- 另一套机器人距离分数

结果就是：

- 搜索层认为 A 更近
- 执行层又把 B 重新排到前面

这就造成了“看起来配置了最近优先，但最后没按最近走”。

### 根因 2：真正执行的是 `goal_xy`，但排序却不围绕实时 `goal_xy`

frontier 系统里至少有两个不同语义的点：

- `anchor_xy`：frontier 的身份点
- `goal_xy`：最终真正发给 Nav2 的落脚点

如果排序只看 frontier anchor，或者在生成最终可执行 `goal_xy` 之前就先做优先级判断，那么会出现这种错位：

- 某个 frontier anchor 看起来“值得优先”
- 但它最终吸附出来的可执行 `goal_xy` 其实并不近
- 另一个 anchor 虽然原始排序略靠后，但它的真实 `goal_xy` 更近、更适合当前立即执行

也就是说，系统之前更像是在选“哪个 frontier 概念上更顺眼”，而不是选“哪个最终 goal 现在最近”。

### 根因 3：YAML 暴露的是一套权重，但运行时实际上用了隐藏策略

这次最关键的工程问题之一是：

- YAML 里公开的是 `frontier_distance_weight`、`frontier_size_weight`
- 但执行层还存在额外的隐式打分偏好

这样会导致非常典型的配置欺骗：

1. 调参的人以为把 `frontier_size_weight` 调成 `0.0` 就已经是“纯最近优先”
2. 实际运行时仍然被另一套未公开的“粘住上一目标/朝向偏好”影响
3. 结果就是参数解释和真实行为脱节

这类问题比单纯 bug 更危险，因为它会让后续调参方向长期跑偏。

### 根因 4：去重/抑制发生在错误的排序阶段

frontier 系统里有 suppression / NMS 逻辑，用来避免同一区域重复出现多个本质相同的 frontier。

如果 suppression 发生在“最终 live goal 排序之前”，就会出现这种情况：

- 一个较差的目标先被排到前面
- 它提前占据了该邻域
- 更好的近处目标被当作同一区域重复点压掉

于是系统最终留下来的，可能不是该邻域里“最值得去”的目标，只是“最先进入 suppression 流程”的目标。

## 这次修复方案

这次修复的核心不是继续调权重，而是把“最近目标优先”的语义重新拉直。

### 修复 1：执行层显式读取并复用 YAML 的距离/尺寸权重

在 [src/frontier_explorer.cpp](/home/unitree/ros2_ws/src/g1_nav/src/frontier_explorer.cpp#L130) 中，执行层现在直接读取：

- `frontier_distance_weight`
- `frontier_size_weight`

并把它们传入最终目标排序逻辑。

这样做的意义是：

- 配置文件暴露什么，运行时就真的按什么工作
- 不再出现“YAML 看起来是最近优先，实际却混着别的隐式偏好”的情况

### 修复 2：先为每个 frontier 生成可执行 `FrontierTarget`，再排序

在 [src/frontier_explorer.cpp](/home/unitree/ros2_ws/src/g1_nav/src/frontier_explorer.cpp#L286) 之后，当前流程变成：

1. 对每个 active frontier 先调用 `select_frontier_target()`
2. 生成包含 `anchor_xy` 与 `goal_xy` 的 `FrontierTarget`
3. 对 `FrontierTarget` 排序，而不是对原始 frontier 重新打隐藏分

这一步把“frontier 候选”和“最终可执行目标”明确拆开了。

### 修复 3：最终排序以 live `goal_xy` 到机器人的距离为主

在 [src/frontier_goal_selector.cpp](/home/unitree/ros2_ws/src/g1_nav/src/frontier_goal_selector.cpp#L136) 新增了 `sort_frontier_targets_for_selection()`。

它的核心语义是：

- 主排序依据：`goal_xy` 到当前 `robot_xy` 的距离
- 次级排序依据：anchor 距离、frontier size、heuristic distance

也就是说，系统现在优先回答的是：

“哪个目标我现在真正可以去，而且最近？”

而不是：

“哪个 frontier 从历史偏好上更像应该继续追？”

### 修复 4：先按最终目标排序，再做邻域 suppression

当前流程变成：

1. 先生成所有 admissible `FrontierTarget`
2. 再按 live `goal_xy` 距离排序
3. 最后按 `anchor_xy` 邻域做 suppression

这样可以保证同一邻域里保留下来的，是“当前排序最好”的目标，而不是“先碰巧被处理到”的目标。

### 修复 5：补充针对性测试，锁住这个语义

在 [test/frontier_goal_selector_test.cpp](/home/unitree/ros2_ws/src/g1_nav/test/frontier_goal_selector_test.cpp#L136) 增加了两个关键测试：

- `SortsTargetsByNearestLiveGoalDistance`
- `SuppressesLowerRankedTargetsInsideSnapRadiusAfterGoalSorting`

这两个测试的意义非常直接：

- 防止以后又回到“按 anchor 或历史偏好排序”
- 防止以后 suppression 再次跑到错误阶段

## 为什么这种问题很容易再次出现

这类 bug 在 frontier / exploration 系统里很常见，因为系统天然有多层语义：

- frontier 搜索层
- frontier 身份层
- snapped goal 选择层
- 运行时 preempt / blacklist / completion 状态层

只要这些层之间没有明确的单一排序契约，就很容易发生下面几种漂移：

1. 搜索层按“最近 frontier”排，执行层按“最像上一目标”排
2. 配置层暴露一套权重，代码里再偷偷叠一套启发式
3. suppression 用的是 frontier 语义，排序用的是 goal 语义，但两者执行顺序没设计清楚
4. 可视化看的是 frontier marker，真正发出去的是 snapped goal，调试者误以为两者是一回事

本质上，这是一个“跨层目标函数不一致”的问题，不只是一个 if 写错了。

## 以后如何避免

### 原则 1：先定义唯一的选择问题，再写代码

在写任何 frontier 选点逻辑前，先把这句话写清楚：

> 系统这一轮要最大化的到底是什么？

例如这次应该明确成：

> 在所有当前可执行的 `FrontierTarget` 里，优先选择 live `goal_xy` 最近的目标。

如果这句话写不清楚，后面代码里一定会慢慢混进多个隐式偏好。

### 原则 2：搜索层、目标层、状态层必须分开

建议以后都保持这三个层次：

- `FrontierSearch`
  只负责发现 frontier 候选
- `FrontierGoalSelector`
  只负责把 frontier 转成可执行目标
- `FrontierExplorer`
  只负责运行时状态机、黑名单、完成、preempt

一旦 `FrontierExplorer` 又开始自己发明一套排序启发式，就要高度警惕。

### 原则 3：所有影响选择结果的权重都必须显式配置化

以后如果真的需要“前向偏好”“上一目标连续性偏好”之类的行为，也不是不能做，但必须满足两个条件：

1. 参数要显式暴露
2. 文档要明确写出它和“最近 goal 优先”的关系

否则现场会继续出现：

- 明明把尺寸权重调成 0 了
- 为什么系统还是不选最近目标

### 原则 4：如果最终执行的是 snapped goal，就要优先按 snapped goal 排序

这是这次最值得固化的一条规则：

- 如果最后发给 Nav2 的是 `goal_xy`
- 那么优先级最好也围绕 `goal_xy`

除非你有非常强的理由证明：

- 虽然最终执行的是 `goal_xy`
- 但排序故意不围绕 `goal_xy`

否则就很容易出现“理论上在选 frontier，实际上机器人走得并不合理”的错位。

### 原则 5：suppression 必须发生在最终排序之后

如果 suppression 邻域和 snap 邻域表达的是同一物理区域，那么更稳的流程通常是：

1. 先构造最终可执行目标
2. 再排序
3. 最后做 suppression

这样保留下来的才是“这个区域里最优的那个目标”。

### 原则 6：给“排序语义”单独写测试，不要只测能不能选出某个点

以后这种模块至少应长期保留三类测试：

1. 最近 `goal_xy` 必须排在前面
2. 同一 suppression 邻域里必须保留排序更优者
3. YAML 权重变化必须真实影响最终排序

没有这类测试时，最容易发生的就是：

- 功能看起来还正常
- 但排序语义已经悄悄变了

### 原则 7：让日志和可视化直接暴露排序依据

如果后续还要继续调 frontier 逻辑，建议增加两类可观测性：

1. 日志打印候选目标的 `anchor_xy`、`goal_xy`、goal distance、最终 score
2. RViz marker 区分“frontier anchor”和“最终 nav goal”

这样以后再遇到“为什么没选前面那个点”，就不用靠猜。

## 这次修复的工程价值

这次修复最大的价值不是“把一个 if 改对了”，而是把 frontier 选点逻辑重新拉回到一个可解释的契约上：

- YAML 里配置的距离优先，运行时真的生效
- 最终执行的是哪个 goal，排序就围绕哪个 goal
- suppression 发生在正确阶段
- 关键排序语义有测试兜底

这会让后续 frontier 调参更可信，也能减少“明明改了参数却看不出行为变化”的无效试错。

## 建议后续动作

1. 保持当前 `FrontierTarget` 排序语义不要再回退到 explorer 内部隐式重打分
2. 如果未来确实要引入“连续性偏好”或“前向偏好”，必须显式参数化并加测试
3. 在 RViz 或日志里增加 `anchor_xy` / `goal_xy` / goal distance 输出，便于现场验证
4. 以后所有 frontier 相关设计文档都明确写清“排序对象到底是 frontier anchor 还是 final nav goal”

## 相关代码

- [src/frontier_explorer.cpp](/home/unitree/ros2_ws/src/g1_nav/src/frontier_explorer.cpp#L130)
- [src/frontier_goal_selector.cpp](/home/unitree/ros2_ws/src/g1_nav/src/frontier_goal_selector.cpp#L19)
- [test/frontier_goal_selector_test.cpp](/home/unitree/ros2_ws/src/g1_nav/test/frontier_goal_selector_test.cpp#L136)

## 验证

本次补文档前，使用隔离到 `/tmp` 的临时 `colcon` 目录重新验证了与该问题最直接相关的测试：

```bash
colcon --log-base /tmp/g1_nav_doc_log build \
  --packages-select g1_nav \
  --base-paths /home/unitree/ros2_ws/src/g1_nav \
  --build-base /tmp/g1_nav_doc_build \
  --install-base /tmp/g1_nav_doc_install \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON

colcon --log-base /tmp/g1_nav_doc_log test \
  --packages-select g1_nav \
  --base-paths /home/unitree/ros2_ws/src/g1_nav \
  --build-base /tmp/g1_nav_doc_build \
  --install-base /tmp/g1_nav_doc_install \
  --event-handlers console_direct+ \
  --ctest-args -R 'frontier_goal_selector_test|frontier_completion_policy_test'
```

结果：

- `frontier_goal_selector_test` 通过
- `frontier_completion_policy_test` 通过
