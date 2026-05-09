# 2026-04-16 最近 4 天会话与文档周报

## 统计范围

- 时间范围：2026-04-13 至 2026-04-16
- 覆盖内容：
  - 已成文档的报告、设计、计划
  - 已提交的代码与脚本改动
  - 截至 2026-04-16 工作区内尚未整理成文档的 session 痕迹

## 一、本周总体结论

最近 4 天的工作主线非常清晰，可以归纳为 4 条并行推进的链路：

1. 编写整机迁移 skill 包，把当前已验证能力迁移到另一台 G1
2. frontier 目标选择与完成语义的纠偏
3. 真机 behind-goal 起步卡顿问题定位与实机修复
4. G1 导航执行器从设计、协议落地到 headless 启动链路打通

整体看，这几天的工作已经从“单点问题调参”逐步转向“体系化可复现、可联调、可迁移”的工程化阶段：

- 4 月 13 日偏重 frontier 语义纠偏与探索链路问题收口
- 4 月 14 日偏重真机卡顿问题修复与整机迁移 skill 包设计落地
- 4 月 15 日偏重导航执行器协议、状态机、POI 语义和 headless 启动整合
- 4 月 16 日则继续围绕执行器可靠性补齐运行循环与异常分支测试

## 二、信息来源

本周报综合了以下材料：

- 报告文档
  - `docs/reports/2026-04-13-frontier-completion-regression-report.md`
  - `docs/reports/2026-04-13-frontier-nearest-goal-selection-report.md`
  - `docs/reports/2026-04-14-behind-goal-stutter-debug.md`
  - `docs/reports/2026-04-15-g1-navigation-executor-architecture-and-server-protocol-report.md`
- 设计与计划文档
  - `docs/superpowers/specs/2026-04-14-g1-auto-mapping-nav-migration-design.md`
  - `docs/superpowers/plans/2026-04-14-g1-auto-mapping-nav-migration-plan.md`
  - `docs/superpowers/specs/2026-04-15-g1-navigation-executor-semantic-poi-design.md`
- 关键提交
  - `77c5a77` `fix: narrow frontier completion matching`
  - `c7d128e` `Improve behind-goal startup and scan safety gating`
  - `80e45a0` `Refine frontier goal handling and realsense launch integration`
  - `e40971a` `docs: add G1 auto mapping migration design`
  - `61ed09b` `docs: add G1 migration implementation plan`
  - `12a0c59` `docs: design G1 navigation executor semantic POI flow`
  - `a7d18e6` ~ `a24aa8a` 导航执行器实现与协议对齐相关提交
- 未提交改动
  - `g1_nav/navigation_executor.py`
  - `g1_nav/navigation_executor_core.py`
  - `test/test_navigation_executor_packaging.py`
  - `test/test_navigation_executor_runtime.py`

## 三、按日期回顾

## 2026-04-13：frontier 语义纠偏
### 1. 修复 frontier completion 语义过宽导致的误抑制

发现并修复了一个很典型、但对探索效率影响很大的问题：

- 机器人到达某个 frontier 后
- 附近新出现的 frontier 有时被错误当成“已经完成”
- 结果系统跳回更远、更旧的 frontier

根因在于：

- proximity auto-completion 用了过宽的 reached radius
- completed history 过滤也复用了这个宽半径
- 导致“同一区域的小漂移”和“相邻但不同 frontier”被混淆

修复后，completion 语义被收紧为：

- 当前 frontier 的 proximity 完成只作用于当前追踪目标
- completed history 使用更小的 anchor drift 容差

配套新增 `frontier_completion_policy_test`，把“同一 frontier 漂移”和“新 frontier 不应被误杀”这两个边界锁住。

### 2. 修复 frontier 最近目标选择失真问题

这部分是 4 月 13 日最关键的一条主线。现场现象不是简单的“参数大小不合适”，而是多层排序语义互相覆盖：

- 搜索层按最近 frontier 排序
- 执行层又引入额外偏好重新打分
- suppression/NMS 发生时机也不对
- 结果 YAML 看起来在配“最近优先”，实际运行却偏向“延续旧目标”或“方向偏好”

本次修复明确了几个核心原则：

- 最终执行的是 `goal_xy`，那排序就应该优先围绕 live `goal_xy`
- 先把 active frontier 转成可执行 `FrontierTarget`
- 再对 `FrontierTarget` 排序
- 最后才做邻域 suppression

新增的关键测试包括：

- `SortsTargetsByNearestLiveGoalDistance`
- `SuppressesLowerRankedTargetsInsideSnapRadiusAfterGoalSorting`

这次修复的工程价值不只是“让机器人更可能选最近目标”，更重要的是把跨层语义重新拉直，减少后续 YAML 调参和运行时真实行为脱节的问题。

## 2026-04-14：behind-goal 卡顿修复与整机迁移 skill 包

### 1. 真机 behind-goal stutter 问题定位完成

围绕“目标在机器人后方时，起步明显卡顿甚至延迟数秒”的问题完成了一轮较完整的实机定位。

最终确认这不是单一 deadband，而是两层因素叠加：

- Nav2 对 behind-goal 经常先给出小幅前进加转向命令，而不是果断原地转向
- 老的 `collision_monitor` 使用固定前向 `Slowdown/Stop` polygon，在前方净空只有约 0.5 m 时，会持续把这类起步命令压成 0

同时还存在一个时序放大器：

- Realsense depth-to-scan 输出在 `camera_depth_frame`
- 下游需要频繁做 `camera_depth_frame -> base` 的时序敏感 TF 解析
- 负载上来后容易出现 stale-source warning 和停走停走现象

### 2. 已落地修复

针对上述问题，4 月 14 日完成了三类修复：

- `collision_monitor` 从固定前向 `Slowdown + Stop` 改为基于 footprint 的 `Approach`
- `realsense_depth_to_scan` 改为 `output_frame: base`
- 对 `scan_time`、`transform_tolerance`、`source_timeout` 做了更贴近现场的时序修正

实机指标对比显示：

- `safe_first` 从约 `1.46s` 降到约 `0.30s`
- `odom_first` 从约 `1.97s` 降到约 `0.70s`
- `tf_move_first` 从约 `7.05s` 降到约 `1.39s`
- `safe_zero_while_nav_nonzero` 从 `270` 降到 `3`

这意味着 behind-goal 情况下的“多秒级不起步”已经被显著缓解。

### 3. Realsense 集成与 frontier 流程继续收口

同一天另一条代码主线体现在 `80e45a0`：

- frontier 目标处理继续围绕最近 live goal 语义收敛
- Realsense launch 集成进一步整理
- 增补 axis escape recovery 相关设计与计划

这说明 4 月 14 日不是孤立修一个实机 bug，而是在把“前端探索策略”和“底层 scan/避障链路”同步拉直。

### 4. 编写用于迁移到另一台 G1 的 skill 包方案，并完成技能树落地

这一天围绕“把当前机器上已验证的自动建图、导航、避障和 Unitree 运行环境迁移到另一台已具备 Lightning SLAM 的 G1”形成了完整的 skill 化方案。

这项工作不是简单写一篇迁移说明，而是把迁移经验整理成一个可重复执行的 skill 包，目标是让后续在另一台机器上能够按统一流程完成：

- 源机器打包
- 目标机器 preflight 检查
- 缺失项动态安装或修复
- 统一环境脚本加载
- 分层验收 DDS、SDK 和 `g1_nav`

从本次 session 留下的实际文件看，skill 树已经落到：

- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md`
- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/agents/openai.yaml`
- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh`
- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/preflight_check.sh`
- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/install_or_repair.sh`
- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh`
- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/install-and-verify.md`
- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/package-manifest.md`
- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/runtime-topology.md`
- `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/troubleshooting.md`

结合设计、计划和实际 skill 文件，这条迁移链路的关键原则已经明确：

- Lightning SLAM 保持外部已有能力，不纳入迁移范围
- skill 迁移目标聚焦：
  - `g1_nav`
  - `g1_cmd`
  - `g1_sim`
  - `realsense-ros`
  - `unitree_ros2`
  - `unitree_sdk2_python`
  - `unitree_sdk2-main`
- 安装策略不是“全量重装”，而是：
  - 先 preflight 探测目标机器状态
  - 再按缺失项做动态 install-or-repair
- CycloneDDS 被定义为“条件性补救项”，不是默认必须整体打包
- skill 工作流要求先验证 DDS / ROS 2，再验证 Python SDK，最后验证 `g1_nav`

从 `references/install-and-verify.md` 可以看到，迁移后的验收流程也已经写成标准步骤，包括：

- 从源机器导出迁移 bundle
- 在目标机解包并运行 `preflight_check.sh`
- 执行 `install_or_repair.sh`
- `source target_env.sh`
- 验证 `/sportmodestate`
- 验证 `/lightning/odometry`、`/lightning/grid_map`、`/scan`
- 最后启动 `g1_auto_explore.launch.py`

这部分工作把经验从“当前机器能跑”进一步提升到了“能封装成 skill、能迁移、能复用、能交付给另一台 G1 执行”的层面。

## 2026-04-15：导航执行器从设计到联调入口打通

### 1. 导航执行器语义与协议设计完成

当天先完成了 `G1 Navigation Executor Semantic POI Design`，核心边界被明确为：

- 大脑/服务端负责语义意图、任务编排、POI 语义信息
- 执行器负责几何真值、POI 本地存储、Nav2 调用和位姿获取

这为后续实现提供了两个关键好处：

- 服务端无需持有 POI 几何真值
- SLAM loop closure 导致的地图漂移可以在机器人本地被吸收

### 2. 导航执行器实现快速落地

4 月 15 日连续提交了多步实现，把执行器从协议草案推进到可运行形态：

- 增加协议基础原语
- 增加本地 POI store 与重投影
- 增加执行器 core state machine
- 增加 `mark_current_poi` 流程
- 接上 Nav2 action bridge、pose provider 与 runtime
- 补齐 packaging 与 runtime 测试

这一串提交说明当天不是“只设计不实现”，而是完成了一次从抽象语义到底层运行骨架的集中交付。

### 3. 以“小脑为具身执行层、大脑为抽象管理层”的方式打通 headless 运行链路

这一部分更高层的意义，不只是新增了一个 headless 启动脚本，而是把机器人侧执行系统进一步明确成“小脑”角色：由机器人本体对上提供稳定的 MCP 工具能力，对下接管导航、位姿、局部空间结构和回环后的几何修正。

按这套自顶向下的设计哲学，系统边界被拉直为：

- 大脑负责抽象层 VLM 管理逻辑
- 大脑负责任务理解、语义决策、工具编排和业务流程
- 小脑负责具身执行、空间感知、导航落地和运行时状态闭环
- 地图中的空间结构变化、SLAM 回环带来的几何调整和 POI 几何真值维护，都留在小脑侧处理

这样做的核心收益是：

- 大脑可以停留在更稳定的语义层和任务层，不必直接背负底层几何漂移
- 小脑可以持续吸收机器人现场的实时状态，包括 TF、局部地图、导航反馈和 loop closure 修正
- 大小脑之间的接口更像“能力调用”而不是“共享底层状态”，从而降低系统耦合度

在这个架构下，`start_navigation_headless.sh` 的价值就不只是“把几个进程一起拉起来”，而是把机器人侧小脑运行时统一收口成一个可稳定拉起、可独立托管、可被上层调用的执行入口。当前它统一管理的核心链路包括：

- `map -> odom` 静态 TF
- `slam2`
- `g1_auto_explore.launch.py`
- `navigation_executor.py`

对应到工程落地，这个 headless 入口已经具备较完整的运行保障能力：

- 短参数和长参数双入口
- `RVIZ` / `Realsense` / `Exploration` 可开关
- `--server-uri` 可覆盖默认 WebSocket 目标
- 预检查 `yaml` / `websockets` 等 Python 依赖
- 自动管理子进程组
- 支持 `Ctrl+C` 一次性回收全部后台进程
- 可选 CPU `cpu4-cpu7` online
- `--dry-run` 模式可用于无副作用检查启动命令

配套测试 `test_headless_navigation_script.py` 已覆盖：

- 默认参数行为
- 短参数组合行为
- `server_uri` 覆盖
- 缺少 `websockets` 时快速失败

因此，这部分工作的汇报重点不应仅理解为“headless 启动链路打通”，而应理解为：机器人侧小脑已经开始以工具化、服务化方式向上承接大脑的抽象控制，并把空间结构与几何一致性的复杂性稳定地留在机器人本体一侧。

### 4. 执行器 JSON 协议与文档对齐

4 月 15 日后半段又做了一轮非常关键的“代码-文档-服务端联调格式”对齐：

- 明确所有业务消息采用顶层平铺字段
- 不使用 `data` / `payload` 外层包裹
- 基础导航消息统一为：
  - `navigate_to`
  - `abort_navigation`
  - `on_progress`
  - `on_arrived`
  - `on_error`
- G1 扩展消息继续沿用相同 flat JSON 风格

这一步看起来像文档整理，实际是在消除一种非常常见的联调风险：

- WebSocket 连通
- 日志看起来也正常
- 但因为字段嵌套层级不一致导致双方实际上不兼容

换句话说，4 月 15 日的成果不是“又写了一篇说明书”，而是把协议歧义显式消掉了。

## 2026-04-16：未成文档 session 补充

截至当前工作区，虽然 4 月 16 日还没有新增正式报告文档，但已经能从未提交改动中看出一条明确的持续修复主线，仍然属于本周工作的延续。

### 1. 给导航执行器补上 ROS spin 异步循环

在 `g1_nav/navigation_executor.py` 中新增了 `_spin_ros_node()`，并在主异步流程中创建 `spin_task`。

这说明前一天的执行器实现已经推进到一个新的稳定性阶段：不仅要能连 WebSocket、能处理协议，还要保证 ROS node 在 asyncio 主循环中持续被 `spin_once()` 驱动。

这类改动通常对应的实际问题包括：

- TF / subscription / timer 回调未被稳定消费
- 执行器虽然连上服务器，但本地 ROS 数据更新不及时
- 运行一段时间后出现“看起来在线，实际内部状态没刷新”的假活性

同时，退出路径中也补上了 `spin_task.cancel()` 与等待清理逻辑，说明这次不是临时加个循环，而是把运行和退出都一起补齐了。

### 2. 补上 `mark_current_poi` 位姿捕获异常分支

在 `g1_nav/navigation_executor_core.py` 中，`mark_current_poi` 从原先直接调用：

- `capture_for_poi()`

变为：

- 捕获异常
- 回传 `on_mark_poi_error`
- 把底层 TF/位姿采集失败暴露为上层可消费的协议错误

从新增测试看，这次明确考虑了类似下面的现场错误：

- `"map" passed to lookupTransform argument target_frame does not exist.`

这类修复很重要，因为它把“底层 ROS/TF 失败”从 silent failure 或 task crash，转成了明确的业务错误事件，更利于云端联调和问题归因。

### 3. 测试继续向稳定性和错误分支收口

配套新增或更新了两类测试：

- `test_navigation_executor_spins_ros_node_in_async_loop`
- `test_mark_current_poi_returns_error_when_pose_capture_raises`

可以看出 4 月 16 日的 session 已经不是在“补功能”，而是在补：

- 运行循环完整性
- 失败可观测性
- 协议级错误上报闭环

这类未成文档工作虽然体量不大，但对把执行器从“能演示”推进到“能长期运行和联调”非常关键。

## 四、本周阶段性产出

### 已形成的文档产出

- frontier completion regression 报告
- frontier 最近目标选择复盘报告
- behind-goal stutter 真机调试报告
- G1 导航接口与 JSON 一致性报告
- G1 自动建图导航迁移设计
- G1 自动建图导航迁移实施计划
- G1 navigation executor semantic POI 设计

### 已形成的代码与脚本产出

- frontier completion 策略修复与单测
- frontier live goal 选择链路修复与单测
- behind-goal 起步卡顿相关配置修复
- Realsense depth-to-scan 参数修复
- `g1-auto-mapping-nav-migration` skill 包
- `package_sources.sh` / `preflight_check.sh` / `install_or_repair.sh` / `target_env.sh`
- `start_navigation_headless.sh` 一键 headless 启动脚本
- `navigation_executor.py`
- `navigation_executor_core.py`
- `navigation_protocol.py`
- `nav2_action_bridge.py`
- `pose_provider.py`
- `poi_store.py`
- 多组 protocol / runtime / packaging / script 测试

## 五、当前遗留风险与关注点

### 1. behind-goal 问题虽然显著改善，但时序边缘问题未完全归零

报告中仍提到偶发：

- `Message Filter dropping message`
- TF cache 提前/过期类告警

这意味着“严重卡死”已经缓解，但高负载边缘条件下的时序稳定性仍值得继续跟。

### 2. 导航执行器仍处于快速收敛阶段

从 4 月 15 日到 4 月 16 日的连续改动看，执行器现在已经跨过“功能能跑”的阶段，正在进入“异常闭环和运行稳定性补齐”的阶段。后续仍要重点观察：

- asyncio 与 ROS spin 的长期兼容性
- TF 不可用时的错误上报完整性
- WebSocket 断连、重连、任务恢复行为
- POI 本地几何真值在长时运行中的一致性

### 3. 迁移方案已有设计和计划，但尚未看到完整落地验收记录

4 月 14 日已经把迁移方案设计得很清楚，但从当前 4 天材料里，还没有看到“在另一台 G1 上完整执行迁移并验收通过”的闭环记录。这是后续最自然的一步。

## 六、建议的下周重点

1. 继续把导航执行器从“功能完成”推进到“异常、断连、长期运行稳定”全部有测试覆盖。
2. 用真实服务端联调一次 flat JSON 协议，尽量把字段名、错误码、状态事件一次锁死。
3. 继续跟踪 behind-goal 修复后的高负载边缘情况，重点观察 TF/scan 时序告警是否还会带来次级行为异常。
4. 选择一台目标 G1，按 4 月 14 日迁移设计与计划走一遍完整 preflight、repair、verify，补一份验收报告。

## 七、一句话总结

最近 4 天的工作已经把 `g1_nav` 从“探索行为调参 + 真机问题点修补”推进到了“可联调、可迁移、可 headless 运行”的新阶段；其中最值得关注的增量，一是整机迁移 skill 包已经形成完整技能树，二是导航执行器链路已经基本搭起骨架，而 4 月 16 日的未成文档 session 正在继续补齐它的运行稳定性和错误闭环。
