# 2026-04-11 G1 Nav 最近几次 Session 汇总

## 时间范围

- 汇总窗口：2026-04-07 至 2026-04-11
- 关注范围：`g1_nav` 最近几轮设计、实现、验证与现场策略收敛
- 本报告定位：给后续继续调参与实机联调的人一个高密度摘要，不替代各专题设计文档

## 总结

最近几次 `g1_nav` session 的主线可以概括成三件事：

1. frontier 搜索阶段开始显式利用 `global_costmap`，减少被高代价碎片串起来的错误 frontier
2. Nav2 运行时从“看静态地图走路径”升级到“动态障碍先停、再重规划、再解释 abort 上下文”
3. embedded recovery 从“等待 / 清图 / 后退”升级成“按障碍方位定向脱困，并保持 recovery 直到真正脱离”

这几轮工作的共同方向不是继续堆参数，而是把失败分层：

- frontier 生成是否正确
- launch / runtime 数据链路是否清晰
- 动态障碍是否能在局部停下、在全局重规划
- 机器人已经陷入障碍时，是否有专门的脱困动作而不是直接把 goal 拉黑

## Session 1：Frontier 搜索开始借助 Costmap 几何

日期主线：

- 2026-04-07

核心问题：

- `grid_map` 上未知边界附近有高代价障碍碎片时，frontier cluster 会被错误连通
- cluster 质心被拉偏后，后面的 snapped goal 只能“修最终落点”，无法修复错误 frontier 本体

本轮收敛：

- 在 frontier 搜索阶段引入 `global_costmap` 过滤，而不是只在 goal 准入阶段才看 costmap
- 通过 `costmap_search_threshold` 保留低代价边缘探索空间，同时切断高代价膨胀碎片造成的假连通

工程意义：

- frontier 质心更接近真实可探索边界
- snapped goal 不再承担“修坏 frontier”的主要职责

参考文档：

- [2026-04-07-frontier-costmap-search-log.md](/home/unitree/ros2_ws/src/g1_nav/docs/reports/2026-04-07-frontier-costmap-search-log.md)
- [2026-04-07-scan-priority-local-costmap-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-04-07-scan-priority-local-costmap-design.md)
- [2026-04-07-scan-priority-local-costmap-plan.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-04-07-scan-priority-local-costmap-plan.md)

## Session 2：Launch 分层与动态障碍 Stop / Replan

日期主线：

- 2026-04-09

核心问题：

- bringup、auto explore、Realsense depth-to-scan 的职责混在一起
- 路径原本可达，但新障碍出现后，系统没有足够快地停下并重规划
- frontier handoff、abort、blocked path 的语义容易混淆

本轮收敛：

- 把 launch 栈拆成三层：
  - `2d_g1_nav2_bringup.launch.py` 负责基础 `slam + nav2`
  - `g1_auto_explore.launch.py` 负责 frontier exploration
  - `realsense_depth_to_scan.launch.py` 负责本地深度障碍层
- 在运行时链路上加入 `nav2_collision_monitor`
- 给 `global_costmap` 增加 `/scan` 动态障碍标记
- 提高 DWB `BaseObstacle` critic 权重
- frontier 结果解释上，开始区分“真正不可达”与“handoff / 上下文中的 abort”

工程意义：

- 新障碍出现时，系统先停，再让 planner 看见，再尝试重规划
- exploration 与感知层启动路径更清楚，便于单独调试
- abort 不再被粗暴地一律解释成“当前 frontier 必须拉黑”

相关提交：

- `6223539 feat: harden frontier navigation against blocked paths`

参考文档：

- [2026-04-09-g1-nav-launch-layering-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-04-09-g1-nav-launch-layering-design.md)
- [2026-04-09-g1-nav-launch-layering-plan.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-04-09-g1-nav-launch-layering-plan.md)
- [2026-04-09-nav2-dynamic-obstacle-stop-replan-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-04-09-nav2-dynamic-obstacle-stop-replan-design.md)
- [2026-04-09-nav2-dynamic-obstacle-stop-replan-plan.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-04-09-nav2-dynamic-obstacle-stop-replan-plan.md)

## Session 3：差分化导航与 Embedded Obstacle 定向脱困

日期主线：

- 2026-04-11

核心问题：

- 正常 Nav2 跟踪仍会产生 `y` 方向速度，不符合当前 G1 导航上希望采用的差分近似
- 机器人 footprint 已经压进障碍物时，原有 recovery 只会 wait / clear / backup，无法按障碍相对方位脱困
- embedded recovery 应该留在 Nav2 BT 内部，不应立刻把 goal 拉黑

本轮收敛：

- 将正常导航的 `y` 方向速度能力关掉：
  - `FollowPath.max_vel_y = 0.0`
  - `FollowPath.vy_samples = 1`
  - `velocity_smoother.max_velocity[1] = 0.0`
  - `velocity_smoother.min_velocity[1] = 0.0`
- 新增 `EscapeObstacle` BT 插件：
  - 订阅 local costmap 和 published footprint
  - 统计 footprint 内 lethal overlap cell
  - 在 base frame 下判断前后左右哪一侧压力更大
  - 只发布单轴反向速度，直到连续观测确认已脱离
- 两棵 embedded recovery BT 都改成：
  - `CancelControl`
  - `EscapeObstacle`
  - 清 local/global costmap

本轮额外硬化：

- success 不再依赖重复 tick，而依赖新的 costmap / footprint 观测
- embedded 但算不出有效 escape command 时直接 `FAILURE`，让 BT 进入下一层 recovery
- 新增 `max_escape_duration`，避免 Level1 recovery 无限 `RUNNING`
- 对 blackboard `node` / `tf_buffer` 做显式校验，避免空指针崩溃

工程意义：

- 正常导航阶段更加贴近差分模型
- 脱困行为从“盲目退”升级为“按障碍相对位置定向退”
- embedded recovery 保持在 Nav2 BT 内部，不会马上污染 frontier blacklist 语义

参考文档：

- [2026-04-11-g1-embedded-obstacle-escape-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-04-11-g1-embedded-obstacle-escape-design.md)
- [2026-04-11-g1-embedded-obstacle-escape-plan.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-04-11-g1-embedded-obstacle-escape-plan.md)

## 验证状态

本轮 2026-04-11 的 embedded escape 实现已经完成并做过针对性验证：

- isolated `colcon build` 通过
- 目标 gtest 通过：
  - `escape_obstacle_action_test`
  - `frontier_navigation_result_policy_test`
  - `frontier_goal_selector_test`
- Python 集成契约测试通过：
  - `test_realsense_nav2_integration.py`

此外，本次汇总与 push 前还修复了 `g1_nav` 仓库本地 `.git` 中的零字节 loose object 损坏问题，恢复了正常的 `status / commit / push` 流程。

## 当前剩余问题

还没有闭环的点主要有：

- `EscapeObstacle.max_escape_duration` 仍需要结合真机再调
- embedded escape 的真实 TF / topic 时序表现还需要更多现场观察
- 极端地图噪声下，局部 overlap 方向是否会抖动，仍需关注
- `embodied` 内网远端当前在这个环境里无法解析 DNS，本次 push 会优先走 `origin`

## 建议的下一步

1. 先做真机回放或实地测试，重点观察 embedded escape 是否会过早切到 Level2
2. 把 `max_escape_duration` 和 `clear_confirmations` 作为现场调参重点
3. 若后续仍出现 frontier / abort 语义混杂，优先沿着“结果解释层”继续分离，而不是重新堆 recovery patch

## 相关文档索引

- [2026-04-03-g1-navigation-methodology-report.md](/home/unitree/ros2_ws/src/g1_nav/docs/reports/2026-04-03-g1-navigation-methodology-report.md)
- [2026-04-07-frontier-costmap-search-log.md](/home/unitree/ros2_ws/src/g1_nav/docs/reports/2026-04-07-frontier-costmap-search-log.md)
- [2026-04-09-g1-nav-launch-layering-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-04-09-g1-nav-launch-layering-design.md)
- [2026-04-09-nav2-dynamic-obstacle-stop-replan-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-04-09-nav2-dynamic-obstacle-stop-replan-design.md)
- [2026-04-11-g1-embedded-obstacle-escape-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-04-11-g1-embedded-obstacle-escape-design.md)
