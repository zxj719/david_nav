# 周报：David 导航（上周 2026-07-27 ~ 08-03）

分支：`feature/dwb-control-opt`
（更早的直线振荡分析与标定工具链工作属上上周，详见
`2026-07-24-weekly-nav-chassis-improvements.md`，此处不重复。）

---

## 一、蛇形走位根因定论

直线蛇形由**两个延迟共同造就**，闭环实测（bag cmd↔TF 互相关，r=0.86 以上）：

| 延迟源 | 实测量 | 说明 |
|---|---|---|
| **SLAM 延迟** | lightning map→odom TF 滞后 ~0.3s | 位姿反馈滞后吃掉控制环相位；也曾把 RotationShim 的坐标变换打挂 |
| **底盘响应延迟** | cmd→整车响应 600–900ms（软件链仅 60ms） | **电量低时响应进一步变慢**；DWB 前向仿真按零延迟假设，滞后即振荡 |

排除项（均有数据）：双轮恒定不平衡（纠偏净偏置仅 1–10% 且变号）、
过热（幅度逐 goal 秒级重置）、执行增益（0.77–0.83，满电/常态一致）。

## 二、上周改进（四项）

### 1. 目标选点连续外扩
POI 压在 costmap 代价里时，从 snap 半径起按 0.25m 逐圈外扩（至 3m）取第一个
可达点，圈内离机器人最近优先；**到达扩圈点即判导航成功**（旧语义在目标落于
膨胀区时报"仍不可达"，服务端只会原地重试同样失败）。参数
`nav_goal_max_snap_radius` / `nav_goal_snap_step` 可配，配合 global costmap
cost 值调整使用。

### 2. RotationShim 固化（最短方向原地旋转）
- 根因修复：Humble 上游 shim 的 TF 变换 tolerance **写死 0.1s**，SLAM 滞后
  ~0.3s 下每周期失败、静默回退 DWB 且拖垮 20Hz 控制循环（443 次失败 ↔ 455 次
  丢拍）。以 **workspace overlay 补丁版**根治（tolerance 参数化，读
  `FollowPath.transform_tolerance: 1.0`），`~/ros2_ws/src/nav2_rotation_shim_controller`；
- 执行兜底：原地旋转 kick floor `0.35→0.55`（实测破启值）、`max_angular_z
  0.5→0.6`，shim 旋转速度 0.55 对齐——即使 shim 偶发回退，DWB 原地旋转也
  转得动；滚动态参数不受影响（直通不抬升）。
- 回归验证指标：bringup 日志 Extrapolation Error = 0、无 missed-rate 刷屏、
  大角度旋转走最短方向。

### 3. 冗余兜底 + 日志
- **WS 事件送达保证**：修复"先出队再发送"的丢事件 bug（断联瞬间 on_arrived
  永久丢失 → 服务端超时判导航失败，即"明明到点却上报失败"）。事件跨重连暂存、
  发送成功才移除、重连后自动重发；应答帧统一走队列。附回归测试。
- **断联可归因日志**：连接建立/断开（时长、未送达事件数、距最后收到服务端帧
  秒数）、终态帧审计（`OUT sent / RESENT`）、半死连接告警（3 个心跳周期无
  服务端帧）。
- 其他兜底：headless pre-flight 残留清场（上代 nav2/wheeltec 等退净再启动，
  根治 TF 两棵树与 lifecycle SIGABRT）；测试沙箱防误杀真机栈的逃生门。

### 4. agent 端通信链路改造
断联根因定位：executor↔上位机此前**走 WiFi 而非物理链路**（日志特征
"no close frame received or sent" = TCP 无握手死亡）。已改为**网线物理连接，
不再断联**；第 3 项的送达保证与日志保留为第二道防线。

## 三、本周计划（对准两个延迟源）

1. **底盘响应延迟**：按《STM32 速度环调参指导》提带宽——排查速度指令是否
   经过位置环、固件斜坡/编码器滤波、速度环 Kp/Ki；**覆盖低电量工况**
   （电量低响应更慢，标定与验收需记录电池电压、低电复测）；
   验收：满载 wz 阶跃 90% ≤ 200ms（alias 9 直接出数）。达标后
   `max_vel_theta 0.25→0.50` 回退，直线速度上限恢复。
2. **SLAM 延迟**：与 SLAM 侧推进 lightning map→odom 发布滞后压缩
   （目标 <0.1s）——它同时影响控制环相位、RotationShim 与一切 map 系闭环组件。
3. 回归验证：overlay shim 实车指标（见上）；有线链路下与服务端联调重连补发
   语义（服务端需接受旧 request_id 的 on_arrived，会话不绑连接）。

---
关联：`2026-07-17-dwb-straight-line-limit-cycle-analysis.md`（振荡机理）、
`2026-07-23-stm32-velocity-loop-tuning-guide.md`（本周固件调参依据）、
`2026-07-24-weekly-nav-chassis-improvements.md`（上上周详版）
