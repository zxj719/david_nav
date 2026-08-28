# 周报：David 导航/底盘控制改进（2026-07-17 ~ 07-24）

分支：`feature/dwb-control-opt`
核心成果：**直线行走正弦振荡问题基本解决** —— wz 指令 RMS 从 0.32–0.40 降到
**0.10**、饱和占比从 10–65% 降到 **0%**（07-24 实跑验证），配套建成底盘标定
工具链、修复多项工程可靠性问题。

---

## 1. 直线正弦振荡：从定性猜测到定量根治

### 1.1 诊断路径（三份递进的分析报告）

**07-17 指令侧取证**（bag 全挂，靠 wheeltec 串口日志 47k 条指令逆推）：
- 振荡是**控制极限环**，不是硬件走不直：wz 指令 6.6–9.4s 周期近对称交替
  （自相关 +0.72），单 goal 净角度积分仅占纠偏活动量 1–10% 且符号不定
  → **排除"恒定双轮不平衡"假设**；
- 幅度每个新 goal 秒级重置、24 分钟连跑末尾反而最干净
  → **排除"过热退化"假设**；
- 三重根因：①底盘执行增益未标定（外环模型失配）；②david_move 死区抬升地板
  0.15 == DWB 上限 → vx 执行集合退化为 {0, 0.15}、曲率失真最大 4×；
  ③BT 1Hz 重规划把参考线锚在机器人自己身上 → 无直线记忆、振荡中心漂移。

**07-23 闭环实测**（bag 修复后首次 cmd↔实际响应互相关，r=0.86–0.97）：
- STM32 首轮重调后 **wz 增益 2.5× → 0.73–0.83，已修复**；
- 新主导根因浮出：**cmd→整车响应纯滞后 600–900ms**，其中 ROS 软件链仅
  60ms → 全部在固件/机械侧；
- 蛇形量化：横向峰值 34–55cm、RMS 6.7–17cm、波长 ~1.3m。

**07-24 收敛验证**：新配置（新固件 + 软件整形停用 + wz 限幅止血）下
38.5s 直行 wz RMS **0.10**、峰值 0.145、零饱和、跟线方向偏差 4°。

| 指标 | 07-17/23 基线 | 07-24 现状 |
|---|---|---|
| wz 指令 RMS | 0.32–0.40 | **0.10** |
| wz 饱和占比 | 10–65% | **0%** |
| 蛇形横向峰值 | 34–55 cm | 视觉消失（待 bench 定量） |
| wz 执行增益 | ~2.5×（旧固件） | 0.73–0.83 |

### 1.2 落地的控制链路调整

- **`cmd_shaping` launch 开关**（默认 true 不影响他人）：`:=false` 时
  velocity_smoother 转 OPEN_LOOP 直通@20Hz（消 ~100ms 平滑时延）、
  david_move 线速度抬升归零 —— 已在 `launch_nav_with_log.sh` 与
  `start_navigation_headless.sh` 临时固定 false（STM32 已自带闭环，避免双重补偿）；
- **`max_vel_theta 0.50 → 0.25` 止血**（临时）：固件带宽修复前压极限环幅值；
  回退条件 = wz 阶跃 90% ≤ 200ms；
- 拐角可行性由"w=0.5 硬转"改为 DWB 降速过弯（cmd_shaping off 后
  min_vel_x=0 生效，低速档实测已原样到达底盘：串口出现 0.037/0.075/0.112 档）。

## 2. 底盘标定工具链（alias 9）与固件协同

### 2.1 `scripts/calibrate_plant_gains.{sh,py}`（新建）

一键滚动态 v/wz 执行增益标定：v 直行阶跃 / wzstat 原地 / wzroll 滚动叠加三组，
外部真值 + 轮速计双观测，安全框 + GT 闭环自动回位，跑完自动输出
增益/纯滞后/死区标记与修正系数建议；`--analyze` 支持离线重分析。

工具本身经历三轮实战迭代，沉淀了几个重要发现：
- **slam_gt 与 lightning loc 均不稳**：`/lightning/loc/odometry` 实测偶发
  **单帧 6.9m / π rad 重定位跳变**（一次跳变把端点法速率打成 3.1 m/s）。
  真值定稿为纯 lightning 阶梯：激光里程计 > TF > loc（兜底），分析层加
  **跳变缝合 + Theil-Sen 稳健斜率**；
- **旧"v 增益 ~1.2×"结论存疑**：稳健重分析后 v ≈ 0.94–1.05，与轮速计自报一致
  —— 旧数字是跳变真值的伪影；
- **原地旋转 cmd=0.55 恰卡 stick-slip 边界**（yaw 率中位仅 0.15 rad/s）：
  homing 重调为 0.70 + 预测性提前停（lead = |ω实测|×0.45s）+ 位姿中值滤波
  + 越框三连击判定。

### 2.2 STM32 调参指导文档（新建）

`docs/reports/2026-07-23-stm32-velocity-loop-tuning-guide.md`：面向固件同学，
核心方针"**不动增益、只提带宽**"。含实测基线表、硬性验收指标
（满载 wz 阶跃 90% ≤200ms / 增益 0.95–1.05 / 低速 0.05 可持续 / 左右轮差 <3%）、
按嫌疑排序的排查步骤（头号：速度指令勿裹进位置环；二号：固件斜坡/编码器滤波）、
常见坑与迭代记录模板。alias 9 即每轮验收工具。

## 3. 导航行为修复（07-24 三项）

1. **POI 压在 static costmap 里 → 连续外扩找可达点**：goal_resolver 从两档
   固定半径（0.5/1.0m）改为 0.25m 步长逐圈外扩至 3.0m，圈内离机器人最近优先；
   参数 `nav_goal_max_snap_radius` / `nav_goal_snap_step` 可配；
2. **原地旋转方向随机、绕超半圆 → 重新启用 RotationShim** 包裹 DWB：
   按最短角度方向确定性转到与路径朝向差 <45° 再交 DWB。历史"shim 卡死"的
   前提（旧底盘大死区+无 kick floor）已消除，注释含一行回退方法；
3. **大角度原地旋转卡死需人推 → 原地/滚动参数分工**（用户方案）：
   `min_stationary_angular_z: 0.35` 不再随 cmd_shaping 归零 —— 纯旋转保留
   kick floor（shim 原地转 0.5=链路上限），滚动态（v,w 同时非零）不抬升。

47 个单元测试全过；已清 pycache + colcon 重建。

## 4. 工程可靠性与工具链

- **rosbag 录制 bug**：`--output` 指向已存在目录 → 此前多晚 run **零 bag**。
  修复为 bag 落 `<run>/bag/` 子目录（两个入口脚本）；
- **bag 瘦身**：`/lightning/grid_map` 移出默认录制 —— 3.4h 实测吃掉 **107GB**
  且把 TF 频率从 92Hz 压到 11Hz（IO 挤占 = 额外环路时延）；`--full-bag` 保留；
- **headless pre-flight 残留清理**（07-24 晨三连败复盘）：新增清上代 nav2 全家
  + 残留 wheeltec（仅自起底盘时）、**TERM 后轮询等退净（≤8s）再 SIGKILL 补刀**
  —— 只 kill+sleep 1s 正是 controller_server lifecycle SIGABRT 的 race 来源；
  pattern 采用"launch 形态/安装路径形态"防误伤编辑器；
- **RViz costmap 集成**：lslidar RViz（alias 2）加 global/local costmap 显示
  （Durability=Transient Local 关键）+ slam_toolbox `/map` 底图。

## 5. 实车排障记录（速查）

| 现象 | 根因 | 处置 |
|---|---|---|
| POI 秒转 idle，d2g=0.00 | local costmap 60% 致命格包围（定位错位画图）+ map TF 丢失 | 重给初始位姿 + 清 costmap；d2g=0.00 是未收反馈的默认值 |
| wheeltec "can not open serial port" | 固件调参后底盘 USB 串口未回插（总线无 CP2102/CH343；CH340 是 IMU 的） | 回插验证 lsusb + /dev/wheeltec_controller |
| 串口 50Hz 收指令但车纹丝不动（激光里程计 82s 净位移 1.5cm） | 急停/电机使能/动力电（STM32 侧不执行） | 处理后恢复；键盘遥控单指令可快速判别 |
| TF 两棵树 / controller SIGABRT | 快速连续重启不清场 | pre-flight 清理已自动化（§4） |
| "机器人全在代价里、controller 不执行"疑问 | 实为膨胀渐变区显示效果；该 run 无 No-valid-trajectories 且指令正常 | 真因是上一行的底盘不执行 |

## 6. 遗留与下周计划

1. **固件带宽验收**：`9 --tests v,wzstat,wzroll` 量新固件纯滞后是否已 ≤200ms
   （周初基线 600–900ms）；达标后 `max_vel_theta 0.25 → 0.50` 回退；
2. **完整长直线 + controller_bench** 定量蛇形残留（目标横向峰值 <0.1m）；
3. **RotationShim / 外扩 / 原地抬升** 三项新行为实车回归；
4. local costmap `inflation_radius 0.40 < circumscribed 0.553` 的几何不一致
   （global 已修 0.60，local 未动）——需连 `cost_scaling_factor` 一起调后窄道实测；
5. 上位机 WS 服务（172.16.22.205:8101）本周持续拒连，POI 下发链路待联调。

---
关联文档：`2026-07-17-dwb-straight-line-limit-cycle-analysis.md`、
`2026-07-23-stm32-velocity-loop-tuning-guide.md`
主要改动文件：`scripts/calibrate_plant_gains.{sh,py}`（新）、
`launch/2d_david_nav2_bringup.launch.py`、`config/nav2_params_2d.yaml`、
`david_nav/navigation_goal_resolver.py`、`david_nav/navigation_executor.py`、
`start_navigation_headless.sh`、`navigate_semantic_pose_with_debug_bag.sh`、
`scripts/launch_nav_with_log.sh`、`~/.bashrc`（alias 9）、
lslidar RViz 配置（lightning_ws）
