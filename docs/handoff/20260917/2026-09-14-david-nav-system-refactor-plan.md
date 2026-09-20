# David 导航系统重构与优化总方案

**日期**：2026-09-14
**范围**：定位与感知接口 · 导航能力 · 运动控制与底盘 · 任务与技能 · 反应与安全 · 对外接口，外加 配置层 / 可观测与运维 两个横切层和 地图与点位 数据域；横切目标：轻量化 / 可迁移 / 可追溯
**仓库**：`embodied_navigation`（David AGX `172.16.22.51`，`~/ros2_ws/src`，分支 `feature/dwb-control-opt`，基线 `977e29f`）
**执行方**：机器人上的 Codex；Claude 只出方案
**版本记录**：
- r1 初稿（导航 / 行为策略 / 参数 / 上位机服务四模块）
- r2 待决策项落定、alias 0 当前取值、版本基线、模块划分评审
- r3 wheeltec 底盘驱动与键盘遥控抽取
- **r5（本版）**：odom 话题再纠正——alias 3 实际发布 `/lightning/loc/odometry`，Nav2 现接线 `/lightning/odometry` 无数据，是否接入列 TODO-O1（§2.1）；开始自动执行循环（`refactor_loop/RUNBOOK.md`）
- **r4**：按用户确认的模块划分整体重写；采纳改进项（单一任务管理器、定位健康守护、离线闭环仿真、doctor、adapter C++ 化、调参回流、共享环境包、协议握手 TODO）；纠正三处错误结论（见 §2.1）；foxglove_bridge / import-linter / MCAP 插件一律只列 TODO、不安装

**配套文档**：[`2026-09-14-david-nav-executor-refactor-spec.md`](2026-09-14-david-nav-executor-refactor-spec.md) 是模块④任务管理器内核与模块⑥ WebSocket 适配器的代码级细化；包归属以本文 §5.3 为准。

---

## 0. 名词与流程（先读）

| 名词 | 意思 |
|---|---|
| **alias 3 / 0 / 4 / 5** | David `~/.bashrc` 的数字快捷键。**3** = lightning 定位（后台 RViz + 雷达 + IMU + `run_loc_online`，**起来后需人在 RViz 里手动给初始位姿**）；**0** = 高精度入位导航；**4** = 生产导览（含 WebSocket 执行器）；**5** = 只起底盘。"3+0" / "3+4" 指先起 3、完成手动定位，再起 0 或 4 |
| **run（一次运行）** | 从敲下 alias 到 Ctrl-C 的完整一次 |
| **运行记录** | 每次 run 自动生成的目录：跑的是什么代码、什么参数、发生了什么、占了多少资源 |
| **机器人档案** | 一台机器人的全部差异化参数（尺寸、坐标系、话题、限速、死区、底盘驱动）。**换机器人只改它** |
| **模式叠加** | 在档案之上只写差异的小文件，如"精确入位模式把到点容差收紧到 0.10 m" |
| **场地档案** | 地图、点位文件、agent server 地址 |
| **渲染** | 把"档案 + 叠加 + 场地"合成为各节点真正读取的参数文件与行为树，并存进运行记录 |
| **绑定表** | 说明档案里一个值要写到最终参数的哪几个位置，从构造上保证"必须相等的参数"相等 |
| **任务管理器** | 机器人上**唯一**有权下发导航/入位/运动任务的组件；rviz、WebSocket、命令行都只是它的入口 |
| **技能** | 任务管理器可执行的一类任务：POI 导航、两段式入位，将来的运动原语 |
| **chassis_adapter** | 底盘适配层：所有速度指令的最后一道软件关口（限幅、死区抬升、超时停车、退出零速） |
| **组合进程（composition）** | 把多个 ROS 2 节点装进同一个进程，省进程、内存和 DDS 发现开销 |
| **navrun** | 新的启动管家（Python），取代 1221 行 `start_navigation_headless.sh` 与 precise launch 里的底盘探测 |
| **doctor** | `navrun doctor`：把已知的坑写成自动检查的一条命令 |
| **离线闭环仿真** | 假底盘（积分速度发 odom）+ 假定位（发 TF）+ 真 Nav2，不动车也能跑完一次导航 |
| **BT** | Nav2 行为树 XML，决定导航失败或卡住时做什么 |
| **golden（金样）** | 重构前录下的"标准答案"，重构后逐项比对证明行为没变 |
| **MCAP** | rosbag2 的一种带索引的存储格式（本轮不装，TODO） |

---

## 1. 约束与目标

### 1.1 硬约束

| # | 约束 |
|---|---|
| **C1** | **每一个提交之后，3+0 与 3+4 都必须能顺利启动并完成一次导航。** 允许修改 `~/.bashrc` 的 alias 0 / 4 / 5 指向新入口 |
| C2 | Codex 不下发任何运动指令（零速除外）。需要车动的验证写成步骤交用户执行 |
| C3 | bench 类 alias（6/7/8/9/10）不得"静默坏掉"：要么保持可用，要么在 Phase 报告里列出变更 |
| C4 | alias 3（`~/lightning_ws_new/src`）属定位侧边界，本方案只读取、只建议 |
| C5 | **本轮不安装新软件**：foxglove_bridge、import-linter、rosbag2 MCAP 插件等均列 TODO；Python / pip 版本以当前机上为准 |

### 1.2 目标（按优先级）

1. **可追溯**：每次 run 留下完整、自描述的运行记录；关键事件（含脱困触发）在终端与日志里即时可见。
2. **可迁移**：换机器人 = 新建一份档案（话题、驱动、底盘参数），不改代码；参数有正式文档和自动校验。
3. **模块化**：6 个运行时模块 + 2 个横切层 + 1 个数据域，边界由契约和 import 规则保证。
4. **轻量化**：减少进程数、CPU、内存、写盘；每项优化都有前后测量。
5. **鲁棒性**：单一运动权限、底盘多级超时、定位健康守护、执行器缺陷修复。

### 1.3 本轮只预留、不实现

- 基于雷达点云的实时避障（绕行，或停车并通过 WebSocket 上报状态）
- alias 0 挂 WebSocket 服务
- 给 server 直接提供 前进 / 后退 / 左右旋转 接口
- 新的脱困行为策略（见 TODO-E1）
- 与 server 的协议握手（见 TODO-P1）

---

## 2. 决策、纠正、实测取值、版本与 TODO

> 本节优先级最高，正文与本节冲突时以本节为准。

### 2.1 r4 纠正（此前结论有误，已全文撤回）

| 此前说法 | 纠正（用户确认） | 影响 |
|---|---|---|
| "alias 3 下 `/lightning/odometry` 无发布者，Nav2 与执行器订阅的是死话题" | **r5 再纠正：原判断正确**。用户确认 alias 3 实际发布的是 `/lightning/loc/odometry`；Nav2（controller_server、bt_navigator）与执行器现订阅的 `/lightning/odometry` 无数据。不接时车仍能导航，但 DWB 取不到当前速度、`mark_current_poi` 取不到 odom 位姿 | 档案 `localization.odom_topic: /lightning/loc/odometry`；Nav2 接线单列 `nav2.odom_topic`，现值 `/lightning/odometry` 等值保留；是否及如何接入见 TODO-O1；`check --live` 仍须在定位完成后检查 |
| "BT 横向脱困被 david_move 清零，是个 bug" | **横向脱困本身是错误设计**：差速底盘不应有横向速度，david_move 清零 `linear.y` 是**正确**的。 | 脱困策略重新设计列 TODO-E1（确认后方无障碍才后退，否则停下）；本轮不改脱困行为，只把**脱困触发条件打印到日志和终端**（§10.3） |
| "`named_poses.json` 采点帧未知，需向生产方确认" | 该文件是**手动添加**的，本轮不处理。 | 删除原 TODO-F1；原 TODO-F2 降为备注 |

### 2.2 决策

| # | 决策 |
|---|---|
| Q1 | 导航启动**默认不再杀**机械臂组 `arm_node` / MoveIt；检测到时告警并记事件。档案开关 `preflight.kill_foreign_stacks: false` |
| Q2 | 按 §7.5 隔离 legacy，Phase 7 前逐项列出供确认，隔离不删除 |
| Q3 | 脱困：本轮行为不变 + 触发可见（§10.3）；新策略 TODO-E1 |
| Q4 | Nav2 组合进程：实测后决定（TODO-L4；`component_container_isolated` 已装） |
| Q5 | 导航地图降采样 0.10 m：实测后决定（TODO-L6） |
| Q6 | precise 模式默认录轻量 bag（sqlite3） |
| Q7 | alias 3 侧改动交定位侧（TODO-Q7） |
| Q8 | 定位 odom 话题为 `/lightning/loc/odometry`；Nav2 是否接入见 TODO-O1（§2.1） |
| D1 | 执行器相关代码独立成包（归属见 §5.3） |
| D2 | 不要求 Python 3.8 兼容，以当前 3.10.12 为准；GO2W 复用 TODO-V1 |
| D3 | websockets 以当前加载的 pip 16.1（`~/.local`）为准；依赖固化 TODO-V2 |
| D4 | 未提交修改与未跟踪生产文件由用户先提交 |
| D5 | 入位结构化事件本轮做 |
| D6 | 执行器采点帧保持 `base_link`（行为不变），与 `base_footprint` 分开书写（§2.3） |
| M1 | **采纳 r2 模块划分评审全部要点**（§5） |
| M2 | **采纳改进项**：单一任务管理器、定位健康守护、离线闭环仿真、doctor、adapter C++ 化、调参回流、共享环境包；协议握手列 TODO-P1 |

### 2.3 base_link ≠ base_footprint（务必区分）

`david_nav/urdf/david.urdf:98-101`：`base_link → base_footprint` 固定变换 **xyz = (0.165, 0, −0.4075)，rpy = 0**。

| 帧 | 含义 | 发布者 |
|---|---|---|
| `base_footprint` | 双轮中点的地面投影 = **旋转中心**，在 base_link 前 16.5 cm、下 40.75 cm | URDF（robot_state_publisher） |
| `base_link` | 机身正向本体帧 | lightning（`map→odom→base→base_link`） |
| `base` | SLAM 内部帧，与 IMU 同姿态（机身 −y 为前进） | lightning；**导航禁用** |

| 使用者 | 帧 |
|---|---|
| Nav2 全部 `robot_base_frame`、入位、`pointcloud_to_laserscan.target_frame`、BT `EscapeObstacle`、`scan_priority_layer`（C++ 写死） | `base_footprint` |
| 执行器位姿（`mark_current_poi` 采点、STATUS、snap 候选排序） | `base_link` |
| `semantic_pose_navigator`（legacy） | `base` |
| wheeltec `/odom`（不发 TF） | header `odom_combined`，child `base_footprint` |

规则：档案里分键书写、禁止合并（§12.2）；`navcfg validate` 在 `poi_capture != nav_base` 时打印 WARN 与偏移；文档、日志、注释里不得把二者统称"base"。备注：用 base_link 采的点交给 Nav2 到达，旋转中心会沿朝向偏 16.5 cm —— 目前点位为手动添加，本轮不处理。

### 2.4 alias 0 当前会用到的值（静态核实：launch 声明 + yaml）

> 运行时是否被覆盖需 Phase 0 `ros2 param dump` 核实（TODO-M0）。

| 组件 | 取值 | 来源 |
|---|---|---|
| Nav2 参数文件 | `config/nav2_params_2d_precise.yaml` | `precise_nav_bringup.launch.py:174` |
| velocity_smoother（直通） | yaml：`max_velocity [0.20,0,0.5]`、`min_velocity [-0.20,0,-0.5]`、`velocity_timeout 1.0`、`deadband 0`；launch 覆盖：`OPEN_LOOP`、`20 Hz`、加减速 ±99 | precise yaml；`2d_david_nav2_bringup.launch.py:144-149` |
| behavior_server | `max_rotational_vel 0.8`、`min_rotational_vel 0.4`、`rotational_acc_lim 3.2`、`cycle_frequency 10`、`simulate_ahead_time 3.0`、`global_frame odom`、`robot_base_frame base_footprint`、`transform_tolerance 1.0` | precise yaml |
| DWB / goal checker | `max_vel_x 0.15`、`max_vel_theta 0.50`、`xy_goal_tolerance 0.10`（两处）、`yaw_goal_tolerance 0.4363`、critics 含 `RotateToGoal` | precise yaml |
| Nav2 odom 话题 | `/lightning/odometry`（alias 3 不发布该话题，无数据；TODO-O1） | precise yaml；用户确认 |
| david_move | `input cmd_vel_smoothed`、`output /cmd_vel`、`max_linear_x 0.20`、`min_linear_x 0.0`、`min_stationary_angular_z 0.55`、`max_angular_z 0.6`；代码默认 `cmd_vel_timeout 2.0`、`publish_hz 50`、`min_angular_z 0.0`、`stationary_angular_threshold 0.025` | launch `:340-398`；`david_move.py` |
| dock_navigator | `goal_topic /dock_goal`、`approach_dist 0.5`、`xy_ok 0.10`、`yaw_ok 0.4363`、`max_yaw_retries 0`、`base_footprint`、`map`；代码默认 `nav_timeout 120 s`、`spin_timeout 30 s`、停稳 `0.02 rad/s / 0.4 s / 8 s` | precise launch；`dock_navigator.py:56-70` |
| 地图 | `daolan_map_nav.yaml`：`trinary`、`0.05 m`、`origin [-10.0496, -11.6018, 0]`，image 指向 lightning 数据目录 | 该 yaml |
| 感知 | scan bridge 关（无 /scan）、collision_monitor 关 | launch 默认 |
| 底盘驱动 | `/dev/wheeltec_controller`、`115200`、`odom_frame_id odom_combined`、`robot_frame_id base_footprint`、`gyro_frame_id gyro_link`、odom 缩放 1.0；发 `/odom` `/imu` `PowerVoltage`，不发 TF | `turn_on_wheeltec_robot.launch.py`；`wheeltec_robot.cpp:450-472` |

### 2.5 版本基线（以当前为准，本轮不安装任何东西）

| 类别 | 版本 |
|---|---|
| 系统 | Ubuntu 22.04，Jetson Linux R36.4.7，12 核 / 61 GB |
| Python | 3.10.12 |
| ROS 2 Humble | nav2 1.1.20、rosbag2 0.15.16、rclcpp_components 16.0.19（`component_container_isolated` 可用）、nav2_collision_monitor 1.1.20、teleop_twist_keyboard |
| pip（`~/.local` 优先） | websockets 16.1、PyYAML 6.0.3、psutil 5.9.0、Jinja2 3.1.6、numpy 2.2.6、pytest 6.2.5、flake8 4.0.1 |
| **未安装，均为 TODO，本轮不装** | import-linter（TODO-I1）、rosbag2 MCAP 插件（TODO-L5）、foxglove_bridge（TODO-L7）、twist_mux（不需要） |
| 外部依赖 | `serial`（wjwwood fork，MIT）在 `~/scout_ws/src/serial-ros2`，经 wheeltec 工作空间 underlay 引入 |

实现约束：资源采样用 psutil；渲染只用 PyYAML + 标准库（BT 模板用 `string.Template`）；依赖 import-linter 的门禁在未安装时 skip 并提示；manifest 记录上表版本，漂移告警。

### 2.6 TODO 总表

| ID | 内容 | 时机 |
|---|---|---|
| TODO-M0 | `ros2 param dump` 核实 §2.4 运行时生效值 | Phase 0 |
| TODO-O1 | Nav2 odom 接线评估：controller_server（DWB 取当前速度）、bt_navigator（速度估计）现订阅无数据的 `/lightning/odometry`，执行器 `mark_current_poi` 也依赖 odom。候选 (a) `/lightning/loc/odometry` (b) 底盘 `/odom`（编码器速度） (c) 保持不接。行为变更：单独提交 + 运行记录 A/B + 实机冒烟 | Phase 6 |
| **TODO-E1** | **脱困策略重新设计**：差速底盘无横向分量；仅在确认后方无障碍时后退，否则停下（并上报）；后退速度不超过档案 `limits.max_reverse_vel`。本轮不落实，只做触发可见（§10.3） | 后续 |
| TODO-P1 | 与 server 协议握手 `hello {protocol_version, robot_id, capabilities, run_id}`，需 server 方配合 | 后续 |
| TODO-L4 | Nav2 组合进程 A/B 实测 | Phase 5 |
| TODO-L5 | rosbag2 MCAP 插件：本轮不装，保持 sqlite3 | 后续 |
| TODO-L6 | 导航地图降采样 0.10 m 实测 | Phase 5 |
| TODO-L7 | 显示离机（含 foxglove_bridge）：本轮不做不装 | 后续 |
| TODO-I1 | import-linter：本轮不装，分层契约门禁 skip | 后续 |
| TODO-Q7 | alias 3 侧（`run_loc_online` 输出落盘、RViz 开销）交定位侧 | 后续 |
| TODO-V1 | 复用到 GO2W（Foxy / Python 3.8） | 后续 |
| TODO-V2 | pip 依赖固化（锁版本清单 + 安装脚本）；`package.xml` 中 apt `python3-websockets` 声明改注释说明真实来源 | Phase 7 |
| TODO-V3 | 核查 numpy 2.2.6 与 Humble apt Python 包（如 `tf_transformations`）兼容性（只读检查） | Phase 0 |
| TODO-S1 | sudoers NOPASSWD（`nvpmodel`、`jetson_clocks`）由用户执行 | Phase 4 前 |
| TODO-T1 | 确认 `twist_transform` 有无外部使用者 | Phase 4 |
| TODO-W1 | 抓上行 24 字节状态帧金样（驱动未运行、车静止时只读串口） | Phase 0 |
| TODO-W2 | 实测 `ros2 launch` 下键盘节点读 `/dev/tty` 是否可行 | Phase 4 |
| TODO-W3 | David 档案 `odom_frame_id` 是否改 `odom`（先确认 slam_gt 的 odom→base 桥是否依赖 `odom_combined`） | Phase 4 |
| TODO-W4 | 固件 `SecurityLevel` 是否改 0 作失联保护双保险（Keil 烧写，交用户/固件侧） | 后续 |
| TODO-W5 | udev 规则收敛为一份并入库（需 sudo，用户执行） | Phase 7 |
| TODO-W6 | 旧 wheeltec 工作空间与 `~/scout_ws` serial 是否下线 | Phase 7 后 |
| TODO-H1 | 定位健康守护从 `warn` 切到 `stop`（实机冒烟后） | Phase 6 后 |

---

## 3. 现状诊断（证据）

### 3.1 两条启动链实际拉起了什么

```
alias 3  start_rviz_david001.sh
  ├─ RViz（后台）
  └─ start_loc_david001.sh ─ lslidar_driver · wit_imu_node[python, 空闲 10.4% CPU] · run_loc_online
     （需在 RViz 里手动 2D Pose Estimate，定位成功后才有 map→odom TF 与 /lightning/odometry）

alias 4  start_navigation_headless.sh（bash 1221 行）
  ├─ nvpmodel / jetson_clocks（sudo 密码走环境变量）
  ├─ pre-flight：pgrep 杀 MoveIt、机械臂 arm_node、上代 nav2、残留 RSP、残留底盘
  ├─ DDS 卫生：ros2 daemon stop + 清孤儿共享内存段
  ├─ wheeltec 底盘（wheeltec_robot_node + twist_transform）+ bash 看护（死了重启 1 次）
  ├─ 2d_david_nav2_bringup.launch.py cmd_shaping:=false
  │    robot_state_publisher · controller · planner · behavior · bt_navigator
  │    velocity_smoother(直通) · map_server · lifecycle_manager · david_move
  ├─ 等 /navigate_to_pose → 运行时配置检查（只告警）→ nav2 看护（死了全栈退出）
  ├─ rosbag（轻量话题，默认开）
  └─ navigation_executor.py（前台，tee executor.log）

alias 0  ros2 launch precise_nav_bringup.launch.py
  ├─ pgrep 判定底盘是否在跑 → 没在跑就 bash -c 起 wheeltec
  ├─ 同一 bringup（精确参数快照）· 9 个进程
  └─ dock_navigator
     （无 pre-flight、无 DDS 卫生、无看护、无日志落盘、无 bag、无运行记录）
```

### 3.2 问题清单

| 域 | # | 问题 | 证据 |
|---|---|---|---|
| 编排 | A1 | 同一导航栈两套编排（bash vs launch），alias 4 的历次防护 alias 0 全没有 | §3.1 |
| 编排 | A2 | pgrep 模式会杀机械臂组 `arm_node` 与 MoveIt | `start_navigation_headless.sh:1088-1090` |
| 编排 | A3 | `operator_console/` 与 systemd 单元已失效（路径 `/home/unitree`） | `profiles.yaml` |
| 导航 | N1 | `2d_david_nav2_bringup.launch.py` 607 行，大半是停用分支 | 该文件 |
| 导航 | N2 | 直通 `velocity_smoother` 只剩钳位与 1.0 s 静默停车，多一进程一跳 | §2.4 |
| 导航 | N3 | 9 个独立进程，未用组合进程 | 实测 |
| 导航 | N4 | 全局代价图 0.05 m × 948×813 = 77 万格 | `nav2_params_2d.yaml:374-386` |
| 导航 | **N5** | **生产控制器依赖未入库代码**：`RotationShimController` 来自未跟踪的补丁版 `src/nav2_rotation_shim_controller/` | `git status` |
| 导航 | N6 | 大量停更入口与停用代码 | `git log -1` |
| 底盘 | W1 | **驱动无指令超时 + 固件默认 `SecurityLevel=1` 不做失联保护 ⇒ 上游崩溃后车按最后速度一直走**（与 2026-05-23 事故一致） | `wheeltec_robot.cpp:37-90`；固件 `sys.c:42`、`balance_task.c:214-221` |
| 底盘 | W2–W6 | 每帧 INFO 日志（35 分钟 11 MB）、odom `position.z` 写成 yaw、`twist_transform` 常驻 `/cmd_vel` 发布者、`odom_combined` 不一致、udev 规则三份无记录 | §8.1 |
| 底盘 | W7 | 驱动工作空间不在 git；`serial` 依赖藏在 `~/scout_ws` | 实测 |
| 底盘 | W8 | 两套键盘遥控；Wheeltec 自带的直发 `/cmd_vel`、无上限、默认 domain 1 | §8.1 |
| 任务 | T1 | **dock_navigator 与 WS 执行器是两个独立任务状态机**，各自直接给 bt_navigator 发目标；将来 WS 挂到 alias 0 会互相抢占 | 两份源码 |
| 任务 | T2 | 执行器缺陷：畸形帧拆连接、导航 task 崩溃成僵尸、拷贝安装跑旧码等 | 执行器 spec §2.3 |
| 反应 | B1 | "遇到 X 怎么做"分散在 BT、DWB、执行器、dock、david_move、bash 六处 | 各文件 |
| 反应 | B2 | 脱困：BT 设计含横向分量（差速不适用，david_move 清零是对的）；后退速度 0.25 高于 DWB 0.15；脱困触发的 WARN 在 alias 4 下只进 bringup.log，终端不可见 | BT XML `:26-45`；`robot_embedded_in_obstacle_condition.cpp:147-185`；`escape_obstacle_action.cpp:304-392` |
| 反应 | B3 | 恢复行为曾整体失效数周，缺不变量检查 | 历史事故 |
| 感知 | S1 | 点云→scan 写在导航 launch 里；无定位健康监测（历史上位姿发散时导航照跑） | launch `:227-258` |
| 配置 | P1 | 三份 650–680 行 Nav2 参数靠人工派生 | `diff` |
| 配置 | P2 | 机器人常数散落：footprint 两遍；`base_footprint` 在 yaml/BT/C++/launch；限速在 DWB/smoother/david_move/BT 四处 | 各文件 |
| 配置 | P3 | 参数文档是日记式注释 | `nav2_params_2d.yaml:174` |
| 配置 | P4 | 关键约束只写在注释里，运行时只告警 | `verify_nav2_runtime_config` |
| 观测 | O1 | alias 0 零记录；alias 4 无清单（代码版本、生效参数）；与 alias 3 会话无关联；无结构化事件、资源数据、摘要；`~/.ros/log` 2.1 GB 无清理 | 实测 |
| 环境 | E1 | `ros_env.conf` / DDS 白名单放在 `david_nav/config/`，却被 systemd、脚本、其他组引用 | `ros_env.conf` 头注释 |

---

## 4. 参考的业界做法

| 参考 | 做法 | 对应 |
|---|---|---|
| Clearpath `robot.yaml` | 一台机器人一份 yaml，生成器产出 launch 与参数 | 机器人档案 + 渲染（§12） |
| Autoware `vehicle_info` | 车辆常数集中在车辆参数包，算法包不含车辆常数 | 档案与算法包分离 |
| PickNik `generate_parameter_library` | 参数用 YAML 声明类型/范围/描述，自动生成校验与文档 | 各模块自带 schema（§12.1） |
| Open-RMF fleet adapter | 面向调度的逻辑与面向机器人的 API 分层 | 对外接口 ↔ 任务管理器 ↔ 技能 |
| Nav2 Docking Server | 先到 staging pose 再精确入位，机器人差异插件化 | 入位技能 |
| Nav2 Simple Commander | "导航即库"，隐藏 action 细节 | 统一 Nav2 客户端 |
| Nav2 `use_composition` | Nav2 服务器装进一个容器进程 | TODO-L4 |
| Nav2 Collision Monitor | 控制器之后、底盘之前做停车/减速区域判定 | 安全闸预留位置 |
| launch_ros 事件（`OnProcessExit`、`respawn`、`Shutdown`） | 用事件表达进程死亡处理 | 取代 bash 看护 |
| Strangler Fig / Google Python Style / REP-2004 / AsyncAPI | 渐进替换、代码规范、包质量、协议文档 | 见执行器 spec |

---

## 5. 目标架构

### 5.1 分层

```
┌──────────────────────────────── 运行时模块 ────────────────────────────────┐
│ ⑥ 对外接口      WebSocket 适配器（nav WS v1）  [预留]WS 服务 / 运动原语 API    │
│                      │ 只做协议翻译                                          │
│ ④ 任务与技能    任务管理器（唯一运动权限） ── 技能：POI 导航 · 两段式入位 · [预留]原语 │
│                      │                     ▲ rviz /dock_goal、CLI 也是入口     │
│ ⑤ 反应与安全    恢复行为树 · 脱困触发上报 · 底盘多级超时 · [预留]障碍策略/安全闸  │
│ ② 导航能力      Nav2（规划/控制/代价图/行为）+ 自研插件                          │
│ ③ 运动控制与底盘 chassis_adapter · wheeltec_driver · 键盘遥控                   │
│ ① 定位与感知接口 契约检查 · 定位健康守护 · 点云预处理      —— 实现（lightning）在外部 │
└──────────────────────────────────────────────────────────────────────────────┘
 横切：配置层（schema 归各模块，<robot>_config 组合）   横切：可观测与运维（navrun · 运行记录 · doctor · 离线仿真）
 数据域：地图与点位（版本化 · hash · frame 语义）
```

**依赖规则**：
1. 模块之间只通过话题 / 动作 / 服务 / 配置契约交互；Python 包之间按 §5.4 的 import 规则。
2. **机器人常数只允许出现在 `<robot>_config`**。
3. **只有任务管理器可以向 Nav2 下发 `NavigateToPose` / `Spin` 等任务**；rviz、WS、CLI、bench 走任务管理器入口（bench 在迁移完成前豁免）。
4. **所有速度指令必须经过 chassis_adapter**（遥控也不例外）。
5. 与安全相关的响应（超时停车、脱困、将来的安全闸）不得依赖 Python 任务管理器或 WS 连接存活。

### 5.2 指令链路（本轮 vs 预留）

```
本轮：
  controller / behavior_server ─▶ /cmd_vel_nav    ─┐
  键盘遥控（互斥，navrun 锁）  ─▶ /cmd_vel_teleop ─┴▶ chassis_adapter ─▶ /cmd_vel ─▶ wheeltec_driver ─▶ STM32
                                                     （限幅/死区/超时）         （超时，D3）
预留：
  [原语] ─▶ /cmd_vel_primitive ─┐
  以上各源                       ├▶ [仲裁] ─▶ [安全闸：障碍策略] ─▶ chassis_adapter …
```

本轮 adapter 的输入话题由模式决定（导航模式 `/cmd_vel_nav`，遥控模式 `/cmd_vel_teleop`），不起仲裁与安全闸进程（轻量化）。

### 5.3 包划分

| 模块 | 包 | 类型 | 来源 |
|---|---|---|---|
| ① 定位与感知接口 | `nav_perception_iface` | ament_python | 新；点云→scan 配置从导航 launch 迁入 |
| ② 导航能力 | `nav_plugins` | ament_cmake | 从 `david_nav` 迁出 `ScanPriorityLayer`、BT 插件、BT 模板 |
| ③ 运动控制与底盘 | `wheeltec_driver` | ament_cmake | 从 `turn_on_wheeltec_robot` 抽取 |
| ③ | `chassis_adapter` | ament_python → P5 C++ | 由 `david_cmd/david_move` 通用化 |
| ③ | `chassis_teleop` | ament_python | 由 `david_cmd/david_keyboard` 通用化 |
| ④ 任务与技能 | `nav_tasks` | ament_python | 执行器 spec 的 core（状态机/注册表/处理器/重试/supervisor/snap）+ `dock_navigator`/`dock_finish`/`dock_settle` |
| ⑤ 反应与安全 | 放在 `nav_plugins`（BT 模板、脱困触发上报）与 `chassis_adapter`/`wheeltec_driver`（超时） | —— | —— |
| ⑥ 对外接口 | `nav_executor` | ament_python | 执行器 spec 的 protocol + transport；包名沿用 |
| 横切：可观测与运维 | `nav_bringup` | ament_python | 新：`nav.launch.py`、navrun、navcfg、运行记录、doctor |
| 横切：离线仿真 | `nav_sim` | ament_python | 新；从 `scripts/sim_fake_chassis.py` 起步 |
| 横切：共享环境 | `david_system_env` | 纯数据 | `ros_env.conf`、`fastdds_david.xml`、`load_ros_env.sh` 迁入，旧路径留软链 |
| 横切：配置 | `david_config` | 纯数据 | 机器人档案、模式叠加、URDF、底盘参数 |
| 数据域 | `nav_site_data`（代码）+ `david_config/sites/`（数据） | ament_python | POI 仓库、地图加载与校验 |
| 外部依赖 | `third_party/serial` | vendored | 从 `~/scout_ws/src/serial-ros2` 原样搬入（MIT） |
| 过渡 | `david_nav` / `david_cmd` | 现有 | 保留 bench 工具链（alias 7/8/9/10）、位置环、shim；逐步瘦身 |

物理包可以先合并（如 `nav_perception_iface` 暂放 `nav_bringup` 子目录、`nav_site_data` 暂放 `nav_tasks` 子目录），但 import 边界必须按 §5.4 执行。

### 5.4 import 规则（本轮用单测里的 AST 扫描实现；import-linter 为 TODO-I1）

- `nav_tasks` 不得 import `websockets`、`nav_executor`。
- `nav_executor` 只能通过 `nav_tasks` 的公共 API（`TaskApi`）调用任务管理器。
- `nav_site_data` 不得 import `rclpy`。
- `nav_bringup` 可以 import 各包的配置 schema，不得 import 运行时实现。
- 任何包不得出现 `"base"`、`"base_link"`、`"base_footprint"`、`/lightning/`、`/david001/` 字面量（`david_config` 除外；测试夹具除外）。

---

## 6. 模块①：定位与感知接口

### 6.1 职责

本模块不实现定位，只负责"导航侧与定位/感知之间的契约是否成立"，并给出定位质量的实时判断。

### 6.2 契约

| 契约 | 必须满足 | 检查 |
|---|---|---|
| 定位 | 手动定位完成后发布 TF `map→odom→<frames.body>`，`<frames.body>→<frames.nav_base>` 可查；`localization.odom_topic` 有数据 | `navcfg check --live`（**等到 map→odom 出现后再查 odom**） |
| 底盘 | 订阅 `chassis.cmd_topic`；发布 `chassis.odom_topic`；退出停车 | 同上 + 驱动诊断 |
| 感知（启用时） | `sensors.scan_from_cloud.cloud_topic` 有 `PointCloud2`，可变换到 `frames.scan_target` | 同上 |

### 6.3 定位等待与健康守护（`localization_monitor`，1 个轻量节点，5 Hz）

- **等待手动定位**：navrun 启动后若 `map→odom` 不存在，终端每 5 s 打印一次"等待在 RViz 手动给初始位姿"，记事件 `localization_waiting`；出现后记 `localization_ready`。不因此判启动失败。
- **TF 延迟**：`map→base_footprint` 最新变换时刻与当前时刻差超过 `localization_health.max_tf_age_sec`（默认 0.5）→ WARN 事件。
- **位姿发散**：滑动窗口内，定位位移与底盘 `/odom` 积分位移之差超过 `localization_health.max_divergence_m`（默认 1.0 m / 2 s）→ ERROR 事件（历史上机器人几乎没动、地图位姿却飞出 28 m）。
- **动作**：`localization_health.action: warn | stop`，本轮默认 `warn`（终端 + 日志 + 事件）；`stop` 时通知任务管理器取消当前任务。实机冒烟后切 `stop`（TODO-H1）。
- 输出 `/diagnostics` 与 `/nav/events`。

### 6.4 点云预处理

`pointcloud_to_laserscan` 的全部参数从导航 launch 迁到本模块的 launch 与档案 `sensors.scan_from_cloud`，默认仍关闭（与现状一致）。将来避障的输入预处理也放这里。

---

## 7. 模块②：导航能力

### 7.1 统一入口

```
ros2 launch nav_bringup nav.launch.py robot:=david mode:=tour|precise|bench_slamgt site:=daolan \
    executor:=true|false chassis:=on|off composition:=false run_dir:=<navrun 传入>
```

launch 只描述进程图，读取 navrun 渲染进运行记录的参数文件；不做 pgrep、sudo、文件操作。

### 7.2 bash 功能 → 新归属（Phase 4 逐项验收）

| 现有功能 | 新归属 | 方式 |
|---|---|---|
| nvpmodel / jetson_clocks | navrun `platform` | 档案开关；sudoers 只放行这两条（TODO-S1），不再传密码 |
| pre-flight 清残留 | navrun `preflight` | 档案 `preflight.kill_patterns`；机械臂/MoveIt 默认只告警（Q1）；保留 TERM→轮询 8 s→KILL |
| DDS 卫生 | navrun `dds_hygiene` | 原逻辑搬迁（只删无进程映射的段） |
| 单实例锁 | navrun | `flock /tmp/navrun.lock`，0 / 4 / 5k 互斥 |
| 底盘自起/已在跑跳过 | navrun 探测 + launch | launch 不再 pgrep |
| 底盘死亡重启 1 次 | launch | `respawn` + `OnProcessExit` 计数 → `Shutdown` |
| nav2 死亡全栈退出 | launch | 关键节点 `OnProcessExit → Shutdown` |
| 退出零速 | chassis_adapter + wheeltec_driver + navrun | 三层都发零速 |
| 等 nav2 就绪 | navrun | 复用 `wait_nav_ready.py` 逻辑 |
| 运行时配置检查 | `navcfg check --live` | 完整不变量（§12.4），`--strict` 时失败退出 |
| bag | 运行记录器 | 话题清单进档案；sqlite3 |
| 前台执行器 | launch 管理 | 日志进运行记录 |

### 7.3 轻量化（先测、再改、A/B 对比）

| # | 项 | 预期收益 | 行为保持 / 状态 |
|---|---|---|---|
| L0 | 基线测量 | —— | Phase 0，指标见 §15 |
| L1 | 日志降噪：底盘驱动写帧日志降 DEBUG（§8.3 D5）；`ROS_LOG_DIR` 指进运行记录 | chassis.log 11 MB/35 min → KB 级 | 只影响日志 |
| L2 | 去掉直通 smoother | 少 1 进程、少一跳 | 等价迁移：`behavior_server.max_rotational_vel` 0.8→0.5；adapter `hard_max_angular_z` 保持 0.6（否则 0.55 地板被钳回）；adapter `cmd_vel_timeout` 2.0→1.0；线速度钳位已由 adapter 覆盖。前后 bag 对比 Spin/BackUp/导航 `/cmd_vel` |
| L3 | adapter 发布频率 50→20 Hz | Python 节点 CPU 明显下降 | 冒烟看原地破启 |
| L4 | Nav2 组合进程 | 进程 7→1 | TODO-L4，默认 false |
| L5 | MCAP 存储 | 写盘开销 | **TODO，不装**，保持 sqlite3 |
| L6 | 地图降采样 0.10 m | 代价图格数 ÷4 | TODO-L6 |
| L7 | 显示离机 | 避免机上渲染 | **TODO，不做不装** |
| L8 | chassis_adapter C++ 组件化 | 安全链路确定性 + 省一个 Python 进程 | Phase 5，Python 版等价验证后 A/B |
| L9 | 执行器忙轮询、全局 asyncio 补丁 | —— | 执行器 spec R4/R6 |

### 7.4 去掉 C++ 里的机器人常数

| 位置 | 改法 |
|---|---|
| `scan_priority_layer.cpp:162,282` 写死 `"base_footprint"` | 改为代价层参数 `robot_base_frame`，默认取 costmap 的 `robot_base_frame` |
| `escape_obstacle_action.cpp` 默认值 | 保留为端口默认，XML 模板总是显式传入 |
| `frontier_explorer.cpp` | 进 legacy，不处理 |

### 7.5 清理

- **Phase 0 先入库**（N5）：`src/nav2_rotation_shim_controller/`、`dock_finish.py`、`dock_settle.py`、`fastdds_david.xml`、3 个测试文件、`scripts/dock_bench/`。
- 隔离到 `david_nav/legacy/`（清单 Phase 7 前逐项确认）：`cmd_vel_separator`、`map_warmup_spin`、frontier 探索全套、`collision_monitor_params.yaml`（不再被 launch 引用）、`operator_console/`、`semantic_pose_navigator.py`、停更的 5 个 shell 入口、waypoint / dead_reckoning launch。
- 位置环与 controller_bench 仍被 alias 8 使用，保留。

---

## 8. 模块③：运动控制与底盘

### 8.1 现状（实测）

**驱动**：`~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/src/turn_on_wheeltec_robot`（非 git）。在用的只有 `wheeltec_robot_node`（543 行）、`twist_transform`、`Quaternion_Solution`、launch、`keyboard_teleop_ros2.py`、`wheeltec_udev.sh`；其余为 ROS1 残留。

| 类别 | 内容 |
|---|---|
| 参数（9） | `usart_port_name`、`serial_baud_rate`、`odom_frame_id`、`robot_frame_id`、`gyro_frame_id`、`odom_x_scale`、`odom_y_scale`、`odom_z_scale_positive`、`odom_z_scale_negative` |
| 话题 | 订 `cmd_vel`；发 `odom`(50)、`imu`(20)、`PowerVoltage`(10)；不发 TF |
| 时序 | 100 Hz 读串口；断线约 1 Hz 重连；析构发一帧零速 |
| 下行协议 | 11 字节：`0x7B · 0x00 0x00 · vx · vy · wz`（int16，×1000）`· 前 9 字节异或 · 0x7D`；实测 `wz=0.55 → 7B 00 00 00 00 00 00 02 26 5F 7D` |
| 上行协议 | 24 字节：帧头、Flag_Stop、三轴速度、MPU6050 六轴、电压、第 22 字节异或、帧尾 |

**缺陷**：W1 无指令超时（固件默认也不保护）；W2 每帧 INFO；W3 odom `position.z` 写成 yaw；W4 `twist_transform` 常驻 `/cmd_vel` 发布者且本机无 `/twist_raw` 发布者；W5 `odom_combined`；W6 udev 三份无记录；W7 非 git + serial 藏在 `~/scout_ws`。

**键盘遥控**

| | Wheeltec `keyboard_teleop_ros2.py` | `david_cmd/david_keyboard.py` |
|---|---|---|
| 键位 | `u i o / j k l / m , .` | `w a s d` / 方向键 |
| 发布到 | 直接 `/cmd_vel` | `cmd_vel_safe` → david_move |
| 限幅 | 无；`q/w/e` 每次 +10% 无上限 | 0.20 / 0.50 |
| 死区抬升 | 无 | 有 |
| 松键停车 | 约 0.5 s 后平滑减速 | 0.4 s |
| 其它 | 含全向切换 `b`；脚本默认 `ROS_DOMAIN_ID=1` | `config/cmd.yaml` 默认值过时 |

### 8.2 目标结构

```
wheeltec_driver/                    # 只管"串口 ↔ ROS"，Wheeltec 控制板通用
├── include/wheeltec_driver/{protocol.hpp, byte_stream.hpp, imu_attitude.hpp}
├── src/{protocol.cpp, serial_stream.cpp, imu_attitude.cpp, wheeltec_driver_node.cpp}
├── config/wheeltec_driver.yaml     # 参数 + 单位/范围/默认/说明
├── launch/driver.launch.py
├── udev/99-wheeltec-controller.rules  +  scripts/install_udev.sh
├── test/{test_protocol.cpp, test_driver_node.cpp}   # 字节金样；假串口测超时/重连/退出零速
└── README.md
chassis_adapter/                    # 限幅、死区抬升、指令超时、退出零速；launch/chassis.launch.py = 驱动 + adapter
chassis_teleop/                     # keyboard.py；launch/keyboard_teleop.launch.py = 驱动 + adapter + 键盘
third_party/serial/                 # 原样 vendoring，保留 LICENSE 与来源 commit
third_party/wheeltec_original/      # 旧包快照，COLCON_IGNORE，只作对照与回滚
```

机器人差异全部在 `david_config/chassis/`。

### 8.3 驱动改造（每条一个提交；✓ 保持行为，⚠ 行为变更）

| # | 改动 | 类型 |
|---|---|---|
| D1 | 协议编解码抽纯函数 + 字节金样单测 | ✓ |
| D2 | 串口抽象 `ByteStream`，节点逻辑可假串口测试 | ✓ |
| D3 | 驱动层指令超时 `cmd_timeout_sec`：超时后 20 Hz 发零速帧直到新指令；先以 `0`（关闭）合入，冒烟后单独提交改 `0.5` | 先 ✓ 后 ⚠ |
| D4 | SIGINT/SIGTERM 与析构都发零速帧并记日志 | ✓ |
| D5 | 写帧日志降 DEBUG（`log_tx_frames: false`）；校验失败/重连按秒汇总 WARN | ✓ |
| D6 | odom `position.z` 置 0 | ⚠ |
| D7 | 发布 `/diagnostics`（1 Hz）：串口在线、收帧率、校验错误、重连次数、电压、Flag_Stop | ✓ |
| D8 | 保留原 9 个参数；新增 `cmd_timeout_sec`、`log_tx_frames`、`publish_imu`、`publish_voltage` | ✓ |
| D9 | 可执行名 `wheeltec_driver_node`，节点名保持 `wheeltec_robot`；detect_patterns 同时匹配新旧一个版本 | ✓ |
| D10 | `twist_transform` 不迁移（TODO-T1） | ✓ |
| D11 | `odom_frame_id` 保持 `odom_combined`（TODO-W3） | ✓ |

### 8.4 chassis_adapter

- Phase 4：由 `david_move` 通用化，参数全部来自档案，行为与 §2.4 取值等价（param dump 对比 + 单测）。
- 新增：输入话题由模式决定；SIGINT 先发零速再退出；`/diagnostics`（输入频率、是否处于超时、当前钳位值）。
- Phase 5：C++ 组件化（L8），与 Python 版 A/B。

### 8.5 键盘遥控

- 基于 `david_keyboard.py` 通用化；不采用 Wheeltec 自带脚本；apt 已装的 `teleop_twist_keyboard` 写入 README 作应急备选。
- 键位默认 `w/a/s/d` + 方向键，空格 / `k` 停车，`q` 退出；`layout: uiojkl` 兼容老习惯，不含全向与无上限加速。
- 速度取自档案，档位上限不超过 `hard_max_*`（validate 规则）。
- 链路 `keyboard → /cmd_vel_teleop → chassis_adapter → /cmd_vel → wheeltec_driver`；三道超时：松键 0.4 s、adapter 1.0 s、驱动 0.5 s（D3 生效后）。
- 终端输入：键盘节点直接打开 `/dev/tty`（参数 `tty_device`），使 `ros2 launch` 在 SSH 终端下可用；不可行则回退为包装脚本前台 `ros2 run`（TODO-W2）。
- 互斥：检查 navrun 锁与 `/cmd_vel`、`/cmd_vel_nav` 发布者，导航在跑则拒绝并说明原因。
- `ROS_DOMAIN_ID` 与 DDS 从共享环境读取。
- `keyboard_teleop.launch.py` 参数：`robot:=david`、`start_driver:=auto|on|off`、`layout:=wasd`、`linear_speed`、`angular_speed`。

---

## 9. 模块④：任务与技能

### 9.1 单一任务管理器

```
 rviz /dock_goal   ─┐
 WebSocket 适配器   ├─▶ 入口适配器（只做翻译） ─▶ TaskApi ─▶ 任务管理器 ─▶ 技能
 CLI / 测试         ┘                                      │  同一时刻只有一个活动任务
                                                           │  统一抢占、取消、超时、事件
                                                           └─▶ /nav/events
```

- **进程**：一个 Python 进程 `nav_task_manager`（asyncio + rclpy，内核设计见执行器 spec §4），**WS 适配器作为可选组件在同一进程内加载**（按模式开关），避免多一个进程与跨进程通信。
- **公共 API**（进程内）：`submit(task) / cancel(task_id | current) / status() / subscribe_events()`。跨进程接口预留 `david_nav_msgs/action/RunTask`（本轮不实现）。
- **抢占策略**：按入口配置 `preempt: allow | reject`；默认 WS 允许抢占（与现行 v1 行为一致），rviz `/dock_goal` 在忙时拒绝（与现 dock_navigator 一致）。
- **与遥控的互斥**：进程级由 navrun 锁保证；任务级由任务管理器保证。
- **定位健康联动**：收到 `localization_health` 的 stop 请求时取消当前任务（§6.3）。

### 9.2 技能

| 技能 | 来源 | 行为（保持现状） |
|---|---|---|
| `NavigateToPoi` | 执行器 core | 查 POI → snap（0.5→3.0 m，步长 0.25，到达扩圈点即成功）→ Nav2 → 单次失败 1 s 后重发 1 次 → 耗尽报错 |
| `Dock` | `dock_navigator` + `dock_finish` + `dock_settle` | 预备点后退 0.5 m → 阶段 1 → 阶段 2 → 等停稳 → 判定（xy 0.10 m、yaw 25°）→ 补偿 0 次；**`[dock]` 日志行逐字保持**（dock_bench 正则依赖） |
| [预留] `MotionPrimitive` | —— | 前进/后退/旋转，经 `/cmd_vel_primitive` → 仲裁 → 安全闸 → adapter |

所有技能共用唯一的 Nav2 客户端（`nav_tasks/nav2_client.py`，含"goal 已发出但 handle 未回时被取消"的补刀逻辑与"禁用 `spin_until_future_complete`"的约束）。

### 9.3 模式与入口

| 模式 | 入口适配器 | 技能 |
|---|---|---|
| tour（alias 4） | WebSocket | NavigateToPoi |
| precise（alias 0） | rviz `/dock_goal` | Dock；WS 适配器预留（配置开启即可） |
| bench_slamgt | 无（bench 直连 Nav2，迁移前豁免） | —— |

---

## 10. 模块⑤：反应与安全

### 10.1 行为策略总表（`docs/behavior_policy.md`，唯一真源）

| 触发条件 | 响应（本轮） | 负责方 | 参数键 |
|---|---|---|---|
| POI 在障碍或代价区 | snap 扩圈 | 任务管理器 | `tasks.goal_snap.*` |
| Nav2 单次失败 | 1 s 后重发 1 次 | 任务管理器 | `tasks.nav_retry.*` |
| 重试耗尽 | 上报失败 | 任务管理器 | —— |
| 足迹嵌入致命障碍（连续 3 次确认） | **现有 BT 脱困流程不变** + **触发上报**（§10.3）；新策略 TODO-E1 | BT + 运行记录器 | `reactive.escape.*` |
| 无进展（IsStuck） | 现有恢复轮换不变 + 触发上报 | BT + 运行记录器 | `reactive.recovery.*` |
| 入位 | 两段式 + 停稳 + 判定 | Dock 技能 | `tasks.dock.*` |
| abort | 取消，立即回 paused | 任务管理器 | —— |
| 遥控松键 | 零速 | chassis_teleop | `teleop.key_timeout_sec` |
| adapter 输入静默 | 零速 | chassis_adapter | `chassis.adapter.cmd_timeout_sec` |
| 驱动输入静默 | 零速帧（D3 生效后） | wheeltec_driver | `chassis.driver.cmd_timeout_sec` |
| 底盘驱动死亡 | 重启 1 次，再死全栈退出 | launch | `chassis.max_restarts` |
| Nav2 核心进程死亡 | 全栈退出 | launch | —— |
| 定位 TF 过期 / 位姿发散 | warn（本轮）→ stop（TODO-H1） | 定位健康守护 | `localization_health.*` |
| [预留] 前方动态障碍 | 绕行 或 停车上报 | 障碍策略 / 安全闸 | `reactive.obstacle.policy: none` |

每一行在 Phase 6 前都要有自动化测试或不变量检查。

### 10.2 行为树模板化

- BT XML 变模板，frame、速度、距离从档案渲染，结果存进运行记录。
- 本轮只做**等值渲染**：David 档案渲染出的 XML 与现有 XML 逐节点逐属性相等（golden）。
- `navcfg validate` 对 `kinematics.type: diff_drive` 下 BT 中任何非零横向速度参数给出 **WARN**（不阻止启动），提醒 TODO-E1。

### 10.3 脱困触发可见（本轮实现）

现状：`RobotEmbeddedInObstacle` 与 `EscapeObstacle` 在触发时已打 `WARN_THROTTLE`（`robot_embedded_in_obstacle_condition.cpp:147-185`，`escape_obstacle_action.cpp:304-392`），但 alias 4 下 bringup 输出只进 `bringup.log`，终端看不到；alias 0 下混在大量 launch 输出里。

本轮做法（**不改 C++、不改 BT 行为**）：
1. 运行记录器订阅 Nav2 自带的 `/behavior_tree_log`，当以下节点状态跳变时生成事件：`RobotEmbeddedInObstacle`（进入"嵌入"判定）、`EmbeddedRecoveryLevel1/2`、`EscapeObstacle`、`BackUp`、`Spin`、`IsStuck` 分支、`NavigateRecovery` 重试计数变化。
2. 事件写入 `events.jsonl`，同时由 navrun 在**前台终端**打印一行醒目提示，例如：
   `[06:12:03] ⚠ 脱困触发：RobotEmbeddedInObstacle 确认嵌入 → EmbeddedRecoveryLevel1（task=req_42，位姿 x=14.20 y=22.31 yaw=-0.95）`
3. 同一行写入 `logs/navrun.log`；运行结束的 `summary.md` 统计触发次数与位置。
4. 如 `/behavior_tree_log` 信息不足以得到位姿，从 TF 取当前 `map→base_footprint` 补全。

### 10.4 预留：障碍策略

- 档案键 `reactive.obstacle.policy: none | bypass | stop_and_report`（本轮只接受 `none`）。
- 事件类型 `obstacle_blocked` / `obstacle_cleared`；任务管理器将来把它翻译成 WS 状态。
- 安全闸位置固定在仲裁之后、adapter 之前（§5.2），使将来的运动原语也受避障约束。

---

## 11. 模块⑥：对外接口

- **WebSocket 适配器**（`nav_executor`）：只做 nav WS v1 协议 ↔ `TaskApi` 的翻译；协议、断联重发、心跳、畸形帧处理见执行器 spec（R1 畸形帧不拆连接等缺陷在此修）。
- 线协议逐字节兼容；进程名 `navigation_executor.py` 在 Phase 4 切 alias 前保持。
- **预留**：alias 0 挂 WS = 模式叠加开启适配器；运动原语 = 新增命令处理器提交 `MotionPrimitive` 任务；协议握手 TODO-P1。

---

## 12. 横切：配置层

### 12.1 schema 归属

每个包在自己的 `config/schema.yaml` 里声明它消费的参数（类型、单位、范围、默认、说明、调参经验链接）。`navcfg` 汇总所有 schema 生成 `docs/parameters.md` 并做校验；**没有单独的"参数模块"**。

### 12.2 机器人档案（David，取值 = 当前线上值）

```yaml
schema_version: 1
robot: {name: david, description: 70kg 差速底盘 + 从动脚轮，双臂人形}
kinematics: {type: diff_drive}

frames:
  map: map
  odom: odom
  # base_link 与 base_footprint 是不同的帧（URDF：base_footprint 在 base_link 前 0.165 m、下 0.4075 m），禁止合并
  nav_base: base_footprint         # Nav2 / 入位 / 代价图 / BT
  body: base_link                  # 机身本体帧（lightning 发布）
  slam_base: base                  # SLAM 内部帧，导航禁用
  poi_capture: base_link           # 采点帧；≠ nav_base 时 validate WARN
  scan_target: base_footprint

footprint: [[0.255, 0.30], [0.255, -0.30], [-0.465, -0.30], [-0.465, 0.30]]

limits:
  max_vel_x: 0.15
  max_vel_theta: 0.50
  max_reverse_vel: 0.20            # TODO-E1 新脱困策略使用

chassis:
  adapter:
    hard_max_linear_x: 0.20
    hard_max_angular_z: 0.60
    min_linear_x: 0.0
    min_stationary_angular_z: 0.55
    stationary_angular_threshold: 0.025
    cmd_timeout_sec: 2.0           # L2 后 1.0
    publish_hz: 50                 # L3 后 20
  driver:
    package: wheeltec_driver
    parameters:
      usart_port_name: /dev/wheeltec_controller
      serial_baud_rate: 115200
      odom_frame_id: odom_combined
      robot_frame_id: base_footprint
      gyro_frame_id: gyro_link
      odom_x_scale: 1.0
      odom_y_scale: 1.0
      odom_z_scale_positive: 1.0
      odom_z_scale_negative: 1.0
      cmd_timeout_sec: 0.0         # D3：冒烟后 0.5
      log_tx_frames: false
    detect_patterns: [wheeltec_robot_node, wheeltec_driver_node, twist_transform, launch turn_on_wheeltec_robot]
    startup_delay_sec: 3.0
    max_restarts: 1
  odom_topic: /odom
  cmd_topic: /cmd_vel

teleop:
  layout: wasd
  linear_speed: 0.20
  angular_speed: 0.35
  key_timeout_sec: 0.40

localization:
  provider: lightning
  odom_topic: /lightning/loc/odometry   # alias 3 实际发布（r5）；Nav2 接线见下方 nav2.odom_topic
  aligned_cloud_topic: /lightning/loc/aligned_scan
nav2:
  odom_topic: /lightning/odometry        # Nav2 现接线（alias 3 不发布，无数据），等值渲染保留；TODO-O1
localization_health:
  max_tf_age_sec: 0.5
  max_divergence_m: 1.0
  divergence_window_sec: 2.0
  action: warn

sensors:
  scan_from_cloud: {enabled: false, cloud_topic: /lightning/loc/aligned_scan, scan_topic: /scan,
                    min_height: 0.10, max_height: 1.0, angle_min: -1.5708, angle_max: 1.5708,
                    range_min: 0.3, range_max: 12.0}

platform: {jetson: true, maxn: true, jetson_clocks: true}
preflight: {kill_patterns: [...], kill_foreign_stacks: false, kill_stale_robot_state_publishers: true}
record: {light_topics: [...], storage: sqlite3}
```

模式叠加（`modes/precise.yaml` 等）只写差异；场地档案 `sites/daolan/site.yaml` 写地图、点位文件、server 地址。

### 12.3 绑定表（节选）

```yaml
limits.max_vel_x:
  - controller_server.ros__parameters.FollowPath.max_vel_x
  - controller_server.ros__parameters.FollowPath.max_speed_xy
frames.nav_base:                    # 绑定的是 base_footprint
  - bt_navigator.ros__parameters.robot_base_frame
  - local_costmap.local_costmap.ros__parameters.robot_base_frame
  - global_costmap.global_costmap.ros__parameters.robot_base_frame
  - behavior_server.ros__parameters.robot_base_frame
  - local_costmap.local_costmap.ros__parameters.scan_priority_layer.robot_base_frame
footprint:
  - local_costmap.local_costmap.ros__parameters.footprint
  - global_costmap.global_costmap.ros__parameters.footprint
nav2.odom_topic:                    # 不是 localization.odom_topic（TODO-O1）
  - bt_navigator.ros__parameters.odom_topic
  - controller_server.ros__parameters.odom_topic
```

### 12.4 校验规则（失败即拒绝启动，WARN 除外）

| 规则 | 来源 |
|---|---|
| `behavior_server.global_frame == local_costmap.global_frame` | 恢复行为全废事故 |
| `limits.max_vel_x ≤ hard_max_linear_x`；`max_vel_theta ≤ hard_max_angular_z` | 限速不一致 |
| `min_stationary_angular_z ≤ hard_max_angular_z` | 07-27 地板被钳回 |
| 精确模式两处 `xy_goal_tolerance` 相等；`yaw_goal_tolerance < 2π` ⇒ critics 含 `RotateToGoal` | 08-31 精确版坑 |
| 地图 yaml `mode == trinary` | scale 模式陷阱 |
| 遥控档位上限 ≤ adapter 硬限 | 键盘无上限 |
| `diff_drive` 下 BT 横向速度非零 → **WARN** | TODO-E1 |
| `poi_capture != nav_base` → **WARN** 并打印偏移 | §2.3 |

`navcfg check --live`：等 `map→odom` 出现后，检查关键话题发布者、TF 延迟、各节点实际参数与渲染值一致。

### 12.5 调参回流

bench（alias 7/8/10）调出的参数必须以"模式叠加文件的提交"回到档案；**禁止再生成快照 yaml**。`navrun show <run>` 能导出该 run 的生效参数与叠加差异，便于提交。

### 12.6 换机器人步骤（`docs/porting.md`）

1. 复制 `david_config` → `<robot>_config`，改档案、URDF、底盘参数、场地。
2. `navcfg validate --robot <robot> --mode tour` 通过。
3. 起定位（完成定位）与底盘，`navcfg check --live` 通过。
4. `navrun selftest --robot <robot> --mode tour`（离线闭环仿真）通过。
5. 用户实机冒烟。
6. 验收：`git diff --stat` 只含 `<robot>_config/`。Phase 3 用虚构的 `example_diffdrive_config` 自动化验证。

---

## 13. 横切：可观测与运维

### 13.1 navrun

```
navrun up     --mode tour|precise|chassis|teleop [--robot david] [--site daolan]
              [--record light|full|off] [--no-executor] [--chassis auto|on|off] [--dry-run]
navrun down | status | ls | show <run_id|latest> [--params|--diff|--events]
navrun check [--live] | doctor | selftest --mode ... | gc
```

`up` 顺序：加锁 → 建运行记录 → 写清单 → 渲染 → 校验 → platform → preflight → DDS 卫生 → launch → 等 Nav2 就绪 → 等手动定位（只提示不失败）→ `check --live` → 开始录制 → 前台打印关键事件 → 收尾（零速、停录制、摘要、释放锁）。

实现约束：Python 3.10、标准库 + PyYAML + psutil；每步一个可单测函数；`--dry-run` 不启动任何东西。

### 13.2 运行记录

```
~/nav_runs/
├── latest -> 20260914_101500_tour_977e29f
└── 20260914_101500_tour_977e29f/
    ├── manifest.yaml     # run_id、alias、mode/robot/site、主机、起止时间、exit_reason、git(分支/commit/dirty/diff统计)、
    │                     # 各包版本、§2.5 版本表、ROS_DOMAIN_ID/RMW/DDS profile、launch 参数、地图与点位文件 sha256、server 地址
    ├── code.diff
    ├── params/{inputs, rendered, effective}   # 输入档案 / 渲染结果 / 激活后 param dump
    ├── logs/{navrun.log, launch.log, executor.log, chassis.log, ros/, loc/}
    ├── events.jsonl      # 任务、脱困与恢复触发、入位判定、定位健康、进程启停、WS 连接、校验告警
    ├── metrics/{resources.csv (1 Hz 每进程 CPU/RSS/线程), topics.csv (0.2 Hz 关键话题频率与 TF 延迟)}
    ├── checks.yaml
    ├── bag/              # sqlite3
    └── summary.md        # 时长、任务成败与原因、脱困/恢复触发次数与位置、入位结果、重启与退出原因、TF 延迟分位数、CPU/内存、磁盘
```

- 与 alias 3 关联（只读）：记录 `run_loc_online` 的 pid、启动时间、命令行、配置 sha256，收尾时拷贝本次时间窗内 alias 3 日志目录的新增内容到 `logs/loc/`。
- 遥控模式（5k）同样生成运行记录。
- 保留策略（`gc`，每次 `up` 前）：最近 30 个 run 或 20 GB；超过 7 天删 `bag/`，清单与摘要永久保留。

### 13.3 doctor（已知坑的自动检查）

| 检查 | 对应历史问题 |
|---|---|
| ros2 CLI 守护进程僵死（`--no-daemon` 对照） | bt_navigator 等待 120 s |
| 当前 shell 未加载 DDS 白名单环境 | SSH 查 ROS 图"半瞎" |
| domain 0 出现外来 GID 前缀的节点 | 实验室另一台机器人串入 |
| 地图 yaml 为 `mode: scale` | 未知区被判成障碍 |
| 多个 robot_state_publisher / TF 裂树 | 残留 URDF |
| `/cmd_vel` 多发布者 | 遥控与导航抢底盘 |
| install 目录可执行脚本是陈旧拷贝 | 改了代码不生效 |
| 孤儿 Fast DDS 共享内存段过多 | 同机发现失效 |
| 串口 `/dev/wheeltec_controller` 缺失或权限不足 | 底盘起不来 |
| `map→odom` 不存在 | 尚未手动定位（提示而非报错） |

### 13.4 离线闭环仿真（`nav_sim`）

- `fake_chassis`：由 `scripts/sim_fake_chassis.py` 通用化（输入话题可配，默认 `/cmd_vel`），积分发 `/odom`。
- `fake_localization`：以 odom 积分发 `map→odom`（恒等）与 `odom→base_link`；`base_link→base_footprint` 由 URDF 的 robot_state_publisher 提供。
- `fake_agent_server`：发 `navigate_to` / `abort`，收事件。
- `navrun selftest --mode tour|precise`：`ROS_DOMAIN_ID=99`、真 Nav2 + 假底盘 + 假定位，**跑完一次 POI 导航与一次两段式入位**并断言事件序列；是 Codex 每个提交的门禁。

### 13.5 共享环境包

`ros_env.conf`、`fastdds_david.xml`、`load_ros_env.sh` 迁入 `david_system_env`；旧路径 `david_nav/config/` 保留软链一个版本；变更前在 Phase 报告中列出引用方（systemd 单元、脚本、其他组）。

### 13.6 `.bashrc`（Phase 4）

```bash
# 备份：cp ~/.bashrc ~/.bashrc.bak_before_navrun_$(date +%Y%m%d)
alias 0="export ROS_DOMAIN_ID=0; ~/ros2_ws/src/nav_bringup/scripts/navrun up --mode precise"
alias 4="export ROS_DOMAIN_ID=0; ~/ros2_ws/src/nav_bringup/scripts/navrun up --mode tour"
alias 5="export ROS_DOMAIN_ID=0; ~/ros2_ws/src/nav_bringup/scripts/navrun up --mode chassis"
alias 5k="export ROS_DOMAIN_ID=0; ~/ros2_ws/src/nav_bringup/scripts/navrun up --mode teleop"
# 回滚通道，保留一个版本周期：
alias 0old="export ROS_DOMAIN_ID=0;source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash;ros2 launch david_nav precise_nav_bringup.launch.py"
alias 4old="export ROS_DOMAIN_ID=0;/home/agx/ros2_ws/src/david_nav/start_navigation_headless.sh"
alias 5old="export ROS_DOMAIN_ID=0;cd ~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2;source ./install/setup.bash;ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py;"
```

`scripts/navrun` 是稳定路径的薄壳（负责 source ROS 与共享环境），以后不再需要改 `.bashrc`。

---

## 14. 数据域：地图与点位（`nav_site_data`）

- 场地目录：`david_config/sites/<site>/{site.yaml, maps/, pois/}`；`site.yaml` 写地图 yaml 路径、点位文件、agent server 地址。
- 代码：地图 yaml 加载与 `trinary` 校验；点位文件加载（兼容现有 `named_poses.json` 与 `poi_store.yaml` 格式，**格式不变**）；计算 sha256 写入运行记录；POI 仓库（原执行器 `poi_store.py`）。
- `named_poses.json` 为手动维护的文件，本轮只记录 hash，不做 frame 语义处理。

---

## 15. 轻量化度量口径

| 指标 | 采集 | 口径 |
|---|---|---|
| 导航侧进程数 | resources.csv | navrun 拉起的进程（不含 alias 3） |
| CPU（空闲 / 导航中） | 1 Hz 平均与 P95 | 12 核总量百分比 |
| 常驻内存 | RSS 求和 | MB |
| DDS 参与者数 | `ros2 node list --no-daemon` 按进程去重 | 个 |
| 指令延迟 | `/cmd_vel_nav` → `/cmd_vel`（bag 离线） | ms P50/P95 |
| TF 延迟 | `map→base_footprint` 年龄 | ms |
| 写盘速率 | 运行记录目录增长 | MB/h |

目标值在 Phase 0 基线后由用户确认。

---

## 16. 实施路线

### 16.1 门禁（每个提交）

| 层级 | 谁 | 内容 |
|---|---|---|
| 离线 | Codex | `colcon build --symlink-install` + `colcon test`；渲染 golden；import 规则单测；`navrun up --dry-run` 符合期望 |
| 无运动闭环 | Codex | `navrun selftest --mode precise` 与 `--mode tour`（§13.4，Phase 3 起）；Phase 3 前用"假 TF + 假 action server"的无运动启动检查 |
| 实机冒烟 | **用户** | §16.3；每个 Phase 结束一次，Phase 4 之后每个涉及运行时的提交一次 |

### 16.2 阶段

| Phase | 内容 | 行为变化 | 入口 |
|---|---|---|---|
| **0 基线** | ① 用户提交未提交修改；入库 N5 等未跟踪生产依赖 ② tag `pre-nav-system-refactor` ③ 只读资源采样，测 3+0、3+4 基线 ④ golden：两种模式所有节点 param dump、BT XML、进程列表 ⑤ wheeltec 旧包快照 + TODO-W1 抓上行帧 ⑥ TODO-V3 只读检查 | 无 | 不变 |
| **1 可追溯** | `nav_bringup` 骨架 + 运行记录器 + `navrun wrap -- <旧命令>` 包裹模式 + **脱困触发终端可见（§10.3）** + doctor 首版；alias 0/4 改为包裹旧入口 | 无（旁路观察） | 包裹旧入口 |
| **2 配置层** | 各包 schema、`david_config`、绑定表、`navcfg render/validate/doc`；渲染结果与 Phase 0 param dump 逐键相等；旧 launch 改读渲染文件；派生 yaml 等值证明后删除（bench 那份保留到调参回流落地） | 无 | 包裹旧入口 |
| **3 去写死 + 仿真 + 底盘抽取前半** | C++ frame 参数化；BT 等值渲染；点云预处理迁入模块①；`nav_sim` + `navrun selftest`；`example_diffdrive_config` 迁移测试；wheeltec W1（serial vendoring + 协议层）、W2（驱动节点 D1/D2/D4/D5/D7–D9，D3=0） | 无 | 包裹旧入口 |
| **4 统一入口 + 底盘切换** | `nav.launch.py` + navrun 完成 §7.2 全部对等；chassis_adapter（Python 等价版）；chassis_teleop + `keyboard_teleop.launch.py`；定位契约检查与"等待手动定位"提示；`.bashrc` 按 §13.6 修改 | 无（功能对等） | **切到 navrun** |
| **5 轻量化** | L1 → L3 → L2 → L8(adapter C++) → L4 → L6，一项一提交，附前后运行记录对比；无收益或冒烟退化即回退 | 逐项评估 | navrun |
| **6 任务管理器与对外接口** | `nav_tasks`（单一任务管理器 + NavigateToPoi + Dock 技能，替代 dock_navigator）；`nav_executor` 瘦身为 WS 适配器（执行器 spec 缺陷修复）；`/nav/events`；定位健康守护（warn）；驱动 D3 改 0.5、D6 | 有（缺陷修复、驱动超时） | navrun |
| **7 清理与文档** | legacy 隔离；`david_system_env`；`docs/{architecture, parameters, porting, behavior_policy, events, runbook}.md`；修 CLAUDE.md/README 漂移；删除 `0old/4old/5old`（用户确认后） | 无 | navrun |

### 16.3 实机冒烟清单（用户执行）

**3+0**
1. 起 alias 3，RViz 里手动 2D Pose Estimate，确认定位成功。
2. 起 alias 0：终端显示 run_id、"nav ready"；若定位未完成应看到等待提示。
3. RViz 用 2D Goal Pose（`/dock_goal`）给近距离空旷目标，完成两段式入位，终端打印判定。
4. Ctrl-C：车停、打印摘要路径；`navrun show latest` 可见目标与判定。

**3+4**
1. 起 alias 3 并手动定位。
2. 起 alias 4，执行器连上 server。
3. server 发近距离 `navigate_to`，收到 `on_arrived`。
4. 导航中 `abort_navigation`，≤1 s 车停并收到 paused。
5. Ctrl-C，`navrun show latest` 可见任务结果、资源曲线、生效参数。

**底盘与遥控（Phase 4 起）**
1. alias 5k：`w` 前进约 1 s 松键 0.4 s 内停；`a` 原地左转能起步；空格立即停；Ctrl-C 后车停且 `/cmd_vel` 无残留发布者。
2. alias 4 运行中再起 5k → 拒绝并说明原因。
3. （Phase 6 D3 后，空旷场地、低速）按住前进时 `kill -9` 键盘与 adapter → 车 ≤0.5 s 停。

**脱困可见（Phase 1 起，遇到时观察即可，不刻意制造）**
- 发生嵌入或卡住时，终端出现"⚠ 脱困触发 …"行，`events.jsonl` 与 `summary.md` 有记录。

### 16.4 回滚

- 任一提交 `git revert`。
- Phase 4 之后：`0old / 4old / 5old`，或恢复 `.bashrc` 备份。
- 整仓：`git checkout pre-nav-system-refactor -- .` 后重新 `colcon build --symlink-install`。
- 旧文件（headless 脚本、precise launch、三份 yaml、wheeltec 旧工作空间）在 Phase 7 用户确认前不删除。

---

## 17. 风险

| 风险 | 缓解 |
|---|---|
| 渲染参数与现状不一致导致行为漂移 | param dump 级逐键 golden；Phase 2 不改任何值 |
| launch 事件处理与 bash 看护语义不完全对等 | §7.2 对等表逐项测试；保留 `4old` |
| 任务管理器替换 dock_navigator 后 dock_bench 解析失败 | `[dock]` 日志行冻结 + 事件双写；bench 冒烟 |
| `/dev/tty` 读键在 launch 下不可用 | TODO-W2 回退方案 |
| 驱动超时参数改 0.5 后在高负载下误触发 | 先以 0 合入；`/diagnostics` 观察输入间隔分布再定值 |
| 共享环境包迁移影响其他组 | 旧路径软链一个版本；变更前列引用方 |
| 定位健康阈值误报 | 本轮只 warn，积累运行记录后再切 stop |

---

## 附录 A：旧文件 → 新归属

| 旧 | 新 |
|---|---|
| `start_navigation_headless.sh` | `nav_bringup`（navrun）+ `nav.launch.py` |
| `launch/precise_nav_bringup.launch.py`、`2d_david_nav2_bringup.launch.py` | `nav_bringup/launch/nav.launch.py` |
| `config/nav2_params_2d*.yaml`、`dwb_bench_params.yaml` | `nav_bringup/config/nav2_base.yaml` + `david_config` 档案与叠加 + 绑定表 |
| `behavior_trees/*.xml` | `nav_plugins/behavior_trees/*.xml.tmpl` |
| `src/scan_priority_layer.cpp` 等 C++ 插件 | `nav_plugins` |
| `david_nav/navigation_executor*.py`、`navigation_protocol.py`、`nav2_action_bridge.py`、`navigation_goal_resolver.py`、`pose_provider.py` | `nav_tasks`（内核/技能/Nav2 客户端）+ `nav_executor`（协议/传输）；细节见执行器 spec 附录 A |
| `poi_store.py`、`grid_map_view.py`、`navigation_types.py` | `nav_site_data` / `nav_tasks` domain |
| `dock_navigator.py`、`dock_finish.py`、`dock_settle.py` | `nav_tasks` Dock 技能 |
| `david_cmd/david_move.py` | `chassis_adapter` |
| `david_cmd/david_keyboard.py` | `chassis_teleop` |
| `turn_on_wheeltec_robot`（wheeltec 工作空间） | `wheeltec_driver` + `third_party/serial` |
| `scripts/sim_fake_chassis.py` | `nav_sim/fake_chassis` |
| `config/ros_env.conf`、`fastdds_david.xml`、`load_ros_env.sh` | `david_system_env` |
| `urdf/david.urdf`、`config/maps/*` | `david_config` |

## 附录 B：参考资料

- Clearpath robot.yaml — https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview
- PickNik generate_parameter_library — https://github.com/PickNikRobotics/generate_parameter_library
- Autoware vehicle_info_utils — https://autowarefoundation.github.io/autoware_universe/main/vehicle/autoware_vehicle_info_utils/
- Open-RMF fleet adapter template — https://github.com/open-rmf/fleet_adapter_template
- Nav2 Docking Server — https://docs.nav2.org/tutorials/docs/using_docking.html
- Nav2 Simple Commander — https://docs.nav2.org/commander_api/index.html
- Nav2 Collision Monitor — https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html
- Nav2 bringup（`use_composition`，本机 `navigation_launch.py:95` 实测存在）— https://github.com/ros-navigation/navigation2/tree/humble/nav2_bringup
- Martin Fowler, Strangler Fig Application — https://martinfowler.com/bliki/StranglerFigApplication.html
- 其余（Google Python Style、REP-2004、AsyncAPI、asyncio 任务引用注记）见执行器 spec 附录 B
