# Spec：David 导航任务执行层重构 —— 抽出可复用的 `nav_executor` 模块

**日期**：2026-09-14
**目标仓库**：`embodied_navigation`（David AGX `172.16.22.51`，`~/ros2_ws/src`，分支 `feature/dwb-control-opt`）
**基线**：`977e29f` + 工作区 9 个未提交修改 + 若干未跟踪文件（见 §6 P0）
**执行方**：机器人上的 Codex（本文是给 Codex 的执行规格，每一步都写到可直接落地）
**r4 修订（同日，优先于下文）**：包归属改为——本文 §4 的 core（`core/` 状态机、命令注册表、处理器、重试、TaskSupervisor、goal_resolution、Nav2 客户端）归入 **`nav_tasks`（单一任务管理器）**，并吸收 dock_navigator/dock_finish/dock_settle 作为 Dock 技能；**`nav_executor` 只保留 protocol + transport + WS 适配器**，作为可选组件与任务管理器同进程加载；POI 存储与地图工具归入 `nav_site_data`。分层规则与 import 约束以总方案 §5 为准；import-linter 本轮不安装（TODO-I1），用单测 AST 扫描代替。
**r2 修订（同日）**：§10 的 D1–D6 已按建议落定；不再要求 Python 3.8 兼容（以当前机上环境为准，GO2W 复用为 TODO-V1）。决策、实测取值与 TODO 总表以 `2026-09-14-david-nav-system-refactor-plan.md` §2 为准；"单一任务管理器"已由用户确认采纳（见上方 r4 修订与总方案 §9）。
**硬约束**：Codex **不下发任何运动指令**。需要车动的验证写成步骤交给用户执行。

---

## 0. 名词与流程（先读这一节）

| 名词 | 在本文里的意思 |
|---|---|
| **executor（执行器）** | `navigation_executor.py` 这个进程：作为 WebSocket **客户端**连到 agent server，收 `navigate_to` 等命令，驱动 Nav2，回报 `on_progress/on_arrived/on_error` |
| **agent server** | 机器人外部的"大脑"服务，监听 `ws://<ip>:8101`，给执行器下命令 |
| **nav WS v1 / v1.1** | 执行器与 agent server 之间的 JSON 线协议。v1 = 只认 POI id（`target_id`）；v1.1 = 导航中随时可 `abort_navigation` |
| **POI** | 语义点位（Point of Interest），`id + name + map 位姿 + odom 位姿` |
| **snap（扩圈）** | POI 落在障碍/代价区时，从 0.5 m 起按 0.25 m 逐圈外扩，找最近的可达点当实际导航目标 |
| **core** | 执行器里的纯逻辑状态机（`navigation_executor_core.py`），不碰网络、不碰 ROS |
| **bridge / backend** | 把"去某个位姿"翻译成 Nav2 `NavigateToPose` action 调用的那一层 |
| **端口（port）** | core 依赖的**抽象接口**（Python `typing.Protocol`）。例："能导航到位姿的东西" |
| **适配器（adapter）** | 端口的**具体实现**。例：Nav2 适配器、假导航适配器（测试用） |
| **组装根（composition root）** | 唯一一个把所有具体实现 new 出来并连线的地方（`app/runtime.py`） |
| **shim（垫片）** | 留在旧路径上的几行代码，只负责转调新模块，保证旧命令/旧 import 不坏 |
| **golden 测试（金样测试）** | 重构前先把"输入 → 输出帧"录成固定文件，重构后逐字节比对，证明行为没变 |
| **ADR** | Architecture Decision Record，一页纸记录"为什么这样设计"。用来安放现在散落在代码注释里的事故复盘 |

**执行流程（现状）**：

```
agent server ──WS JSON──▶ navigation_executor.py（收包循环）
                              │ handle_message
                              ▼
                     NavigationExecutorCore（状态机）
                     ├─ LocalPoiStore（查 POI）
                     ├─ NavigationGoalResolver（snap，订阅 grid_map + global costmap）
                     └─ Nav2ActionBridge ──NavigateToPose──▶ bt_navigator / waypoint_follower
                              ▲ event_callback(progress/arrived/error)
                              └──────────────── 回帧入可靠发送队列 ──▶ agent server
```

---

## 1. 目标与非目标

### 1.1 目标

1. **模块化**：把执行器相关的"协议 + 状态机 + ROS 适配 + 传输 + 存储"抽成一个**不带机器人名字**的独立 ament_python 包 `nav_executor`，与 `david_nav` 同级。David 专属的东西（参数、launch、headless 脚本、C++ 插件）留在 `david_nav`。
2. **可复用**：新机器人 / waypoint 栈（G1、GO2W 复用为 TODO-V1） / 精确入位等"其他导航任务"只需 **配置 + 组装**即可复用，不复制代码。
3. **可读性**：全部公共 API 采用 Google 风格 docstring + 类型注解；代码注释只写"为什么"，事故复盘挪到 ADR。
4. **鲁棒性**：修复本次审查发现的 10 个缺陷（§2.3），每个缺陷**先写失败测试再修**，与"搬家"提交严格分开。
5. **行为零变化**：线协议逐字节兼容、CLI 兼容、进程名兼容、关键日志行兼容、POI 存储文件兼容（§7 不变量清单）。

### 1.2 非目标（本轮明确不做）

- 不改 Nav2 参数、行为树 XML、C++ 插件（`scan_priority_layer` / frontier / escape）。
- 不改协议语义（不新增/删除字段，不改 v1 规则）。
- 不改 cmd_vel 链路（`david_move` / separator / mux / wheeltec）。
- 不重写 `start_navigation_headless.sh`（1221 行），只改它调用执行器的那一处（且保持兼容）。
- 不把精确入位（dock）接进 WS 协议——只**预留扩展点**并演示怎么接（§4.4）。
- 不批量重排旧文件格式（避免 `git blame` 噪声）。新规范只强制作用于新包与被移动的代码。

---

## 2. 现状诊断

### 2.1 盘点：`david_nav/david_nav/` 是 25 个 Python 文件平铺，混了 6 个子系统

| 子系统 | 文件（行数） | 与执行器的关系 |
|---|---|---|
| **任务执行器（本次主角）** | `navigation_executor.py`(575)、`navigation_executor_core.py`(317)、`navigation_protocol.py`(180)、`navigation_types.py`(46)、`navigation_goal_resolver.py`(191)、`nav2_action_bridge.py`(257)、`poi_store.py`(177)、`pose_provider.py`(111)、`grid_map_view.py` | 全部 |
| 精确入位 | `dock_navigator.py`(316)、`dock_finish.py`(120)、`dock_settle.py`(124) | 同类"任务编排"，应共享底座 |
| 位置环 / waypoint | `position_controller.py`、`position_control_core.py`、`waypoint_follower.py`、`waypoint_sparsifier.py`、`dead_reckoning_executor.py` | 是执行器可驱动的**另一种 backend**（`--action-name /waypoint_follower/navigate`） |
| 速度链路 | `cmd_vel_separator.py`、`cmd_vel_mux.py` | 无关 |
| 部署配置 | `headless_config.py`（被 `scripts/navigation_headless_config.py` 调用） | 部署层 |
| 其它/遗留 | `semantic_pose_navigator.py`、`map_warmup_spin.py`、`monitor_nav2_frontier_cpu.py`、`odometry_tf_broadcaster.py` | 遗留工具 |

健康底子不错：**90 个 Python 单测 0.25 s 全过**，flake8 仅 14 条告警。core 已经做了依赖注入（测试里有 `FakeBridge` / `FakeGoalResolver`），这是本次重构能低风险推进的前提。

### 2.2 四个维度的问题（带证据）

#### 模块化

| # | 问题 | 证据 |
|---|---|---|
| M1 | 执行器与其它 5 个子系统混在同一个扁平目录、同一个 ament_cmake 包里，无法单独复用或单独测试 | §2.1 |
| M2 | 依赖方向反了：`navigation_goal_resolver` 从 **core** 里 import 数据类型，形成"适配层 ⇄ core"双向耦合 | `navigation_goal_resolver.py:6` |
| M3 | `NavigateToPose` action 客户端写了 **5 份**，各自处理超时/取消/future 等待 | `nav2_action_bridge.py:62`、`dock_navigator.py:92`、`semantic_pose_navigator.py:393`、`scripts/dock_bench/run_dock_bench.py:62`、`scripts/wait_nav_ready.py:44` |
| M4 | "四元数→yaw / 角度归一化"写了 **22 份** | `grep -rn "def .*yaw_from\|def .*wrap"`：`pose_provider.py:8`、`dock_navigator.py:44,49`、`waypoint_follower.py:49`、`position_controller.py:46` …… |
| M5 | `package.xml` 仍依赖 GO2 时代的 `unitree_go / unitree_api / go2_sport / custom_interface` | `package.xml:35-39` |
| M6 | Python 可执行文件用 `install(PROGRAMS)` **拷贝**安装：`install/.../navigation_executor.py` 日期 07-29，与 src 不一致；`ros2 run david_nav navigation_executor.py` 跑的是旧代码。headless 脚本碰巧优先用 src 路径才没暴露 | `CMakeLists.txt:183-193`；`start_navigation_headless.sh:985-988` |

#### 可读性

| # | 问题 | 证据 |
|---|---|---|
| L1 | `main_async` 一个函数 220 行，混了 ROS 初始化、依赖组装、重连、心跳、收包、清理 | `navigation_executor.py:343-563` |
| L2 | docstring 覆盖率低：`navigation_types / pose_provider / headless_config / semantic_pose_navigator` 为 0；core 12 个函数 1 个 docstring | 统计 |
| L3 | 注释是"事故日记"（"2026-07-27 实测……"），读者要读完历史才能理解当前不变量 | `navigation_executor_core.py:81-86,271-276` 等 |
| L4 | 日志方式混用：执行器用 `print` 包装，dock 用 `get_logger()` | `navigation_executor.py:41`、`dock_navigator.py:99` |
| L5 | 手搓 argv 解析：`--server-uri` 漏写参数值时静默当成 ROS 参数透传 | `navigation_executor.py:102-159` |
| L6 | 魔法数：闭环重投影阈值 `0.05 / 0.03` 写死在调用处 | `navigation_executor.py:337-338` |

#### 可扩展性

| # | 问题 | 证据 |
|---|---|---|
| E1 | 命令分发是一长串 `if command.action == ...`，加一条命令要改 core 本体 | `navigation_executor_core.py:131-246` |
| E2 | 用 `hasattr` 探测依赖有没有某方法 —— 接口没被定义出来的信号 | `navigation_executor.py:199,371` |
| E3 | 配置来源 5 处且优先级无文档：代码常量、`NAV_WS_URL`、`NAV_SERVER_URI`、`navigation_headless.yaml`、shell 变量 | `navigation_executor.py:16-38`、`start_waypoint_nav_headless.sh:54` |
| E4 | `robot_base_frame` 默认值三处不一致：执行器 `base_link`、semantic 导航 `base`、dock `base_footprint`（David 上 Nav2 必须用 `base_footprint`） | `navigation_executor.py:23`、`semantic_pose_navigator.py:18`、`dock_navigator.py:61` |
| E5 | 导航终态靠 bridge 回调推给 core，snap→anchor 第二段导航**嵌套在第一段的回调里**执行，控制流难追 | `navigation_executor_core.py:268-294` |
| E6 | 死代码/未用状态：`is_anchor_static_blocked` 已无调用方；`ExecutorState.DISCONNECTED/CONNECTING` 从未被设置 | `navigation_goal_resolver.py:176`、`navigation_executor_core.py:26-27` |

### 2.3 鲁棒性缺陷（本次审查发现，P4 逐个修）

| # | 缺陷 | 触发条件 → 后果 | 证据 |
|---|---|---|---|
| **R1** | 畸形 `mark_current_poi`（缺 `poi`）或 `abort_navigation`（缺 `request_id`）抛 `KeyError`，而收包循环只 catch `ValueError` | 一帧坏消息 → 异常冲出 `async for` → **拆掉 WS 连接**，与 v1 checklist "畸形消息记日志、不要崩" 直接冲突 | `navigation_protocol.py:95-107`；`navigation_executor.py:508-512` |
| **R2** | 后台导航 task 若抛代码级异常，只打一行日志；`active_task` 不清、`state` 卡在 STARTING、**不回 on_error** | agent server 只能等超时才知道失败，且后续 abort 仍指向僵尸任务 | `navigation_executor_core.py:94-103` |
| **R3** | Nav2 feedback 回调里 `asyncio.create_task(...)` 不保存引用 | Python 官方文档明确：事件循环只持弱引用，任务可能在执行中被 GC → 偶发丢 `on_progress` | `nav2_action_bridge.py:180` |
| **R4** | 启动时全局 monkeypatch `asyncio.sleep/wait/Lock`（为兼容 websockets 9.1），但实际加载的是 `~/.local` 里 pip 装的 **websockets 16.1**；`package.xml` 声明的却是 apt 的 9.1 | 部署不可复现（换台机器/换用户即变）；全局补丁影响同进程所有 asyncio 代码 | `navigation_executor.py:67-75,346`；实测 `websockets.__file__` |
| **R5** | Python 入口脚本是拷贝安装（M6） | 改了 src 不重装 → 跑旧代码。历史上已踩过（dock_navigator、david_slam_gt） | `CMakeLists.txt:158-217` |
| **R6** | ROS 与 asyncio 集成用 `spin_once(timeout=0)` + `sleep(0.01)` 忙轮询 | 空闲时 100 Hz 空转；回调延迟最高 10 ms。**先测再改**（§6 P4） | `navigation_executor.py:273-276` |
| **R7** | dock 的日志字符串被 `dock_bench` 用正则解析 —— **日志行是机器接口**，改一个字 bench 就瞎 | 可读性/国际化改动会静默破坏 bench | `scripts/dock_bench/dock_common.py:94-100`；`dock_navigator.py:276-281` |
| **R8** | `dock_navigator` 参数 `goal_topic` 默认 `/goal_pose` —— 它自己的 docstring 就警告这会和 bt_navigator 抢同一目标；只是 launch 碰巧传了 `/dock_goal` | 任何人 `ros2 run` 单独起它即双链路抢目标 | `dock_navigator.py:60` vs `:19-24` |
| **R9** | 生产必需文件未入库：`dock_finish.py`、`dock_settle.py`、`config/fastdds_david.xml`、3 个测试文件；运行垃圾在 src：`agentdb.rvf`、`ruvector.db`、`.codex-backups/`、`*.bak.*` | clone 到新机器直接起不来；重构基线不干净 | `git status` |
| **R10** | 文档漂移：`CLAUDE.md` 仍写 separator 是核心链路、domain=1；`README.md` 路径是 `/home/unitree`、写 Livox；`start_waypoint_nav_headless.sh` 头部说 v1、`:326` 说 "v2 坐标协议"；`dead_reckoning_executor.py:12-13` 声称 waypoint 脚本默认用它，而脚本默认 `/waypoint_follower/navigate` | 新人/agent 按文档操作必错 | 各文件 |

---

## 3. 参考的业界工程案例：借鉴什么、不借鉴什么

| 案例 | 核心做法 | 本次借鉴点 |
|---|---|---|
| **Nav2 `nav2_simple_commander`** | "navigation as a library"：`BasicNavigator` 把 action/lifecycle 细节藏起来，对外只暴露 `goToPose()` 等简单方法 | `adapters/ros/nav2_backend.py` 作为**唯一**的 NavigateToPose 客户端实现，dock / bench / wait_nav_ready 全部复用（修 M3） |
| **Nav2 Docking Server（opennav_docking，有 Humble 分支）** | 对接流程 = 先导航到 **staging pose（预备点）**，再做精确入位；机器人/对接桩差异通过 `ChargingDock / NonChargingDock` **插件**注入 | 我们的两段式入位与之同构。把 `dock_settle / dock_finish` 抽成纯逻辑"技能"，几何（预备点计算）与执行（Spin/导航）通过注入分离。**不直接换成 opennav_docking**：它依赖外部检测（AprilTag 等）且是 C++ 插件，迁移成本高于收益，记为后续可选 |
| **Open-RMF `fleet_adapter_template`** | 严格两层：`fleet_adapter.py`（面向调度系统的逻辑）与 `RobotClientAPI.py`（面向具体机器人的 API 调用）分开，配置单独放 `config.yaml` | 我们的"agent server 协议层"≈ RMF 侧，"Nav2/waypoint backend"≈ RobotClientAPI。换机器人 = 换 backend + 换配置，core 不动 |
| **Autoware Component Interface / AD API** | 组件之间只通过**有文档、有版本**的接口交互；对外 API 与内部实现解耦 | 线协议写成独立的 `protocol/` 子包 + 协议文档（AsyncAPI 格式），版本号显式化；内部重构不得泄漏到协议 |
| **六边形架构（Ports & Adapters，Alistair Cockburn）** | 业务核心只依赖抽象端口，IO 全部在外圈适配器；测试用假适配器 | core 只依赖 `core/ports.py` 的 `typing.Protocol`（PEP 544，Python 3.8 可用） |
| **Google Python Style Guide** | docstring 用 `Args:` / `Returns:` / `Raises:` 分节；import 分组；公共 API 必须类型注解；不用 `assert` 做运行时逻辑；catch-all 只允许在隔离边界 | §5 编码规范直接采用 |
| **ROS 2 代码风格 + `ament_flake8` 配置** | PEP 8；行宽 99（`ament_flake8.ini`：`max-line-length = 99`、`import-order-style = google`）；单引号优先；pep257 docstring | 行宽以 ament 的 **99** 为准（覆盖 Google 的 80），import 顺序与 Google 一致 |
| **REP-2004 包质量等级** | 从版本管理、变更控制、文档、测试、依赖、平台支持、安全等方面分级 | `nav_executor` 目标 **Quality Level 3**：有 README/API 文档、单测、CHANGELOG、显式依赖 |
| **import-linter（layers contract）** | 用配置声明分层，CI 检查"低层不得 import 高层" | 把 §4.1 的依赖规则写成可执行测试，防止架构腐化 |
| **AsyncAPI** | 用机器可读格式描述 WebSocket 等消息驱动 API（channel / message / schema） | `docs/protocol/nav_ws_v1.asyncapi.yaml` 成为协议唯一真源，golden 测试从中取样例 |
| **Strangler Fig（Martin Fowler）** | 新代码在旧系统旁边逐步长出来、逐步接管，系统全程可运行 | 迁移全程保留 shim；每阶段独立可回滚（§6） |
| **ADR（Michael Nygard）** | 每个重大决策一页纸：背景/决策/后果 | 把代码里的事故注释提炼成 7 份 ADR（§6 P7） |

---

## 4. 目标架构

### 4.1 分层与依赖规则

```
┌───────────────────────────────────────────────────────────────┐
│ app/          组装根 · CLI · 配置加载 · 运行时循环                 │  ← 唯一知道"所有具体实现"的层
├───────────────────────────────────────────────────────────────┤
│ adapters/     ros/ (Nav2, TF, costmap)  transport/ (WebSocket)   │  ← 实现 core/ports 里的接口
│               storage/ (YAML POI)                              │
├───────────────────────────────────────────────────────────────┤
│ core/         状态机 · 命令处理器注册表 · 重试策略 · snap 算法      │  ← 只依赖 ports + protocol + domain
│               ports.py（抽象接口）                               │
├───────────────────────────────────────────────────────────────┤
│ protocol/     nav WS v1 消息数据类 · 编解码（纯数据，零 IO）         │
├───────────────────────────────────────────────────────────────┤
│ domain/       位姿/POI 值对象 · 几何工具 · 栅格视图 · 异常层级        │  ← 零依赖（仅标准库）
└───────────────────────────────────────────────────────────────┘
skills/docking/   纯逻辑技能（停稳判据、入位判定），只依赖 domain
```

**依赖规则（由 import-linter 强制）**：

1. 上层可以 import 下层，下层**绝不** import 上层。
2. `domain / protocol / core / skills` **禁止** import `rclpy`、`websockets`、`yaml`、任何 `*_msgs`。（这保证它们在 PC 上无 ROS 环境也能跑单测。）
3. `adapters/ros`、`adapters/transport`、`adapters/storage` 三者**互不 import**。
4. 只有 `app/` 可以同时 import 多个 adapter。

### 4.2 包布局

```
nav_executor/                         # 与 david_nav 同级的新 ament_python 包
├── package.xml                       # format 3；显式声明 rclpy/nav2_msgs/tf2_ros/python3-yaml 等
├── setup.py / setup.cfg              # console_scripts 入口；flake8 配置
├── resource/nav_executor
├── README.md                         # 30 秒上手 + 复用指南（§4.9）
├── CHANGELOG.rst
├── docs/
│   ├── architecture.md               # §4 的正式版
│   ├── protocol/
│   │   ├── nav_ws_v1.md              # 人读版
│   │   └── nav_ws_v1.asyncapi.yaml   # 机读版（唯一真源）
│   └── adr/0001-*.md … 0007-*.md
├── nav_executor/
│   ├── __init__.py                   # __version__；只 re-export 稳定公共 API
│   ├── domain/
│   │   ├── types.py                  # PoseRecord, StoredPoi, Transform2D, CapturedPose, NavGoal, NavOutcome
│   │   ├── geometry.py               # yaw_from_quaternion, quaternion_from_yaw, wrap_to_pi, transform_pose
│   │   ├── grid.py                   # GridMapView（原 grid_map_view.py）
│   │   └── errors.py                 # NavExecutorError 异常层级
│   ├── protocol/
│   │   ├── messages.py               # Command / Event 数据类
│   │   └── codec.py                  # decode_command(payload) / encode_event(event)
│   ├── core/
│   │   ├── ports.py                  # NavigationBackend, PoseSource, PoiRepository, GoalResolver, Clock
│   │   ├── state.py                  # ExecutorState, ActiveTask
│   │   ├── executor.py               # NavigationExecutor：状态机 + 分发
│   │   ├── registry.py               # CommandRegistry
│   │   ├── handlers/                 # 每条命令一个文件
│   │   │   ├── navigate_to.py
│   │   │   ├── abort_navigation.py
│   │   │   ├── mark_current_poi.py
│   │   │   └── update_poi_list.py
│   │   ├── retry.py                  # RetryPolicy（失败消抖，纯逻辑）
│   │   ├── goal_resolution.py        # SnapSearch 纯算法 + DirectGoalResolver
│   │   └── supervisor.py             # TaskSupervisor：后台 task 强引用 + 崩溃兜底
│   ├── adapters/
│   │   ├── ros/
│   │   │   ├── futures.py            # await_rclpy_future（asyncio）/ wait_rclpy_future（线程）
│   │   │   ├── nav2_backend.py       # NavigateToPose 客户端（唯一实现）
│   │   │   ├── tf_pose_source.py     # 原 pose_provider.py
│   │   │   ├── costmap_goal_resolver.py  # 订阅两张栅格 → 调 core.goal_resolution
│   │   │   └── spin.py               # rclpy ↔ asyncio 集成策略
│   │   ├── transport/
│   │   │   ├── ws_client.py          # 连接/重连/心跳/收包
│   │   │   ├── outbox.py             # ReliableOutbox：断联不丢帧
│   │   │   └── proxy.py              # no_proxy 白名单
│   │   └── storage/
│   │       ├── yaml_poi_repository.py
│   │       └── named_poses_importer.py
│   ├── skills/
│   │   └── docking/
│   │       ├── settle.py             # 原 dock_settle.py
│   │       └── finish.py             # 原 dock_finish.py
│   └── app/
│       ├── config.py                 # ExecutorConfig + 加载优先级
│       ├── runtime.py                # 组装根
│       ├── status.py                 # STATUS 行打印循环
│       ├── loop_closure.py           # map→odom 变化后重投影 POI
│       ├── logging.py                # 统一日志（保留稳定前缀）
│       └── cli.py                    # argparse 入口 main()
└── test/
    ├── unit/                         # 无 ROS，PC 上可跑
    ├── golden/                       # 线协议金样（*.jsonl）
    ├── integration/                  # 需 ROS：domain 99 + 假 Nav2 action server
    ├── fakes/                        # FakeBackend / FakePoseSource / FakeAgentServer
    ├── test_architecture.py          # import-linter + "Node 子类不得覆盖父类属性"检查
    └── test_flake8.py / test_pep257.py
```

`david_nav` 里保留（只做瘦身，不搬家）：launch、config、behavior_trees、C++ 插件、headless 脚本、`dock_navigator.py`（改为薄节点，调用 `nav_executor`）、位置环/waypoint、速度链路。

### 4.3 端口定义（core 与外界的全部契约）

> 以下是要落地的**真实代码**，也是 Google docstring 的示范。Python 3.10（当前机上版本，见 §5.2）。

```python
# nav_executor/core/ports.py
"""core 层依赖的抽象端口。

core 只依赖本模块中的 Protocol。ROS、WebSocket、文件系统等具体实现位于
``nav_executor.adapters``，由 ``nav_executor.app.runtime`` 注入。测试使用
``test/fakes`` 中的假实现，它们满足同样的 Protocol。

典型用法:

    backend = Nav2Backend(node, action_name='/navigate_to_pose')
    executor = NavigationExecutor(backend=backend, pose_source=..., pois=...)
"""

from __future__ import annotations

from typing import Awaitable, Callable, Iterable, Optional, Protocol

from nav_executor.domain.types import (
    CapturedPose, NavGoal, NavOutcome, PoseRecord, StoredPoi, Transform2D)

FeedbackCallback = Callable[[float], None]
"""进度回调，参数为剩余距离（米）。实现方必须保证在事件循环线程内调用。"""


class NavigationBackend(Protocol):
    """把一个 map 系位姿目标交给底层导航栈执行。

    实现示例：Nav2 bt_navigator、waypoint_follower、dead_reckoning、测试假件。
    backend 只负责"单次尝试"，失败重试策略属于 core（见 core/retry.py）。
    """

    async def navigate(self, goal: NavGoal,
                       on_feedback: FeedbackCallback) -> NavOutcome:
        """发送目标并等待其到达终态。

        Args:
            goal: 目标位姿及其请求上下文（request_id / sub_id 仅用于日志）。
            on_feedback: 剩余距离回调。实现方无需节流，节流由 core 负责。

        Returns:
            终态结果。``NavOutcome.status`` 取值 SUCCEEDED / CANCELED / FAILED；
            FAILED 时 ``detail`` 为可直接上报给 agent server 的可读原因。

        Raises:
            asyncio.CancelledError: 调用方取消本协程时向上传播。实现方在传播前
                必须尽力取消底层 goal，包括"goal 已发出但 handle 尚未返回"的窗口。
        """
        ...

    async def cancel(self) -> None:
        """取消当前正在执行的 goal。无 goal 时为空操作，任何状态下调用都安全。"""
        ...


class PoseSource(Protocol):
    """机器人当前位姿的来源。"""

    def current_map_pose(self) -> PoseRecord:
        """返回机器人在 map 系下的最新位姿。

        Raises:
            PoseUnavailableError: TF 不可用或超时。
        """
        ...

    async def capture_for_poi(self) -> CapturedPose:
        """同时采集 map 与 odom 位姿，用于标记 POI。

        Raises:
            PoseUnavailableError: 任一位姿不可用。
        """
        ...

    def current_map_from_odom(self) -> Transform2D:
        """返回 map←odom 变换，用于闭环后 POI 重投影。"""
        ...

    def current_slam_session_id(self) -> str:
        """返回当前 SLAM 会话 id。"""
        ...


class PoiRepository(Protocol):
    """POI 的持久化仓库。所有写操作必须原子落盘（写临时文件后 rename）。"""

    def load(self) -> None: ...
    def get(self, poi_id: str) -> Optional[StoredPoi]: ...
    def upsert(self, poi: StoredPoi) -> StoredPoi: ...
    def rename_many(self, id_to_name: Iterable[tuple[str, str]]) -> None: ...
    def prune_session(self, session_id: str, keep_ids: set[str]) -> list[str]: ...
    def reproject_session(self, session_id: str, map_from_odom: Transform2D,
                          translation_threshold_m: float,
                          yaw_threshold_rad: float) -> list[StoredPoi]: ...


class GoalResolver(Protocol):
    """把 POI 锚点解析成实际可导航的目标（必要时做 snap 扩圈）。"""

    def resolve(self, poi: StoredPoi) -> Optional[ResolvedGoal]:
        """Returns: 解析结果；锚点及所有扩圈候选都不可达时返回 None。"""
        ...

    def is_admissible(self, poi: StoredPoi) -> bool: ...


class Clock(Protocol):
    """可注入时钟，测试中用假时钟消除 sleep。"""

    def now(self) -> float: ...
    async def sleep(self, seconds: float) -> None: ...
```

**对现有接口的两个关键改动**（都必须先有 golden 测试覆盖，见 §6 P0）：

1. **终态从"回调推送"改为"返回值"**（修 E5）。`backend.navigate()` 直接返回 `NavOutcome`，snap→anchor 第二段变成 `navigate_to` 处理器里的**顺序代码**，不再嵌套在回调里：

   ```python
   outcome = await self._navigate_with_retry(ctx, resolved.goal)
   if outcome.succeeded and resolved.used_snap and self._resolver.is_admissible(poi):
       outcome = await self._navigate_with_retry(ctx, anchor_goal)   # 第二段
   ```

2. **失败消抖从 bridge 挪到 core 的 `RetryPolicy`**，backend 只做单次尝试。行为要求：CANCELED 不重试、FAILED 重试 `retry_count` 次、间隔 `retry_delay_sec`、耗尽后 `on_error` 文案保持 `导航失败(共尝试{n}次): {detail}` 不变。

   ⚠ **保留一个容易漏的语义**：现在 bridge 收到 CANCELED 会发 `progress(status="paused")`。我们自己 abort/preempt 时这帧会被 core 过滤掉；但**外部取消**（例如有人从 CLI 取消了 nav2 goal）时，它会以当前 request_id 发给 agent server。新实现必须保持："CANCELED 且不是本执行器发起的取消 → 发 `on_progress paused`"。

### 4.4 命令分发：注册表（修 E1，演示扩展点）

```python
# nav_executor/core/registry.py
class CommandRegistry:
    """action 名 → (解码器, 处理器) 的注册表。

    新增一条协议命令不需要修改 NavigationExecutor 本体（开闭原则）。

    Example:
        registry = CommandRegistry()
        registry.register('navigate_to', decode_navigate_to, NavigateToHandler(...))
    """

    def register(self, action: str, decoder: Decoder, handler: Handler) -> None:
        """注册一条命令。

        Raises:
            ValueError: action 已被注册（防止两个处理器静默覆盖）。
        """
```

**加一条新命令的三步**（README 里原样写进"扩展指南"）：以将来可能的 `dock_to` 为例 —
① `protocol/messages.py` 加 `DockToCommand` + 在 AsyncAPI 文档加 message；
② `core/handlers/dock_to.py` 实现处理器（内部复用 `skills/docking`）；
③ `app/runtime.py` 里 `registry.register('dock_to', ...)`。core、transport、其它处理器一行不改。
**本轮只写文档示例，不实现 `dock_to`**（非目标）。

### 4.5 并发模型（写进 `docs/architecture.md` 的"铁律"）

| 规则 | 原因（对应 ADR） |
|---|---|
| 执行器进程内**只有一个 asyncio 事件循环**，core 的所有状态只在该循环线程读写 | 免锁 |
| 任何 `rclpy` future 在 asyncio 里一律用 `adapters/ros/futures.await_rclpy_future` 等待；在线程里一律用 `wait_rclpy_future`（`threading.Event`） | ADR-0005：**禁止** `rclpy.spin_until_future_complete`（2026-09-08 两个执行器抢同一 action client 的 wait set，进程崩溃） |
| 所有后台 task 必须经 `TaskSupervisor.spawn()` 创建 | 修 R3（弱引用被 GC）+ R2（崩溃统一收尸并回 on_error） |
| 导航必须是后台 task，收包循环永不 `await` 导航终态 | ADR-0001：v1.1 "随时可 abort" |
| 发往 server 的**所有**帧都走 `ReliableOutbox` | ADR-0002：断联窗口内的 on_arrived 不能丢 |
| rclpy 节点子类不得定义与 `rclpy.node.Node` 同名的属性/方法 | `handle` 覆盖事故；`test_architecture.py` 用 AST 自动检查 |

`TaskSupervisor`（当前 Python 3.10 没有 `asyncio.TaskGroup`，3.11 才有，所以自己实现）：

```python
class TaskSupervisor:
    """持有后台 asyncio.Task 的强引用，统一处理异常。

    asyncio 事件循环只保存 task 的弱引用，未被引用的 task 可能在执行中被
    垃圾回收（见 Python 文档 asyncio.create_task 的 Important 注记）。

    Attributes:
        active_count: 当前未结束的 task 数。
    """

    def __init__(self, on_crash: Callable[[str, BaseException], None]) -> None:
        self._tasks: set[asyncio.Task] = set()
        self._on_crash = on_crash

    def spawn(self, coro: Awaitable[None], name: str) -> asyncio.Task:
        """创建并托管一个后台 task。

        Args:
            coro: 要运行的协程。
            name: task 名，出现在崩溃日志里。

        Returns:
            已创建的 task，调用方可保存用于 cancel。
        """
        task = asyncio.ensure_future(coro)
        self._tasks.add(task)
        task.add_done_callback(lambda t: self._on_done(name, t))
        return task
```

**R2 的修法**：`NavigationExecutor` 给 supervisor 的 `on_crash` 回调里，如果崩的是当前导航 task → 入队 `on_error("执行器内部错误: <异常类型>")`，清 `active_task`，状态回 IDLE。

### 4.6 配置：一个数据类 + 明确的优先级（修 E3/E4）

```python
@dataclass(frozen=True)
class ExecutorConfig:
    """执行器全部可调参数。字段即文档，默认值与 2026-09-14 线上行为一致。"""
    server_uri: str = 'ws://127.0.0.1:8101'
    action_name: str = '/navigate_to_pose'
    robot_base_frame: str = 'base_link'
    ...
```

**优先级（高 → 低）**：CLI 参数 > 环境变量（仅 `NAV_WS_URL`）> `--config` 指定的 YAML > 数据类默认值。启动时打印一行生效配置和每个值的来源。

| 键 | 默认 | CLI | 说明 |
|---|---|---|---|
| server_uri | `ws://127.0.0.1:8101` | `--server-uri` | 环境变量 `NAV_WS_URL` 可覆盖默认 |
| poi_store_file | `~/.ros/david_nav/poi_store.yaml` | `--poi-store-file` | **路径不变**（兼容） |
| pose_file | 无 | `--pose-file` | named_poses.json，启动导入并清理过期点 |
| action_name | `/navigate_to_pose` | `--action-name` | waypoint 栈用 `/waypoint_follower/navigate` |
| server_timeout | 10.0 | — | 等 action server 的秒数 |
| heartbeat_interval | 30.0 | — | |
| reconnect_delay | 3.0 | `--reconnect-delay` | |
| nav_retry_count / nav_retry_delay_sec | 1 / 1.0 | `--nav-retry-count` / `--nav-retry-delay` | |
| robot_base_frame | `base_link` | 新增 `--robot-base-frame` | r2 决策：保持 `base_link`（行为不变）。⚠ 它与 Nav2 用的 `base_footprint` **不是同一帧**：URDF 中 base_footprint 在 base_link 前 0.165 m、下 0.4075 m。用 base_link 采的 POI 交给 Nav2 到达，旋转中心会沿 POI 朝向偏 16.5 cm（总方案 §2.3；点位文件为手动添加，本轮不处理） |
| odom_topic | `/lightning/odometry` | 新增 | `mark_current_poi` 依赖它。r5：alias 3 实际发布 `/lightning/loc/odometry`，默认值 `/lightning/odometry` 无数据；本轮只暴露为参数、不改默认，接线见总方案 TODO-O1 |
| grid_map_topic / global_costmap_topic | `/lightning/grid_map` / `/global_costmap/costmap` | 新增 | |
| snap_radius / snap_step / max_snap_radius / goal_clearance_radius | 0.5 / 0.25 / 3.0 / 0.1 | — | `fallback_snap_radius`(1.0) 仅兼容读取，外扩上限取两者较大值 |
| loop_closure_translation_m / yaw_rad | 0.05 / 0.03 | — | 原魔法数（L6） |
| progress_min_interval_sec | 1.0 | — | on_progress 节流 |
| pose_stale_after_sec | 0.5 | — | mark_current_poi 位姿过期阈值 |

`navigation_headless.yaml` 的 `executor:` 段**格式不变**，由 `app/config.py` 新增 `load_headless_executor_section()` 读取；`scripts/navigation_headless_config.py` 继续工作。

### 4.7 错误模型

```
NavExecutorError                  # 本包所有异常的基类
├── ProtocolError                 # 线协议问题（带 request_id/sub_id，可能为 None）
│   ├── MalformedCommandError     # 字段缺失/类型错   → 有 request_id 回 on_error，否则记日志丢弃
│   └── UnsupportedActionError    # 未知 action       → 记日志丢弃
├── PoseUnavailableError          # TF/odom 不可用
├── PoiNotFoundError
└── ConfigError
```

**边界规则**（Google 风格"catch-all 只允许在隔离边界"）：只有三个地方允许 `except Exception`：
① 收包循环里单条消息的处理（记日志 + 继续收包，**绝不拆连接** —— 修 R1）；
② `TaskSupervisor` 的崩溃兜底；
③ 重连循环的外层。其它地方必须 catch 具体异常。解码器内部把所有 `KeyError / TypeError / ValueError` 统一转成 `MalformedCommandError`。

### 4.8 日志

- 用 Python `logging`，`app/logging.py` 配置 formatter 使输出**与现在逐字一致**：`[navigation_executor] <msg>`、`[nav2_action_bridge] <msg>`、`[navigation_executor_core] <msg>`（logger 名映射到旧前缀）。
- **冻结的日志行**（可能被脚本/人工 grep，改动需走 ADR）：

  | 日志行模式 | 产生位置 |
  |---|---|
  | `[navigation_executor] NAV ERROR reported to server: request_id=… sub_id=… error_message=…` | 回 on_error 前 |
  | `[navigation_executor] STATUS state=… poi=… phase=… d2g=… x=… y=… yaw=…rad` | 状态循环 |
  | `[navigation_executor] OUT sent: …` / `OUT RESENT(断联重发): …` | outbox |
  | `[nav2_action_bridge] NAV FAILURE attempt i/n request_id=… sub_id=…: …` | 单次失败 |
  | `[dock] ===== 收到目标` / `阶段1 预备点` / `阶段2 入位 ->` / `第N次纯旋转修正` / `===== 成功|失败:` / `入位实测 exy=` | dock_navigator（被 dock_bench 正则解析，R7） |

- P0 阶段 Codex 先执行 `grep -rn "NAV ERROR\|STATUS state=\|OUT RESENT\|NAV FAILURE" ~/ros2_ws/src ~/lightning_ws_new/src 2>/dev/null` 列出所有消费者，写进 `docs/architecture.md` 的"日志接口"一节。

### 4.9 复用方式（写进 README "在其他导航任务中复用"）

| 场景 | 怎么复用 | 需要写的代码 |
|---|---|---|
| David 生产导览（alias 4） | `ros2 run nav_executor navigation_executor --config …` | 0 |
| David waypoint 栈 | `--action-name /waypoint_follower/navigate` | 0 |
| G1 / GO2W / 新机器人 | 在各自分支 `git checkout feature/david_main -- nav_executor` 拿到包；只写一份机器人的 YAML（frame、话题、server 地址） | 0（限 Humble / Python 3.10 机器人；GO2W(Foxy/3.8) 复用为 TODO-V1） |
| 不走 WebSocket、本地脚本直接调 | `from nav_executor.adapters.ros.nav2_backend import Nav2Backend` + `await backend.navigate(goal, cb)` | 几行 |
| 换一种导航后端（例如自研 lightweight_nav） | 新写一个满足 `NavigationBackend` Protocol 的类，在 runtime 里注入 | 1 个类 |
| 新的任务编排节点（像 dock_navigator） | 复用 `adapters/ros/futures`、`domain/geometry`、`skills/*` | 只写编排本身 |

---

## 5. 编码与文档规范

### 5.1 工具与配置（新包 `setup.cfg`）

```ini
[flake8]
# 与 ament_flake8 (Humble) 默认配置一致，colcon test 与本地 flake8 结论相同
extend-ignore = B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202
import-order-style = google
max-line-length = 99
```

- `test/test_flake8.py` / `test/test_pep257.py` 用 `ament_flake8` / `ament_pep257` 标准模板，`colcon test` 时执行。**新包必须 0 告警**。
- 引号：新代码用**单引号**（ROS 2 风格指南），JSON 键名等需要转义时除外。移动过来的旧代码在"搬家"提交里**不改引号**，单独一个 `style:` 提交统一，保证搬家 diff 可审。
- 类型检查：机器上没装 mypy，**不作为门禁**；PC 端可选跑 `mypy nav_executor/domain nav_executor/protocol nav_executor/core`。

### 5.2 Python 与依赖版本（r2：以当前机上环境为准）

**以当前为准**：Python 3.10.12；websockets 16.1（`~/.local` pip，实际加载的版本）；PyYAML 6.0.3；psutil 5.9.0；pytest 6.2.5；flake8 4.0.1；ROS 2 Humble（nav2 1.1.20）。启动日志与运行记录 manifest 记录这些版本，漂移时告警。依赖固化为 TODO-V2。

**TODO-V1（本轮不做、不强制）**：将来复用到 GO2W（Foxy / Python 3.8）时，再评估以下约束：

- 每个文件首行 `from __future__ import annotations`（这样 `list[str]`、`X | None` 写在注解里没问题）。
- **注解之外**不得出现 `X | None`（例如 `isinstance(x, int | str)` 不行）、`match` 语句、`dataclass(slots=True)`、`asyncio.TaskGroup`、`str.removeprefix`（3.9）。
- 运行时要用的类型别名写成 `Optional[...]` / `Union[...]`。
- `test/unit` 在 PC 上用 3.8 虚拟环境跑一遍作为兼容性证据（可选但推荐）。

### 5.3 Docstring（Google 风格）

**语言约定**：分节关键字 `Args:` / `Returns:` / `Raises:` / `Attributes:` / `Example:` 用英文（工具要识别）；描述内容用中文；标识符用英文。

**模块**：

```python
"""nav WS v1 协议的消息编解码。

本模块是线协议的唯一实现。字段定义以 docs/protocol/nav_ws_v1.asyncapi.yaml
为准，二者不一致时以文档为准并修代码。

本模块不做任何 IO，也不依赖 ROS。
"""
```

**类**：一句话说明"实例代表什么" + `Attributes:`（仅公共属性）。

**函数**：

```python
def decode_command(payload: Mapping[str, Any]) -> Command:
    """把一帧 agent server 下行 JSON 解码成命令对象。

    v1 规则：navigate_to 只认 ``target_id``；残留的 v2 字段 ``target_pose`` /
    ``label`` 直接忽略，不会走坐标直达。

    Args:
        payload: ``json.loads`` 后的顶层对象。

    Returns:
        对应 action 的不可变命令数据类。

    Raises:
        MalformedCommandError: 必需字段缺失或类型错误。异常上带有尽力提取的
            ``request_id`` / ``sub_id``，调用方据此决定能否回 on_error。
        UnsupportedActionError: ``action`` 不在已知集合内。
    """
```

**必须写 docstring 的范围**：所有公共模块、公共类、公共函数/方法、所有 `Protocol` 方法。私有函数（`_` 开头）超过 10 行或有非显而易见逻辑时也要写。

### 5.4 注释

- 只写"为什么"，不复述代码。
- **事故复盘不再写在代码里**。代码只保留一行不变量 + ADR 链接：

  ```python
  # 不变量：导航必须在后台 task 里跑，收包循环不得 await 导航终态。
  # 见 docs/adr/0001-navigation-runs-as-background-task.md
  ```

- 日期化的历史叙述搬进对应 ADR 的"背景"节，**一个字都不丢**。

### 5.5 测试组织

| 目录 | 依赖 | 跑在哪 | 内容 |
|---|---|---|---|
| `test/unit` | 仅标准库 + pytest | PC / 机器人 | domain、protocol、core（配 fakes）、outbox、retry、snap 算法、skills |
| `test/golden` | 同上 | 同上 | 场景脚本 → 下行帧序列，与 `*.jsonl` 逐字节比对 |
| `test/integration` | ROS Humble | 机器人，`ROS_DOMAIN_ID=99` | 真 rclpy + 假 NavigateToPose/Spin action server + FakeAgentServer（websockets） |
| `test/test_architecture.py` | import-linter | 同上 | 分层契约 + Node 属性覆盖检查 |

import-linter 配置（放 `setup.cfg`；**本轮不安装**，TODO-I1，未安装时测试 skip，由 AST 扫描单测暂代）：

```ini
[importlinter]
root_package = nav_executor

[importlinter:contract:layers]
name = nav_executor 分层
type = layers
layers =
    nav_executor.app
    nav_executor.adapters
    nav_executor.core
    nav_executor.protocol | nav_executor.skills
    nav_executor.domain

[importlinter:contract:adapters-independent]
name = 适配器之间互不依赖
type = independence
modules =
    nav_executor.adapters.ros
    nav_executor.adapters.transport
    nav_executor.adapters.storage

[importlinter:contract:pure-core]
name = 内层不得依赖 ROS/网络/YAML
type = forbidden
source_modules =
    nav_executor.domain
    nav_executor.protocol
    nav_executor.core
    nav_executor.skills
forbidden_modules =
    rclpy
    websockets
    yaml
```

---

## 6. 迁移计划（Strangler Fig：每一步独立提交、系统全程可用、可单独回滚）

> 每个阶段结束的**统一门禁**：`colcon build --symlink-install --packages-select nav_executor david_nav` 通过；`colcon test` 通过；原有 90 个测试仍通过；golden 全通过。

### P0 基线与安全网（不动任何生产代码）

1. **未提交修改由用户处理，Codex 不得替用户提交**。先 `git status` + `git diff --stat` 列给用户，等用户确认提交或 stash。当前是：9 个已修改（含 `nav2_params_2d*.yaml`、`precise_nav_bringup.launch.py`、`dock_navigator.py`）+ 未跟踪的 `dock_finish.py`、`dock_settle.py`、`fastdds_david.xml`、`test_dock_*.py`、`test_nav2_frame_consistency.py`、`scripts/dock_bench/`（R9）。
2. 用户确认后打 tag：`git tag pre-nav-executor-refactor`。
3. `.gitignore` 增加：`agentdb.rvf*`、`ruvector.db`、`.codex-backups/`、`*.bak.*`、`.pytest_cache/`。**只加忽略规则，不删文件**。
4. **补特征测试（characterization tests）**，在旧代码上跑绿后提交，放 `david_nav/test/test_executor_golden.py`，金样放 `david_nav/test/golden/`。至少覆盖以下场景，每个场景录"下行帧序列 + 关键状态"：

   | 场景 | 断言 |
   |---|---|
   | G1 navigate_to 正常到达 | `[]` → progress(running) → on_arrived |
   | G2 POI 不存在 | on_error `目标点不存在: X` |
   | G3 target_id 缺失 / 类型错 | on_error，bridge 未被调用 |
   | G4 snap 后到达且锚点仍不可达 | 只发一次 on_arrived，不发起第二段 |
   | G5 snap 后到达且锚点变可达 | 发起第二段，第二段到达后才 on_arrived |
   | G6 snap 无可达点 | on_error `目标点不可达，且未找到可行扩圈点` |
   | G7 单次失败 → 重试成功 | 不发 on_error，最终 on_arrived |
   | G8 连续失败耗尽 | on_error `导航失败(共尝试2次): …` |
   | G9 导航中 abort（request_id 匹配） | 立即 progress(paused)，之后旧 goal 的 canceled 事件不再外发 |
   | G10 abort request_id 不匹配 / 空闲 abort | 无帧、不 cancel |
   | G11 导航中新 navigate_to 抢占 | 旧 task 被 cancel，旧结果不以新 request_id 外发 |
   | G12 外部取消（非本执行器发起的 CANCELED） | progress(paused) 以当前 request_id 外发（§4.3 保留语义） |
   | G13 mark_current_poi 成功 / 位姿过期 / 取位姿异常 | ack + success / ack + error |
   | G14 update_poi_list | 只改名、落盘 |
   | G15 断联期间产生 on_arrived → 重连 | 重连后先重发滞留帧，日志 `OUT RESENT(断联重发)` |
   | G16 `--help` 输出、`_parse_cli_args` 全部旧参数 | 文本/字典一致 |

   G12、G15 现有测试未覆盖，**必须新写**。
5. 录一份 `poi_store.yaml` 样例（脱敏后）作为存储兼容金样。

**回滚**：P0 只加测试和 .gitignore，无需回滚。

### P1 建包 + 纯逻辑搬家（domain / protocol / skills）

1. 新建 `nav_executor` 包骨架（§4.2），`package.xml` 使用 `ament_python`。
2. 移动（用 `git mv` 保留历史；跨包移动时先 `git mv` 再小改 import）：
   - `navigation_types.py` → `domain/types.py`
   - `grid_map_view.py` → `domain/grid.py`
   - 新建 `domain/geometry.py`：`yaw_from_quaternion(x, y, z, w)`、`quaternion_from_yaw(yaw)`、`wrap_to_pi(a)`、`transform_pose(tf, pose)`，并配单测（含 ±π 边界、四元数非归一化输入）。
   - `navigation_protocol.py` → `protocol/messages.py` + `protocol/codec.py`
   - `dock_settle.py` / `dock_finish.py` → `skills/docking/settle.py` / `finish.py`
3. 在旧路径留 **shim**，保证 `waypoint_follower.py`、`dock_navigator.py`、旧测试的 import 不坏：

   ```python
   # david_nav/david_nav/navigation_protocol.py
   """已迁移至 nav_executor.protocol。本文件仅为兼容旧 import 保留，勿新增内容。"""
   from nav_executor.protocol.codec import *  # noqa: F401,F403
   from nav_executor.protocol.messages import *  # noqa: F401,F403
   ```

4. `david_nav/package.xml` 增加 `<exec_depend>nav_executor</exec_depend>`。
5. 本阶段**不修 bug、不改逻辑**，protocol 解码器行为保持原样（R1 留到 P4）。

**验证**：门禁 + `python3 -c "import david_nav.navigation_protocol, nav_executor.protocol.codec"`。
**回滚**：`git revert` 本阶段提交；shim 保证下游不受影响。

### P2 core：端口 + 注册表 + 处理器拆分

1. 写 `core/ports.py`（§4.3 原样）、`core/state.py`、`core/registry.py`、`core/retry.py`、`core/supervisor.py`。
2. `navigation_executor_core.py` 拆成 `core/executor.py` + `core/handlers/*.py`。
3. `navigation_goal_resolver.py` 拆两半：
   - 纯算法 → `core/goal_resolution.py`：`SnapSearch.resolve(anchor_xy, robot_xy, grid, costmap) -> Optional[xy]`，输入是 `GridMapView`，**无订阅**。
   - 订阅两张栅格 + 调算法 → P3 的 `adapters/ros/costmap_goal_resolver.py`。
   - 删除无调用方的 `is_anchor_static_blocked`（E6），在 ADR-0003 记录"为什么删"。
   - 删除从未被设置的 `ExecutorState.DISCONNECTED/CONNECTING`（E6）。
4. 终态改为返回值、重试挪进 core（§4.3 两个关键改动）。
5. 用 `test/fakes/fake_backend.py` 把旧的 `FakeBridge / HangingBridge` 场景全部迁到新接口。

**验证**：门禁；**P0 的 G1–G16 全绿是本阶段唯一的放行标准**，尤其 G5、G11、G12。
**回滚**：revert；旧 `navigation_executor_core.py` 在 shim 化之前一直保留原实现，本阶段结束时才替换为 shim。

### P3 适配器 + 组装根 + CLI

1. `nav2_action_bridge.py` → `adapters/ros/nav2_backend.py`（单次尝试）+ `adapters/ros/futures.py`（asyncio 版取自 `_await_rclpy_future`，线程版取自 `dock_navigator.wait_future`，两份 docstring 里的事故说明搬进 ADR-0005）。保留"goal 已发出但 handle 未回来时被 cancel → handle 迟到后补 cancel"的逻辑（`nav2_action_bridge.py:197-212`）。
2. `pose_provider.py` → `adapters/ros/tf_pose_source.py`；四元数换用 `domain.geometry`。
3. `poi_store.py` → `adapters/storage/yaml_poi_repository.py`；`_populate_poi_store_from_named_poses_json` → `named_poses_importer.py`。`poi_store.py:18` 硬编码的 `"map"` 改为取目标 frame 参数。**YAML 文件格式逐字段不变**（P0 金样比对）。
4. `navigation_executor.py` 拆分：
   - 重连/心跳/收包 → `adapters/transport/ws_client.py`
   - `_drain_outbound_queue` + pending holder → `adapters/transport/outbox.py` 的 `ReliableOutbox`（`put()` / `drain(send)` / `pending_count`），配独立单测：send 抛异常时帧不丢、重连后先发滞留帧。
   - `_bypass_proxy_for_server` → `adapters/transport/proxy.py`
   - `_print_status_loop` → `app/status.py`；`_watch_loop_closure` → `app/loop_closure.py`
   - 依赖组装 → `app/runtime.py`；参数 → `app/config.py`；入口 → `app/cli.py`（argparse，`parse_known_args` 把未知参数交给 ROS，**旧参数名全部保留**）
5. `nav_executor/setup.py` 注册入口：

   ```python
   entry_points={'console_scripts': [
       'navigation_executor = nav_executor.app.cli:main',
   ]},
   ```

6. **旧入口保持可用**（进程名是接口）：`david_nav/david_nav/navigation_executor.py` 改成保留 shebang、保留可执行位的 shim：

   ```python
   #!/usr/bin/env python3
   """兼容入口。实现已迁移至 nav_executor.app.cli。

   文件名与路径不可改：start_navigation_headless.sh:985 直接执行本文件，
   start_navigation_headless.sh:464,576 与 navigation_headless.yaml 的
   cleanup.process_matchers 用 "navigation_executor.py" 匹配进程。
   """
   from nav_executor.app.cli import main

   if __name__ == '__main__':
       main()
   ```

**验证**：门禁 + `test/integration`（domain 99）：FakeAgentServer 发 navigate_to → 假 NavigateToPose server 回 SUCCEEDED → FakeAgentServer 收到 on_arrived；再测 abort、断联重连重发。
**回滚**：revert 后旧 `navigation_executor.py` 恢复原实现；headless 脚本全程未改。

### P4 鲁棒性修复（每个缺陷一个提交，先红后绿）

| 顺序 | 缺陷 | 先写的失败测试 | 修法 |
|---|---|---|---|
| 1 | R1 | FakeAgentServer 依次发 `{"action":"mark_current_poi","request_id":"r"}`（缺 poi）、`{"action":"abort_navigation"}`、`"not json"`、`[1,2]`，再发合法 navigate_to → 断言**连接未断开**且合法命令被执行 | 解码器统一抛 `MalformedCommandError`；收包循环单条消息级 catch-all（§4.7 边界①） |
| 2 | R2 | FakeBackend 的 `navigate` 抛 `RuntimeError` → 断言外发 on_error、`active_task is None`、state=IDLE、下一条 navigate_to 能正常执行 | `TaskSupervisor.on_crash` |
| 3 | R3 | 单测断言 feedback 路径不直接调 `asyncio.create_task` 且 supervisor 持有引用 | feedback → `supervisor.spawn` 或 `loop.call_soon` 同步入队 |
| 4 | R8 | 单测：不带参数构造 dock 节点时 `goal_topic == '/dock_goal'` | 默认值改 `/dock_goal`（launch 本来就传这个，生产行为不变） |
| 5 | R4 | 启动日志打印 `websockets.__version__` 与 `__file__`；版本 < 10 时才在 **ws_client 内部**做兼容，删除全局 monkeypatch | 依赖决策见 §10 D3 |
| 6 | R6 | **先测量**：机器人上起执行器空闲 60 s，`pidstat -p <pid> 1 60` 记录 CPU；再对比方案 | 若空闲 CPU ≥ 3%：改为 rclpy `SingleThreadedExecutor` 跑在独立线程，回调经 `loop.call_soon_threadsafe` 回到事件循环；否则只把轮询间隔做成参数，**不改架构**。结论写进 ADR |

### P5 安装与部署卫生（修 R5 / M5）

1. `david_nav/CMakeLists.txt` 中执行器相关的 `install(PROGRAMS david_nav/navigation_executor.py …)` 保留（shim 很小，拷贝也不会陈旧），并在旁边加注释：
   `# 仅为 shim；实现在 nav_executor 包（symlink-install 生效）`。
2. `dock_navigator` 同样改为 shim + 实现放 `david_nav/david_nav/dock_navigator.py` 的 `main()`，CMake 安装的是 shim（彻底消除"改了 src 忘了 cp 到 install"）。
3. 验证 symlink：`ls -l ~/ros2_ws/install/nav_executor/lib/python3.10/site-packages/nav_executor/` 中 .py 应为指向 src 的链接；若不是，在 README 写明"改完需重新 colcon build"。
4. `david_nav/package.xml` 移除 GO2 遗留依赖 `unitree_go / unitree_api / unitree_interfaces / custom_interface / go2_sport`。**先** `grep -rn "unitree_go\|unitree_api\|go2_sport\|custom_interface" david_nav --include=*.py --include=*.cpp --include=*.hpp --include=CMakeLists.txt` 确认无引用，有引用则保留并在报告里列出。

### P6 精确入位复用底座

1. `dock_navigator.py` 改为使用 `nav_executor.skills.docking`、`nav_executor.domain.geometry`、`nav_executor.adapters.ros.futures`，删除本地 `yaw_of / wrap / wait_future`。
2. `run_action` 保持线程模型（`MultiThreadedExecutor` + 工作线程），**本轮不改成 asyncio**（风险/收益不划算，写进 ADR-0006 的"后果"节）。
3. R7：在不删除任何现有 `[dock]` 日志行的前提下，新增结构化事件话题 `~/events`（`std_msgs/String`，JSON：`{"event":"measured","exy":…,"eyaw":…,"settled":true,"stamp":…}`）。`dock_bench` 迁移到订阅话题是**后续任务**，迁移完成前日志行冻结。
4. `semantic_pose_navigator.py`（G1 时代遗留，默认路径 `/home/unitree/...`、frame `base`）：文件头加 `DEPRECATED` 说明与替代方式（`nav_executor` 的 Nav2Backend），**不删除**。

### P7 文档

1. `nav_executor/README.md`：一句话定位 / 30 秒上手 / 架构图 / 配置表（§4.6）/ 复用指南（§4.9）/ 扩展指南（§4.4）/ 测试命令。
2. `docs/architecture.md`：§4 正式版 + 冻结日志行及其消费者清单。
3. `docs/protocol/nav_ws_v1.asyncapi.yaml` + `nav_ws_v1.md`：从 `protocol/messages.py` 反写；v1 与 v1.1 差异单列；`test/golden` 的样例帧必须能通过 AsyncAPI schema 校验（可选工具 `asyncapi validate`，PC 端跑）。
4. ADR（从代码注释提炼，原文不丢）：

   | ADR | 标题 | 来源注释 |
   |---|---|---|
   | 0001 | 导航作为后台 task 运行（v1.1 随时可 abort） | `navigation_executor_core.py:81-86,173-176` |
   | 0002 | 可靠发送队列：断联不丢终态帧 | `navigation_executor.py:244-251,513-514` |
   | 0003 | 到达扩圈点即视为导航成功；删除 static-blocked 判定 | `navigation_executor_core.py:271-276`、`navigation_goal_resolver.py:177-184` |
   | 0004 | nav2 单次失败静默重试（失败消抖） | `nav2_action_bridge.py:14-17` |
   | 0005 | 禁用 spin_until_future_complete；统一 future 等待工具 | `dock_navigator.py:129-147`、`nav2_action_bridge.py:69-71` |
   | 0006 | 入位先等停稳再测量与补偿 | `dock_finish.py` 模块 docstring |
   | 0007 | named_poses.json 是点位唯一真源，启动时清理过期点 | `poi_store.py:68-73`、`navigation_executor.py:197-198` |

5. 修文档漂移（R10）：`david_nav/CLAUDE.md` 的 cmd_vel 链路（separator 已停用）与 domain 说明；`README.md` 的 `/home/unitree`、Livox；`start_waypoint_nav_headless.sh:326` 的 "v2" 字样；`dead_reckoning_executor.py:12-13` 的错误说法。**只改文字，不改逻辑**。
6. `CHANGELOG.rst`：`0.1.0 (2026-09-xx)` 首版，列出 R1–R10 修复。

### P8 门禁固化

- `nav_executor/test/test_architecture.py`：跑 import-linter（未安装时 `pytest.skip` 并提示安装命令）+ AST 检查所有 `Node` 子类不覆盖父类属性。
- `colcon test --packages-select nav_executor` 纳入 README 的"提交前必跑"。

**工作量粗估**：P0 M、P1 S、P2 L、P3 L、P4 M、P5 S、P6 M、P7 M、P8 S（S≈半天，M≈1 天，L≈2 天）。P2/P3 风险最高，必须等 P0 金样齐全再开始。

---

## 7. 兼容性不变量（任何阶段都不能破）

| # | 不变量 | 怎么验证 |
|---|---|---|
| I1 | 线协议下行帧逐字节一致（键名、键序、`round(remaining_distance, 3)`、`ensure_ascii=False`、中文错误文案） | golden G1–G16 |
| I2 | 旧 CLI 参数全部可用，语义不变；未知参数透传 ROS | G16 |
| I3 | `david_nav/david_nav/navigation_executor.py` 路径存在、可执行，进程命令行含 `navigation_executor.py` | `test -x` + `pgrep -f navigation_executor.py` |
| I4 | `ros2 run david_nav navigation_executor.py` 仍可启动 | 集成测试 |
| I5 | `~/.ros/david_nav/poi_store.yaml` 读写格式不变，旧文件可直接加载 | 存储金样 |
| I6 | §4.8 冻结日志行不变 | 单测断言 + P0 列出的消费者 |
| I7 | `dock_navigator` 的节点名、参数名、launch 传参方式不变 | `ros2 param list /dock_navigator` 对比 |
| I8 | 旧 import 路径（`david_nav.navigation_protocol` 等）可用到下一个大版本 | shim 导入测试 |
| I9 | 不改 nav2 参数 / BT / C++ / launch 默认值 | `git diff pre-nav-executor-refactor -- david_nav/config david_nav/behavior_trees david_nav/src david_nav/include` 为空（P7 的文字修正除外） |

---

## 8. 验证清单

### 8.1 Codex 可自行完成（不涉及运动）

- [ ] 每阶段门禁通过（§6 开头）
- [ ] golden G1–G16 全绿
- [ ] `test/integration`（`ROS_DOMAIN_ID=99`，假 action server，**不启动底盘**）全绿
- [ ] import-linter 三个契约通过
- [ ] 新包 flake8 / pep257 零告警
- [ ] P4-R6 的 CPU 测量数据写入 ADR
- [ ] `ros2 run nav_executor navigation_executor --help` 与旧 `--help` 输出一致

### 8.2 需要用户在真机执行（Codex 只写步骤，不代劳）

1. 正确启动序列 `3`（lightning，在 rviz 里点 2D Pose Estimate 给初始位姿）→ `4`（生产 headless）。
2. 看 `executor.log` 首行生效配置，确认 `server_uri / action_name / robot_base_frame` 与重构前一致。
3. 从 agent server 发一个**近距离、空旷**的 `navigate_to`，确认收到 `on_progress` → `on_arrived`。
4. 导航中发 `abort_navigation`，确认 ≤ 1 s 收到 `on_progress(paused)` 且车停。
5. 导航中拔掉/断开 agent server 5 s 再恢复，确认日志出现 `connected (… 断联 …s)`，到点后 server 能收到 on_arrived（可能带 `RESENT`）。
6. 发一帧畸形 `mark_current_poi`（缺 poi），确认**连接不断**、日志有记录（R1 回归）。
7. alias `0` 精确入位跑一次，确认 `[dock]` 日志行与重构前格式一致；alias `10` dock_bench 能正常解析。

---

## 9. 回滚方案

- 每个阶段是独立提交（P4 每个缺陷单独一个），任一步出问题 `git revert <sha>`。
- 整体回滚：`git checkout pre-nav-executor-refactor -- david_nav && git rm -r nav_executor`，然后 `colcon build --symlink-install --packages-select david_nav`。
- 因为 P1–P3 期间旧入口始终是可执行的 shim、headless 脚本从未修改，**任何时刻 alias 4 都能用旧行为跑**。

---

## 10. 待用户决策（r2：已按建议落定——D1=A 独立包；D2=否，以当前 Python 3.10 为准，GO2W 为 TODO-V1；D3=A pip 16.1；D4=用户先提交；D5=本轮做；D6=保持 base_link，与 base_footprint 的区别见总方案 §2.3）

| # | 决策点 | 选项 | 建议 |
|---|---|---|---|
| D1 | 新模块形态 | A. 同级独立包 `nav_executor` / B. `david_nav` 内子包 `david_nav/executor/` | **A**：名字不带机器人，其它分支可单独 checkout，依赖可单独声明 |
| D2 | 是否保持 Python 3.8 兼容（为 GO2W / Foxy 复用） | 是 / 否（放开用 3.10 语法） | **是**：代价仅是不能用 TaskGroup/match，已有替代写法 |
| D3 | websockets 以哪个为准 | A. pip 16.1（现在实际在跑，装在 `~/.local`）/ B. apt 9.1（`package.xml` 声明的） | **A**，并在 README 写明安装命令与最低版本；删除全局 asyncio 补丁 |
| D4 | 9 个未提交修改 + 未跟踪的 dock 文件 | 用户先提交 / stash / 部分丢弃 | 用户先提交（它们是生产在用的代码） |
| D5 | dock 结构化事件话题（P6-3） | 本轮做 / 推迟 | 本轮做（只增不删，零风险） |
| D6 | 执行器 `robot_base_frame` 默认值（E4） | 保持 `base_link` / 改 `base_footprint` | 本轮**保持**；改之前先实测两个 frame 在 David 上的 yaw 是否一致（`base` 帧有 IMU 旋转 quirk 的前科），一致就维持现状 |

---

## 附录 A：文件映射总表

| 旧路径（`david_nav/david_nav/`） | 新路径（`nav_executor/nav_executor/`） | 旧路径保留为 |
|---|---|---|
| `navigation_types.py` | `domain/types.py` | shim |
| `grid_map_view.py` | `domain/grid.py` | shim（waypoint_follower 在用） |
| 各处 yaw/wrap 函数 | `domain/geometry.py` | 各文件改为 import（仅执行器与 dock 相关文件；bench 脚本不强制） |
| `navigation_protocol.py` | `protocol/messages.py` + `protocol/codec.py` | shim |
| `navigation_executor_core.py` | `core/executor.py` + `core/state.py` + `core/handlers/*` + `core/retry.py` + `core/supervisor.py` | shim |
| `navigation_goal_resolver.py` | `core/goal_resolution.py` + `adapters/ros/costmap_goal_resolver.py` | shim |
| `nav2_action_bridge.py` | `adapters/ros/nav2_backend.py` + `adapters/ros/futures.py` | shim |
| `pose_provider.py` | `adapters/ros/tf_pose_source.py` | shim |
| `poi_store.py` | `adapters/storage/yaml_poi_repository.py` | shim |
| `navigation_executor.py` | `app/{cli,runtime,config,status,loop_closure,logging}.py` + `adapters/transport/{ws_client,outbox,proxy}.py` + `adapters/storage/named_poses_importer.py` | **可执行 shim（路径是接口）** |
| `dock_settle.py` / `dock_finish.py` | `skills/docking/settle.py` / `finish.py` | shim |
| `dock_navigator.py` | 留在 david_nav，改用新包 | — |
| `headless_config.py` | 留在 david_nav（部署层） | — |
| `test/test_navigation_executor_{protocol,poi,runtime}.py`、`test_nav2_action_bridge.py`、`test_grid_map_view.py`、`test_dock_{settle,finish}.py` | `nav_executor/test/unit/` 下对应文件 | 旧测试在 P3 结束前保留并继续跑，确认新旧一致后删除 |

## 附录 B：参考资料

- Nav2 Simple Commander API — https://docs.nav2.org/commander_api/index.html
- nav2_simple_commander (Humble) — https://docs.ros.org/en/humble/p/nav2_simple_commander/
- Nav2 Docking Server — https://docs.nav2.org/tutorials/docs/using_docking.html ；opennav_docking（含 Humble 分支）— https://github.com/open-navigation/opennav_docking
- Open-RMF fleet adapter template — https://github.com/open-rmf/fleet_adapter_template ；Mobile Robot Fleet Integration — https://osrf.github.io/ros2multirobotbook/integration_fleets.html
- Autoware interface design / AD API — https://www.ncnynl.com/ros2docs/en/autoware/design/autoware-interfaces/ad-api/
- Google Python Style Guide — https://google.github.io/styleguide/pyguide.html
- ROS 2 Code style（Humble 源文件）— https://github.com/ros2/ros2_documentation/blob/humble/source/The-ROS2-Project/Contributing/Code-Style-Language-Versions.rst
- ament_flake8 默认配置 — https://github.com/ament/ament_lint/blob/humble/ament_flake8/ament_flake8/configuration/ament_flake8.ini
- REP 2004 Package Quality Categories — https://ros.org/reps/rep-2004.html
- import-linter Contract types — https://import-linter.readthedocs.io/en/latest/contract_types.html
- AsyncAPI WebSocket 教程 — https://www.asyncapi.com/docs/tutorials/websocket
- Martin Fowler, Strangler Fig Application — https://martinfowler.com/bliki/StranglerFigApplication.html
- Python asyncio.create_task（任务弱引用注记）— https://docs.python.org/3/library/asyncio-task.html
- PEP 544 Protocols — https://peps.python.org/pep-0544/
