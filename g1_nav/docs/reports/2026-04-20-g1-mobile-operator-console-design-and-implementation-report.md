# G1 手机操作台设计与实现报告

> 手机浏览器启停 `start_navigation_headless.sh` 并实时查看 RViz 等效可视化的
> 完整实现方案。基于 g1_nav 现有代码构建，不改动 ROS 2 主通信链路。

---

## 1. 概述

### 1.1 目标

在 **Unitree G1 + Ubuntu 22.04 + ROS 2 Humble + Cyclone DDS** 环境中，构建一套
面向手机浏览器的移动操作台，实现：

1. **手机启停导航栈** —— 在手机浏览器上点击按钮，启动或停止
   `start_navigation_headless.sh`（TF + Lightning SLAM + Nav2 + 前沿探索 +
   Navigation Executor 全链路）
2. **手机查看可视化** —— 在手机浏览器上看到与 RViz 等效的地图、机器人位姿、
   路径规划、全局/局部代价地图、激光扫描
3. **不改动已有实现** —— 复用 `start_navigation_headless.sh` 的进程编排逻辑
   和 `navigation_executor.py` 的 WebSocket 协议

### 1.2 非目标

本方案明确不做：

- 跨公网访问（仅局域网）
- 手机原生直接加入 Cyclone DDS 图
- 安卓 APK 原生封装
- 多用户权限系统
- 完整替代 RViz2 的高级交互（2D Pose Estimate、插件调试等）

### 1.3 约束

- 手机与机器人在同一局域网
- 使用浏览器/PWA，不先做原生 APK
- 不改 ROS 2/Cyclone DDS 主通信链路

---

## 2. 现状分析

### 2.1 g1_nav 已有的能力

本方案不是从零开始。`g1_nav` 已经实现了关键基础设施：

#### 2.1.1 `start_navigation_headless.sh` —— 进程编排器

这是一个单 shell 多进程管理器，按顺序启动 4 个子进程：

| 序号 | 名称 | 命令摘要 | 作用 |
|------|------|----------|------|
| 1 | `map->odom tf` | `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom` | 发布 map→odom 静态 TF |
| 2 | `slam2` | `ros2 run lightning run_slam_online --config default_livox.yaml` | Lightning SLAM 在线建图 |
| 3 | `auto` | `ros2 launch g1_nav g1_auto_explore.launch.py use_rviz:=false enable_realsense_scan_bridge:=true enable_exploration:=true` | Nav2 全栈 + 前沿探索 + Collision Monitor + g1_move |
| 4 | `navigation executor` | `ros2 run g1_nav navigation_executor.py --server-uri ws://...` | WebSocket 客户端，接收远程导航指令 |

关键工程特性：

- 使用 `setsid` 为每个子进程建立独立进程组，停止时整组 SIGTERM/SIGKILL
- Ctrl+C 触发有序清理：TERM → `SHUTDOWN_GRACE_SECONDS` 等待 → KILL
- 启动前会 enable cpu4-cpu7、清空 POI store、校验 workspace 和 YAML 路径
- 支持 `--dry-run` 预览所有命令
- 默认 `ROS_DOMAIN_ID=30`

#### 2.1.2 `navigation_executor.py` —— WebSocket 控制协议

已实现机器人侧 WebSocket 客户端，协议完备。

**入站命令（服务端 → 机器人）：**

| action | 关键字段 | 功能 |
|--------|----------|------|
| `navigate_to` | `request_id`, `sub_id`, `target_id` | 导航到指定 POI |
| `abort_navigation` | `request_id` | 中止当前导航 |
| `mark_current_poi` | `request_id`, `sub_id`, `poi.id`, `poi.name` | 标记当前位置为 POI |
| `update_poi_list` | `version`, `poi_list` | 同步 POI 语义目录 |

**出站事件（机器人 → 服务端）：**

`on_progress` / `on_arrived` / `on_error` / `on_mark_poi_ack` /
`on_mark_poi_success` / `on_mark_poi_error`；30 秒心跳 `ping`/`pong`。

**当前连接目标：** `ws://172.16.21.205:8100/ws/navigation/executor`

#### 2.1.3 已在运行的 ROS 2 topic

```text
# TF
/tf, /tf_static

# SLAM（Lightning）
/lightning/grid_map           # 实时 OccupancyGrid
/lightning/odometry           # Odometry

# Nav2
/map, /plan, /local_plan, /goal_pose
/global_costmap/costmap, /global_costmap/costmap_updates
/local_costmap/costmap,  /local_costmap/costmap_updates

# 传感器
/scan                         # LaserScan（pointcloud 或 realsense 桥接）
/utlidar/cloud_livox_mid360

# 控制
/cmd_vel, /cmd_vel_nav, /cmd_vel_safe

# 前沿探索
/explore/frontiers            # MarkerArray
/explore/resume
```

### 2.2 缺什么

相对于目标，缺：

1. **HTTP API 启停入口** —— 目前只能 SSH 进去执行 shell 脚本
2. **ROS ↔ Web 可视化桥** —— 手机浏览器没法直接订阅 DDS topic
3. **手机友好的 UI** —— 需要一个移动端控制面板
4. **自启动** —— 机器人开机后相关服务应自动起来

本报告的工作就是补齐这 4 块。

---

## 3. 架构设计

### 3.1 分层架构

```text
┌─────────────────────────────────────────────────────────────┐
│ 手机浏览器                                                    │
│ ┌─────────────────────────┐  ┌───────────────────────────┐  │
│ │ 控制面板                  │  │ Foxglove Web              │  │
│ │ (HTML/JS，由 G1 托管)     │  │ (app.foxglove.dev)        │  │
│ └────────────┬────────────┘  └────────────┬──────────────┘  │
│              │ HTTP                       │ WebSocket        │
└──────────────┼────────────────────────────┼──────────────────┘
               │                            │
               ▼                            ▼
┌─────────────────────────────────────────────────────────────┐
│ G1 机器人 (Ubuntu 22.04 + ROS 2 Humble)                     │
│                                                             │
│ ┌──────────────────────┐   ┌──────────────────────────────┐ │
│ │ launch_manager.py    │   │ foxglove_bridge              │ │
│ │ :8080 HTTP API +     │   │ :8765 WebSocket              │ │
│ │ 静态文件托管           │   │ (ros-humble-foxglove-bridge) │ │
│ └──────────┬───────────┘   └──────────────┬───────────────┘ │
│            │ subprocess                   │ DDS subscribe   │
│            ▼                              ▼                 │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ start_navigation_headless.sh                            │ │
│ │  ├─ tf publisher                                        │ │
│ │  ├─ slam2 (Lightning)                                   │ │
│ │  ├─ auto (Nav2 + Frontier + CollisionMonitor + g1_move) │ │
│ │  └─ navigation_executor (ws://172.16.21.205:8100)       │ │
│ └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 端口与地址

| 端口 | 组件 | 协议 | 说明 |
|------|------|------|------|
| 8080 | launch_manager | HTTP | 控制面板 + REST API |
| 8765 | foxglove_bridge | WebSocket | Foxglove topic 桥接 |
| 8100 | navigation_executor → 服务端 | WebSocket（出站）| 已有，保持不变 |

### 3.3 两层控制的职责分离

新增的 `launch_manager` 和已有的 `navigation_executor` 是两层不同的控制，
职责明确区分：

| 维度 | launch_manager | navigation_executor |
|------|----------------|---------------------|
| 控制粒度 | 整个导航栈的启停 | 单次导航任务的下发 |
| 协议 | HTTP REST | WebSocket JSON |
| 类比 | 开关机 | 遥控器 |
| 端口 | 8080 (入站) | 8100 (出站) |
| 本次是否新增 | **新增** | **保持不变** |

用户完整操作流程：

1. 手机打开 `http://G1_IP:8080` 控制面板
2. 点击"启动导航" → launch_manager 调用 `start_navigation_headless.sh`
3. 脚本内部自动启动 navigation_executor，连接 `ws://172.16.21.205:8100`
4. 手机打开 Foxglove Web，连接 `ws://G1_IP:8765`，看到实时可视化
5. 手机 App（或服务端）通过已有 WebSocket 协议发送 `navigate_to`
6. 任务完成后，手机点击"停止导航" → 整栈停止

### 3.4 为什么这样架构

#### 为什么不用 Node-RED

**实际需求极简**：启停一个 shell 脚本 + 查看状态 + 看日志。一个轻量
Python HTTP 服务足够，省去 Node.js/Node-RED 的安装、维护、升级负担。

#### 为什么用 Foxglove，不自己写渲染器

Foxglove 原生支持 OccupancyGrid、Path、LaserScan、TF、MarkerArray、
`costmap_updates` 增量流，手机浏览器打开即可用。自写 WebGL 渲染器要
几周时间，性价比极低。

#### 为什么不用 rosbridge_suite

`rosbridge_suite` 基于 JSON，对 OccupancyGrid 这类大消息性能差。
Foxglove Bridge 是 C++ 实现，协议是二进制 CDR，官方推荐优先选它。

#### 为什么不让 navigation_executor 多兼一职

`navigation_executor` 是"导航指令执行器"，与"进程管理"是不同抽象层。
把启停 shell 脚本的能力塞进 executor 会让它既管得着 bash 又管得着 Nav2，
单点故障面过大。分离后 executor 的语义仍然是"机器人侧导航指令客户端"。

---

## 4. 模块实现

### 4.1 Module A: launch_manager.py

一个单文件 Python HTTP 服务，管理 `start_navigation_headless.sh` 的生命周期。

**依赖：** Python 3.8+、PyYAML（G1 默认自带）。无第三方 Web 框架依赖。

**核心类：**

- `ManagedProcess` —— 单个 shell 脚本的子进程封装，包含状态机、日志缓冲、
  SIGTERM 级联清理
- `ProcessManager` —— 多 profile 管理器
- `LaunchManagerHandler` —— `http.server.BaseHTTPRequestHandler` 子类，实现
  REST API + 静态文件托管

**HTTP API：**

```text
GET  /api/health                              # 所有 profile 的状态汇总
GET  /api/profiles                            # profile 列表
GET  /api/profiles/{name}/status              # 单个 profile 状态
GET  /api/profiles/{name}/logs?tail=N         # 最近 N 行日志
POST /api/profiles/{name}/start               # 启动（可选 body 覆盖选项）
POST /api/profiles/{name}/stop                # 停止
GET  /                                        # 静态前端（index.html）
```

**状态响应示例：**

```json
{
  "profile": "navigation",
  "state": "running",
  "pid": 12345,
  "started_at": "2026-04-20T15:20:00+00:00",
  "uptime_seconds": 3600.2
}
```

**关键实现要点：**

- **进程组管理**：子进程通过 `subprocess.Popen(..., start_new_session=True)`
  独立建立 session，停止时 `os.killpg(pgid, SIGTERM)` 触发脚本自身的 trap
  清理逻辑，保证 ROS 子进程被级联回收
- **跨平台**：`os.killpg`/`os.getpgid` 仅 POSIX 可用，Windows 开发时回退到
  `process.terminate()`，使开发机上的单测也能通过
- **日志捕获**：后台线程从 `stdout`（合并了 stderr）逐行读入
  `collections.deque(maxlen=2000)`，ring buffer 防止内存泄漏
- **防重复启动**：`start()` 时检查 `self.process.poll()` 是否仍在运行
- **白名单命令**：只执行 `profiles.yaml` 中预定义的命令，不接受任意 shell
- **CORS**：响应头包含 `Access-Control-Allow-Origin: *`，便于手机浏览器访问
- **优雅退出**：SIGTERM/SIGINT 时先 `manager.stop_all()` 再关闭 HTTP 服务

### 4.2 Module B: profiles.yaml

Profile 白名单，当前定义 4 个：

| profile 名 | 用途 |
|-----------|------|
| `navigation` | 完整导航栈（推荐默认） |
| `navigation_no_explore` | 无前沿探索，手动下发目标 |
| `explore_with_rviz` | 带 RViz，用于有显示器的场景 |
| `explore_headless` | SLAM + Nav2 + 探索，不含 executor |

每个 profile 声明：命令路径、描述、额外环境变量。

### 4.3 Module C: web/index.html —— 手机控制面板

单个 HTML 文件，纯原生 JS + CSS，无构建工具。

**页面结构：**

```text
┌──────────────────────────────────────────┐
│  G1 Navigation Console                    │
├──────────────────────────────────────────┤
│  Status                                   │
│  ● Running    PID 12345   1h 23m         │
├──────────────────────────────────────────┤
│  Profile                                  │
│  [navigation ▾]                           │
│  Full navigation stack: ...               │
│  [  Start  ]  [  Stop  ]                 │
├──────────────────────────────────────────┤
│  Logs                                     │
│  [start_navigation_headless.sh] ...       │
│  [start_navigation_headless.sh] ...       │
├──────────────────────────────────────────┤
│  Visualization                            │
│  [ Foxglove Web → ws://G1_IP:8765 ]      │
├──────────────────────────────────────────┤
│  Services                                 │
│  ● navigation  ● explore_headless         │
└──────────────────────────────────────────┘
```

**设计要点：**

- **暗色主题**：降低户外强光下的眩光
- **大按钮 + 触控安全距离**：适合手指操作
- **实时轮询**：状态每 2s 刷新、日志每 3s 刷新
- **Foxglove 链接**：自动使用 `window.location.hostname` 拼接 ws URL
- **Meta 标签**：`viewport` 固定缩放、`apple-mobile-web-app-capable` 支持
  "添加到主屏幕"后全屏运行
- **错误态**：连接失败时显示 Disconnected，不白屏

### 4.4 Module D: web/foxglove-layout.json

Foxglove 预设布局，主 panel 为 3D 视图，订阅所有关键可视化 topic：

| Topic | 可视化形式 |
|-------|-----------|
| `/lightning/grid_map` | OccupancyGrid 底图 |
| `/global_costmap/costmap` | 代价地图叠加 |
| `/local_costmap/costmap` | 局部代价地图 |
| `/scan` | 激光点云 |
| `/plan` | 蓝色全局路径 |
| `/local_plan` | 黄色局部路径 |
| `/goal_pose` | 目标点箭头 |
| `/explore/frontiers` | 前沿 MarkerArray |
| `/tf`, `/tf_static` | TF 树 |

视角设置为 `follow-pose`，跟随 `base_link`。

### 4.5 Module E: systemd 服务

#### foxglove-bridge.service

```ini
[Unit]
Description=Foxglove Bridge for ROS 2 Humble
After=network-online.target

[Service]
Type=simple
User=unitree
Environment=ROS_DOMAIN_ID=30
ExecStart=/bin/bash -lc "source /opt/ros/humble/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

#### launch-manager.service

```ini
[Unit]
Description=G1 Navigation Launch Manager
After=network-online.target

[Service]
Type=simple
User=unitree
WorkingDirectory=/home/unitree/ros2_ws/src/g1_nav/operator_console
ExecStart=/usr/bin/python3 /home/unitree/ros2_ws/src/g1_nav/operator_console/launch_manager.py --port 8080
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### 4.6 Module F: deploy.sh

一键部署脚本，三步：

1. `rsync` 同步 `operator_console/` 到 G1
2. 检查并安装 `ros-humble-foxglove-bridge` 和 `python3-yaml`
3. 安装/启用/重启 systemd 服务

支持 `deploy.sh --check` 预览所有将要执行的操作。

---

## 5. 接口契约

### 5.1 可视化 Topic 契约

通过 foxglove_bridge 必须可见的 topic：

```yaml
required:
  - /tf                              # TransformStamped[]
  - /tf_static                       # TransformStamped[]
  - /lightning/grid_map              # OccupancyGrid
  - /scan                            # LaserScan
  - /plan                            # Path（全局）
  - /local_plan                      # Path（局部）
  - /global_costmap/costmap          # OccupancyGrid
  - /local_costmap/costmap           # OccupancyGrid

optional:
  - /global_costmap/costmap_updates  # OccupancyGridUpdate
  - /local_costmap/costmap_updates   # OccupancyGridUpdate
  - /explore/frontiers               # MarkerArray
  - /lightning/odometry              # Odometry
  - /goal_pose                       # PoseStamped
  - /cmd_vel_safe                    # Twist
```

### 5.2 Launch Manager API 契约

所有响应 Content-Type: `application/json; charset=utf-8`。

**成功：**

```json
{"ok": true, "pid": 12345}
```

**失败：**

```json
{"ok": false, "error": "Navigation is already running"}
```

HTTP 状态码：200 成功，404 profile 不存在，409 状态冲突（例如重复启动）。

### 5.3 navigation_executor 协议契约

**不变。** 详见
`docs/reports/2026-04-15-g1-navigation-executor-architecture-and-server-protocol-report.md`。

---

## 6. 安全设计

| 层级 | 机制 | 实现位置 |
|------|------|----------|
| 命令执行 | 白名单 profile，只能运行 `profiles.yaml` 中的命令 | launch_manager.py |
| 网络暴露 | 仅监听内网，不暴露公网 | systemd + 防火墙 |
| 跨域 | CORS 明确允许 OPTIONS | launch_manager.py |
| 停止策略 | SIGTERM → grace 等待 → SIGKILL，避免孤儿进程 | ManagedProcess.stop() |
| 审计 | 每次启停记录到 systemd journal | logger.info |

---

## 7. 开发环境划分：本地 Windows vs G1 机器人

这是本项目的**核心工程决策**：严格区分"可脱离硬件抽象的"和"强依赖
硬件/ROS 的"两类工作，让 Phase 1 可以在没有 G1 的时候完整推进。

### 7.1 本地 Windows 已完成的部分

| 任务 | 产出 |
|------|------|
| HTTP 服务框架 | launch_manager.py 主体代码 |
| 进程管理逻辑 | ManagedProcess 状态机、日志 ring buffer |
| Profile 注册表 | profiles.yaml + `load_profiles()` |
| REST API | start/stop/status/logs/health 6 个端点 |
| CORS 与安全头 | HTTP 中间件 |
| 单元测试 | 17 个测试，全部通过（跨平台 echo/ping/sleep）|
| HTTP 集成测试 | 本地端口启动服务 + http.client 调用 |
| 手机 UI 骨架 | web/index.html（纯 vanilla JS）|
| Foxglove layout | web/foxglove-layout.json |
| systemd 模板 | 2 份 .service 文件 |
| 部署脚本 | deploy.sh（含 `--check` 预览模式）|

**验证方式：**
`cd operator_console && python test_launch_manager.py -v`
在 Windows 上即可跑通 17/17 测试。

### 7.2 必须在 G1 机器人上完成的部分

| 任务 | 为何必须本地 |
|------|-------------|
| 安装 `ros-humble-foxglove-bridge` | G1 的 apt source |
| 验证 `/lightning/grid_map` 能否被 Foxglove 渲染 | Lightning SLAM 实际消息类型 |
| 验证 Nav2 costmap `_updates` 是否正常配对 | Nav2 运行时行为 |
| 验证 QoS 兼容性 | DDS 运行时配置 |
| `ros2 topic list` 审计 topic 命名 | 实际 ROS 图 |
| SIGTERM 停止后所有 ROS 进程是否被回收 | 进程组管理在真实环境 |
| systemd enable/start 并验证自启 | G1 的 systemd 环境 |
| 手机 Wi-Fi 连接体验 | 实际网络和带宽 |
| 大带宽 topic 对手机流畅度的影响 | costmap/scan 数据量 |

### 7.3 开发边界判断原则

> **能脱离硬件抽象出来的，都可以在远程仓库推进；
> 任何和"真实 ROS 图、真实设备、真实网络、真实性能"绑定的，
> 都必须回到机器人本地。**

---

## 8. 部署与验证

### 8.1 G1 上的一次性部署

```bash
# 1. 拉取代码
cd ~/ros2_ws/src/g1_nav && git pull

# 2. 预览部署步骤
cd operator_console && bash deploy.sh --check

# 3. 实际部署（如果从开发机远程推送，第一个参数是 G1 IP）
bash deploy.sh 172.16.22.130
```

### 8.2 健康检查

```bash
# HTTP API 健康
curl http://G1_IP:8080/api/health

# Foxglove bridge 是否可达
nc -z G1_IP 8765

# systemd 服务状态
systemctl status foxglove-bridge launch-manager
```

### 8.3 手机验证清单

- [ ] 打开 `http://G1_IP:8080`，看到控制面板
- [ ] 点击"Start"，状态变为 Running，PID 显示
- [ ] 日志区域有 `start_navigation_headless.sh` 输出
- [ ] 打开 Foxglove Web，连接 `ws://G1_IP:8765`，粘贴
  `foxglove-layout.json` 后看到：
  - [ ] 实时地图（/lightning/grid_map）
  - [ ] 机器人位置和朝向（TF 跟随 base_link）
  - [ ] 全局路径（蓝色）/ 局部路径（黄色）
  - [ ] 代价地图叠加
  - [ ] 激光扫描
- [ ] 点击"Stop"，所有 ROS 进程被清理（`pgrep -af ros2` 应为空）
- [ ] Wi-Fi 断开重连后，控制面板和 Foxglove 均能自动恢复

---

## 9. 风险与对策

### 9.1 风险：`/lightning/grid_map` 非标准 OccupancyGrid

Lightning SLAM 的地图消息类型若为自定义类型，Foxglove 无法直接渲染。

**对策：** 本地第一步验证。不兼容则新增一个轻量转换节点（发布到 `/map` 为
标准 `nav_msgs/OccupancyGrid`），加入 `start_navigation_headless.sh` 启动序列。

### 9.2 风险：QoS 不匹配

foxglove_bridge 订阅失败通常是 QoS 不匹配（reliable vs best-effort）。

**对策：** foxglove_bridge 默认会尝试多种 QoS。若仍失败，用
`ros2 topic info -v <topic>` 审计发布方 QoS，在 bridge 侧配置匹配。

### 9.3 风险：SIGTERM 无法回收所有进程

`start_navigation_headless.sh` 虽然用 `setsid` 分组，但 `ros2 launch`
内部的 Python 节点可能对 SIGTERM 响应慢。

**对策：** 脚本自身已有 TERM → 等待 → KILL 的二段清理，可调大
`SHUTDOWN_GRACE_SECONDS`。`start_g1_auto_explore.sh` 还有
`FALLBACK_KILL_REGEX` 做兜底。launch_manager 停止时也走同样的进程组路径，
语义一致。

### 9.4 风险：Foxglove Web 依赖外网

`app.foxglove.dev` 需要从互联网加载前端资源，数据链是直连 G1。
如果现场只有机器人局域网（无互联网出口），Foxglove Web 打不开。

**对策：**
- A 级：手机双网（4G + Wi-Fi），流量走蜂窝，数据直连 Wi-Fi
- B 级：自托管 Foxglove Studio 静态文件到 G1 nginx，完全离线可用
- C 级：备用方案使用 Foxglove Desktop（笔记本）

### 9.5 风险：navigation_executor 的 server-uri 固定

当前硬编码 `ws://172.16.21.205:8100`，网络拓扑变化会连接失败。

**对策：** launch_manager 启动时通过环境变量或 profile 字段覆盖
`--server-uri`，profiles.yaml 预留扩展。

### 9.6 风险：Wi-Fi 带宽对 costmap/scan 的影响

全分辨率 costmap 可能每帧数百 KB，低带宽 Wi-Fi 下手机体验会卡顿。

**对策：**
- 先观察实际带宽占用（Foxglove 右下角有吞吐指标）
- 必要时在 Nav2 侧降低 `global_costmap` 更新频率，或改用 `_updates` 增量流
- Foxglove layout 中大带宽 panel 可默认关闭

---

## 10. 后续演进路线

| 阶段 | 能力 |
|------|------|
| v1（本次）| 单机一键启停 + Foxglove 可视化 |
| v2 | Profile 参数化（body 动态覆盖 `--server-uri` 等）|
| v3 | PWA 化，支持"添加到主屏幕"和离线控制缓存 |
| v4 | 自托管 Foxglove Studio，完全离线 |
| v5 | 多机器人注册中心，一个 Web 管多台 G1 |
| v6 | 权限分级（Operator / Developer / Expert）|

---

## 11. 文件清单

本次新增位于 `operator_console/`：

```text
operator_console/
├── launch_manager.py           # HTTP API 主程序（~330 行）
├── profiles.yaml               # 4 个白名单 profile
├── test_launch_manager.py      # 17 个单元/集成测试
├── deploy.sh                   # 一键部署脚本（支持 --check）
├── web/
│   ├── index.html              # 手机控制面板
│   └── foxglove-layout.json    # Foxglove 预设布局
└── systemd/
    ├── foxglove-bridge.service
    └── launch-manager.service
```

不改动任何现有文件。`start_navigation_headless.sh` 保持不变。

---

## 12. 验收标准

本方案 v1 合格的充要条件：

1. [x] Windows 开发机上 `python test_launch_manager.py` 全部通过（17/17）
2. [ ] G1 上 `deploy.sh` 执行成功，两个 systemd 服务处于 active
3. [ ] `curl http://G1_IP:8080/api/health` 返回 200
4. [ ] 手机浏览器打开控制面板，能看到 profile 列表和状态
5. [ ] 点击 Start 后，`start_navigation_headless.sh` 在 G1 上运行，`ros2 node list` 能看到 SLAM / Nav2 / executor 节点
6. [ ] 点击 Stop 后，所有 ROS 进程被清理
7. [ ] Foxglove Web 连接成功，能看到 map + pose + path + costmap + scan 五项
8. [ ] 断网重连后两条链路均能自动恢复
9. [ ] G1 重启后 foxglove_bridge 和 launch_manager 自启

当前状态：**第 1 项已完成，第 2-9 项待 G1 本地验证。**

---

## 13. 参考

- Foxglove ROS 2 接入文档：https://docs.foxglove.dev/docs/getting-started/frameworks/ros2
- Foxglove Bridge：https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge
- Foxglove 3D panel：https://docs.foxglove.dev/docs/visualization/panels/3d
- ROS 2 Launch：https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
- 本项目相关报告：
  - `2026-04-15-g1-navigation-executor-architecture-and-server-protocol-report.md`
  - `2026-04-16-headless-navigation-cpu-profiling-report.md`
