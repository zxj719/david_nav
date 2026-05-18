# david_nav 未提交改动记录 (2026-05-18)

本文件记录 `feature/david_main` 分支上本次提交前积压的 david_nav 改动，便于回溯。

## 1. 移除 Realsense depth-to-scan 本地避障链路

整套基于 Realsense 深度图转 `/scan` 的本地避障方案已下线，导航的 `/scan` 重新只由 Livox 点云桥接提供。

### 删除文件

- `david_nav/config/realsense_depth_to_scan.yaml`
- `david_nav/launch/realsense_depth_to_scan.launch.py`
- `david_nav/plan/realsense_depth_to_scan_principle.md`
- `david_nav/plan/realsense_local_obstacle_nav2_integration_plan.md`
- `david_nav/plan/realsense_scan_cpu_monitor.py`
- `david_nav/test/test_realsense_cpu_monitor.py`
- `david_nav/test/test_realsense_nav2_integration.py`

### 配套清理

- `david_nav/package.xml`：移除 `realsense2_camera` 的 `exec_depend`。
- `david_nav/CMakeLists.txt`：移除 `realsense_nav2_integration_test`、`realsense_cpu_monitor_test` 两个 pytest 注册项。
- `david_nav/launch/2d_david_nav2_bringup.launch.py`：删除 `enable_realsense_scan_bridge` LaunchConfiguration / DeclareLaunchArgument / IncludeLaunchDescription；`scan_observation_active` 直接等于 `enable_scan_bridge`。
- `david_nav/launch/david_auto_explore.launch.py`：移除 `enable_realsense_scan_bridge` 参数透传；`enable_scan_bridge` 默认值改为 `true`。
- `david_nav/david_nav/headless_config.py`、`david_nav/scripts/navigation_headless_config.py`：从运行时配置 / shell 输出里移除 `enable_realsense_scan_bridge`、`HEADLESS_ENABLE_REALSENSE_SCAN_BRIDGE`。
- `david_nav/start_david_auto_explore.sh`：移除 `--realsense / --no-realsense / -s` 命令行选项与 `REALSENSE` 变量；从 `FALLBACK_KILL_REGEX` 中去掉 `realsense_depth_to_scan` 和 `depthimage_to_laserscan_node`。
- `david_nav/navigate_semantic_pose_with_debug_bag.sh`：移除 `--realsense / --no-realsense` 命令行选项与 `REALSENSE` 变量；bringup 命令固定 `enable_scan_bridge:=true`。
- `david_nav/start_navigation_headless.sh`：移除 `--rviz / --no-rviz / --realsense / --no-realsense` 命令行选项；改用 profile 字段驱动 USE_RVIZ / USE_SCAN_BRIDGE / USE_EXPLORATION，并在日志中打印这三项。
- `david_nav/README.md`：同步移除 Realsense 相关章节，明确局部避障输入仅来自 Livox 点云。
- `david_nav/config/navigation_headless.yaml`：移除 `enable_realsense_scan_bridge` 字段；删除 `navigation_lidar` 这一冗余 profile；`navigation` / `navigation_no_explore` 的 `enable_scan_bridge` 全部改为 `true`。

### 设计取舍

旧方案在 Jetson 上 CPU 占用偏高，且与 Lightning SLAM 的 `/scan` 时间戳同步常出现抖动，触发 DWB 控制器对障碍物的虚警。下线 Realsense 深度桥接后，导航对动态障碍物的反应仅依赖较快的重规划，README 已同步更新已知局限。

## 2. navigation_executor.py 增加 WebSocket 自动重连

`david_nav/david_nav/navigation_executor.py`：

- 新增 CLI 参数 `--reconnect-delay SEC`（默认 3.0 秒），并加到帮助文本。
- 主循环改为 `while True` 包裹 `websockets.connect(...)`，连接断开 / 失败后等待 `reconnect_delay` 自动重连；过去是单次连接，断开即退出。
- `spin_task.cancel()` 移到外层 `finally` 中，确保多次重连期间不会重复取消已结束的任务。
- `start_navigation_headless.sh` 启动 executor 时透传 `--reconnect-delay "${HEADLESS_RECONNECT_DELAY}"`，对应 `headless_config.py` / yaml 中的 `executor.reconnect_delay`。

目的：使 executor 与 operator-console 等上位服务的 WebSocket 链接具备“先于上位机启动 / 上位机临时不可达”的容错能力，避免 executor 进程因 server 未就绪直接退出。

## 3. start_navigation_headless.sh：profile 化 + 路径泛化

- 删除 `--rviz / --no-rviz / --realsense / --no-realsense` 等命令行覆盖；rviz、scan bridge、exploration 全部从所选 headless config profile 读取。
- 调用 `navigation_headless_config.py` 解析配置时改为先捕获 stdout，再 `eval`，并在解析失败时打印明确错误并退出 1。
- 默认 workspace 路径由硬编码的 `/home/unitree/ros2_ws` 改为 `${HOME}/ros2_ws`；`SEMANTIC_MAP_DIR` 同步改为 `${HOME}/semantic_map`。
- `SUDO_PASSWORD` 默认值由 `123` 改为空字符串，避免在新机器上误用默认值。
- 帮助文本同步更新，强调所有切换都由 profile 控制。
- 启动日志增加 `lidar scan bridge`、`exploration` 两行，便于排查实际跑的是哪个 profile。

## 4. navigate_semantic_pose_with_debug_bag.sh：路径泛化 + 默认开 RViz

- `AUTO_WORKSPACE / SEMANTIC_MAP_DIR / BAG_ROOT` 默认值由 `/home/unitree/...` 改为 `${HOME}/...`。
- `RVIZ` 默认值改为 `true`，便于调试时直接看 RViz。
- 移除 `--realsense / --no-realsense` 参数与帮助文本；bringup 直接传 `enable_scan_bridge:=true`。
- 脚本 mode 由 644 改为 755，可直接执行。

## 5. operator console 改动

`david_nav/operator_console/launch_manager.py`：

- 移除 POST 启动 profile 时对 request body 的运行时覆盖（`no_exploration / no_realsense / rviz`）；行为完全由 `profiles.yaml` 中的 command 决定，避免运行时拼接命令带来的歧义。

`david_nav/operator_console/profiles.yaml`：

- `navigation_no_explore` profile 的 command 改用 `--profile navigation_no_explore`，与 headless config 的 profile 名对齐；不再通过 `--no-exploration` 这种已删除的 CLI 开关来切换。

## 6. navigation_headless.yaml 其它调整

- `executor.server_uri` 由 `ws://127.0.0.1:8101/ws/navigation/executor` 改为 `ws://172.16.22.205:8101/ws/navigation/executor`，对接当前 operator console 所在主机。
- `cleanup.term_timeout_sec` 5 → 10，`cleanup.kill_timeout_sec` 2 → 3，给 Nav2 / SLAM 子进程更宽裕的退出窗口，减少残留进程。
- 删除 `navigation_lidar` profile：lidar 已是默认 scan 源，单独 profile 失去意义。

## 7. poi_store.py 支持 source_path 引导

`david_nav/david_nav/poi_store.py` 中 `LocalPoiStore`：

- 构造函数新增可选参数 `source_path: Path | None`。
- `load()` 先尝试从 `source_path` 读入（视为出厂 / 引导默认值），再叠加从 `path` 读取的覆盖内容。
- 拆出私有方法 `_load_from(path)` 复用读取逻辑。

用途：允许 executor 用 `named_poses.json` 等只读 POI 集合作为初始内容，运行时增删保存到本地可写的 `poi_store.yaml`，两者解耦。

## 8. 未变更但相关的事项

- `semantic_pose_navigator.py` 仅文件权限由 644 改为 755，内容无 diff；本次 commit 会带上该 mode 变更。
- `david_nav/test/test_headless_navigation_script.py`：删除 `enable_realsense_scan_bridge:=false` 断言，并整个移除 `test_headless_navigation_flags_toggle_rviz_realsense_and_exploration` 用例（该用例覆盖的 `--rviz / --realsense / --exploration` CLI 开关已不存在）。
- `navigate_semantic_pose_with_debug_bag.sh` 中 `Usage` 注释里的 `/home/unitree/semantic_map` 也同步替换为 `${HOME}/semantic_map`。
