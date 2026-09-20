# 重构入口的快捷命令编号

用户决定：**现有快捷命令一律不动**，重构后的入口一律用新编号。原则是"原编号 + 10"；
其中重构后的 `0` 因为 `10` 已被入位测评台架（dock_bench）占用，用户 2026-09-18 改定为 **`20`**；
2026-09-20 用户把 `4` 的重构入口也定为 **`24`**，即实际口径是**原编号 + 20**。

| 现有编号 | 现在是什么 | 重构后编号 | 重构入口命令 | 状态 |
|---|---|---|---|---|
| `0` | 精准入位导航（`precise_nav_bringup.launch.py`，含 8101 上位机接口） | **`20`** | `navrun up --mode precise` | **已加入 `~/.bashrc`（2026-09-18）** |
| `4` | 无界面导览导航 | **`24`** | `navrun up --mode tour` | **已加入 `~/.bashrc`（2026-09-20）** |
| `5` | 底盘（驱动 + 速度适配） | `15` | `navrun up --mode chassis` | 待加 |
| `5k` | 键盘遥控（PLAN §13.6 的计划编号） | `15k` | `navrun up --mode teleop` | 待加 |
| `7` / `8` | 控制器调参台架 | `17` | `navrun up --mode bench_slamgt` | 待加 |

## 已部署的两条

```bash
alias 20="export ROS_DOMAIN_ID=0;set -o allexport;source ~/ros2_ws/src/david_nav/config/ros_env.conf;set +o allexport;source /opt/ros/humble/setup.bash && source ~/ros2_ws_refactor/install/setup.bash;ros2 run nav_bringup navrun up --mode precise --takeover"
alias 24="export ROS_DOMAIN_ID=0;set -o allexport;source ~/ros2_ws/src/david_nav/config/ros_env.conf;set +o allexport;source /opt/ros/humble/setup.bash && source ~/ros2_ws_refactor/install/setup.bash;ros2 run nav_bringup navrun up --mode tour --takeover"
```

## 为什么必须加载 `ros_env.conf`（2026-09-18 实机定位到的故障）

首次实机启动（未加载该文件）时 Nav2 卡在生命周期启动的第一个节点：地图服务器配置成功，但它的 `change_state` 响应发不回管理器（`failed to send response ... client will not receive response`），管理器无限等待，后面的代价地图、规划器、控制器全部没有配置激活，RViz 因此空白。

机制：定位（`3`）与 RViz 都加载了白名单 XML，只在回环与有线网卡上通告；未加载该文件的导航侧会在包括 WiFi 在内的所有网卡上通告。两边的通告集合不对称时，服务的请求方向先配上、响应方向还没配上，响应就被丢掉了。

证据：不加载时 1 次启动卡死；加载后连续 3 次启动分别在 15 / 21 / 18 秒内完成全部激活，`failed to send response` 0 次，全局代价地图正常发布，`map` 到导航基准帧的变换正常，停止后无残留进程。

注意：现有生产 `0` 同样没有加载这个文件（`4` 的 headless 脚本有加载）。这是一处潜在风险，但本次没有改动生产，交由用户决定。

- 环境与现有 `0` 的区别：除了上面这一条（`20` 会加载 DDS 环境文件），其余逐项对齐——domain 0、只 source 一个工作区，区别只在于工作区换成
  `~/ros2_ws_refactor`，入口换成重构后的 `navrun up`。
- `~/.bashrc` 改动前已备份到 `~/david_nav_backups/20260918_alias20/bashrc.bak`；
  改动前 md5 `695bd38f807e689f084ed249ae855536`，改动后 `113291af807cdd888eed0f9d191caa6b`。
- 只新增了一行 alias 与三行注释，现有 `0`/`4`/`5` 等未做任何修改。

## DDS 配置：已核对，不再是阻塞项

重构工作区的 `david_system_env/config/ros_env.conf` 与生产 `david_nav/config/ros_env.conf`
内容一致，都指向同一份生产白名单 XML `/home/agx/ros2_ws/src/david_nav/config/fastdds_david.xml`。
现有 `0` 本身并不加载该文件，`20` 与它保持一致；若实机出现"看不到定位节点"的现象，
先按 RUNBOOK 加载 `ros_env.conf` 再排查，而不是改 domain 或删白名单。

## 还没做的事

`20` 目前只验证到：别名在交互式 shell 里能正确展开；`navrun up --mode precise --dry-run`
能解析出完整进程图并退出码 0；离线体检 OK 5 / WARN 2（DDS 变量未设、ROS 日志目录过大）/ FAIL 0。
**从未在实机上真正启动过**，实机冒烟按 PLAN §16.3 进行，需要先由现场启动 `3`。

## 回退

现有 `0` / `4` / `5` 一个字都没改；`20` 出任何问题，直接回去用 `0`。
要整段撤掉 `20`，删除 `~/.bashrc` 末尾那段带 `2026-09-18` 注释的四行即可，
或用上面的备份文件覆盖回去。

## 用 `24` 之前要知道的两件事（2026-09-20）

1. `--takeover` 只对**重构版**入口有效：它请的是持有 navrun 租约的那一方。生产的 `0` / `4`不持有该租约，`takeover` 会报 inactive 直接放行。
2. 但启动前的预检**会**按既有 kill 规则停掉生产导览栈（`launch david_nav 2d_david_nav2_bringup` 及其 nav2 节点）。也就是说 `4` 正在跑时敲 `24`，生产那套会被停掉换成重构版——这是既定设计，不是意外。
   底盘驱动若由现场单独起着，预检会把它识别为外部驱动并保护，不会停它。
