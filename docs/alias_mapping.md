# 重构入口的快捷命令编号

用户决定：**现有快捷命令一律不动**，重构后的入口一律用新编号。原则是"原编号 + 10"；
其中重构后的 `0` 因为 `10` 已被入位测评台架（dock_bench）占用，用户 2026-09-18 改定为 **`20`**。

| 现有编号 | 现在是什么 | 重构后编号 | 重构入口命令 | 状态 |
|---|---|---|---|---|
| `0` | 精准入位导航（`precise_nav_bringup.launch.py`，含 8101 上位机接口） | **`20`** | `navrun up --mode precise` | **已加入 `~/.bashrc`（2026-09-18）** |
| `4` | 无界面导览导航 | `14` | `navrun up --mode tour` | 待加 |
| `5` | 底盘（驱动 + 速度适配） | `15` | `navrun up --mode chassis` | 待加 |
| `5k` | 键盘遥控（PLAN §13.6 的计划编号） | `15k` | `navrun up --mode teleop` | 待加 |
| `7` / `8` | 控制器调参台架 | `17` | `navrun up --mode bench_slamgt` | 待加 |

## 已部署的那一条

```bash
alias 20="export ROS_DOMAIN_ID=0;source /opt/ros/humble/setup.bash && source ~/ros2_ws_refactor/install/setup.bash;ros2 run nav_bringup navrun up --mode precise"
```

- 环境与现有 `0` 逐项对齐：同样是 domain 0、同样只 source 一个工作区，区别只在于工作区换成
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
