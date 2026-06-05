# Chassis 驱动跑中崩溃导致 Ctrl+C 不停车 / 末段不旋转对齐

> 2026-05-23 — 分析 `start_navigation_headless.sh` 一次长跑（约 19 分钟后）
> 出现"按 Ctrl+C 后机器人没有停车而继续滑行"与"导航末段方向偏 ~90° 却不旋转
> 对齐"两个现象的根因。

## 1. 故障样本

- Run id: [`~/rosbags/david_nav_headless_20260522_174926/`](file:///home/agx/rosbags/david_nav_headless_20260522_174926)
- 日志：
  - `logs/bringup.log` — nav2 + cmd_vel_separator 全部输出
  - `logs/chassis.log` — wheeltec_robot_node 输出
- 用户主观观感（按导航顺序的最后一个 POI 是 POI_004 `a1_door (5.427, -9.624)`）：
  1. **末段：** 机器人朝向跟目标点夹角越来越大（最后 ~90°+），但底盘并没有
     停下来旋转对齐，而是基本直线前进甚至越走越远。
  2. **Ctrl+C 关停后：** 机器人没立刻停，继续向前滑行。

## 2. 时间轴

UTC（人类时间），由 `bringup.log` / `chassis.log` 的 epoch 转换：

| t (HH:MM:SS) | 事件 |
|---|---|
| 17:49:31 | `wheeltec_robot serial port opened` ([chassis.log:9](../../../../rosbags/david_nav_headless_20260522_174926/logs/chassis.log)) |
| 18:00–18:08 | 多个 POI 正常完成（POI_005..010 各自 `Reached the goal`） |
| **18:08:08** | **`serial::IOException(5) Input/output error` at `unix.cc:488`，wheeltec_robot_node `terminate called` → 进程死亡 (exit code -6)**。最后一帧 serial write：`vx=0.500 vy=0.000 wz=0.000`。 |
| 18:08:53 | nav2 继续 issue 下一段：`new /plan goal in "map": (5.427, -9.624) [70 poses]`（POI_004） — 此时**底盘已死 45 s**，但 nav2 / separator 完全不知道。 |
| 18:09:16 | bt_navigator `RobotEmbeddedInObstacle confirmed (3/3) on local costmap` → cancel goal — 因为机器人在无控状态下直线穿过物理障碍，lidar 已经显示 robot embedded。`controller_server: Goal was canceled. Stopping the robot.` — 但 "Stopping" 实际无效，下游已无人收。 |
| 18:09:31 | 用户 Ctrl+C → `signal_handler(SIGINT/SIGTERM)` 群发，nav2 stack 关停。cmd_vel_separator 出 `ExternalShutdownException` 异常退出。 |
| 之后 | STM32 还保留死前最后一帧 `v=+0.5 w=0`，没有 watchdog 归零（至少这次没看到），机器人继续向前。 |

## 3. 两症状共同根因

### 3.1 cmd_vel 链路从底盘一端断了

`wheeltec_robot_node` 是 `/cmd_vel` 的**唯一**消费者（separator 输出 →
wheeltec 读 → 写串口 → STM32）。它一死：

- `/cmd_vel` 上的所有命令直接丢黑洞 — nav2 / separator 在那 80 多秒里完全
  在自言自语，没人执行。
- `/odom` 也没了 — wheeltec 同进程负责 publish。

### 3.2 STM32 端没有 cmd_vel watchdog（或太长）

STM32 P 控收到的最后一帧是 `v=+0.5 w=0`，固件没有"N ms 没新帧就归零"的
watchdog 兜底，所以一直按 0.5 m/s 跑下去。这是**症状 1（Ctrl+C 不停车）的
直接原因**：关停时刻 cmd_vel 链路根本无效，发不发零都一样。

### 3.3 cmd_vel_separator 没把 /odom stale 报出来

[cmd_vel_separator.py:435-440](../../david_nav/cmd_vel_separator.py) 在
`odom_age > 0.5s` 时会把 `actual_v_safe / actual_w_safe` 强置 0（让 brake
判断不会被 stale 值骗住），但是：

- 日志里 `OUT mode=... act: v=... w=...` 字段打的是 raw `self._actual_v /
  self._actual_w`（[cmd_vel_separator.py:702-707](../../david_nav/cmd_vel_separator.py)），不是 safe 版本。所以 odom 失联了，log 里 act 值仍然是死前最后那个数，看上去一切正常。
- 这次日志里 `act: v=+0.638 w=+0.053` 从 [bringup.log:1885](../../../../rosbags/david_nav_headless_20260522_174926/logs/bringup.log) 到 [bringup.log:1958](../../../../rosbags/david_nav_headless_20260522_174926/logs/bringup.log) **38 秒一字不变**，0 抖动 — 真实 odom 流不可能这么静，但 separator 没标 WARN。

也就是 separator 没有"对外通报 odom 失联"的机制 → 上层 / nav2 / 用户终端
都拿不到这条线索。

### 3.4 末段 90° 偏差不旋转 = nav2 在指挥死掉的链路

POI_004 那段（[bringup.log:1883-1939](../../../../rosbags/david_nav_headless_20260522_174926/logs/bringup.log)）：

```
line 1885  STATE=LINEAR    a2g=+66.5° d2g=3.49m  in=(0.500, 0.500)  act=(0.638, 0.053)
line 1892  STATE=ANGULAR   a2g=+78.1° d2g=3.31m  in=(0.363, 0.500)  act=(0.638, 0.053)
line 1903  STATE=ANGULAR   a2g=+110.6° d2g=3.15m in=(0.363, 0.500)  act=(0.638, 0.053)
line 1922  STATE=ANGULAR   a2g=+134.9° d2g=4.48m in=(0.363, 0.500)  act=(0.638, 0.053)
line 1956  (post-cancel)   a2g=+171.9° d2g=6.86m in=(0.363, 0.000)  act=(0.638, 0.053)
```

- `a2g` 从 +66° 涨到 +172° → 机器人朝向跟目标点夹角持续扩大
- `d2g` 从 3.49m 涨到 6.86m → **机器人远离了目标点**
- 但 act 完全不变 → odom 死了，反馈环全断
- separator 切 ANGULAR mode 并喊 `w=+0.5` — **完全没用**，wheeltec 已死

也就是 nav2 + separator 这次"努力指挥转向对齐"，但全是空喊。

### 3.5 Ctrl+C 关停时也没发零

[start_navigation_headless.sh](../../start_navigation_headless.sh) 的 cleanup
是直接对所有 managed process 群发 SIGTERM → SIGKILL。没有"先发一帧 zero
cmd_vel 再 kill"的兜底。即便这次 chassis 早死了无关紧要，将来 chassis 健在
而 nav2 关停时，这里还是会留一个"最后一帧非零 cmd → 关进程"的窗口。

## 4. 直接原因 vs 深层原因

**直接原因**：`serial::IOException(5) Input/output error` 在 unix.cc:488，
USB-串口侧读写出错。可能是 USB 物理抖动 / EMI / 线缆松 / 内核 USB stack 内
部错误。

**取证盲点 — 这次定位不到硬件根因**：

- `dmesg -T` 当前 ring buffer 最旧 entry 是 2026-06-05，事故 05-22 已经滚出去了。
- `journalctl --since 2026-05-22 18:00 --until 2026-05-22 18:15` 返回 `No entries`。
  根因：`/etc/systemd/journald.conf` 是 `#Storage=auto`，且 `/var/log/journal/`
  目录不存在 ⇒ journald 只保留 runtime（`/run/log/journal/`），重启 + ring
  rotation 后历史全丢。
- **建议**（独立于本次修复）：开 persistent journal：
  ```
  sudo mkdir -p /var/log/journal
  sudo systemd-tmpfiles --create --prefix /var/log/journal
  sudo systemctl restart systemd-journald
  ```
  下次再出现 chassis 抖动可以 `journalctl -k --since ...` 直接取证。

**深层原因**（系统设计层面）：

1. STM32 固件没有 cmd_vel watchdog → 上层一断，底盘按最后帧惯性跑。
2. `start_navigation_headless.sh` 启动后不再监控 chassis 进程是否活着，挂了
   无人察觉。
3. cmd_vel_separator 不报告 /odom stale，关键 fail-mode 被掩盖在"看似正常"
   的日志里。
4. 关停流程没有"send zero cmd_vel before kill"。

## 5. 修复优先级

| # | 修法 | 防御什么 | 难度 |
|---|---|---|---|
| 1 | **STM32 固件加 cmd_vel watchdog**（200-500 ms 没新帧 → 自动归零）| 所有"上层节点死/网络断/SIGKILL"场景全部通杀 | 改 STM32 |
| 2 | `start_navigation_headless.sh` 加 chassis liveness 监控（每 5 s `pgrep wheeltec_robot_node`，挂了立刻让脚本自己 exit）| chassis 中途死掉，user 不再"83 秒后才发现"| 改 .sh，简单 |
| 3 | `cmd_vel_separator` 检测 `odom_age > N s` → log WARN + publish (0,0) 直到 odom 恢复 | 上层 / 用户 / 日志都拿到 fail-mode 信号；同时不再硬推 cmd 把状况掩盖 | 改 py，~10 行 |
| 4 | cleanup 时先 `ros2 topic pub --once /cmd_vel ... linear:0 angular:0`，再 SIGTERM 群发 | 正常关停时刻不让最后一帧非零留在 STM32 | 改 .sh |
| 5 | dmesg 看 18:08:08 附近有没有 `usb disconnect` / `ttyUSB`，定位是硬件还是驱动问题 | 决定要不要先修硬件 | sudo 一下 |

#1 是治本（无论上层怎么挂，底盘自带兜底）。#2-#4 是软件层面纯防御兜底，可
以马上做。

## 5.1 已实装（2026-05-23）

软件兜底三件 (#2 / #3 / #4) 已合入。STM32 watchdog (#1) 留给固件侧。

### `start_navigation_headless.sh`

| 改动 | 函数 / 位置 |
|---|---|
| chassis liveness watcher：每 `CHASSIS_LIVENESS_INTERVAL_SEC`（默认 5s）`kill -0` 一次 wheeltec PID，挂了**先 `kill -USR1` 主脚本**而不是直接 cleanup | `start_chassis_liveness_watch` |
| SIGUSR1 → 主脚本里 `handle_chassis_death`：调一次 `restart_chassis_once`（最多 `CHASSIS_MAX_RESTARTS=1` 次预算），用启动时缓存的 `CHASSIS_LAUNCH_CMD_STR` + `CHASSIS_LOG_FILE` 重新拉一次 wheeltec_robot；新进程过了 `CHASSIS_STARTUP_DELAY_SEC` 还活着就重新挂 watcher 继续跑；起不来 / 预算用尽再走 cleanup | `restart_chassis_once` + `handle_chassis_death` + `trap USR1` |
| cleanup 时**先**发 0.6s 的 zero `/cmd_vel`（`ros2 topic pub --rate 20 ... timeout 0.6`）再走 SIGINT 群发 | `publish_zero_cmd_vel` → 在 `cleanup()` 最前面调用 |
| chassis PID 单独存到 `CHASSIS_PID`（之前只挂在 `LAST_STARTED_PID` 上，后面起 Nav2 会被覆盖）| `main()` chassis 启动后 |

环境变量开关：
- `CHASSIS_LIVENESS_INTERVAL_SEC` — watcher 周期，默认 5s
- `CHASSIS_MAX_RESTARTS` — 整次脚本运行内允许的重启次数预算，默认 1（"再启动一次"）
- `ZERO_CMD_VEL_ON_SHUTDOWN` — true/false，默认 true
- `ZERO_CMD_VEL_DURATION_SEC` — 发零持续时间，默认 0.6s

设计取舍：
- 重启预算 **不会** 因长时间稳定运行而重置 — 一次跑里挂两回基本是硬件问题，不
  再硬撑。
- watcher 走 SIGUSR1 而不是 SIGINT，因为 INT 在 trap 里会走 cleanup→exit
  路径，没法做"试一次再决定"。
- restart 时复用 `start_managed_process`，新 chassis 进入 `CHILD_PIDS` 数
  组，后续 cleanup 仍能正常杀掉。

### `cmd_vel_separator.py`

| 改动 | 行 |
|---|---|
| 新增三参：`odom_stale_sec=1.0`、`odom_stale_action='zero'`、`odom_stale_log_period_sec=2.0` | 158-162 |
| 新增状态：`_odom_stale` 边沿标记 + `_odom_stale_last_log_t` 独立 throttle | 265-266 |
| `_publish_tick` 里加 odom stale 段：entry/exit WARN + 周期 WARN + `action=='zero'` 时把 `out` 强置 Twist()（v=0, w=0）。Mode 本身不动，odom 恢复后下个 tick 自然接回 | 673-703 |

### 行为验证矩阵（人脑过的）

| 场景 | 之前 | 现在 |
|---|---|---|
| 正常 Ctrl+C，chassis 健在 | nav2 节点直接被 INT；最后一帧不一定是 0 | cleanup 先发 0.6s 的零，再 INT |
| 跑中 chassis 崩（一次）| 没人察觉，nav2 继续算 cmd 进黑洞 | 5s 内 watcher 发 SIGUSR1 → `restart_chassis_once` 拉起新 wheeltec_robot，过 `CHASSIS_STARTUP_DELAY_SEC` 还活着就重新挂 watcher 继续跑 |
| 跑中 chassis 崩（重启又挂 / 起不来）| n/a | restart 失败 → `cleanup` + `exit 1`；用户会看到 `Chassis restart failed — triggering full shutdown` |
| 跑中 `/odom` 停了但 chassis 没崩（极少见，如 wheeltec 半挂）| log act 字段一直显示死前的值，没人知道 | separator 1s 后 WARN，并把 `/cmd_vel` 强压到 0 |
| `/odom` 恢复 | n/a | 下个 tick `_odom_stale` 清零，输出按 mode 正常接回 |

## 6. 验证方式（修完之后）

- **bench 模拟**：跑 nav2 + chassis，到某个时间点 `kill -9 wheeltec_robot_node`
  - 期望：脚本自己退出（#2）；STM32 在 ~300ms 后归零（#1）；separator log
    出现 `odom stale 1.0s+`（#3）。
- **正常关停**：跑完一段任务，按 Ctrl+C
  - 期望：先发一帧 zero 再 kill；底盘明显刹停（#4 + #1）。

## 7. 相关文件

- 这次数据：[`~/rosbags/david_nav_headless_20260522_174926/`](file:///home/agx/rosbags/david_nav_headless_20260522_174926)
- 启动脚本：[start_navigation_headless.sh](../../start_navigation_headless.sh)
- separator：[cmd_vel_separator.py](../../david_nav/cmd_vel_separator.py)
- BT condition：[robot_embedded_in_obstacle_condition.cpp](../../src/robot_embedded_in_obstacle_condition.cpp)
- 全链路图：[CLAUDE.md §2](../../CLAUDE.md)
