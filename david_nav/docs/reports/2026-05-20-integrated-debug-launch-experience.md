# 2026-05-20 一键启动集成 Debug 经验

> 真机 nav 调试反复跑、反复 Ctrl-C，日志很容易丢、残留进程很容易污染下一次。这
> 份文档总结这一轮迭代后形成的"一键启动 + 自动落盘 + 自动清理"工具链。

## 1. 三个启动入口（按场景挑）

| 场景 | 入口 | 特点 |
|---|---|---|
| Debug / 直接发 goal | `scripts/launch_nav_with_log.sh` | 终端可见 + tee 到 `/tmp/nav2_latest.log` |
| 完整带 bag + action goal | `navigate_semantic_pose_with_debug_bag.sh` | 录 rosbag、跑 semantic_pose_navigator |
| Headless（无 GUI）| `start_navigation_headless.sh` | 服务器 / 远程调试 |

三个入口**底层都跑同一个 `2d_david_nav2_bringup.launch.py`**，差别只在外围。

## 2. `launch_nav_with_log.sh` 关键设计

### 2.1 自动 log 落盘 + symlink
```bash
LOG_FILE="/tmp/david_nav_logs/nav2_${stamp}.log"
LOG_LATEST="/tmp/nav2_latest.log"   # 总是指向最新一次 → 任何 agent 都可以 tail -F 它
ln -sfn "$LOG_FILE" "$LOG_LATEST"
```
后续 debug 永远 `cat /tmp/nav2_latest.log` 就行，不需要每次找新路径。

### 2.2 前飞清理（pre-flight）

**杀所有现有 robot_state_publisher**：
```bash
for pid in $(pgrep -f robot_state_publisher 2>/dev/null); do
  [[ "$pid" == "$SELF_PID" ]] && continue
  kill -9 "$pid" 2>/dev/null
done
```
nav2 launch 起来后会自己跑一个 RSP，残留的（MoveIt 那套或上次没退干净的）任何
RSP 都会让 base_link 出现多个父亲，**tf2 把树切两半**，立刻报 "two unconnected
trees"。一刀切干净。

**避免 pkill -f 误杀自己**：`pkill -f "move_group"` 会匹配脚本自身（脚本源码里
有 "move_group" 字符串），把自己也杀了（exit 144）。换成 `pgrep + 排除 $$/$PPID
+ kill`：
```bash
for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
  [[ "$pid" == "$$" ]] && continue
  [[ "$pid" == "$PPID" ]] && continue
  kill "$pid" 2>/dev/null
done
```

### 2.3 自动 brake_trace 录制
默认 `RECORD_BRAKE=1` 就开。后台启 `record_brake_trace.py`，CSV 落到
`/tmp/david_nav_logs/brake_trace_<stamp>.csv`，订阅 `/cmd_vel_nav` →
`/cmd_vel_smoothed` → `/cmd_vel_safe` → `/cmd_vel_raw` → `/cmd_vel` + `/odom` +
`/plan` + 动作 status。脚本 trap EXIT/INT/TERM 自动 kill recorder。

### 2.4 不破 ROS_DOMAIN_ID
**不 `export ROS_DOMAIN_ID`**，继承父 shell。这点很重要：lightning loc 和
wheeltec 在 domain 0，如果 wrapper 强制 30，nav2 + RViz 关在沙箱里**听不到**
他们的 TF / topic，看起来一切起来了但 `/tf` 是空的，map → odom 永远不来。

## 3. `navigate_semantic_pose_with_debug_bag.sh` 关键修补

### 3.1 移除 verify_runtime_config 的 set -e 杀脚本
原版函数硬编码 `EXPECTED_NAV2_FRAME=map` 要求 local_costmap 也用 `map` 帧。但
**REP-105 规定 local_costmap 应该用 `odom`**（rolling window 抗 map→odom 跳变）。
验证失败 → `return 1` → set -e → 整个脚本退出 → trap cleanup → SIGINT 杀所有
managed processes，包括 RViz。

外在表现：跑了 `.sh` 不到 15 秒整套 nav2 + RViz 一起没了，用户根本来不及点
"2D Pose Estimate"。

**修法**：改成 warning-only，返回 0：
```bash
if [[ "$local_frame" != "odom" ]]; then
  log "WARN: local_costmap.global_frame=${local_frame} (expected odom per REP-105)"
fi
return 0
```

### 3.2 ROS_DOMAIN_ID 默认 30 → 0
让 nav2 / RViz / lightning / wheeltec 跑在同一 DDS 域，**否则它们互相看不见**，
症状是 RViz 报 "frame map does not exist"，但 lightning 明明在跑。

### 3.3 复用 launch_nav_with_log.sh 的 pre-flight
同样的"杀所有 RSP / MoveIt 残留"逻辑搬进来。

## 4. RViz 启动延迟

`2d_david_nav2_bringup.launch.py` 把 rviz_node 包进 `TimerAction`：
```python
rviz_node = TimerAction(
    period=rviz_delay_sec,  # 默认 12.0 秒
    actions=[rviz_inner_node],
    condition=IfCondition(use_rviz),
)
```

为什么要等：nav2 lifecycle 全 Activated 约 6-10s。RViz 如果在 Configuring 阶段
就订阅 costmap，订阅成功但 publisher 还没 advertise，等 publisher 起来后 QoS
握手时机不好就**永远收不到数据**。等 12s 之后再起，publisher 已经 advertise 好，
RViz 立刻拿到 latched data（前提是 Durability=Transient Local，见坑文档）。

## 5. 可选 collision_monitor

`enable_collision_monitor:=true/false` launch arg：
- **false（默认）**：collision_monitor 节点 + 它的 lifecycle_manager 都不启；
  david_move 改读 `/cmd_vel_smoothed`（绕过 collision_monitor 这一跳）。
- true：恢复原链路，带障碍 gating。

debug 阶段建议关，避免它在原地旋转时把 footprint 扫过 scan 返回点导致一直
slow-mode。

## 6. brake_trace CSV 字段定义

每 50ms 一行：

| 列组 | 字段 |
|---|---|
| 时间 | `t_unix` |
| **cmd_vel 全链** | `cmd_vel_nav_v/w`、`cmd_vel_smoothed_v/w`、`cmd_vel_safe_v/w`、`cmd_vel_raw_v/w`、`cmd_vel_v/w` |
| odom 实测 | `odom_v/w/x/y` |
| 当前 goal | `plan_goal_x/y`、`plan_age_ms` |
| 事件 | `action_status`、`event=ACTION_STATUS_CHANGE→SUCCEEDED` |

最关键的几列：**`cmd_vel_v/w`**（最终发到 wheeltec 的）+ **`odom_v/w`**（实测）。
对比这两个看 separator 输出和 chassis 响应的 tracking 误差。

## 7. Debug 工作流

```bash
# 1. 起 chassis 周边（lightning loc、wheeltec、lidar、IMU 各自独立）
ros2 run lightning run_loc_online --config ~/lightning_ws/.../default_c16.yaml &
cd ~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/ && source ./install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py &

# 2. 起 nav2（任选一个 launcher）
~/ros2_ws/src/david_nav/scripts/launch_nav_with_log.sh use_rviz:=true enable_scan_bridge:=false
# 或：
~/ros2_ws/src/david_nav/navigate_semantic_pose_with_debug_bag.sh --target <pose_id>

# 3. RViz 起来后：
#    a) 点 "2D Pose Estimate"，机器人当前位置 → lightning 收到 /initialpose
#       开始发 map → odom → map 显示
#    b) 点 "2D Nav Goal" 发 goal
#       或者 .sh 已经在跑 semantic_pose_navigator 自动发 action goal

# 4. 跑完看：
#    /tmp/nav2_latest.log         — 完整 launch + 所有节点输出
#    /tmp/david_nav_logs/brake_trace_<ts>.csv — cmd_vel 链路 + odom 数据
```

## 8. Trace 速读

```bash
# separator 状态转换序列
grep "STATE" /tmp/nav2_latest.log | head -30

# brake 触发了吗
grep "BRAKE_" /tmp/nav2_latest.log | head -20

# goal 到达时刻
grep "ACTION_STATUS_CHANGE" /tmp/nav2_latest.log

# 看 cmd_vel 全链路在某个时刻的状态
awk -F, 'NR==1 || $1 > <某个时间戳>' /tmp/david_nav_logs/brake_trace_*.csv | head -20
```

## 相关文档

- [高负载底盘 nav 调参方法](2026-05-20-nav-tuning-for-heavy-load-low-torque-chassis.md)
- [Nav2 调试常见坑（RViz 视角）](2026-05-20-rviz-and-tf-pitfalls.md)
