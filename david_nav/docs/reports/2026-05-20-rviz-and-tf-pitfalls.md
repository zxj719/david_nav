# 2026-05-20 Nav2 调试踩过的坑（RViz / TF 视角）

> 经常 RViz 是"问题最先暴露"的地方——你看到的是"屏幕上没东西"或"frame X
> doesn't exist"，但真正的 bug 经常在 DDS / TF / lifecycle / QoS 这些 layer。
> 这份文档汇总这一轮调试踩过的 RViz-相关坑 + 对应根因和修法。

## 1. RViz 不显示 costmap / map（最常见症状）

### 1.1 ROS_DOMAIN_ID 不匹配（→ frame map does not exist）
**症状**：RViz 全局状态错误 `Frame [map] does not exist`，但你确认 `lightning_slam`
进程在跑。

**根因**：launch wrapper 强制 `ROS_DOMAIN_ID=30`，nav2 + RViz 在 domain 30，
lightning / wheeltec 在 domain 0（默认），**DDS 跨不到**。

**检查**：
```bash
cat /proc/<lightning_pid>/environ | tr '\0' '\n' | grep ROS_DOMAIN
cat /proc/<rviz_pid>/environ | tr '\0' '\n' | grep ROS_DOMAIN
```

**修法**：让所有进程在同一 domain（通常 0），或者所有进程都 `export ROS_DOMAIN_ID=<同一值>`。

### 1.2 QoS Volatile vs Transient Local（→ 偶尔有偶尔没）
**症状**：RViz 启动早于 nav2 时看不到 costmap；重启 RViz 后能看到。

**根因**：nav2 costmap 用 **Transient Local** durability 发布（latched）；
RViz config 里 Map display 默认 Volatile 订阅。tf2 工具会兼容，但 RViz 早期
版本的 QoS 协商有时会错过 latched 消息。

**修法**：把 RViz config 里所有 OccupancyGrid / Map display 的 `Durability Policy`
改成 `Transient Local`：
```yaml
Topic:
  Durability Policy: Transient Local
  Reliability Policy: Reliable
```

适用 topic：`/lightning/grid_map`、`/global_costmap/costmap`、`/local_costmap/costmap`。

### 1.3 没设过 /initialpose（→ 看不到 map）
**症状**：launch + RViz 都正常起来，但 map 不显示。

**根因**：lightning loc（localization mode）启动后会**等 `/initialpose`**。没收到
就不发 `map → odom` TF。RViz Fixed Frame=map，TF 链断，整张 map 不渲染。

**检查**：
```bash
ros2 topic info /tf -v   # 看 lightning_slam 是否在 publishers 里
# 如果在，但 sniff 一段 /tf 没看到 map -> odom，就是没初始位姿
```

**修法**：RViz 工具栏点 "2D Pose Estimate"，在 map 上点机器人大概位置。lightning
收到 `/initialpose` 后立刻开始发 `map → odom`。

### 1.4 双 robot_state_publisher（→ tf has two or more unconnected trees）
**症状**：planner_server 报错 `Could not find a connection between 'map' and
'base_footprint' because they are not part of the same tree. Tf has two or more
unconnected trees.`

**根因**：上一次 launch 没退干净，留了一个旧 RSP 进程；新 launch 又起一个 RSP，
两个 RSP 都在 `/tf_static` 上发 `base_link → wheel_link` 等等。tf2 内部不允许一
个 frame 有多个父亲 / 多个发布者按时间窗交替"占领"，**把树切成两棵互不连通的**。

**检查**：
```bash
pgrep -af robot_state_publisher | grep -v claude
# 应该只有 1 个；如果有 2 个或更多，就是这个 bug
```

**修法**：在启动 nav2 前 pre-flight 杀掉所有现有 RSP。nav2 launch 会起自己的。
```bash
for pid in $(pgrep -f robot_state_publisher); do
  kill -9 "$pid" 2>/dev/null
done
```

### 1.5 MoveIt URDF 反向声明 base_link 父亲（→ world → base_link 冲突）
**症状**：同上"two unconnected trees"，但 RSP 只有 1 个。

**根因**：MoveIt 的 `david_moveit_config/config/david.urdf.xacro` 包含 Gazebo
风格的 `<link name="world"/> <joint><parent>world</parent><child>base_link</child></joint>`。
RSP 加载后发 `world → base_link` 静态 TF。同时 nav2 launch 的 `base_alias_tf_node`
发 `base → base_link`。**base_link 有两个父亲**（`world` 和 `base`）。

**修法**：要么杀 MoveIt-side RSP（pre-flight 已加），要么改 URDF 移除 world 节
点（**不推荐**，会影响其他 MoveIt 工作流）。

## 2. nav2 启动后 ~15 秒被杀（→ 看不见任何 nav2 状态）

**症状**：跑 `.sh`，RViz 起来 15 秒后整个 launch 退出，所有 nav2 节点收到
SIGINT/SIGTERM。

**根因**：`.sh` 的 `verify_nav2_runtime_config` 函数硬编码要求 `local_costmap.global_frame == "map"`，但 REP-105 是 `odom`。验证失败返回 1，配合 `set -e` →
脚本退出 → trap cleanup() → 杀所有 managed processes。

**修法**：把 verify 改成 warning-only，永远 return 0：
```bash
if [[ "$local_frame" != "odom" ]]; then
  log "WARN: local_costmap.global_frame=${local_frame} (expected odom per REP-105)"
fi
return 0
```

## 3. costmap 时间戳错误（→ RViz 不显示 / 显示但跳变）

**症状**：costmap topic 在发，但 RViz 不显示；或显示后位置错乱。

**根因**：某个 nav2 节点错误地使用 `use_sim_time=true` 但场景没有 /clock 发布者。
节点的内部时钟停在 0 或者按某种 fallback 累计。costmap 消息 header 的 stamp 变成
`sec=299`（相对开机时间），而 TF 用真实 unix 时间（1779243xxx）。RViz 按 unix 时间
找 frame 找不到。

**检查**：
```bash
ros2 topic echo /global_costmap/costmap --once | grep "stamp\|frame_id" | head -4
# 期望 stamp.sec 是 unix 时间（10 位数）
```

**修法**：
1. 检查所有 nav2 节点的 `use_sim_time` 应为 `false`。
2. 检查 `/clock` topic 是否有"幽灵"发布者（earlier 跑的 sim 留下的 transient）：
   ```bash
   ros2 topic info /clock -v
   ```
   如有但没在发，杀掉发布者；或重启 ros2 daemon。

## 4. d2g=n/a 在临门一脚时突然出现（→ goal_brake 不触发）

**症状**：separator log 里 `d2g` 从 2.0m 平滑减到 1.0m 然后**突然变 n/a**，机器人
冲过 goal 不停。

**根因**：`tf_buffer.transform(goal, robot_frame)` 用 goal message 自带的 stamp
做 lookup。goal stamp 是发出时的时间；机器人移动几秒后 TF buffer（~10s 窗口）已
经丢弃了那个时间的变换 → ExtrapolationException → 返回 None → goal_brake 不触发。

**修法**：transform 前把 goal stamp 改成 0（让 TF 用 latest-available）：
```python
stamped = PoseStamped()
stamped.header.frame_id = goal.header.frame_id
stamped.header.stamp = Time()   # builtin_interfaces.msg.Time(), sec=0
stamped.pose = goal.pose
goal_in_robot = self.tf_buffer.transform(stamped, robot_frame, ...)
```

## 5. 键盘遥控不响应（→ separator 一直发零速）

**症状**：nav2 跑着没事，但你想 keyboard_teleop 接管时机器人响应极差或不动。

**根因**：separator 在 IDLE 模式下也以 20Hz 发零速到 `/cmd_vel`，跟 keyboard_teleop
抢话题。20 个零速 / 秒 vs 10 个键盘命令 / 秒，wheeltec 大部分时间看到的是 0。

**修法**：separator 加 silent-on-idle —— IDLE 模式持续 > `idle_grace_sec`（0.5s）
且 input_age > timeout_sec 时**完全不发**，让外部 publisher 接管：
```python
silent = (self.silent_on_idle
          and self.mode == Mode.IDLE
          and input_age > self.timeout_sec
          and idle_age > self.idle_grace_sec
          and not goal_brake_fire)
if silent:
    return  # 不 publish
```

## 6. collision_monitor lifecycle 卡住等 service

**症状**：launch 日志反复打 `lifecycle_manager_collision_monitor: Waiting for
service collision_monitor/get_state...`

**根因**：collision_monitor params 里 `polygons: []`（空列表）。nav2_collision_monitor
**需要至少一个 polygon 实体**才能完成配置；空列表让节点初始化失败，进程死了，
lifecycle_manager 永远等不到 get_state service。

**修法**：要么保留至少一个 polygon（但可以 `enabled: False`）让节点能配置成功；
要么干脆把节点从 launch 里排除（用 `condition=IfCondition(enable_collision_monitor)`，
默认 false）。后者更彻底，链路上 `velocity_smoother → david_move` 直连。

## 7. RViz 配置里没勾选 / 复选框关掉了显示

**症状**：所有 topic 都在发、所有 TF 都对，但 RViz 还是不显示。

**检查清单**：
1. RViz 左侧 Displays 面板里对应项的复选框是否打勾
2. 该 display 是否报红色错误图标（hover 看错误提示）
3. Fixed Frame 是不是 `map`（一般 nav2 配置）
4. Tool 设置：Initial Pose Topic / Nav Goal Topic 是不是 `/initialpose` 和
   `/goal_pose`（确保用默认值就行）

## 8. Quick diagnose 命令汇总

```bash
# 1. TF 树到底有什么边
python3 -c "
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
rclpy.init(); n = Node('s'); edges = set()
qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(TFMessage, '/tf_static', lambda m: [edges.add((t.header.frame_id, t.child_frame_id)) for t in m.transforms], qos)
n.create_subscription(TFMessage, '/tf', lambda m: [edges.add((t.header.frame_id, t.child_frame_id)) for t in m.transforms], 50)
s=time.time()
while time.time()-s<3: rclpy.spin_once(n, timeout_sec=0.1)
for p,c in sorted(edges): print(f'{p} -> {c}')"

# 2. 谁在发 /tf
ros2 topic info /tf -v 2>&1 | grep -E "Publisher count|Node name"

# 3. 节点重名警告（双 RSP 等）
ros2 node list 2>&1 | grep "WARNING"

# 4. costmap timestamp 是不是 unix 时间
ros2 topic echo /global_costmap/costmap --once | grep -A2 "stamp:"

# 5. 节点 use_sim_time 检查
for n in controller_server planner_server bt_navigator velocity_smoother; do
    echo -n "/$n: "; ros2 param get /$n use_sim_time 2>&1 | tail -1
done
```

## 相关文档

- [高负载底盘 nav 调参方法](2026-05-20-nav-tuning-for-heavy-load-low-torque-chassis.md)
- [一键启动集成 debug 经验](2026-05-20-integrated-debug-launch-experience.md)
