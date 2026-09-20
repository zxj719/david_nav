# 怎么读终端输出与运行日志（操作人员版）

面向在车上启动导航的人。不需要懂代码，照着查就行。
文中的"运行目录"指 `~/nav_runs/<时间>_<模式>_<提交号>/`，`~/nav_runs/latest` 永远指向最近一次。

## 一分钟版：出问题先看这三样

```bash
ros2 run nav_bringup navrun show latest | head -30     # 这次跑成没成、怎么结束的
cat ~/nav_runs/latest/summary.md                        # 给人看的摘要
grep -E "error|failed|_exited" ~/nav_runs/latest/events.jsonl | tail -20   # 出错的事件
```

终端刷得太快看不清时，**不要盯着终端**——上面三样是同一次运行的完整记录，事后看更准。

## 终端上会出现的六类行

| 长这样 | 是谁打的 | 要不要管 |
|---|---|---|
| `[INFO] [launch]: ...`、`[INFO] [controller_server-5]: process started with pid [...]` | launch 自己 | 不用，正常启动过程 |
| `[controller_server-5] [WARN] [1789897035.7] [controller_server]: ...` | **某个节点自己的 ROS 日志**，由 launch 转发；`-5` 是该节点在本次启动里的序号 | 看严重度：WARN 多数是状态，ERROR 才要查 |
| `navrun [info] startup_step_finished: {...}` | navrun 的运行事件，**同时写进 `events.jsonl`** | 这是排查的主线 |
| `localization_health [warn] localization_odom_missing: {...}` | `/nav/events` 上的结构化事件回显（带完整数据） | 同上，可事后在 `events.jsonl` 里翻 |
| `navrun: run record /home/agx/nav_runs/... (ctrl_c, exit 130)` | 收尾那一行：**运行目录在哪、怎么结束的** | 每次都该看一眼 |
| `[ros2run]: Process exited with failure 1` | `ros2 run` 把上面那个退出码原样报出来 | 数字含义见下表 |

## 为什么看着有很多重复

三个原因，都是正常的：

1. **同一件事有两条来源**。节点自己打一条给人看的 ROS 日志，navrun 再把同一个事件按结构化格式回显一条（带数值、同时落盘）。例如：
   - `[localization_monitor-3] [WARN] ... : localization_odom_missing` ← 节点自己
   - `localization_health [warn] localization_odom_missing: {"tf_age_sec": 0.42, ...}` ← 事件流
2. **周期性状态摘要不是新事件**。它们每隔固定时间重打一次，只是在说"这个状态还在"。
   2026-09-20 的一次真实启动里：底盘串口摘要 165 条（每秒 1 条）、定位缺 odom 32 条（每 5 秒 1 条）。
3. **Nav2 每个生命周期节点启动时固定 4 行**（"lifecycle node launched" ＋ 3 行说明），11 个节点就是几十行。

**只看关键行的过滤办法**：

```bash
# 只看 navrun 的事件与结束行，去掉节点刷屏
grep -vE "^\[[a-z_]+-[0-9]+\]" ~/nav_runs/latest/logs/console.log | less

# 只看警告与错误
grep -E "\[(WARN|ERROR)\]|\[warn\]|\[error\]" ~/nav_runs/latest/logs/console.log | less

# 看哪类行最多（定位刷屏来源）
sed -E "s/\[[0-9]{10}\.[0-9]+\]//g; s/[0-9]+\.[0-9]+/N/g" ~/nav_runs/latest/logs/console.log \
  | sort | uniq -c | sort -rn | head
```

## 退出码怎么读

`[ros2run]: Process exited with failure N` 里的 `N`，以及 `navrun: run record ... (原因, exit N)`：

| 退出码 | 含义 | 典型原因 |
|---|---|---|
| `0` | 正常结束 | 遥控模式里按 q 退出 |
| `1` | **运行起来了但中途失败** | 某个关键子进程提前退出（见下面的"原因"列） |
| `2` | **还没真正开始就失败** | 配置/参数非法、锁被别人占着、准备或清理阶段出错 |
| `3` | 只出现在 `navrun down` 和带 `--live` 的检查命令 | 请求被接受但清理没完成；或定位仍未就绪 |
| `130` | 你按了 Ctrl-C | `=128+2`，正常停止 |
| `143` | 被 `navrun down` 或别的入口接管停掉 | `=128+15`，也是正常停止 |
| `128+N` | 被第 N 号信号结束 | 同上规律 |

括号里的**原因**（`exit_reason`）对照：

| 原因 | 意思 |
|---|---|
| `ctrl_c` / `terminated` | 你停的 / 被别人协作停掉，正常 |
| `<名字>_exited` | 名为 `<名字>` 的关键子进程自己退出了。`launch_exited` = 导航栈那一段；`executor_launch_exited` = 上位机执行器那一段；`record_exited` = 录包 |
| `prepare_failed` / `startup_failed` / `cleanup_failed` / `record_failed` | 对应阶段失败，退出码 2 |
| `keyboard_quit` | 遥控模式正常退出 |

一个容易困惑的点：`executor_launch_exited` 配 `exit 1` 时，执行器**可能是"正常退出"（退出码 0）**的——
但它在该活着的时候退出了，所以仍按失败记 1。真正的原因要看 `logs/executor_launch.log` 的最后几行。

## 事件文件的最后两行

```
{"source": "navrun",      "type": "run_stopped", "data": {"exit_code": 1, "exit_reason": "executor_launch_exited"}}
{"source": "event_flush", "type": "failed",      "data": {"exit_code": 1}}
```

- **看 `run_stopped`**：这才是本次运行的结论。
- 最后那条 `event_flush` 的 `finished`/`failed` 说的是**末尾事件发布器自己**的结果，不是运行成败；
  它失败一般只影响"最后几条事件有没有发到 ROS 话题上"，**不影响运行目录里的记录完整性**。

## 运行目录里哪个文件回答什么问题

| 文件 | 回答 |
|---|---|
| `summary.md` | 这次跑了多久、怎么结束的、关键指标（给人看） |
| `events.jsonl` | 完整时间线：每一步、每个告警、每次进程起停 |
| `logs/console.log` | 终端上看到的全部内容（事后可慢慢翻） |
| `logs/launch.log` | 导航栈那一段的启动输出 |
| `logs/executor_launch.log` | **上位机执行器那一段**——执行器起不来时先看它 |
| `logs/loc/` | 定位那边的日志摘录 |
| `checks.yaml` | 启动前后的只读体检结果 |
| `params/rendered/` | 这次**实际用的**参数（怀疑参数不对就看这里，不要看源码里的模板） |
| `manifest.yaml` | 代码版本、环境、输入文件指纹 |

## 常见刷屏与告警对照

| 你看到的 | 意思 | 该怎么做 |
|---|---|---|
| `serial 1s summary: ... online=0` | 底盘控制板**没连上**（计数全零＝连都没连上，不是通信错误） | 查底盘电源与 USB；`lsusb` 里应出现 `1a86:55d4` 或 `10c4:ea60`，并生成 `/dev/wheeltec_controller` |
| `serial 1s summary: ... online=1` | 底盘正常，这行只是心跳 | 不用管 |
| `localization_odom_missing` | 导航侧订的里程计话题没有数据（已知项，定位发的是另一个话题名） | 已知、不影响定位与导航，不用现场处理 |
| `localization_tf_stale` | 定位的坐标变换一时没跟上 | 偶发可忽略；持续出现且 RViz 里位姿不动，去看定位那边 |
| `chassis_cmd_missing: /cmd_vel: no subscriber`、`chassis_odom_missing` | 底盘没接上的另一种表述 | 同"底盘控制板没连上" |
| `parameter_missing: ... rendered but not declared` | 体检发现渲染了某些节点没声明的参数（长期如此，49 条是基线） | 数量没变就不用管；突然变多再报 |
| `inactive_parameter_selector` | 参数文件里有本次没启动的节点的段落 | 正常，不用管 |
| `connecting to ws://... / connection failed / reconnecting in 3.0s` | 上位机没开或地址不对 | 上位机开着就会自动连上；一直连不上查地址与网络 |
| `navrun event publication: RCLError` | 关闭瞬间有事件没发出去 | 正常，末尾发布器会补；记录不受影响 |
| `node_identity: /robot_state_publisher: expected one node, observed 2` | 机械臂那套自带一个同名节点，导航这套又起了一个 | 正常现象（2026-09-20 起本流程不再自动结束它）。实测未出现 TF 冲突告警；若某天同时出现 `TF_REPEATED_DATA` 之类，再找人处理 |
| `preflight_warning: foreign stack retained` | 发现了不属于本栈的其它 ROS 进程，**按策略保留没动它** | 正常保护行为（例如机械臂那套） |
| `another navrun owns /tmp/navrun.lock` | 已经有一套导航在跑 | 用带接管的入口（`20`/`24`），或先 `navrun down` |

## 三个别误判

1. **WARN 不等于故障**。上面一半的刷屏是"状态还在"的心跳，不是新问题。
2. **终端最后一行不等于结论**。看 `navrun: run record ... (原因, exit N)` 那一行，或事后 `navrun show latest`。
3. **启动成功不等于车能动**。底盘没连上时导航照样起得来（`startup_ready`），但车不可能动。

## 要找人帮忙时，请带上这些

```bash
R=$(readlink -f ~/nav_runs/latest)
echo $R                                  # 运行目录
tail -5 $R/events.jsonl                  # 怎么结束的
tail -20 $R/logs/executor_launch.log     # 执行器起不来时
tail -40 $R/logs/console.log             # 最后的终端内容
```

把这四段贴出来，基本能直接定位问题；只贴一句"启动失败"通常查不了。
