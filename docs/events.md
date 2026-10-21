# 导航事件契约

全局类型为 [NavEvent.msg](../david_nav_msgs/msg/NavEvent.msg)：
`event_id / stamp / source / event_type / severity / data_json`。
stamp为有限Unix wall秒，不是ROS仿真时间；data_json为对象且数值有限。
身份与task_id不同，task_id可以重复。事件是观察，不是控制命令或实体停稳回执。

## 生产者与含义

| source | 事件 | 时间与所有权边界 |
|---|---|---|
| `tasks` | admitted/phase/progress/paused/cleanup/fault及succeeded/failed/canceled | 终态在释放软件所有权后；外部CANCELED的paused仍非终态 |
| `dock` | started/approach_checked/stage_started/measured/spin/after_spin/no_pose，全局带dock_前缀 | 测量事件不等于任务成功；`~/events` String保留同一原始stamp/id |
| `localization_health` | waiting/ready、TF或odom缺失/过期/未来、clock_reset/numeric_range、health_ok/divergence；另有 map_odom_lag_persistent/map_odom_future/map_odom_lag_numeric_range/map_odom_lag_recovered | 每次状态变化或5s，另有5Hz diagnostics；ready仅表示map→odom可查；map_odom_* 只在进入告警与首次恢复时发 |

任务/Dock具体字段与生命周期见 [任务事件](../nav_tasks/docs/events.md)。
生产者使用reliable/volatile/keep-last depth10；晚订阅者没有历史保证。
任务/Dock复用原节点，不新增观察线程；health是独立5Hz节点且无取消/速度接口。

`navrun` 自己的生命周期事件（预检、启动、检查、退出，含最终 `run_stopped`）由父进程直接写进
本地 `events.jsonl`，**不发布**到 `/nav/events`。W46 删除了原来向 DDS 的投影、游标/租约文件和
末尾补发进程 `event_flush`，因为没有任何消费者从话题读取 navrun 事件，本地文件里已经有完整记录。

## 本地记录

monitor保留JSONL五顶层键：`time/source/event/severity/data`。
原event_id存入 `data.__nav_event.event_id`；不把接收时间替换为原发生时间。
有限JSON、深度/尺寸/ID约束和4096项LRU成功后去重；重启/淘汰/部分写入不是exactly-once。
坏消息与发布失败只诊断，不改变任务结果，不输出完整敏感异常体。
monitor 和 navrun 父进程向同一个 `events.jsonl` 追加时，经文件锁协调。

## 交付限制

预检失败只写本地记录，不为报告失败而启动 ROS。诊断失败不改导航退出码。
DDS ACK 不等于消费者处理或落盘。volatile 启动发现窗口、多 monitor 接收、崩溃恢复均有边界；
不承诺 exactly-once、完整历史、零延迟或零开销。事件缺失不能单独证明动作未发生。
运行记录/部署审核见 [runbook](runbook.md)，实现细节见 [nav_bringup](../nav_bringup/README.md)。
