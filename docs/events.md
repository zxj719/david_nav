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
| `navrun` | 生命周期/预检/启动/检查/退出的有限字段投影 | 来自原events.jsonl，持久UUID+字节offset身份，原记录时间不变 |
| `localization_health` | waiting/ready、TF或odom缺失/过期/未来、clock_reset/numeric_range、health_ok/divergence | 每次状态变化或5s，另有5Hz diagnostics；ready仅表示map→odom可查 |

任务/Dock具体字段与生命周期见 [任务事件](../nav_tasks/docs/events.md)。
生产者使用reliable/volatile/keep-last depth10；晚订阅者没有历史保证。
任务/Dock复用原节点，不新增观察线程；health是独立5Hz节点且无取消/速度接口。

## 本地记录与防回声

monitor保留JSONL五顶层键：`time/source/event/severity/data`。
原event_id存入 `data.__nav_event.event_id`；不把接收时间替换为原发生时间。
有限JSON、深度/尺寸/ID约束和4096项LRU成功后去重；重启/淘汰/部分写入不是exactly-once。
坏消息与发布失败只诊断，不改变任务结果，不输出完整敏感异常体。

navrun发布器只投影本地source=navrun的安全字段，不广播command/env等原载荷，
忽略已带外来元数据的记录。每批128片段/1MiB、单行64KiB，部分行等待完整，
文件替换/缩短封闭。run-local publisher lease和持久cursor防活owner重叠，
自身namespace在接收端排除。allowlist不是任意字符串的通用秘密检测器。

## 末尾事件与交付限制

原monitor关闭后，父进程完成cleanup并写真实run_stopped，再启动一次受管临时
event_flush续原cursor；up租约保持至helper也排空。0.5s发现、3s协作预算、
父进程独立5s执行期限，超限后关闭/升级但保留owner直到证明退出。
预检失败不强启ROS；helper自身终态只写本地source=event_flush，避免递归补发。
诊断失败不改导航退出码。

cursor是处理位置，DDS ACK也不等于消费者处理或落盘。volatile启动发现窗口、
失败后已推进游标不自动耐久重放、多monitor接收、崩溃恢复均有边界；不承诺
exactly-once、完整历史、零延迟或零开销。事件缺失不能单独证明动作未发生。
运行记录/部署审核见 [runbook](runbook.md)，实现细节见 [nav_bringup](../nav_bringup/README.md)。
