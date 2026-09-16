# nav_sim

PLAN §13.4 / P3.4 的可配置离线假件，不依赖真实驱动，不打开串口。

- `fake_chassis` 订阅 `cmd_topic`（默认 `/cmd_vel`），以固定 `dt=0.02` 按旧
  `david_nav/scripts/sim_fake_chassis.py` 的显式 Euler 规则积分，发布 `odom_topic`
  （默认 `/odom`）。仅使用 linear.x / angular.z；默认静止，延续旧脚本持有最后指令的语义。
  必填 `odom_frame`、`body_frame`；新消息明确填 child_frame_id，避免假定位猜测机器人帧。
- `fake_localization` 接收上述里程计，发布恒等 map→odom 和积分 odom→body。
  必填 `map_frame`、`odom_frame`、`body_frame`，三者不同；URDF/robot_state_publisher
  独占 body→nav_base（导航旋转中心），假定位不发布这一边。输入帧或姿态非法时丢弃样本。
  可选 `output_odom_topic` 镜像到导航测试的 odom 接线；不会改生产档案。
- `fake_agent_server` 仅监听 127.0.0.1:18101，默认只收事件、不发命令。
  可选 JSON `--script` 需要显式 `--allow-goals`。一条脚本是
  `{"command": {"action": "navigate_to", "request_id": "case-a", "sub_id": 0,
  "target_id": "poi-a"}, "wait_event": "on_arrived"}`；脚本文件顶层为列表。
  取消 action 必须是旧协议 `abort_navigation`。匹配事件屏障同时核对 request_id/sub_id，
  `--events FILE` 追加保存所有对象帧；同一时刻只允许一个客户端。

三个入口均要求显式 `ROS_DOMAIN_ID=99`，ROS 节点还检查实际 context 的域。域隔离不是现场运动授权，不能将假服务端接给生产
执行器。RUNBOOK S1 禁止自动闭环发非零速度，本包测试仅调用纯函数和内存假件；不运行带目标
脚本，也不启动 Nav2。仅一个临时 loopback socket 测试用 ping/pong 验证已装 WebSocket API；
不发送导航命令。真 Nav2 闭环运行的审核事项见交接包 `REVIEW_PENDING.md`。

纯测试覆盖原脚本积分等价、ROS 消息/TF 的内存映射、非默认机器人帧、协议字节、屏障超时、
多余状态帧及隔离门禁。`colcon test --packages-select nav_sim` 不启动 ROS 节点。
共享档案帧键来自其它包 schema；此包独有参数列于 `config/schema.yaml`。

## 闭环自检的事件验收核心（P3.5进行中）

`selftest_evidence.check_evidence` 接收独立的NavEvent记录与WS帧，核对一个普通POI、
随后一个两段Dock的完整事件证据。它不启动ROS、不产生目标，也不模拟或冒充Nav2。
真实runner/CLI、隔离场景和实跑验收仍待；此模块的 `complete` **仅指事件证据完整**。

每例必须有唯一admission、正确POI与模式阶段、同task/sub/generation、终态成功且释放所有权。
两例必须由同一producer按序完成，第二例generation递增；Dock还必须按顺序通过完整
轮廓校验、approach/final两个阶段及停稳位置/角度容差测量。校验目标及预备点几何，
不把“到达预备点”或单个WS on_arrived当成Dock成功。该健康场景预期0次旋转补偿；
额外Spin/失败/暂停/取消/多余阶段、丢失事件或提前终态均失败，不更改实际Dock行为。

ROS记录标准化为`event_id/source/event_type/severity/stamp/data`映射；同ID完全相同的
重送可去重，冲突重送拒绝。选中任务的producer序号必须递增，WS到达帧各一次且顺序正确。
两个传输的接收顺序不作跨通道比较；wall clock回退不影响序号判断。每通道最多5万条、
序列化单NavEvent最多64KiB；这不是传输解析器，runner仍须限制接收大小和落盘量。
其它健康事件不提供任务成功证据。报告不回显原始payload。

单测除了损坏/缺失/乱序轨迹，还用真实TaskManager、NavigateToPoi、DockAwareNavigation、
DockWorkflow、静态footprint算法和协议编码器生成事件，运动/时钟全部内存替身。
它证明验收器与领域事件契约一致，不证明DDS、真实Nav2控制或实体制动。
