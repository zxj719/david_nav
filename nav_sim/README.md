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
执行器。RUNBOOK S1 禁止自动闭环发非零速度；常规包测仅调用纯函数和内存假件，不启动
Nav2。显式沙箱检查可以启动隔离 Nav2，但仅发布零速、不发导航命令。
真 Nav2 闭环运行的审核事项见交接包 `REVIEW_PENDING.md`。

纯测试覆盖原脚本积分等价、ROS 消息/TF 的内存映射、非默认机器人帧、协议字节、屏障超时、
多余状态帧及隔离门禁。`colcon test --packages-select nav_sim` 不启动 ROS 节点。
共享档案帧键来自其它包 schema；此包独有参数列于 `config/schema.yaml`。

## 闭环自检的事件验收核心（P3.5进行中）

`selftest_evidence.check_evidence` 接收独立的NavEvent记录与WS帧，核对一个普通POI、
随后一个两段Dock的完整事件证据。它不启动ROS、不产生目标，也不模拟或冒充Nav2。
隔离场景、受管启动器和完整runner/CLI已实现；带目标闭环实跑验收仍待，此模块的
`complete` **仅指事件证据完整**，不等于运行器最终结果。

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

## 隔离场景与零速启动检查

`selftest_scene.prepare_scene` 离线生成独占目录、合成地图、普通POI与两段Dock、共享
导航图及私有参数。支持David/example_diffdrive的tour/precise档案，保留选中档案的
几何、帧、速度限制、目标容差和Nav2调参；只替换测试数据源与接线。不读取现场地图或POI，
不执行平台预检/主机清理，不启动真实底盘。固定地图为12米方形、5厘米分辨率、占用边界；
要求轮廓至少3点且各坐标绝对值小于2米，不声称适配任意尺寸机器人。

所有ROS节点与话题置于随机 `selftest_<32位十六进制>` 命名空间；包括TF、NavEvent、
速度、地图、里程计和动作名。假件积分步长20ms，body→nav_base仍由RSP负责。
`selftest.json` 保存资产哈希、图和验收预期；其中 `executed: false` 是不可变的准备快照，
不是运行结果。父进程必须持有生成时返回的digest，不能从随后可变的文件重算“预期hash”。

`selftest_plan.load_scene` 校验父进程digest、每项资产、路径/软链和进程白名单。
`python3 -m nav_sim.selftest_launch --run-dir DIR --plan-sha256 SHA
--start-verified-fixture` 仅供受管父进程调用；要求domain99，不创建目标客户端或WS服务。
父进程必须先检查现场栈，使用OwnedProcess独占进程组，保持WS事件循环可响应直至退出，
再核对 `process_events.jsonl` 的逐进程启动/退出记录。Humble交互LaunchService不会
转发只发给leader的SIGINT，因此 `close_fixture` 异步等待自有进程组SIGINT后的退出，
超时清理不算成功；仅leader状态不能证明所有子进程正常退出。

显式执行 `bash sandbox_tests/run_startup_checks.sh`：脚本加载Humble、重构install及
正式DDS白名单后覆盖domain99，发现现场栈则拒绝启动。检查example_diffdrive两种模式，
被动loopback WS只收帧、不发命令；逐个确认Nav2 lifecycle ACTIVE、参数和map→nav_base TF，
随后仅给adapter私有输入发布默认全零Twist，核对输出全零。动作端点可发现不等于ACTIVE，
adapter未收到输入前保持静默也不是错误。检查最后要求所有13个进程有正常退出记录。
这证明真实安装栈的零速接线与收尾，不证明轨迹跟踪、停稳或POI/Dock闭环。沙箱文件不匹配
`test_*.py`，不会被常规colcon测试自动启动。

## 完整协调器与navrun入口（闭环运行尚待审核）

`navrun selftest --mode tour|precise [--robot NAME]` 由nav_bringup懒加载可选nav_sim，
避免包依赖成环。默认robot=david；可用 `--source-root SRC` 读源码资源，否则读安装索引。
`--output-dir DIR` 必须是新目录，省略则保留独占临时目录；失败也不自动删除证据。
必须显式选择以下一种，不选择即报错，绝不默认发任务：

- `--prepare-only`：离线写合成场景和哈希计划，不导入运行器、不起ROS、不绑定端口；
  允许非99环境。输出 `kind=prepared, executed=false, complete=false`，不是零写dry-run。
- `--startup-only`：domain99且无用户栈时，起真实私有Nav2＋假底盘/定位；观察所有lifecycle
  ACTIVE、地图、TF/odom、WS与进程后正常关闭，不创建速度发布器、不派发目标。
  成功输出 `passed=true, complete=false, kind=startup_only`，不声称闭环通过。
- `--allow-goals`：操作人员明确允许的完整运行，domain99/无用户栈门禁仍生效；依次发送一个
  普通POI与两段Dock的navigate_to。当前RUNBOOK S1不允许agent自动执行这个选项；
  域隔离/假底盘并不自动构成运动授权。

`--timeout SEC` 默认180，必须有限正数，是包括启动在内的总协作期限；超时进入独立有界
清理，不是强制限制每次原生ROS调用耗时。运行器不做主机preflight、daemon/shm清理或全局
按名杀进程。启动前两次只读检查现场栈，检查与启动不是系统级互斥，操作者不得同时起其它栈。

观察器使用私有Context和显式SingleThreadedExecutor，不启用进程全局文件日志或rosout；
每轮最多16次非阻塞spin，lifecycle查询最多每节点2Hz。只订阅私有NavEvent、map、odom和TF。
WS仅绑定动态loopback端口，只接受一次连接，断开/重连失败，禁止重放命令；ping只答pong。
WS/ROS数据入口拒绝重复键、非对象/非有限值/深度超限；每帧64KiB、总5万条且8MiB，
ROS内层JSON先限60KB以预留信封空间，溢出即失败而非静默截断。落盘observations.jsonl的
通道信封有少量额外开销；磁盘日志和launch日志不包含在该内存证据上限内。

每例除匹配WS到达和任务释放所有权，还须新鲜map→nav_base TF满足目标xy/yaw容差，
并看到本例独立fake-chassis odom位移大于5cm；不会单凭Goal succeeded报成功。
最终还必须通过原事件验收器，两个任务/入位阶段/轮廓校验/停稳测量完整，且所有子进程
正常退出、WS和ROS资源清理完成。取消（包括收尾时再次取消）、异常/超时/错误事件均不能
成功；屏蔽取消直到自有资源清理完成，再写失败结果并传播取消，不丢弃进程句柄。
等待子进程退出期间继续每20ms消费ROS，随后再有界排空200ms，避免TF积压挡住最后的
任务清理失败事件；它不是跨进程DDS应用层持久交付保证，丢失必需事件仍会失败。
`result.json` 与不可变selftest.json分离，只有 `kind=closed_loop, complete=true` 才是完整
自测成功。CLI退出0表示所选种类通过，1表示已运行但实验失败，2表示配置/门禁错误，130表示中断。

显式 `bash sandbox_tests/run_coordinator_checks.sh` 只运行真实CLI的startup-only两模式，
检查上述实际观察与清理路径，仍要求complete=false。常规包测用内存端口检查两任务协调、
独立位姿/位移、错误事件、断连和取消清理；这些不构成真Nav2闭环执行证据。
