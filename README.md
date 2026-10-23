# David 导航工作树

David 的 ROS 2 Humble 导航栈。`main` 自 2026-10-22 起是 **W46 减法之后的版本**：在重构版的基础上删掉了
文档形状门禁和事件回流链，并补上了定位滞后检测。它在 David 的隔离工作区 `~/ros2_ws_slim` 里通过了全量测试和
实车短距离导航验证。

**部署说明**：快捷键 20/24/21 目前仍运行 `~/ros2_ws_refactor`（`cd7d463`），切换之前不受 `main` 影响；
生产工作树 `~/ros2_ws`（快捷键 0/4）同样不受影响。定位（雷达 + IMU + Lightning）由现场用快捷键 `3` 启动，
本仓只负责导航，从不修改或启停它。

**要做重构/删减？先读 [重构手册](docs/refactor_playbook.md)。** agent 按
[refactor-task 技能](.claude/skills/refactor-task/SKILL.md) 执行；禁止用定时循环自动重构。

新来的 agent：先读 [CLAUDE.md](CLAUDE.md)（硬边界与操作规矩），再读本文（项目地图），
需要细节时按下表跳转。状态真源是交接包里的 `refactor_loop/PROGRESS.md`，不是本文。

## 当前状态（2026-10-22）

| 项 | 值 |
|---|---|
| 管线 | `precise`、`tour`、`tour + OA` 共用唯一 navrun/navcfg/Nav2 管线 |
| 测试 | 以当前提交的全量 `colcon build/test` 为准（W46 时为 2572 个，0 失败）；沙箱另跑，见 [重构手册](docs/refactor_playbook.md) |
| 实机 | 10-22 `tour` 三次短距离导航都是 SUCCEEDED，关停干净；09-18 精准入位与取消通过（旧版本） |
| 部署 | 快捷键 20/24/21 仍指向 `~/ros2_ws_refactor`（`cd7d463`），尚未切到 `main` |
| 已知问题 | 避障（OA）数据源仍是 aligned_scan（待现场选型）；底盘驱动指令超时 `cmd_timeout_sec` 仍为 0.0（关闭），要等实车冒烟后再开启 |

## 三十秒看懂数据流

```text
机器人档案 + 模式差异 + 场地 + schema/绑定表
  → navcfg 纯渲染与校验 → navrun 冻结出一份 run bundle → nav.launch 进程图
  → Nav2 控制器/恢复行为 → 可选平滑器 → chassis_adapter → wheeltec_driver

上位机 WS / ROS Dock 入站 → nav_executor 同进程组装根
  → nav_tasks.api → 唯一 TaskManager → NavigateToPoi / Dock 技能
  → 共享 Nav2Client / ActionSession → Nav2 动作

外部定位 TF + 底盘 odom → nav_perception_iface 健康观察（只告警，不干预）
任务/入位/健康事件 → /nav/events → monitor → events.jsonl + summary.md
navrun 自身的生命周期事件 → 父进程直接写 events.jsonl（不经 DDS）
```

三条不变量：**任务只有一个 owner**；纯逻辑层不碰 ROS/YAML/WS；
`frames.body` / `frames.nav_base` / `frames.poi_capture` 是三个不同的帧，永不合并。
展开版见 [架构与所有权](docs/architecture.md)。

## 我要改 X，该动哪里

| 想做的事 | 去哪 |
|---|---|
| 改速度、容差、代价图、行为树选择 | `david_config/robot/david.yaml` 或 `david_config/modes/<模式>.yaml`，**不要**直接改生成出来的参数文件 |
| 加一个新参数 | 先在 `nav_bringup/config/schema.yaml` 声明，再在档案里填值；`navcfg doc` 重新生成 `docs/parameters.md` |
| 改 Nav2 参数名到档案的映射 | `nav_bringup/config/nav2_bindings.yaml` |
| 换一台机器人 | 当前不承诺，见 [多机器人支持现状](docs/porting.md)；`example_diffdrive_config` 只是测试夹具 |
| 改启动了哪些进程 | `nav_bringup/nav_bringup/launching/graph.py`（纯数据决策，不碰 ROS） |
| 改任务状态机/技能 | `nav_tasks/`（纯逻辑在 `domain`/`core`/`skills`，ROS 适配在 `adapters/ros`） |
| 改上位机协议 | `nav_executor/nav_executor/protocol/`；协议文档与金样在 `docs/protocol/`，**不能为测试绿重新生成金样** |
| 改点位数据 | `david_config/sites/daolan/`；POI 采集用 `frames.poi_capture`（base_link），与导航基准帧差 0.165 m |
| 加一次运行要记录的东西 | `nav_bringup/nav_bringup/recorder/` |

## 包一览

| 包 | 职责 |
|---|---|
| `nav_bringup` | `navrun` / `navcfg`：配置渲染与校验、启动图、运行记录、体检、离线闭环自测的宿主 |
| `nav_tasks` | 纯领域与技能、唯一任务管理器、共享 Nav2 客户端、入位、站点数据 |
| `nav_executor` | WS 协议与传输、ROS 入站、同进程组装根；旧入口是转发到这里的薄壳 |
| `nav_perception_iface` | 定位/底盘接口契约与 5 Hz 健康观察（默认只告警） |
| `nav_plugins` | 行为树恢复插件与模板 |
| `nav_sim` | 假底盘/假定位/假上位机与**离线闭环自测**（已跑通，两种模式各一次） |
| `wheeltec_driver` / `chassis_adapter` / `chassis_teleop` | 驱动、速度适配、键盘遥控 |
| `david_config` | David 的机器人/模式/场地数据 |
| `example_diffdrive_config` | 测试夹具：沙箱与真实 Nav2 接线测试用的虚构机器人配置，不是部署目标 |
| `david_system_env` | 显式加载的共享 ROS/DDS 环境数据，没有自动 hook |
| `david_nav_msgs` | 事件等消息契约 |
| `nav2_rotation_shim_controller` / `third_party/serial` | 当前控制链需要的补丁与驱动依赖 |

`third_party/wheeltec_original` 是忽略构建的原始快照。

## 常用命令

两个 CLI 的可执行文件**不在 PATH 上**，实际调用是
`ros2 run nav_bringup navrun …` / `ros2 run nav_bringup navcfg …`；
下面为简洁省略前缀。任何 ROS 命令之前都要先按 [CLAUDE.md](CLAUDE.md) 第 2 节加载环境。

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws_slim/install/setup.bash

navrun up --mode precise --dry-run      # 只预览：不建目录、不起进程、不碰 ROS
navrun up --mode precise                # 真启动（会起底盘驱动，见 CLAUDE.md 的安全约定）
navrun up --mode precise --takeover     # 锁被占时先请对方协作退出再接管；不强杀
navrun up --mode precise --chassis off  # 不起驱动：车物理上不可能动，用于无运动验收
navrun doctor                           # 只读体检
navrun ls                               # 列出运行记录（JSON）
navrun show latest                      # 查看最近一次运行记录
navrun status                           # 查询当前持有者；从不按 PID 发信号
navrun down                             # 协作式请求停止，不强杀
navrun check --run-dir ~/nav_runs/latest  # 用冻结文件重新校验；ROS 检查需 --live
navrun selftest --mode precise --allow-goals --output-dir /tmp/st1  # 需 ROS_DOMAIN_ID=99
navcfg validate --robot david --mode precise      # 档案 + 模式 + 场地 + URDF 的跨文件校验
navcfg render --profile david_config/robot/david.yaml --mode david_config/modes/tour.yaml --output /tmp/nav2.yaml
navcfg doc --source-root ~/ros2_ws_slim/src --output docs/parameters.md  # 改 schema 后重新生成
```

构建与全量测试：

```bash
cd ~/ros2_ws_slim
colcon build --symlink-install
colcon test && colcon test-result --all
```

只跑某个包的 pytest 时**必须进到包目录**（各包有同名测试文件，一次性把所有 `test/`
交给 pytest 会因模块重名而无法收集）：

```bash
cd ~/ros2_ws_slim/src/nav_bringup && python3 -m pytest test -q
```

## 一次运行留下什么

`~/nav_runs/<时间>_<模式>_<提交>/`：`manifest.yaml`（git 状态、环境、输入指纹）、
`events.jsonl`（完整时间线）、`summary.md`（给人看的摘要）、`params/`（输入、渲染、
实际生效三份）、`checks.yaml`（体检）、`launch.json`（启动图）、`logs/`、可选 `bag/`。
`~/nav_runs/latest` 是软链。排查现场问题**先看这里**，不要只翻控制台。

## 重构经验与 agent 规则

两次重构的教训详见 [docs/refactor_playbook.md](docs/refactor_playbook.md)，最重要的几条：

- **不要让 AI 定时循环做重构。** 2026-09 用每 4 小时一轮的自动循环，4 天新增约 6 万行 Python；
  测试涨到 8 倍，其中 27% 只检查注释格式。原因是实车验收做不了，循环只能在测试和文档上证明"完成"。
- **一次只做一张已批准的变更单**，缺现场证据就写 `WAITING_FIELD` 并停下。
- **只在隔离工作区开发**（部署树是 symlink-install，改了就等于改现场）；基线一律用干净检出。
- **删除时测试跟着机制走**，删除前后比对测试清单，意外删除必须为 0；现场修复当天入库。

给 agent 的可执行流程：[.claude/skills/refactor-task/SKILL.md](.claude/skills/refactor-task/SKILL.md)
（模板与命令见同目录 [REFERENCE.md](.claude/skills/refactor-task/REFERENCE.md)，
测试清单比对脚本是 `scripts/test_inventory.py`）。Claude Code 在本仓里会自动发现这个技能；
其他 agent（例如 Codex）请在任务说明里显式指向它。

## 文档索引

| 文档 | 内容 |
|---|---|
| [CLAUDE.md](CLAUDE.md) | 硬边界、环境陷阱、验证规矩（**动手前必读**） |
| [docs/refactor_playbook.md](docs/refactor_playbook.md) | **重构手册**：两次重构的事故与教训、下一次的流程与完成标准 |
| [.claude/skills/refactor-task/SKILL.md](.claude/skills/refactor-task/SKILL.md) | 给 agent 的重构任务技能（硬性禁区、步骤清单、触发线） |
| [docs/architecture.md](docs/architecture.md) | 架构与所有权、模式、任务与关闭顺序 |
| [docs/console_and_logs.md](docs/console_and_logs.md) | **操作人员版**：怎么读终端输出、退出码含义、日志去哪找 |
| [docs/runbook.md](docs/runbook.md) | 现场操作与诊断步骤 |
| [docs/parameters.md](docs/parameters.md) | 参数目录（由 schema 自动生成，勿手改） |
| [docs/porting.md](docs/porting.md) | 多机器人支持现状（当前不承诺） |
| [docs/behavior_policy.md](docs/behavior_policy.md) | 行为与安全边界 |
| [docs/events.md](docs/events.md) | 事件契约与交付限制 |
| [docs/agent_navigation_tools.md](docs/agent_navigation_tools.md) | Agent 可调用的导航工具、事件和任务编排边界 |
| [docs/protocol/](docs/protocol/) | 上位机 WS 协议、AsyncAPI、ADR |
| [docs/alias_mapping.md](docs/alias_mapping.md) | 现场快捷键与重构版入口的对应 |
