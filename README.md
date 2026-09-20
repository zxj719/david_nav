# David 导航工作树

ROS 2 Humble 导航重构工作树，分支 `refactor/nav-system`，目录 `~/ros2_ws_refactor`。
生产工作树是 `~/ros2_ws`，**本仓的改动不会自动进入生产**。定位（雷达 + IMU + Lightning）
由现场用快捷键 `3` 启动，本仓只负责导航，从不修改或启停它。

新来的 agent：先读 [CLAUDE.md](CLAUDE.md)（硬边界与操作规矩），再读本文（项目地图），
需要细节时按下表跳转。状态真源是交接包里的 `refactor_loop/PROGRESS.md`，不是本文。

## 当前状态（2026-09-20）

| 项 | 值 |
|---|---|
| 任务 | 59 项自动任务全部完成；余 7 项 `WAITING_USER`（需现场配合或需决策） |
| 测试 | `colcon test` **4129 / 0 / 0 / 0**（0 跳过），构建约 15 秒、测试约 2 分 45 秒 |
| 实机 | 09-18 精准入位与取消通过；09-20 完成一次**不让机器人移动**的整机验收 |
| 部署 | 重构版精准入口已在 `.bashrc` 挂成快捷键 `20`；生产 `0` 未被替换 |
| 待现场 | 资源基线、旧入口改 wrap、终端读键、轻量化 A/B、驱动冒烟、odom 接线、legacy 合并 |

## 三十秒看懂数据流

```text
机器人档案 + 模式差异 + 场地 + schema/绑定表
  → navcfg 纯渲染与校验 → navrun 冻结出一份 run bundle → nav.launch 进程图
  → Nav2 控制器/恢复行为 → 可选平滑器 → chassis_adapter → wheeltec_driver

上位机 WS / ROS Dock 入站 → nav_executor 同进程组装根
  → nav_tasks.api → 唯一 TaskManager → NavigateToPoi / Dock 技能
  → 共享 Nav2Client / ActionSession → Nav2 动作

外部定位 TF + 底盘 odom → nav_perception_iface 健康观察（只告警，不干预）
任务/入位/健康/navrun 事件 → /nav/events → monitor → events.jsonl + summary.md
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
| 换一台机器人 | 复制一份 `*_config` 包，见 [换机器人](docs/porting.md)；`example_diffdrive_config` 是可运行的样例 |
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
| `nav_perception_iface` | 定位契约、可选点云转激光、5 Hz 健康观察（默认只告警） |
| `nav_plugins` | 代价图/行为树插件与模板 |
| `nav_sim` | 假底盘/假定位/假上位机与**离线闭环自测**（已跑通，两种模式各一次） |
| `wheeltec_driver` / `chassis_adapter` / `chassis_teleop` | 驱动、速度适配、键盘遥控 |
| `david_config` / `example_diffdrive_config` | 机器人/模式/场地数据；后者用于验证"换机器人只改配置" |
| `david_system_env` | 显式加载的共享 ROS/DDS 环境数据，没有自动 hook |
| `david_nav_msgs` | 事件等消息契约 |
| `david_nav` / `david_cmd` | 兼容入口与旧工具，**不能整包删**，隔离提案见 `docs/legacy_isolation.md` |
| `david_slam_gt` / `nav2_rotation_shim_controller` / `third_party/serial` | 保留的依赖与补丁，本次不改行为 |

18 个参与构建；`third_party/wheeltec_original` 是忽略构建的原始快照。

## 常用命令

两个 CLI 的可执行文件**不在 PATH 上**，实际调用是
`ros2 run nav_bringup navrun …` / `ros2 run nav_bringup navcfg …`；
下面为简洁省略前缀。任何 ROS 命令之前都要先按 [CLAUDE.md](CLAUDE.md) 第 2 节加载环境。

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws_refactor/install/setup.bash

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
navcfg doc --source-root ~/ros2_ws_refactor/src --output docs/parameters.md  # 改 schema 后重新生成
```

构建与全量测试：

```bash
cd ~/ros2_ws_refactor
colcon build --symlink-install
colcon test && colcon test-result --all
```

只跑某个包的 pytest 时**必须进到包目录**（各包有同名测试文件，一次性把所有 `test/`
交给 pytest 会因模块重名而无法收集）：

```bash
cd ~/ros2_ws_refactor/src/nav_bringup && python3 -m pytest test -q
```

## 一次运行留下什么

`~/nav_runs/<时间>_<模式>_<提交>/`：`manifest.yaml`（git 状态、环境、输入指纹）、
`events.jsonl`（完整时间线）、`summary.md`（给人看的摘要）、`params/`（输入、渲染、
实际生效三份）、`checks.yaml`（体检）、`launch.json`（启动图）、`logs/`、可选 `bag/`。
`~/nav_runs/latest` 是软链。排查现场问题**先看这里**，不要只翻控制台。

## 文档索引

| 文档 | 内容 |
|---|---|
| [CLAUDE.md](CLAUDE.md) | 硬边界、环境陷阱、验证规矩（**动手前必读**） |
| [docs/architecture.md](docs/architecture.md) | 架构与所有权、模式、任务与关闭顺序 |
| [docs/runbook.md](docs/runbook.md) | 现场操作与诊断步骤 |
| [docs/parameters.md](docs/parameters.md) | 参数目录（由 schema 自动生成，勿手改） |
| [docs/porting.md](docs/porting.md) | 换机器人 |
| [docs/behavior_policy.md](docs/behavior_policy.md) | 行为与安全边界 |
| [docs/events.md](docs/events.md) | 事件契约与交付限制 |
| [docs/protocol/](docs/protocol/) | 上位机 WS 协议、AsyncAPI、ADR |
| [docs/alias_mapping.md](docs/alias_mapping.md) | 现场快捷键与重构版入口的对应 |
| [docs/legacy_isolation.md](docs/legacy_isolation.md) | 旧代码隔离提案（待逐项确认） |
