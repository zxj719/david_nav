# 重构进度（唯一真源）

> 当前目标状态：blocked（未完成）。2026-09-17 第3次连续复核：生产启动文件变更仍未确认，生产status指纹仍为2b46c35450efb573317140c44f5c1f12，生产HEAD977e29f、refactor HEADc6199da及.bashrc指纹未变，无活跃colcon/pytest。已将目标标记为blocked（未完成），停止自动重复检查；需用户确认是否保留该生产变更及更新基线后恢复。5个未提交文件和已完成240项增量测试证据保留；本次未改源码、提交、回滚、重定基线或重跑测试，未修改或启停3。

> ⚠ 2026-09-17 生产指纹变化：本轮开始为da1ef50b0f796a7d12d71374dc3815bd，收尾为2b46c35450efb573317140c44f5c1f12；生产HEAD仍977e29f、.bashrc仍695bd38f807e689f084ed249ae855536。只读核对发现新增状态项为M david_nav/launch/2d_david_nav2_bringup.launch.py，移除这一状态行后恰好恢复旧指纹。该文件将output_cmd_vel_topic从固定/cmd_vel改为LaunchConfiguration(david_move_output_topic)，新增同名参数，默认/cmd_vel，说明提及精准模式速度复用。远端mtime为2026-09-17 09:28:10 +0200。来源未确认；未执行生产回滚、覆盖、清理或自动重定基线。按RUNBOOK §4.6停止本轮实现/提交，等待确认这项生产变更及基线处理。

> 2026-09-17 最新：HEADc6199da（另有42bd363）：修复模式覆盖绕过绑定保护和配置名尾部换行校验，合计18项新回归，增量分别145/185绿。另有5文件说明批未提交：三个模块54处说明、3段历史原文归档ADR、4项边界测试，增量240绿（12.55s），非doc AST与c6199da一致。测试已终态、本轮未构建/全量；上次全量仍为5ea1813的3980/0/0/2。收尾发现生产指纹变化，按规则暂停，保留5文件工作树及两端相同副本，不提交、不stash。草稿态剩239无CJK及混合说明；P7.4/总目标未完成，本轮不改或启停3、不发运动指令、不安装或部署。

> 上轮连通记录：本轮提交/全量验证完成后文档同步中SSH断开；只读探测后原IP恢复，uptime约1分钟，重启原因未知、非本轮操作。已重新核实HEAD5ea1813、源码干净和生产/.bashrc指纹不变；本轮测试结果来自断连前完整终态，不将重连当重测。随后重新同步报告与RUNBOOK并核对SHA。

> 现场状态：本轮初检仅见run_loc_online/rviz2，按S7纯测试；收尾生产launch出现新增修改，来源待确认。未启停定位/导航，不自动接受新的生产指纹。

- 状态值：`todo` / `doing` / `done` / `blocked` / `WAITING_USER`
- 类型：`AUTO` = 循环可自主完成；`USER` = 需要用户参与（循环跳过）
- 标记 `[sandbox]` = 需在 `ROS_DOMAIN_ID=99` 沙箱启动 ROS 进程（受限模式跳过）
- 用户最新边界（2026-09-15）：不修改任何 `3` 相关代码/配置/启动脚本（Lightning/uslam、雷达与 IMU 链路）；定位由用户手动操作，相关问题只读诊断并留档，不作为继续导航重构的阻塞项。
- **2026-09-15 09:47 起自动循环（/loop）按用户要求停止，交给 Codex 继续执行**；交接说明见 `weekly_report/codex_handoff_david_nav_refactor/HANDOFF.md`。本文件仍是任务状态的唯一真源

## 基准记录

| 项 | 值 |
|---|---|
| 生产仓库 HEAD | `977e29f`（`feature/dwb-control-opt`，2026-09-14 12:08 核对） |
| 生产工作树指纹 `git status --porcelain \| md5sum` | `da1ef50b0f796a7d12d71374dc3815bd`（2026-09-15 04:43 重新基线：用户直接要求改造 alias 0，见下方"用户直接要求的生产变更"U-1；此前为 `9dfbbc59ed33cfb0677a1e0d8c153f91`） |
| `~/.bashrc` md5 | `695bd38f807e689f084ed249ae855536` |
| 重构分支 | `refactor/nav-system`（worktree `~/ros2_ws_refactor/src`；基线快照提交 `93676a5`，父提交 `977e29f`） |
| 最近一次全量测试 | 2026-09-17 HEAD5ea1813：独立shell fresh18包build15.4s/test2m32s，**3980 tests / 0 errors / 0 failures / 2 skipped**，全链exit0；bringup1445绿，专项82绿。证据staging/p74_linked_trigger_codex/{build_full,test_full,result_full}.log。两运行非doc AST与82399fa一致，源码干净、生产指纹保持。既有版权/缺jsonschema两skip及告警保留。用户要求后续按风险增量，不每轮全量；未全量的后续提交不得冒用本条结果。 |

## Phase 0 基线

| ID | 任务 | 类型 | 依赖 | 状态 | 提交 / 备注 |
|---|---|---|---|---|---|
| P0.1 | 首轮核对：主机、生产 HEAD、工作树指纹、.bashrc md5 写入"基准记录" | AUTO | — | done | 2026-09-14 12:08 |
| P0.2 | 创建 worktree `~/ros2_ws_refactor/src`（分支 `refactor/nav-system`，基于生产 HEAD）；不改生产工作树 | AUTO | P0.1 | done | 创建后生产指纹不变 |
| P0.3 | 基线快照提交：把生产工作树的**已修改跟踪文件**与**生产必需的未跟踪文件**（`nav2_rotation_shim_controller/`、`david_nav/david_nav/dock_finish.py`、`dock_settle.py`、`david_nav/config/fastdds_david.xml`、`david_nav/test/test_dock_*.py`、`test_nav2_frame_consistency.py`、`david_nav/scripts/dock_bench/`）复制进 worktree 提交；排除 `agentdb.rvf*`、`ruvector.db`、`.codex-backups/`、`*.bak*`、`.pytest_cache/`、`__pycache__/`；提交信息列出清单 | AUTO | P0.2 | done | `93676a5`，29 文件；生产分支不动（替代 PLAN P0 "用户先提交"）。注意：worktree 顶层另有 `apriltag/`(324K)、`apriltag.rar`、`cmd.md`，P0.4 构建前确认 apriltag 是否为 colcon 包（depth≤3 未发现 package.xml） |
| P0.4 | 重构工作空间独立构建 + 全量测试，记录基线测试数（预期 90 个 Python 单测 + gtest） | AUTO | P0.3 | done | `8fa6e2a`（`src/colcon.meta`：shim 包 BUILD_TESTING=OFF，与生产 CMakeCache 一致）、`d1b7a7b`+`5d668b5`（headless 脚本测试加 sudo/nvpmodel/jetson_clocks/pgrep 假件——dry-run 也会调真实 sudo 与 pgrep 杀进程）、`48a5916`（补注册 8 个从未被 colcon 运行的 Python 测试）、`34e289d`（dock_bench 脚本式自检改 pytest 包装）。结果见"基准记录" |
| P0.5 | `.gitignore` 增加垃圾清单（仅重构分支） | AUTO | P0.3 | done | `4388e37`（补 `*.bak.*`、`agentdb.rvf*`、`.codex-backups/`、`.fastdds_*`；原有 `*.bak_*` 不匹配点号形式） |
| P0.6 | 执行器线协议特征测试 G1–G16（SPEC §6 P0），在旧代码上跑绿 | AUTO | P0.4 | done | `48a5916` `test/test_executor_golden.py` 12 例全绿：补齐 G1 字节帧、G6、G8（core+bridge 端到端错误文案）、G12（外部取消→paused，且**现状不清 active_task、保持 NAVIGATING**——已固化，重构若改需 ADR）、G2/G9/G13 字节帧、G16 help 与默认 CLI 字典、冻结日志行；G2–G5、G7、G9–G11、G13–G15 已由原测试覆盖 |
| P0.7 | [sandbox] golden：用重构工作空间的**基线提交**起 `2d_david_nav2_bringup`（tour 参数、precise 参数各一次；`cmd_shaping:=false`；不起底盘；静态 TF 伪造 map→odom→base_link），lifecycle 激活后 `ros2 param dump` 全部节点 → `david_nav/test/golden/params/{tour,precise}/`；拷贝 BT XML；记录进程列表 | AUTO | P0.4 | done | `fe1ac78`：`david_nav/test/golden/capture_nav2_param_golden.sh` + 两种模式各 11 个节点参数 dump、节点列表、MANIFEST。tour/precise 差异仅在 controller_server（RotateToGoal critic、xy 0.1、yaw 0.4363），与参数文件一致。BT XML 已在仓库、生效路径记录在 bt_navigator dump 中，未另拷。沙箱采集偶发失败（见 C-4），脚本需重跑 |
| P0.8 | TODO-W1：**仅当** `wheeltec_robot_node` 不在运行时，只读 2 秒 `/dev/wheeltec_controller`（115200 raw）抓上行 24 字节帧作为金样；否则保持 todo | AUTO | P0.4 | done | 2026-09-14 17:14，无进程占用；先 `stty raw -echo`（防止 tty 回显把收到的字节写回 STM32）再只读。960 字节/40 帧（20 Hz）全部通过 XOR 校验；静止、`flag_stop=1`、电压原始值 4338（按驱动公式≈4.34 V，推测电机电源未开）。3 帧入库 `third_party/wheeltec_original/golden/`（`aa52ace`） |
| P0.9 | TODO-V3：只读检查 numpy 2.2.6 与 `tf_transformations` 等导入是否正常，结论写备注 | AUTO | — | done | numpy 2.2.6（`~/.local`）；`tf_transformations`(transforms3d 0.4.2)、`rclpy`、`sensor_msgs_py` 正常；`cv_bridge` 能导入但打印 numpy 2 ABI 告警 `_ARRAY_API not found`（该模块按 numpy 1.x 编译）。本仓 Python 代码不 import numpy / tf_transformations，当前无影响；新代码若用 cv_bridge 需注意 |
| P0.10 | wheeltec 旧包快照入 `third_party/wheeltec_original/`（COLCON_IGNORE）+ `third_party/serial/` vendoring（保留 LICENSE 与来源） | AUTO | P0.3 | done | `aa52ace`：serial 来自 `~/scout_ws/src/serial-ros2`（上游 .git 已损坏未复制；MIT 原文在其 README）；wheeltec 旧包原样快照 + COLCON_IGNORE；`third_party/README.md` 记录来源；文件权限 777→644/755；`serial` 包单独构建通过 |
| P0.11 | 清理 `david_cmd` 既有 lint 旧账（flake8 21 条、pep257），使全量测试通过 | AUTO | P0.4 | done | `1786a3d`，行为不变；`test_pep257` 改为 `--add-ignore D213 --add-select D212`（见"待确认"） |
| P0.U1 | 基线资源测量（真实 3+0、3+4 各一次） | USER | — | WAITING_USER | 需用户运行 |

## Phase 1 可追溯

| ID | 任务 | 类型 | 依赖 | 状态 | 提交 / 备注 |
|---|---|---|---|---|---|
| P1.1 | `nav_bringup` 包骨架（ament_python、setup.cfg flake8、lint 测试、docstring 规范检查单测、README） | AUTO | P0.4 | done | `f898cf0`：`nav_bringup.lint.docstrings`（AST 检查 Args 覆盖每个参数、Returns/Yields/Raises、模块/类/函数 docstring）+ flake8 + pep257(google) + 规则单测与自检，15 测试全绿。其他新包复用方式见包 README |
| P1.2a | 运行记录器纯 Python 核心：运行目录与 `latest`、manifest（git/版本/环境/文件哈希/时长/退出原因）、`code.diff`、`events.jsonl`、`resources.csv`(psutil) 与汇总、`summary.md`、保留策略 gc | AUTO | P1.1 | done | `3f65daf`；51 个新测试，flake8/pep257(google)/docstring 策略全绿 |
| P1.2b | 运行记录器 ROS 侧：生效参数 dump（复用 golden 采集的跳过/重试规则）、`topics.csv` 关键话题频率与 TF 延迟、日志收集；[sandbox] 集成测试 | AUTO | P1.2a | done | `8991976`（`navrun dump-params`：沿用 golden 采集规则——跳过无参数服务的辅助节点、每次调用超时、重试、输出非空才算成功；报告重名节点）、`d01d662`（`navrun monitor`：一个节点同时做 BT 触发、关键话题频率与发布者数、map→机器人 TF 延迟 → `metrics/topics.csv`（0.2 Hz）；summary 增加话题频率、整段运行无发布者的话题提示、TF 延迟 P50/P95/最大；`watch-bt` 并入）、`c991ad0`（沙箱集成检查 `sandbox_tests/`，不被 colcon 收集；第 5 轮 domain 99 实跑 2/2 通过、28 s、无残留进程）、`e8db127`（沙箱检查改用临时 `ROS_LOG_DIR`，实测一次完整运行前后 `~/.ros/log` 文件数不变，见 C-5）。日志收集：命令输出已由 P1.3 的 `console.log` 覆盖；`ROS_LOG_DIR` 进运行记录归 P5.1；alias 3 日志关联拆为 P1.2c |
| P1.2c | alias 3 关联（只读）：记录 `run_loc_online` 的 pid、启动时间、命令行、配置 sha256；收尾时拷贝本次时间窗内 alias 3 日志目录的新增内容到 `logs/loc/` | AUTO | P1.2a | done | `abab4eb`：`recorder/linked_process.py` + `navrun wrap --loc-process EXE [--loc-config-flag NAME] [--loc-log GLOB]`。开始时按 argv[0] 文件名在 /proc 找进程（`ros2 run` 包装进程不算），记 pid、其它实例、启动时间、命令行、`--config` 配置 sha256（相对路径按进程工作目录解析）与日志大小 → manifest `linked.loc`；结束时把本次运行期间日志新增的部分（文件变小则整份）与运行期间新写的日志摘录到 `logs/loc/`，每个文件最多保留末尾 20 MB。实地核对：alias 3 → `start_rviz_david001.sh` → `start_loc_david001.sh` 前台 `ros2 run lightning run_loc_online --config <yaml>`，终端输出不落盘；ROS 日志在 `~/.ros/log/run_loc_online_<pid>_*.log`（用 `{pid}` 通配选中），雷达/IMU 包装输出在 `<ROOT>/log/david001_loc_launch/*.out`。schema 增 `localization.process_name/config_flags/log_globs`。13 个新单测 |
| P1.3 | `navrun wrap -- <旧命令>` 包裹模式 + 单测 | AUTO | P1.2a | done | `3c33ebe`：`navrun wrap` 建运行目录/manifest/code.diff → 子进程输出同时写终端与 `logs/console.log` → 资源采样线程 → 收尾写 summary；Ctrl+C 只记录（子进程同进程组自己收到），SIGTERM/SIGHUP 转发；退出码按 128+N 记录信号死亡；启动前 gc（保护本次运行）。16 个新测试含真实 SIGINT/SIGTERM |
| P1.4a | 脱困触发可见（纯逻辑 + 订阅节点）：`bt_triggers` 规则/去抖/计次、`navrun watch-bt` 订阅 `/behavior_tree_log` + TF 位姿 → `events.jsonl` + 终端行 + `logs/navrun.log`；summary 按节点统计 | AUTO | P1.2a | done | `1c134f2` + `d3eb9ad`（PLAN §5.4 修正：`--base-frame` 改为必填，不再默认 base_footprint；新增 `nav_bringup.lint.literals` AST 检查，禁止包代码出现机器人帧名与机器人话题前缀）：31 个新测试（含"每条规则的节点名都存在于 David 两棵恢复树"校验、导入 watcher 不引入 rclpy 的子进程校验）；`spin()` 本身需 ROS 图，留给 P1.4b 沙箱集成测试。PLAN §10.3。已核对 Nav2 Humble `RosTopicLogger`：QoS(10)、**含 →IDLE 跳变**（BT.CPP 默认 show_transition_to_idle），规则只看跳入目标状态 + 0.5 s 去抖 |
| P1.4b | `navrun wrap` 自动拉起 `navrun monitor`（独立子进程，崩溃不影响主命令；需要 `--base-frame` 与话题清单，P2.2 档案就绪前由 wrap 参数传入）+ 生效参数 dump；[sandbox] 集成测试 | AUTO | P1.4a | done | `3aaf46e`：`wrap --monitor-base-frame FRAME [--monitor-topic T]` 在独立会话里跑 monitor（终端 Ctrl+C 只到主命令；输出并入终端与 `console.log`；资源采样含 monitor；主命令退出后 SIGINT、10 s 后 SIGKILL；monitor 崩溃或起不来只记 warn 事件，不改退出码）。dump 时机从"收尾"改为"就绪时"：monitor `--ready-action`（wrap 默认 `/navigate_to_pose`）看到 `<action>/_action/status` 有发布者（只有 action server 发布，rviz 客户端不算）即记 `ready_action_available` 并在独立会话跑 `dump-params`（收尾时 Nav2 已退出，dump 不到）。新增 `recorder/sessions.py`。13 个新单测；沙箱检查 3/3 通过（含 wrap + 假 action server 端到端，74 s，无残留）。PLAN §10.3 / §13.2 |
| P1.5a | `navrun doctor` 离线只读检查：ROS/DDS 环境变量、Fast DDS 孤儿共享内存段、地图 yaml mode、重复 robot_state_publisher、拷贝安装脚本过期、底盘串口设备（只 `os.access`，不打开） | AUTO | P1.1 | done | `30f56d7`：6 项检查 + `--json`；仅 FAIL 时退出码 1；单个检查抛异常转为 FAIL 不影响其它；17 个新测试。David 实跑（生产工作区，只读）：OK 4 / WARN 2——当前 shell 未加载 DDS 白名单、Fast DDS 共享内存段 26 个中 18 个无进程映射（佐证 C-4） |
| P1.5b | `navrun doctor` 在线检查：ros2 daemon 是否 stale、话题重复发布者、map→odom TF 新鲜度；[sandbox] | AUTO | P1.5a | done | `a0e2f31`：`doctor --live`——`ros2_daemon`（守护进程在跑时对比 `--no-daemon` 直接发现，只查询、不启停）、`command_publishers`（`--cmd-topic` 多发布者告警）、`foreign_hosts`（发布者 GID 主机字节不止一种即告警，提示"本机器人另一台计算机"属正常情况）、`map_to_odom`（短命 rclpy TF 探测，缺失为提示）；解析器按 David 实采的 `ros2 topic info --verbose` 输出编写，带缓存 runner。离线新增 `ros_log_dir`（≥1 万条目或 1 GiB 告警，C-5）。沙箱会话夹具移入 `sandbox_tests/conftest.py`；沙箱检查 4/4 通过（84 s）。PLAN §13.3 中"domain 0 外来 GID 前缀"由 `foreign_hosts` 覆盖。原备注：附带：P2.2 档案就绪后，doctor 的串口设备路径、地图目录、`david_nav` 包名改为从档案读取（当前默认值含 David 路径；不在 §5.4 禁用字面量清单内，但不符合"换机器人只改配置"）；另加 ROS 日志目录（`ROS_LOG_DIR` 或 `~/.ros/log`）文件数与体量告警（C-5） |
| P1.U1 | alias 0/4 改为 `navrun wrap` 包裹旧入口 | USER | P1.3 | WAITING_USER | 需用户同意改 .bashrc |

## Phase 2 配置层

| ID | 任务 | 类型 | 依赖 | 状态 | 提交 / 备注 |
|---|---|---|---|---|---|
| P2.1 | 各包参数 schema 格式 + 汇总器 | AUTO | P1.1 | done | `b19f1d3`：`nav_bringup.navcfg.schema`——格式 v1（type/description/default(null=未设)/unit/range/choices/items/required/tuning）严格校验；`ParameterCatalog` 汇总并拒绝"两包声明同一键"与"键嵌套在另一键下"；`check_value` / `check_profile`（缺必填=错误，未声明键=警告）；按 AMENT_PREFIX_PATH（overlay 优先）与源码树（尊重 COLCON_IGNORE）发现 schema。`nav_bringup/config/schema.yaml` 声明 platform.* / preflight.* / record.*，默认值与机器人无关且保守（David 取值 P2.2 写入档案）。33 个新测试。PLAN §12.1 |
| P2.2a | `david_config` 包 + 机器人档案 `robot/david.yaml`（PLAN §12.2，含 `nav2.odom_topic` 现值），取值由测试逐项对照 P0.7 生效参数、`david_move`/键盘配置、bringup launch、wheeltec 驱动 launch | AUTO | P2.1 | done | `809a224`：新数据包 `david_config`（ament_python，无代码）。`test_robot_profile.py` 10 项对照：控制器限速、代价图 footprint 与各帧、BT/behavior/smoother 的帧与 odom 话题（P0.7 tour+precise 生效参数）；adapter = 导航时 david_move 生效参数；teleop = `david_cmd/config/cmd.yaml` 的 david_keyboard；点云转 scan = bringup launch；驱动参数 = wheeltec 驱动 launch 默认值；§12.4 限速规则；对工作空间全部包 schema 无 error。另记档案键 `localization.process_name/config_flags/log_globs`（供 `navrun wrap --loc-process`）与 `record.monitor_topics` |
| P2.2b | 场地档案 `sites/daolan/`（地图 yaml + 点位文件 + agent server） | AUTO | P2.2a | done | `029eeaf`：`site.yaml` + `maps/daolan_map_nav.yaml`（与 `david_nav` 生产副本逐字节相同，测试防漂移，Phase 4 起由 launch 读场地档案后去重）；`setup.py` 递归安装 `sites/`；4 个新测试（david_config 共 16）。实地核对：bringup 默认地图 `david_nav/config/maps/daolan_map_nav.yaml`；执行器默认 POI 库 `~/.ros/david_nav/poi_store.yaml`、点位源 `~/semantic_map/named_poses.json`（手动维护，只记 hash）；server `ws://192.168.123.186:8101`（`navigation_headless.yaml`） |
| P2.2c | 模式叠加 `modes/{tour,precise,bench_slamgt,teleop}.yaml`（只写差异；teleop 需带上 `david_cmd/config/cmd.yaml` 里 david_move 的遥控取值：min_linear_x 0.20、min_angular_z 0.20、min_stationary_angular_z 0.35、max_angular_z 0.50）+ URDF；与 P2.3 绑定表一起设计 | AUTO | P2.2a | done | `0eed23d`：四种模式叠加、档案新增 goal/nav2/adapter 键、David URDF 副本与防漂移测试；与 P2.3 的基础树/绑定表一起验证。`nav_bringup` 229、`david_config` 27 tests 全绿；tour/precise/bench 渲染等值（footprint 按多边形归一化） |
| P2.3 | `nav2_base.yaml` + 绑定表 + `navcfg render` | AUTO | P2.2 | done | `2a15885`：纯 Python 渲染引擎与 `navcfg render` CLI；`0eed23d`：`nav2_base.yaml`、12 项绑定表及 schema 必填键。David 上两包构建通过，测试分别 229 / 27，全绿 |
| P2.4 | golden：渲染结果与 P0.7 param dump **逐键相等**（tour、precise） | AUTO | P2.3, P0.7 | done | `e7ca86c`：基础树采用 tour/precise 的 launch 生效值（`/map`、`require_live_scan=false`、smoother OPEN_LOOP 20 Hz 与 pass-through 加速度），bench overlay 显式保留原 shaping 值；16 组 mode×node golden 投影逐键比较，固定 shared 数量与 rendered-but-undeclared 集合，footprint 按多边形、BT 路径按文件名归一化。`david_config` 43 个纯测试全绿；受限模式下两包低优先级构建通过 |
| P2.5 | `navcfg validate` 全部规则（PLAN §12.4）+ 单测 | AUTO | P2.3 | done | `32c98c8`：新增离线校验器与 CLI，覆盖 schema、限速、话题/帧一致性、URDF base_link→base_footprint 偏移、模式叠加及允许的差速底盘横向参数例外。`nav_bringup` 237 个纯测试全绿；David 四种模式实跑均 0 errors / 19 个预期 warnings |
| P2.6 | `navcfg doc` 生成 `docs/parameters.md` + 同步检查测试 | AUTO | P2.1 | done | `455ddd4`：由全部包的 `config/schema.yaml` 确定性生成 Markdown 参数表（类型、单位、必填/默认、范围、选项、说明、调参链接），CLI 支持源码树、显式 schema 与安装空间发现；3 个新增测试含提交文档同步门禁，flake8/pep257、低优先级构建和安装后 CLI 复验通过 |

## Phase 3 去写死 · 仿真 · 底盘驱动前半

| ID | 任务 | 类型 | 依赖 | 状态 | 提交 / 备注 |
|---|---|---|---|---|---|
| P3.1 | `nav_plugins`：迁入 C++ 插件；`scan_priority_layer` frame 参数化（默认行为不变）+ gtest；cpplint/uncrustify | AUTO | P0.4 | done | 迁移 `dc6b501`、风格门禁 `f663861`、参数化 `715d4b3`；默认继承父 costmap 的 robot_base_frame，显式覆盖优先、空帧拒绝；TF 与 marker 共用参数。42 项纯测试/lint 全绿，独立 domain 99 插件加载/参数沙箱 3/3（先红后绿）。移走重构 install 的 3 个旧插件软链及旧资源标记到 staging/p31_codex/stale_install_backup，均可恢复，未动生产 |
| P3.2 | BT XML 模板 + 等值渲染 golden | AUTO | P2.3 | done | `a44a324`：三棵 BT 数据模板、显式绑定表、8 个恢复参数 schema 与 David 档案值；navcfg render --bt-output-dir 同时生成 XML 并绑定三处 Nav2 绝对路径；4 模式×3 树逐节点逐属性 golden，25 个新增测试，安装后 CLI 复验通过；19 个预期 WARN 保留。`f1ad1e3` 另用真实 Nav2 控制节点 tick 两棵树及模板，确认脱困成功后规划/跟踪各重发一次，在 FollowPath 成功前保持 RUNNING（不启 ROS） |
| P3.3 | 点云预处理与定位契约检查迁入 `nav_perception_iface`（默认关闭，行为不变） | AUTO | P2.3 | done | 新包保留旧 enable_scan_bridge 显式覆盖及 11 个参数金样；定位就绪前只提示等待，之后核对 TF/实际定位 odom，不改 Nav2 接线。35 个新纯测试/lint；受影响包构建通过，累计 698 tests / 0 errors / 0 failures / 2 既有 skips。ROS 实时采样归 P4.5，健康守护归 P6.7。提交号见 LOG |
| P3.4 | `nav_sim`：fake_chassis / fake_localization / fake_agent_server + 单测 | AUTO | P1.1 | done | 41 个新增测试/lint；旧脚本 100 tick×4 工况精确等价；纯消息/TF 回调、v1 协议字节与事件屏障、loopback ping/pong、环境和真实 context 域门禁。WS 默认被动，不发目标；全仓 11 包构建、739 tests / 0 errors / 0 failures / 2 既有 skips。提交号见 LOG |
| P3.5 | [sandbox] `navrun selftest --mode tour|precise`：跑完一次 POI 导航与一次两段式入位 | AUTO | P3.4, P2.4 | doing | 15fb8cb事件oracle、9cada06私有场景/启动器；562a48d完整WS协调器、只读ROS观察、独立位姿/odom位移证据、逐子进程审计、抗协作取消清理、result.json和navrun可选CLI完成。7b9876d增加退出期持续ROS消费及200ms末尾有界读取，防积压后的清理错误漏判。prepare-only/startup-only/allow-goals互斥且默认拒绝执行；仅closed_loop完整通过才complete=true。53新纯检查，包1153/171全绿；实际example两模式startup-only 2/2、David/example两模式安装prepare-only 4/4；无目标/非零速度。实际POI＋Dock闭环受S1禁止，不能以模型/启动替代，保持doing。 |
| P3.6 | `example_diffdrive_config` 迁移验收测试（只改配置即可渲染/校验通过） | AUTO | P2.5 | done | 新增虚构差速机器人：全部帧/话题/限速/轮廓、0.04 m URDF 偏移和自包含场地不同；两模式逐绑定校验、安装后 CLI render/validate、输入 SHA256 不变与无 David 残留。9 项测试/lint 全绿；本提交仅新增本配置包，不改任何公共实现。实际闭环和现场验收仍未做 |
| P3.7 | `wheeltec_driver` 协议层纯函数 + 字节金样 gtest | AUTO | P0.10 | done | 独立纯 C++ wheeltec_protocol 库，无 ROS/串口依赖；14 个 gtest 覆盖 3 帧 P0.8 实录、下行金样、65536 个 signed 标量、截断/非法输入/单 bit 校验损坏及 float 精度。cpplint/uncrustify 全绿；全仓 13 包构建与 771 tests / 0 errors / 0 failures / 2 既有 skips。NOTICE 保留未明确的上游许可问题 |
| P3.8 | 驱动节点 D2、D4、D5、D7、D8、D9（D3 参数存在但默认 0）+ 假串口测试 | AUTO | P3.7 | done | `6e2388a` D2；`638a449` D3 默认 0；`f2d7c78` D4 pre-shutdown 幂等零速；`3b0845d` D5 降噪；`fc456e9` D7 诊断；`cfe169f` D8 发布开关/schema；`994389f` D9 新旧可执行名兼容。全仓 13 包构建通过、878 / 0 / 0 / 2；节点沙箱 8/8、INT/TERM 假串口自信号检查各通过。David 导航档案关闭底盘 IMU，不改 3。D3=0.5 与 D6 仍待人工 |

## Phase 4 统一入口 · 底盘切换准备

| ID | 任务 | 类型 | 依赖 | 状态 | 提交 / 备注 |
|---|---|---|---|---|---|
| P4.1 | `nav.launch.py`（tour / precise / bench_slamgt / chassis / teleop） | AUTO | P2.4, P3.2 | done | `05cd6ee` 五模式进程图；`9cd5dd5` 安装入口、只读 bundle 校验、全量解析可执行文件后才构造进程、有界重启与就绪门控。纯测试覆盖五模式/生命周期/外部驱动/精确 argv；6/6 沙箱检查含实际安装入口 + 零运动假底盘。全仓 13 包构建、943 / 0 / 0 / 2。P4.2 负责生成完整 bundle，P4.3/P4.4 补适配层与遥控包；当前仍不可替换生产 alias |
| P4.2 | `navrun up` 完成 PLAN §7.2 对等表全部项 + dry-run 测试 + [sandbox] 集成测试 | AUTO | P4.1, P1.2 | done | 既有 bundle/锁/platform/preflight/DDS/录制/生命周期见前序 LOG；`bc0cc91` 完成真实 tour/precise 无目标启动、192/210 项声明参数核对和 strict 拒绝。`489f715` 用同一冻结图的 core/executor 两个 launch 阶段，将前台 WS 启动延后到完整 check/strict 通过；保留 launch 管理、原始 argv/ready 门控、关键退出监管及先执行器后 core 清理。定位中途丢失重新等待与采样，不返回 3 中断默认 up。全仓 1541/0/0/2；本轮隔离 11/11，执行器分支使用纯/私有子进程测试，不开启实际 WS 或运动。生产入口和现场验收仍 P4.U1/U2；不清实际 DDS。 |
| P4.3 | `chassis_adapter`（Python，与 david_move 等价，param dump 对比） | AUTO | P2.2 | done | `70793b8`：纯限幅/死区/整数纳秒 ROS 超时，保留单次超时零速后静默；1 Hz 诊断、INT/TERM 先零速后销毁。27 项单测含 >3000 组旧方法对照、参数金样；真实 domain 99 param dump 两模式一致，退出仅零速。安装 chassis.launch.py 与统一 chassis 模式补齐驱动＋适配层（修正此前驱动-only 中间实现），外部驱动保持不动。9/9 相关隔离检查，全仓 14 包、1143 / 0 / 0 / 2。`bda3ccd` 先修旧测试 ROS 发布路径，普通测试不再启 ROS |
| P4.4 | `chassis_teleop` + `keyboard_teleop.launch.py`（`/dev/tty` 读键；无 TTY 时优雅报错）+ 单测 | AUTO | P4.3 | done | `236b6d2` 纯键位/TTY；`e04efd1` 独立入口；`78c86fe` 排他 flock/TTY/socket 交接；`69ca344` 父侧组合 up 与终端兜底。`5402e9a` 安装便捷 launch（robot/start_driver/layout/linear_speed/angular_speed）复用同一 navrun 所有权；源配置字节不改，覆盖值单独冻结、非法/超限提前拒绝、死区语义保留；外层明确转发关闭信号、失败不报成功。22 项新普通测试，5 项安装 launch 隔离检查覆盖 q、单进程 SIGINT、私有控制终端 Ctrl-C、发布者冲突、无 TTY，全部恢复/清理/记录验证；合并回归 21/21，仅零速假底盘。全仓 15 包、1456/0/0/2。未切生产 5k，真实 SSH/实机验收仍 P4.U2 |
| P4.5 | 定位"等待手动定位"提示与 `check --live` | AUTO | P3.3 | done | `2590cc7`、`ca64e2b`、`41687be`、`bc0cc91` 已完成只读实时参数/TF/独立契约采样和前置定位等待。`489f715` 默认遇到检查中途定位丢失时关闭旧观察者、重新等待并完整采样，旧错误不复用；显式正 wait-timeout 保留 pending=3，配置/检查器错误和中断不重试。6 项新纯测试先红后绿，真实合成 TF 开关验证恢复/INT/显式限时 3/3；连同既有 live-check、前置等待和真实 Nav2 strict 共 11/11。全仓 1541/0/0/2。只限启动检查，持续健康仍归 P6.7，不改或启停 3。 |
| P4.U1 | 按 PLAN §13.6 改 `.bashrc`（0/4/5/5k + old 回滚）并实机冒烟 PLAN §16.3 | USER | P4.2, P4.4 | WAITING_USER | |
| P4.U2 | TODO-W2 实测 launch 下 `/dev/tty` 读键 | USER | P4.4 | WAITING_USER | |

## Phase 5 轻量化（代码准备；是否启用需实测）

| ID | 任务 | 类型 | 依赖 | 状态 | 提交 / 备注 |
|---|---|---|---|---|---|
| P5.1 | L1 日志降噪配置（驱动 `log_tx_frames=false`、`ROS_LOG_DIR` 进运行记录） | AUTO | P3.8, P1.2 | done | 现有驱动/档案（cfe169f 及前序）默认关闭帧日志，显式开启仅 DEBUG；recorded startup/生命周期（69ca344 及前序）将所有自有子进程 ROS_LOG_DIR 指到 run/logs/ros。`bc0cc91` 真实 tour/precise 检查日志目录及 launch 实际落点；本轮内存驱动日志开关 1 项＋自有 INT/TERM 2 项通过（log_noise.log），未开真实串口。默认值/schema 普通测试与全仓 1522/0/0/2 通过。实机日志体量/资源 A/B 仍 P5.U1，不宣称生产已切换。 |
| P5.2 | L2/L3 以模式叠加形式提供（默认关闭，即现行为），附等价性测试 | AUTO | P4.3 | done | `773a906`：六个 tour/precise 的 L2/L3/组合实验模式，显式 --mode-file 选择并冻结原文/来源哈希；五个默认模式不改。L2 去掉 smoother 进程与 lifecycle 项，Nav2 直连原 adapter 输入，恢复旋转上限 0.5、timeout 1.0、hard_max 0.6；L3 publish_hz=20。schema/图/实时检查同步，11 项新配置测试先红后绿；9 项纯内存整形/时序测试含 24846 组有效差速输入，0.55 地板不变。合并选项真实示例 Nav2 tour/precise 2/2，180/198 项声明参数一致，51/49 旧未声明项保留。全仓 1561/0/0/2。仅假底盘/合成定位，无目标/非零发布；时序、越界/混合横移输入、实机运动/CPU A/B 和启用决定仍 P5.U1。 |
| P5.3 | L8 `chassis_adapter` C++ 组件 + 与 Python 版等价测试 | AUTO | P4.3 | done | `150d569`：同包 ament_cmake/ament_cmake_python 混合构建，保留默认 Python 入口；新增 ROS-free C++ core、独立 chassis_adapter_cpp 与 chassis_adapter::AdapterNode 组件。profile implementation 显式选择，节点名/参数文件不变；tour/precise 两个实验文件已安装，默认模式未改。5 项跨语言测试共 12500 组含混合横移/阈值/时钟回退/超时的序列一致，7 项选择/模式测试；组件 pre-shutdown 和卸载用共享锁定状态先发 5 零速、阻止后续回调恢复输出。实际零速 DDS、整数参数、单/多线程容器、navrun/遥控共 15/15；全仓 1588/0/0/2。旧 setup 元数据与构建/安装目录已备份退役；实机性能/停车 A/B 和启用仍 P5.U1，无 3/生产变更。 |
| P5.4 | L4 组合进程 launch 选项（默认 false） | AUTO | P4.1 | done | `aca5da9`：navrun --composition / launch composition:=true|false，冻结选择、旧包默认 false；七个 Nav2 节点合一，显式 CPP adapter 可同容器，Python/driver/dock/感知/WS 独立。逻辑参数/话题/lifecycle 不变，容器也接完整 YAML 以保留子 costmap。物理可执行/组件资源与库预检；90 秒有界加载、失败退出、自有清理、容器不重启；已有导航容器拒绝而非自动杀，其他定位容器不误判。15 项先红后绿，另采集/退出/占用边界有红绿证据；48 项新增普通测试。真实 Nav2 tour/precise+CPP 参数 192/210 项一致，51/49 旧未声明项保留；含 strict/失败/超时/容器退出共 6/6（409.57 s），最终边界后补验 4/4。全仓 1636/0/0/2，29 文件哈希一致；仅域 99 假底盘/合成定位/零速，无目标、不改 3/生产。CPU/运动 A/B 仍 P5.U1；首轮强制收尾 monitor 异常另记 REVIEW_PENDING。 |
| P5.U1 | L1–L4、L6 实机 A/B 与启用决定 | USER | P5.1–P5.4 | WAITING_USER | |

## Phase 6 任务管理器 · 对外接口

> 当前进展以本节末“P6 旧入口和精准 launch 统一迁移（884e4d5 / 58b9226）”及navrun/宿主续作为准。P6.1–P6.4实现已接通真实入口，状态doing表示完整验收未完成，不再表示shim/配置/宿主未实现。真实ROS验收按S7暂缓，继续可自主的P6.5修复、事件/健康和P7，不在已完成金样/入口上重复准备。

| ID | 任务 | 类型 | 依赖 | 状态 | 提交 / 备注 |
|---|---|---|---|---|---|
| P6.1 | `nav_tasks` 内核：ports / registry / state / retry / supervisor（SPEC §4），协议/领域纯逻辑迁移（SPEC P1–P2） | AUTO | P0.6 | doing | 纯内核/ports/registry/state/retry/supervisor及POI技能、存储、观察、public API已接真实组装根与旧shim（884e4d5）。G1–G16/配置/CLI及所有权纯检查通过；完整ROS生命周期验收仍待，R1/R2/R3归P6.5单独修复；详见下方续作和LOG。 |
| P6.2 | Nav2 统一客户端 + NavigateToPoi 技能；G1–G16 全绿 | AUTO | P6.1 | doing | 单一Nav2Client/ActionSession承载Navigate+Spin，迟到handle补cancel、期限与清理证明、POI/直接Dock路由均已接真实入口。新链路G1–G16及旧金样通过；无运动真实宿主/两机器人两模式startup-only已通过，带动作组合验收仍受S1限制，不把纯检查或启动通过当任务闭环。 |
| P6.3 | Dock 技能（替代 dock_navigator；`[dock]` 日志行冻结测试用 dock_bench 正则校验）；[sandbox] 假 action server 集成测试 | AUTO | P6.1 | doing | DockWorker/冻结日志/完整静态footprint/配置桥/ROS入站/多线程宿主/旧Dock shim均已实现，精确模式同一owner且0.05停稳。旧standalone16参数数据化，5°/2次与precise25°/0次分开；R8已由b767ea9解决，不再待修。33d49ad纯参数/配置转换移到skills，app兼容reexport与全部函数AST不变；实际带目标Dock验收仍受S1限制，不能仅以startup-only替代。 818698b补修Dock入口终止close任务的重试，5纯内存故障用例/连续5轮绿，仍保留失败入口并要求宿主回调静默，不把此项称为真实动作验收。 6768d2c修TaskRuntime内部settle取消后重试，8内存用例及五轮复测绿；不改变活跃shield/成功缓存，不跳过失败清理。 |
| P6.4 | `nav_executor` 瘦身为 WS 适配器（SPEC P3）；旧入口 shim | AUTO | P6.2 | doing | WS适配器/public API/可靠outbox/CLI/信号root及两个旧入口shim已接通（884e4d5）；旧precise launch与navrun均单manager。四次源码/安装help、实现路径及文件落点通过，完整两轮2349/0/0/2。SPEC真实ROS集成/共享Dock宿主退出验收仍待；可继续独立P6.5，不重复迁移入口。 |
| P6.5 | SPEC 鲁棒性修复 R1、R2、R3、R8（先红后绿，各一提交）；R6 仅做测量脚本 | AUTO | P6.4 | done | R1 43e110f、R2 22f4087、R3 06a4ce2、R8 b767ea9、R6 9bfa9d6；5470016补R4元数据闭环，executor/sim安装Requires-Dist实际16.1、无全局asyncio补丁、未安装。5新检查及executor336/sim171/david_nav304门禁绿（executor仍1缺schema skip），旧打包检查按D3更新依赖所有权而非删除。R6专项29绿，恢复后受影响包独立build/test308/0/0/0通过再提交。工具完成，无实际CPU基线或调度改动；后续88820ad完整全仓2426/0/0/2。 |
| P6.6 | `NavEvent.msg` + `/nav/events` 生产者（任务管理器、Dock、navrun） | AUTO | P6.2 | done | 88820ad消息、60a719c任务/Dock、fde9309消费、9d31306 navrun有界发布；c830291补实际up/monitored wrap收尾：原monitor清理后写真实run_stopped，再受管临时publisher续游标/ID；0.5s发现、3s协作/5s父超时，保留锁直至cleanup证明，无新增常驻DDS。预检失败留本地不强启ROS，helper终态source=event_flush本地记录避免递归；不改导航退出状态。957包测、62本机专项、实际生成CDR无ROS探针通过；volatile发现/崩溃重放及真实性能/DDS验收限制详README/C-9。PLAN §9.1/§13.2。 |
| P6.7 | 定位健康守护节点（默认 `warn`）+ 单测 | AUTO | P3.3 | done | 9608659原观察节点/纯核/schema/docs；a8b2ba7修复真实Humble rcutils_logger同一调用点从warn切info时ValueError退出，按等级分开调用点，文本/等级/策略不变。假日志器回归与真实ROS日志器红绿验证，包102/0/0/0；实际两模式启动健康节点持续运行并正常退出。仅warn，无运动接口，TODO-H1不实现；最新样本配对/轮式父帧/现场误报与性能见C-10及README。PLAN §6.3。 |
| P6.U1 | 驱动 D3=0.5、D6 odom z 修复的实机冒烟与启用 | USER | P3.8 | WAITING_USER | |
| P6.U2 | TODO-O1 odom 接线 A/B | USER | — | WAITING_USER | 本循环不切换 |

P6 最新接续（`28568e6`，覆盖上文较早子步骤的“CLI/外层待做”备注）：普通POI工厂/WS组件/配置之后，已注册nav_executor自己的navigation_executor入口，旧david_nav入口不变。help实际安装输出逐字相等，python -S确认help不导入ROS/YAML/WS；启动打印生效配置及库版本。私有Context/显式executor不装rclpy信号关闭钩子，100Hz轮询保持旧架构；INT/TERM/HUP仅请求stop，清理成功前保留ROS回调，迟到动作终态/重复cancel/回滚失败均有纯假件覆盖。新增25项，全仓1992/0/0/2；无用户栈时域99实际ROS+WS ping三种信号退出3/3，无目标/速度。Humble executor第一次shutdown=False会粘滞保留node/context，无法证明清理时可能需人工恢复，见REVIEW；未在沙箱出现。Dock配置仍I/O前拒绝，Dock/Spin、旧shim及P6/P7余项未完成，P6.1/P6.2/P6.4保持todo。证据staging/p64_outer_codex/及LOG、outer-runtime.md。

P6 Spin续作（`eba9e2d`）：Nav2Client.spin接纯SpinGoal/SpinBackend，默认不创建Spin资源；显式endpoint启用后两种动作只用一个ActionSession。保持signed yaw、纳秒截断、5秒server wait，返回单次终态；allowance是服务端预算，不替代未来Dock的发包10秒/结果spin_timeout+5等本地期限。新后端不新增[dock]日志或修改旧补偿策略；Dock仍被组装根拒绝，完整P6/P7未完成。详见LOG与staging/p62_spin_codex/。

P6 Dock流程续作（`8331f73`）：迁移的是同步编排纯组件，不是已启用的Dock运行入口；必须在受管worker内执行，不能阻塞ROS/asyncio线程。DockConfig所有策略显式传入，不把helper的5°/2次误当David的25°/0次。动作port仍需实现旧server/send/result期限与逐动作日志、取消后清理证明；未来线程桥接要等待worker退出，不能只cancel asyncio waiter。现golden从未改dock_navigator.py提取三方法做独立参考，切旧节点前需冻结参考fixture，不能让对照随新实现一起变化。用户栈状态每轮重新核查。

P6期限续作（`29548c3`）：ActionSession.execute新增可选send_timeout_sec/result_timeout_sec，默认None不改现有普通POI/Spin路径；等待bridge由TaskSupervisor托管，原ROS future不被取消。本地期限与源future的TimeoutError明确分开，已完成future优先于零秒期限。SEND_TIMEOUT/RESULT_TIMEOUT仅在清理证实结束后返回，否则抛出清理异常并保持busy，禁止Dock据此继续下一段。23项新增纯测试、相关67项通过，全仓2101/0/0/2；四文件已提交，生产指纹不变。尚未把旧Dock的5/10/分段秒数或冻结逐动作日志接到新API，没有启用Dock运行；受管worker/共享Nav2Client桥接仍为下一步，P6.2/P6.3保持todo。证据staging/p62_deadlines_codex/。

P6线程续作（`1b80f81`，覆盖上段“桥接/动作日志待做”）：DockWorker显式非daemon线程运行原同步流程，_DockMotion阻塞端口将阶段投递唯一loop/Nav2Client，完整raw feedback+extra距离回loop。navigate_pose/spin_action共享原clients/session和建消息逻辑，接5秒server/10秒send/分段result；on_timeout只作日志观察且finally清理，异常不吞成False。worker异常/清理失败保留busy，取消/close可重试；重复execute.cancel等线程真正退出，丢失close waiter不丢cleanup，完成的旧cleanup不重跑以免取消新动作。取消后的pose/sleep/check与最终结果均有隔离；外部CANCELED/ABORTED仍按旧实测判定。39项新检查、相关169项全绿，全仓2140/0/0/2，12文件已提交。当前未改旧节点或public factory，实际Dock端口/配置/入站/MultithreadedExecutor宿主接线、旧shim及P6/P7仍待。新增golden还读取旧run_action，切旧节点前应连同此前3方法冻结参考。证据staging/p63_worker_codex/。

P6工厂续作（`9140d06`，覆盖上段factory/地图TF待做）：RuntimeConfig.dock接受完整不可变DockRuntimeConfig，缺配置的Dock IDs仍在I/O前拒绝；配置None时普通资源数不变。DockObservations独立map订阅为reliable/transient-local/depth1，复制旋转原点完整快照；Dock TF用独立导航帧、旧2秒timeout与yaw公式，不能复用采点帧。公共create_runtime接同一manager/唯一backend/worker，注册direct dock，并在普通snap/retry前按ID路由；Dock整任务仅一次、失败保留旧ABORTED诊断/单次包装与审计，普通POI行为不改。完整raw反馈携DockGoal身份/代次和extra距离回loop，scalar进度节流；关闭顺序先manager/worker再backend/map/odom，超时资源保留可重试。新43项、全仓2183/0/0/2；SDK纯消息探针16组字节相等，无ROS init/DDS/目标。外层profile/CLI尚不产生DockRuntimeConfig，现console仍拒绝Dock；下一步同进程ROS入站/MultithreadedExecutor宿主与外层配置，不是部署或完整P6完成。证据staging/p63_factory_codex/。

### P6 参数桥续作

P6参数桥续作（`baaa4f4`）：公共DockNodeConfig及dock_config_from_parameters/declare_dock_parameters接完整旧16参数，保持名称、默认值来源、footprint字符串及ROS覆盖；缺项/未知/非法值提前拒绝，不替机器人选择helper默认策略。nav_executor新增--dock-config-file（profile/headless/flat YAML同名选择），接受完整flat或既有dock_navigator.ros__parameters文件；endpoint显式CLI/YAML/profile覆盖优先，其余按原file值，ResolvedConfig.task_config接公共factory，audit打印有效Dock值。通过旧navrun的实际David/示例precise渲染对照，无renderer/profile改动；0.4363rad/0次/0.05rad/s保留。新59项（参数43、配置15、factory链路1），全仓2242/0/0/2、18文件已提交。**只接配置桥**：ROS入站、MultithreadedExecutor宿主、同进程可选WS和旧shim尚未完成；console遇Dock IDs或显式Dock文件仍I/O前拒绝。P6.1–P6.4保持todo，完整目标active。切旧节点前除四个workflow/action参考方法外，也冻结新的旧参数声明名参考。证据staging/p63_parameters_codex/，生产指纹不变。

### P6 ROS 入站续作

P6 ROS入站续作（`b33442a`）：api.create_dock_ingress返回空owner，open建depth1 PoseStamped与reentrant NavigateToPose服务，核心状态通过loop处理，ROS execute callback线程等待内部Future，Dock实际同步流程仍在原受管worker/共享Nav2Client。REJECT忙策略/原日志、header按配置frame解释/z忽略、全部反馈字段及extra距离、身份/代次/取消隔离保留。TaskApi.submit可选on_settled回执在释放所有权后发出，抢占/close不再让ROS动作等不到结尾；cancel新增generation在transition lock内核对，旧WS事件/外部CANCELED/R2行为不变。source元数据透明进TaskEvent，WS仅转发websocket/旧空source，避免ROS生成ID被误报为server请求到达；普通事件订阅仍能观察ROS。close封口/等待所属任务清理，不销毁ROS资源，必须保持loop/ROS回调→异步等待executor quiescence（含SDK结果发布尾部）→destroy。新增25项（manager6、ingress19），全仓2267/0/0/2、11文件已提交。**仍未接外层MultiThreadedExecutor宿主**，console gate与旧入口不改，P6.1–P6.4保持todo；下一步宿主/同进程可选WS/旧shim，再P6.5–P7。详见dock-ingress.md及LOG，总目标active。

### P6 外层宿主续作

P6宿主续作（`87c5fc9`）：新nav_task_manager与navigation_executor console共用一个root，显式完整Dock文件启用同一TaskManager/共享Nav2Client；ROS参数声明/最终覆盖在factory前应用，解析remap后的入站与Navigate/Spin端点不得同名。默认WS不变，--no-websocket或executor.websocket_enabled=false关闭WS导入/连接/proxy/WS观察任务，冻结help不改。Dock节点名保留dock_navigator，MultiThreadedExecutor沿用CPU数且最少2线程，普通节点和SingleThreadedExecutor路径不改。

新增TaskRuntimeApi.settle与backend.settle：停止入站→join WS→ingress.close→runtime.settle（保持ROS/loop活，关闭任务/worker/session但保留clients/subscriptions）→停spin→host.quiesce（辅助线程，等待SDK shutdown、pool join、已排队结果发送Task）→runtime.close/ingress.destroy→TF/node/context。丢失waiter/反复取消不丢cleanup；未知SDK布局、首次shutdown False/异常、未完成队列Task保留资源不假成功。实际Humble base shutdown不join pool，Future.set_result还会enqueue发送Task；后者有独立红绿证据。适配仅承诺David已审查的_executor/_pending_tasks/_tasks_lock布局，不承诺其他SDK；不取新DDS请求，不保证远端收到结果。SDK升级需重验。

23文件已提交、工作树干净，新增32项（host20、outer12）、相关117/117；17包构建15.7s，全仓2299/0/0/2、2m11s，两包lint通过。SDK实际Task/Future纯探针无ROS init/entities/goals。生产status MD5 da1ef50b0f796a7d12d71374dc3815bd与.bashrc MD5 695bd38f807e689f084ed249ae855536不变。当前用户雷达/IMU在运行，S7下跳过真实ROS宿主无目标检查；不修改/启停3，不发ROS目标/速度。下一步先冻结旧workflow/action四方法、参数声明与CLI旧实现参考，再统一切重构旧shim/launch，防止两个运动owner同启；其后P6.5–P7。旧alias/生产不因此加载U2，总目标active。

### P6 固定金样续作

`9a724ca`（8文件）与`05c0556`（4文件）把87c5fc9时的navigation_executor.py、dock_navigator.py、precise_nav_bringup.launch.py原字节固定到david_nav/test/golden/legacy_entries/*.py.txt；README记录来源路径/末次变更8c4861b或35cdda7/独立SHA256。nav_executor CLI金样、nav_tasks workflow三方法/run_action/16参数名金样、nav_bringup site-binding金样均改读固定夹具，不再随将迁移的入口/launch变化。新增4项完整性及runpy路径检查，当前源码零行为变更、旧入口仍未切。旧david_nav自测仍有直接import旧模块，切shim时须迁移这些参考测试并增加真正shim测试，不能简单删除断言或改金样迎合实现。

两次完整验证分别2302/0/0/2与2303/0/0/2，17包构建14.6s/15.9s、测试2m7s/2m10s。最新专项CLI29、Dock113、site绑定2通过；相关包lint通过，最终工作树干净。S7下仅纯检查，生产与.bashrc指纹不变。仍未满足“全部AUTO完成+两轮完整验证”总完成条件。

下一步实际统一启动的已核实接点：nav_bringup/launching/graph.py目前precise同时起旧dock与WS，应改成一个共享manager；phase筛选依赖进程spec.name==navigation_executor；navrun/preparation.py仅按ProcessSpec.parameter_file生成参数artifact，所以新CLI所需完整Dock文件及有效（含mode覆盖）robot-profile快照必须一并生成，不能只改executable。executor_binding.py目前禁WS直接返回，需要保留ROS-only无site/WS依赖；tour与precise的ready_action、站点POI路径、ROS-only/Dock策略不应丢失。旧precise launch同样不能留下两个新owner；需保留navigation_executor.py路径/shebang/可执行位。preflight profile匹配列表当前没有nav_task_manager，新入口匹配要同步且不动3保护。相关定位/IMU问题仍跳过；P6/P7及旧shim未完成。

### P6 navrun 单一任务管理器启动图（9782092）

16文件已提交，navrun新图实际使用nav_executor/nav_task_manager，不再同时启动旧Dock与WS。precise+WS只有一个共享owner，仍在deferred executor phase通过Nav2 readiness后启动；precise无WS仅一个core-phase ROS Dock；tour无WS无manager，bench/chassis/teleop不新增manager。**时序差异**：precise+WS的Dock不再提前随core启动，而与WS一起等完整启动检查；ROS-only时序不变。旧david_nav precise launch、两个旧入口和生产alias尚未切换，不可与新图并启。

ProcessSpec.configuration_files列出额外CLI输入，preparation生成模式合并后的executor_profile.yaml以及完整dock.yaml，load_options在启动前检查存在性。显式传入profile/action/Dock/WS开关，防止示例机器人回落David默认值；保留已有site argv/空格路径及ROS-only不读取WS站点绑定。历史bundle缺快照须重新prepare，不静默回填；optional executable resolver继续预检安装，不新增会形成colcon环的nav_bringup→nav_executor依赖。

真实argv→load_configuration的6种机器人/模式组合发现example_diffdrive缺executor.grid_map_topic/poi_store_file，补其自身/demo/map与~/.ros/example_diffdrive/poi_store.yaml数据（红3/绿6）。验证mode覆盖、独立采点/导航帧、footprint/yaw/0次重试/0.05停稳、站点路径/Dock IDs完整到达public task config，无ROS或storage runtime。preflight识别新manager/兼容console/ros2 run入口，发现已有owner则拒绝navrun启动，shell/editor纯文本不误判；不添加kill规则、不执行真实清理（红12/绿17）。原图红7/绿12；相关179通过，附原preflight及lint49、executor lint3通过。

全仓2338/0/0/2，17包build14.7s/test2m17s；16文件SHA一致、diff --check通过、最终工作树干净。现场雷达/IMU在运行，S7只纯检查/低负载构建，真实ROS宿主/新图无目标检查暂缓。生产status MD5 da1ef50b0f796a7d12d71374dc3815bd与.bashrc MD5 695bd38f807e689f084ed249ae855536不变。

下一步协调旧navigation_executor.py、dock_navigator入口和precise launch统一迁移，保留旧路径/shebang/可执行位、独立Dock历史5°/2次与precise25°/0次差异；旧david_nav算法对照改读已冻结金样，另测真实shim，不能同时启动两个new root。随后P6.5 R1/R2/R3/R8、事件/健康/P7和两历史skip。总目标仍active，未满足全部AUTO完成+两轮fresh全绿。

### P6 旧入口和精准 launch 统一迁移（884e4d5 / 58b9226）

884e4d5（25文件）把两个旧程序变为约12行薄入口：navigation_executor.py→nav_executor.app.cli.main，dock_navigator.py→app.dock_cli.main。main退出码传播；路径/shebang保留，检查发现执行器原source mode为100644，现补成100755，非系统授权配置。CMake仍安装兼容小副本，实际实现解析到重构源码（symlink-install），不再维护旧重复运行实现。david_config/dock/standalone.yaml安装完整16参数，与固定旧AST逐项相等；独立Dock仍0.087/2次、0.05rad/s，ROS参数最终覆盖；R8的/goal_pose缺陷刻意留下一独立提交。dock_cli默认读该数据、强制关闭WS；显式--dock-config-file可另选策略，help不查资源。

旧precise launch同步只起一个manager：WS+Dock经旧等待脚本在Nav2就绪后一起启动；无WS时单一core-phase Node随原底盘延时。八个ROS覆盖保持/dock_goal、0.4363/0次等旧精准值。已有旧/新manager时整次launch不启动新进程、不kill；WS配置/入口缺失也不再留下半套本地Dock。pgrep仍是保守快照而非原子全局锁，禁止混启独立手工/旧进程。直接执行旧executor带--dock-poi-id须给完整--dock-config-file；不再通过外部Dock进程转发，旧precise launch已自动给齐。生产未部署。

旧david_nav六处算法/CLI测试改读已固定.py.txt（断言不删），新测试独立检查实际shim转发/argv/退出码/可执行位。首轮full2348出现1失败，定位David site测试仍查活动入口常量，改读同一冻结参考保留原断言后绿。沙箱验收脚本仅改配置：WS场景移除第二个Dock，完整策略+精准ROS覆盖交同一root；各场景POI路径均临时目录。新增纯测试检查其command builders和无第二owner，不执行ROS fixture/目标。

新增11项；专项43及88通过，后补shim+lint12/site4；实际source/install各两次--help、安装副本字节、完整策略与实现路径检查通过，无ROS init。完整2349/0/0/2，build31.7s/test2m19s后提交884e4d5。随后58b9226仅2文件6行样式整理：4处历史docstring、unused import及长行；12文件flake8与4文件pep257零问题、专项20通过，第二次full2349/0/0/2、build13.8s/test2m13s。初次失败和各绿证据保留staging/p64_shims_codex/。25文件本地远端SHA一致，最终干净。

现场雷达/IMU在运行，S7只纯测试/低负载构建，真实共享Dock宿主/launch及目标验收未执行，不修改或启停3、不发目标/速度。生产status MD5 da1ef50b0f796a7d12d71374dc3815bd与.bashrc MD5 695bd38f807e689f084ed249ae855536不变。下一步P6.5 R1（缺字段、畸形目录、非JSON后同连接仍处理合法指令）、R2/R3/R8分别红绿提交；R6仅测量脚本，然后事件/健康/P7和两历史skip。P3.5等现场/ROS步骤仍跳过；所有AUTO完成尚未成立，连续绿不等于目标完成。

### 最新续作：P6.5 R1（43e110f）

畸形消息隔离已完成，14文件单独提交、工作树干净。解码器统一MalformedCommandError并保留CommandParseError/ValueError兼容；未知action仍仅忽略。缺字段/坏形状/数值转换异常携带可恢复request_id/sub_id，有ID经原outbox回on_error，否则记日志继续。同一消息处理中的异常不再拆WS连接；真实socket迭代、ping/heartbeat/outbox发送失败仍重连，CancelledError仍向上传播。合法G1–G16帧/文本不变。

目录在调用存储前整批校验并物化：未知ID缺name也拒绝整批，合法未知ID仍忽略；这是R1明确修复，不再声称与旧畸形输入等价。没有实现通用存储I/O回滚或handler副作用回滚，任务崩溃清理留R2。ADR-0007记录该边界。

新增26项（新文件25+未知action带ID用例1）。初始红23失败/1通过；修复后专项85通过，真实127.0.0.1临时WS服务证实连续坏消息后合法指令在同一连接完成，ROS/动作仅内存假件。最终17包构建15.2s、全仓2375/0/0/2、测试2m19s，证据staging/p65_r1_codex/。14文件本地远端SHA一致，生产status及.bashrc两基准MD5不变。

本轮初检雷达/IMU在运行，按S7纯测试/低负载构建；后来SSH中断，重连时uptime约5分钟、未见原栈进程，重启原因未知，非本轮操作。断连前全量命令未留下日志或存活构建，确认后重新验证；不将丢失SSH当成验证通过。全程不修改/启停3、生产或权限配置，不启动ROS、不发目标/速度。下一步R2先红：FakeBackend.navigate抛RuntimeError后可靠on_error、清理成功再释放active/回IDLE、下一指令正常；清理失败不得假报空闲，旧G12外部CANCELED仍保留。再R3/R8分别提交、R6测量脚本、事件/健康/P7。总目标active，2历史skip仍待P7.4。

### 最新续作：P6.5 R2（22f4087）

12文件单独提交，11项新增测试。_run不再吞代码异常，由TaskSupervisor按当前worker/name/generation与异常对象匹配上报一次FAILED（执行器内部错误: 异常类型），实际WS adapter输出既有on_error。另托管恢复协程，取得同一transition lock后二次检查active对象，后端清理成功才清active/回IDLE并发一次FAILED完成回执。清理失败仍ERROR/持有所有权，允许cancel/preempt/close重试，不无界自旋；正常cancel和G12外部CANCELED不变。已报告崩溃后再abort不追加误导paused；本次清理之前已记录的worker原异常可忽略，清理中产生的新worker异常仍令该次失败。ADR-0008详述。

红9失败；实现后核心24绿，补同ID抢占在恢复前排队、取消期间新异常两项。首次整包回归发现1条超长行及旧完成回执期待CANCELED，按R2改为保留FAILED后最终17包build15.4s、全仓2386/0/0/2、test2m23s。32项关键检查PYTHONASYNCIODEBUG=1重复10轮全部通过。证据staging/p65_r2_codex/，12文件SHA一致、最终干净。初检雷达/IMU14878/14914及15106/15119在运行，S7仅纯测试/低优先级双并发构建，不启停/修改3，不发ROS目标/速度，生产和.bashrc指纹不变。

下一步R3反馈托管专项：当前ActionSession._queue_feedback已call_soon_threadsafe→同步_feedback，无裸create_task；supervisor强引用/反馈线程与取消隔离已有覆盖。先核验当前新链路和保留旧bridge的实际可达性，针对旧缺陷与新保证补明确回归/ADR，勿为制造“修复”退回弱引用或改反馈频率。然后R8默认/dock_goal、R6仅测量脚本、事件/健康/P7。总目标active，所有AUTO尚未完成，两历史skip仍待P7.4。

### 最新续作：P6.5 R3 / R8（06a4ce2 / b767ea9）

R3机制已在88d3bc4的ActionSession引入、884e4d5接入口：call_soon_threadsafe持有同步callback/message，不创建反馈Task。本轮06a4ce2补4项测试/ADR-0009（6文件），同一禁止裸spawn检查对保留旧bridge第180行红、对新任务运行包绿；100帧真实线程→loop队列在GC后不丢且完成后可回收，无新增Task，取消前排队及结束后迟到反馈不污染替换任务。旧bridge仅历史参考/测试使用，new packages已有禁止反向导入检查。没有人为修改当前反馈调度/频率。专项42通过，受影响两包构建/测试全绿；当时总结果2390含其余包上轮结果，不作fresh全仓宣称。

R8 b767ea9（9文件）仅将安装数据standalone.yaml的goal_topic由/goal_pose改/dock_goal；裸dock_cli→实际配置/公共参数声明测试先红后绿。冻结旧node参考不改，对比明确仅此一项差异；其余15值、16参数名、5°/2次、0.05和显式ROS覆盖均保持，precise原本/dock_goal不变。安装真实_default_policy与源码模块落点纯探针通过，无ROS init/目标。专项10通过；最终fresh17包build15.0s、全仓2391/0/0/2、test2m14s，源码SHA一致、工作树干净。

中途主机再度不可达，旧SSH会话最终255；重连uptime约1分钟、原colcon等进程消失、test日志半截且result不存在。未执行重启/用户进程信号；无持久化上一boot journal，原因未明。预检测试为假进程/假信号，生产指纹和14个相关文件SHA不变，确认终止后将中断日志保留test_interrupted_boot.log再完整重跑。证据staging/p65_r3_codex/和p65_r8_codex/。初检雷达/IMU运行，始终S7式低负载纯验证，不起ROS、不修改/启停3、不发目标/速度。

下一步R6只做只读空闲资源测量脚本（当前pidstat未在PATH，禁止安装；可用既有psutil/采样器但需处理首样本预热、PID重用、完整窗口和单核CPU百分比，不能伪造实测）。不自动启动被测栈、不因此改100Hz调度。随后事件/健康/P7和两历史skip；全部AUTO仍未完成，总目标active。

### 当前未提交续作：R6测量工具（连接中断；仅本地交接更新）

8文件：nav_executor/app/idle_profile.py、test/test_idle_profile.py、setup.py、README/CHANGELOG、docs/migration.md/idle-profiling.md/adr-0011-r6-measurement.md（前两路径分别位于包nav_executor/nav_executor与nav_executor/test）。显式PID只读Linux stat，无ROS/WS/目标/信号/新依赖；输出新目录metadata/CSV/summary，不覆盖旧路径。baseline百分比留空，按总CPU增量/单调时长算单核百分比；PID重用、计数/时钟回退、退出/权限/中断给不完整证据。R6还须>=60秒、<=1秒间隔、预期数量/最大间隔合格和人工确认idle，才给3%布尔比较，否则null。不改100Hz运行架构、没有导航CPU实测。

初专项功能通过但lint有行长；稀疏两端采样错误合格的问题已先红后绿修正。远端最后专项27过/1个测试装饰器行长失败；全量构建和已安装0.03秒自有shell诊断已跑（complete=true/r6_eligible=false），全量测试中途SSH断开，原本有该lint待修。最新本地test文件修好行长并新增不等间隔加权测试，本机24项全绿、长度检查零问题，但尚未上传。远端test状态未知，**不可推断因SSH255已结束；恢复后先核对进程/日志，再上传单文件及重跑**。

原session80118/读取98372均255；.51多次No route/timeout，按RUNBOOK只读22端口扫描和ED25519匹配未找到新地址，未登录其它主机。没有确认本次是否重启；未执行重启/3操作。测试附近真信号接口已核查为fake或自有非ROS子进程，未找到reboot/poweroff调用，原因仍不明。生产最后已知两MD5不变；断连后无法复核。staging/p65_r6_codex保存targeted、cadence_red、targeted_verified/targeted_final（都有已知lint失败）、build_verified、installed_probe/installed_self_diagnostic、test_verified（状态未知）；result_verified是否生成待查，不用旧结果代替新验证。

本轮文档只能本地更新，尚未同步远端；代码未提交、不能标P6.5 done。先完成R6门禁与提交，再P6.6事件（PLAN实际§9.1；PROGRESS旧§6.4引用已漂移）、R7 Dock ~/events String JSON、P6.7健康/P7。保留日志和草稿，不覆盖生产或启动3。总目标active。

### R6 续作：专项与构建通过，再次失联（2026-09-16 15:10 CST；仅本地记录）

第一次中断现已核实：重连 uptime 约1分钟、无原 colcon/pytest/compiler，test_verified 半截含NUL、result_verified不存在；保留 test_interrupted_boot.log。读全 RUNBOOK/当前PROGRESS，核对生产两MD5一致，确认无活跃验证后上传最新test文件，八文件SHA全部一致。本机27项纯门禁、远端29项（含flake8/pep257/docstrings/architecture）绿。17包低负载build29.0s成功，安装入口0.03s自有shell诊断成功但r6_eligible=false，不是导航CPU实测。

第二次full进入nav_executor（收集308项，最后读到22%）又断连：24514/15883均255，直接43494超时、邻居INCOMPLETE。远端原测试是否仍运行、是否又重启、本次result_recovered状态未知；**不要推断SSH结束等于测试结束，不再盲目循环重跑全仓**。证据新增 targeted_recovered/build_recovered/installed_recovered/test_recovered.log 与 installed_self_recovered/；输出目录不可重用。下次先只读状态检查，再核对受影响包门禁和全量结果。

当前boot journal仅有本次启动，出现系统既有2分钟硬件watchdog配置但无复位因果证据；不改系统。test_recorded_startup已只读核验使用假平台步骤和自有非ROS子进程信号。根因仍需人工核实，不能推断是测试或排除相关性。未动3/生产/权限/串口/目标；最后可读时生产指纹仍匹配，断连后无法再核。本地五文档尚未同步；无新源码草稿，仍8文件未提交，commit草稿已上传staging。目标active，不算P6.5完成；之后继续事件/健康/P7。

## Phase 7 清理 · 文档

| ID | 任务 | 类型 | 依赖 | 状态 | 提交 / 备注 |
|---|---|---|---|---|---|
| P7.1 | `david_system_env` 迁移（旧路径软链） | AUTO | P1.1 | done | 55034e9：三个原资产逐字节迁入纯数据包，旧source路径相对链接；无自动环境hook/ROS节点。旧包构建/运行依赖新包，安装先解析新文件REALPATH，避免隔离安装悬链；实际symlink及普通隔离资产安装/四路径source等价探针绿。17纯资产用例+2样式+CTest=20项，三包1281/0/0/1；引用清单/部署边界写新包docs/migration.md及C-11。不改生产或3，不切原绝对profiles路径/域/白名单。 |
| P7.2 | legacy 隔离提案（只在重构分支移动，清单写入"待确认"） | AUTO | P6.4 | done | 5f7acb5：docs/legacy_inventory.yaml与legacy_isolation.md列8组42文件、拟议目标、引用与保留项。9新纯检查，nav_bringup977全绿；无候选实际移动/删测试/取消安装。旧碰撞配置仍被条件launch引用、separator仍有位置环/bench消费者、5个shell身份未确认，见C-12；逐项批准/实施属P7.U1。 |
| P7.3 | 文档：architecture / parameters / porting / behavior_policy / events / runbook；修 CLAUDE.md 与 README 漂移 | AUTO | P6.* | done | 61b444d：六命名文档与root/package指南齐备，参数表仍由schema生成；纠正域号、separator链、全局清理/push等过时指导，并修包README/事件/schema说明漂移。15新链接/CLI解析/指南测试，专项18绿，四包1751/0/0/1；只改文档/schema描述/测试，不改运行逻辑。ca42041继续补SPEC R10：waypoint启动log由v2纠正v1，dead_reckoning说明明确默认waypoint/显式选择，semantic入口DEPRECATED并说明public API；仅文字、3新回归/AST及单行hash证明。CHANGELOG已列R1–R10与历史阶段区别，R6实测仍未完成。P7.A1/A2后续已完成，原状态为历史。 |
| P7.A1 | PLAN§13.1运维CLI补齐：navrun down/status/ls/show/check/gc；§12.5的show生效参数/叠加差异导出 | AUTO | P4.2, P1.2a | done | 535ab18 ls/show，f063b78 check，6723571 status/down；6ee078b补gc预览/应用。24新纯测试，nav_bringup1121/0/0/0；共享up/wrap保留原预算但跳过不完整/坏记录，fd相对删除与原inode复核、up租约及每root维护锁。永久留manifest/summary；失败可能部分删除，未清理真实用户记录。C-14/C-15记录管理范围/清理边界。 |
| P7.A2 | PLAN§12.6 navcfg validate --robot选择器与mode名工作流 | AUTO | P2.5 | done | 21cf8ba：保留原显式文件接口，新robot/mode/site名选择source或installed资源；复用discover_inputs/prepare_bundle，不调用plan_up/主机观测/ROS/写run。唯一site自动选，多site显式指定，chassis/teleop无site依赖；身份/模式叠加/原安全规则均重查。29新测试、nav_bringup1150全绿，安装David五模式/样例四模式绿。真实可执行/在线/实体及并发原子性不在离线证明范围。 |
| P7.A3 | 补SPEC P7协议与决策文档：docs/protocol/nav_ws_v1.asyncapi.yaml、nav_ws_v1.md、金样/schema对应、ADR背景任务/可靠outbox/单次重试/POI真源及冻结日志消费者清单 | AUTO | P7.3 | doing | f31b3d2：13文件，协议/四ADR/日志及README已交；11新测试（10通过、1缺jsonschema skip），executor331/0/0/1。旧golden不变，7段历史注释/docstring逐字保存，有效消费者逐项核查。完整payload及官方AsyncAPI文档校验尚未运行，TODO-ASYNCAPI-1；S4不安装，不把字段对应当完整验证。已有ADR0007-R1保留。 |
| P7.4 | 全仓 Google 规范收尾：新包 lint 零告警；docstring 规范检查全绿 | AUTO | 全部 AUTO | doing | 42bd363修绑定覆盖保护，c6199da修四入口完整匹配，18项新增回归，增量145/185绿；54处说明及4边界测试仍5文件未提交，增量240绿、AST/历史归档核对通过。因生产指纹变化暂停，不能称源码干净或说明批已提交。草稿态239无CJK；schema/C-16/R6/实机与最终条件未完成。 |
| P7.U1 | legacy 清单确认、sudoers、udev、固件 SecurityLevel、合并策略 | USER | — | WAITING_USER | |

## 用户直接要求的变更（不是循环自主做的）

| # | 内容 | 状态 | 说明 |
|---|---|---|---|
| U-1 | 2026-09-15 用户要求：记录当前位姿为 `~/semantic_map/named_poses.json` 的 POI_017；alias 0 并入 alias 4 的 server 模块（navigation_executor，状态上报与 POI 接收相同），agent 经 8101 下发去 POI_017 时走两段式精准入位 | done（生产已落地，待用户重启 alias 0 实测） | 重构分支 `8c4861b`（`david_nav` 229 tests 全绿，含新单测与 golden 同步更新）。生产：`dock_navigator.py`（新增 `dock_to_pose` 动作接口，NavigateToPose 类型，支持结果/取消/feedback）、`navigation_executor.py`（`--dock-poi-id` / `--dock-action-name`，dock POI 不扩圈、失败不重发）、新模块 `dock_routing.py`、`precise_nav_bringup.launch.py`（nav2 就绪后起 executor，参数 `start_executor/server_uri/pose_file/dock_poi_ids=POI_017/nav_ready_timeout`）；install 里两个拷贝件已同步、`dock_routing.py` 已补软链；未重编译。备份 `~/david_nav_backups/20260915_alias0_server/`。`.bashrc` 未改（alias 0 仍是 `ros2 launch david_nav precise_nav_bringup.launch.py`）。POI_017 按用户确认取 04:22 的 base_footprint 位姿（04:32 时有人挪动过机器人）。生产指纹因此重新基线（见上表） |
| U-2 | 2026-09-15 用户要求：dock 类 POI 在发送预备点前校验完整车身轮廓不占静态地图占用格；脱困后重发规划/路径跟踪，避免恢复动作直接令导航成功；停稳角速度阈值放宽到约 0.05 rad/s | done（仅重构分支，未改生产） | `35cdda7`：静态地图 footprint 面积碰撞检查（含旋转地图原点、越界/未收到地图保守拒绝）；两棵 BT 让 EmbeddedRecoveryGuard 包住完整规划/FollowPath 管线；默认阈值 0.05 rad/s。相关纯测试 65 个全绿，`david_nav` 低优先级构建通过，新增测试已由 CMake 注册 |
| P0.12 | [sandbox] 运行 `david_nav/sandbox_tests/run_sandbox_checks.sh`（假 nav2 + 真 dock_navigator：两段目标与成功、取消；真 executor + 假 agent server：POI_017 → on_arrived） | AUTO，done | 第 6 轮 09:16 domain 99 实跑 **3/3 通过**（21.7 s，无残留进程）：dock_to_pose 两段目标坐标正确并 SUCCEEDED、取消只发出一段并 CANCELED、真 executor 收到 navigate_to POI_017 后经两段式入位回 on_arrived。04:43 那次因用户 3+0 在运行被拒；第 6 轮第一次尝试被拒是误报——自己的 ssh 命令行里含被监视的进程名，`pgrep -f` 匹配到了启动脚本的 shell；修复已由 `05afd2e` 提交，命令行故意带被监视进程名时两个沙箱套件 3/3、4/4 通过，flake8 通过 |

## 待确认（循环遇到的设计取舍）

| # | 取舍 | 当前做法（最保守） | 需要用户决定吗 |
|---|---|---|---|
| C-1 | ament_pep257 默认规则要求 docstring 摘要写在第二行（D213），与 Google 风格（摘要在第一行，D212）冲突 | 旧包（david_cmd、david_nav）：沿用 ament 规则但改为 D212；新建/迁入包：`--convention google`（同时要求公共 API 有 docstring） | 否，符合"Google 工程规范"目标；如有异议可改 |
| C-2 | G12 外部取消（非本执行器发起的 CANCELED）后，core 不清 `active_task`、状态停在 NAVIGATING | 作为现状固化进 golden 测试，不修 | 重构到 P6.5 时是否把它当缺陷修复（回 IDLE），届时列入 ADR |
| C-3 | PLAN P0 原要求"用户先提交未提交修改"；循环改为在重构分支上做基线快照提交 `93676a5` | 生产分支与工作树保持原样 | 否 |
| C-4 | David 上 `/dev/shm` 现有 26 个 Fast DDS 共享内存段（历史上该数量曾导致同机发现整体失效）；第 3 轮沙箱 bringup 两次偶发失败（一次 `Node not found`，一次 lifecycle `change_state` 应答超时） | 循环按 S3 不清理 `/dev/shm`，只靠重试 + 冷却；不影响 3+0/3+4（它们启动时自带 DDS 卫生清理） | 可选：你方便时重启一次或跑一次 alias 4 的 DDS 卫生，后续沙箱任务（P3.5、P4.2）会更稳 |
| C-5 | David 的 `~/.ros/log` 有 40,684 个文件、2.1 GB（最早 2025-07-14；每条 ros2 命令、每个 ROS 进程各写一个）。来源前几名：python3 16,148、tf2_echo 8,091、rviz2 1,268、robot_state_publisher 1,215、run_loc_online 619。第 5 轮前的沙箱检查也往里写过少量文件 | 循环不删除用户文件；沙箱检查已改用临时 `ROS_LOG_DIR` 并在结束时删除；P5.1 让导航运行把 `ROS_LOG_DIR` 指进运行记录（受保留策略约束）；P1.5b 的 doctor 增加日志目录体量检查 | 可选：确认后可清理旧日志；tf2_echo 八千多个日志说明有脚本在反复调用 `ros2 run tf2_ros tf2_echo`，可顺带排查 |
| C-6 | 速度平滑器线速度上限 `velocity_smoother.max_velocity[0]` = 0.20，DWB `max_vel_x` = 0.15；参数文件注释写"两者必须一致"，07-16 DWB 改为 0.15 后没有同步 | 渲染时不绑定，保持 0.20（行为不变） | 是：平滑器上限跟随 `limits.max_vel_x`（0.15），还是绑定到底盘硬限 0.20 |
| C-7 | `recoveries_server` 参数段在 Humble 中没有对应节点（已由 behavior_server 取代），其 `global_frame: map` 与恢复行为实际用的 odom 不一致，但不生效 | 为与原文件等值保留，帧名也绑定到档案 | 否（清理阶段可删除） |
| C-8 | DDS 孤儿判定的实际映射可见性不足：2026-09-15 agx 只读检查有 117 个 Permission denied；且 /proc 快照不是跨 DDS 进程的原子屏障 | 不完整或变化中的证据一律保留段并告警；逐文件复查身份与映射，只在临时目录测试删除，不清真实 /dev/shm、不停 daemon、不改权限 | 最终审核确认实际维护窗口和必要可见性；不作为继续其它 AUTO 的阻塞 |
| C-9 | PLAN/PROGRESS要求NavEvent但未列字段、时间与最终诊断生存期 | 五字段语义+Unix wall stamp/event_id；任务/Dock已接，消费者原时间/LRU4096；navrun持久UUID:offset、有界游标/独占租约与字段投影，不改旧JSONL。c830291使用顺序临时末尾publisher续发真实run_stopped，不增常驻节点；3s协作/5s父执行期限，cleanup保留所有权；预检拒绝只留本地，helper自身终态本地source=event_flush避免无穷递归 | 部署前审核新增临时收尾开销、QoS volatile/早期任务发现窗口、cursor与DDS ACK不等于应用处理/持久化、失败后非自动耐久重放、多monitor非exactly-once、白名单/约5秒采样。tail代码已接；真实DDS/性能现场验收不阻塞其它AUTO |

### C-10：导航健康观察的实现边界（2026-09-16）

健康节点只观察TF与底盘Odometry，不依赖IMU，不改/启停3，不创建运动或取消接口。
最新TF与最新轮式积分位姿在5Hz tick配对，各自新鲜度须≤0.5s；不插值同步。
wheel child→导航基准点按消息时间转换，轮式header父帧用新增chassis.odom_frame，
与定位frames.odom独立（David分别为odom_combined/odom）。窗口首对航向差对齐坐标轴，
比较两条位移向量残差而非绝对坐标；≤2s窗口/最多4096对，短窗口也判断，
缺失/过期/未来/时钟回退清历史。残差>1m报ERROR，但不归咎哪一提供方。
配置stop明确拒绝，自动取消TODO-H1尚未实现/未启用，不能靠改一个YAML启用。
部署前审核最新样本时间误差、滑移/重定位误报、阈值及单常驻节点开销；
真实DDS/定位质量验收不由纯假件或CDR探针替代，不阻塞其它AUTO。

## WAITING_USER 汇总

旧代码隔离审核（C-12）：基线55034e9，docs/legacy_inventory.yaml逐项42文件与拟议
david_nav/legacy目标，8组覆盖PLAN§7.5。未移动/删除任何候选，未取消编译/安装/测试。
旧collision_monitor_params仍有条件launch引用，PLAN无引用前提不成立；separator有
位置环/bench消费者，不能破坏alias8。PLAN未列5个停更shell名称，清单5候选身份
明确未确认。保留旧nav/headless/shim/bench/golden/配置，外部消费者未穷举。
P7.2提案已交，不等于P7.U1批准实际移动；不动生产/3/console服务。

共享环境部署审核（C-11）：P7.1只迁所有权，不改三资产字节。ros_env.conf仍明确
指向生产 `/home/agx/ros2_ws/src/david_nav/config/fastdds_david.xml`；旧source路径保留
相对软链，安装时REALPATH→symlink或兼容副本。当前重构XML实际UDP仅loopback+SHM，
文件中的历史有线网卡说明与有效配置不完全一致；本次不据注释加地址、不放WiFi。
迁移不代表所有运行入口已切到重构XML；部署/跨机器时审核绝对路径和外部引用、
旧软链保留期。其它工作区/生产systemd/3未穷举或改动，不能因本包迁移要求修改3。

运行查询导出边界（C-13）：535ab18只读ls/show，无ROS/信号/锁操作。unfinished
是记录状态，不证明owner存活；descriptor固定目录，但并发文件写入仍非原子快照。
默认摘要不含原始manifest命令/environment/linked，但显式参数/事件/summary可含敏感值，
分享前审核。diff是冻结robot→robot+mode，Nav2与teleop覆盖分别导出，不计算runtime差异，
不自动生成第二份配置真源或回写模式。ls/show每文件8MiB、记录32MiB、4096项、8层，超限拒绝；
映射别名拒绝；大记录需人工按原工件分析，不以截断结果冒充完整。gc已由6ee078b完成，
其删除边界见C-15，不转成人工编码待办。

合作控制边界（C-14）：6723571 status/down仅新up租约owner，非wrap/手工/3的全局停止。
同uid内核peer+代次握手；无PID信号、无新线程/daemon/socket文件，复用observer的0.1s轮询。
正常finalize+同代次租约释放才cleanup_complete，crash/换inode/新owner/超时不假成功或强杀。
unknown不证明请求未接收；初始准备、尾部、日志阻塞可无响应，人工可查看前台/记录。
active不是Nav2 ready，inactive不是实体停车证明；少量IPC开销/真实停止验收仍待。

清理边界（C-15）：6ee078b gc默认应用、dry-run不写；30份/20GiB逻辑字节/7天bag规则保持。
共享up/wrap安全筛选仅schema1且runid/结束时间/真实无硬链manifest与summary齐全的记录；
未知/未结束/不安全内容跳过，预算不保证限制这些保留项。manifest、summary永久不删，latest不改。
应用持up lease+per-root lease；fd相对/no-follow检查、planned inode复核防可检测目录替换，
symlink只unlink本身。根路径祖先需可信；不承诺管理员挂载变化或同uid最后unlink竞态隔离。
删除增量执行，失败可部分完成、无内置恢复；实际回收磁盘块不等于报告逻辑字节。
本轮仅test私有临时记录，未清理真实用户数据。部署使用前先dry-run核目标；不阻塞其余AUTO。

版权声明核实（C-16）：原ament_copyright实际执行15 errors / 19 files，不是缺依赖。
david_cmd包内6个运行模块、3个scripts、conftest与5个单测缺声明，完整清单在REVIEW_PENDING。
package维护者/git作者信息不直接证明每个文件的权利主体/年份，LICENSE末尾模板也无证据；
不伪造版权头、不改检查范围换绿。下一步只读追溯来源，如无可靠声明需项目方确认。
这项和headless Ctrl+C历史skip仍不满足最终仅允许缺依赖skip的条件；不阻塞其它AUTO。

P0.U1、P1.U1、P4.U1、P4.U2、P5.U1、P6.U1、P6.U2、P7.U1
