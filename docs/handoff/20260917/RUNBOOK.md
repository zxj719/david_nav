# David 导航重构 · 自动执行循环手册

> 每一轮 /loop 迭代**从头读本文件**，然后按 §4 执行一轮。本文件是循环的唯一操作规程。
> 规格来源：`../2026-09-14-david-nav-system-refactor-plan.md`（总方案，简称 PLAN）与
> `../2026-09-14-david-nav-executor-refactor-spec.md`（执行器 spec，简称 SPEC）。
> 进度：`PROGRESS.md`（任务状态，唯一真源）；日志：`LOG.md`（每轮追加）。

> 2026-09-17 用户要求加速：不再每轮/每次提交都跑全量。采用下述按风险验证规则，
> 合并同主题文档审查批次，优先真实缺陷与未完成的功能/验收项；安全规则不变。

---

## 1. 目标与完成定义

**目标**：在独立分支上按 PLAN / SPEC 完成重构中**不需要人工参与**的全部工作，使：

1. `colcon test` 在重构工作空间内**全部通过**（0 failures、0 errors；允许的 skip 只有"依赖未安装"类，且需在 PROGRESS 列明）。
2. 新建与迁入的代码**符合 Google 工程规范**：
   - Python：Google Python Style Guide —— 公共模块/类/函数有 docstring（`Args:` / `Returns:` / `Raises:` 分节），公共 API 有类型注解，命名与 import 分组符合规范；由 `ament_flake8`（ROS 默认配置，行宽 99）、`ament_pep257`、以及本仓自带的 docstring 规范检查单测（AST 扫描）强制。
   - C++：Google C++ Style（ROS 2 采用的变体）—— 由 `ament_cpplint` 与 `ament_uncrustify`（ROS 默认配置）强制；只作用于新建或迁入的包，未迁移的旧 `david_nav` C++ 豁免。
   - 通用：每个包有 README、参数 schema/文档、单测；提交粒度小、信息完整。
3. 生产环境（3+0、3+4）在整个过程中**不受任何影响**。

**完成（DONE）**：PROGRESS 中所有 `AUTO` 任务为 `done`，且连续两轮迭代的完整验证（§4 步骤 6 全量模式）都通过。之后每轮只做全量验证与报告，不再改代码；并按 §6 通知用户。

---

## 2. 硬性安全规则（任何情况下不得违反）

| # | 规则 |
|---|---|
| S1 | **不下发任何运动指令**。唯一允许向速度话题发布的是零速，且只在 `ROS_DOMAIN_ID=99` 沙箱内 |
| S2 | **不修改**：`~/ros2_ws/src`（生产工作区，含其 git 分支/暂存/工作树）、`~/ros2_ws/{build,install}`、`~/.bashrc`、`/etc/**`、`~/lightning_ws*`、`~/wheelhumanoidrobot/**`、`~/scout_ws/**`、`~/semantic_map/**`、`~/rosbags/**` |
| S3 | **不杀、不重启、不信号**任何不是本轮自己在沙箱里启动的进程；不执行 `ros2 daemon stop`（除非 `ROS_DOMAIN_ID=99` 且仅针对本轮环境）；不清理 `/dev/shm` |
| S4 | **不安装软件**（apt / pip / 源码安装一律禁止）。foxglove_bridge、import-linter、rosbag2 MCAP 插件等为 TODO |
| S5 | **不 push**、不改远端；只在 David 本机的 `refactor/nav-system` 分支上提交 |
| S6 | **不打开真实串口** `/dev/wheeltec_controller`，唯一例外见任务 P0.8（驱动进程不存在时只读 2 秒） |
| S7 | 若检测到用户的导航/定位/底盘栈正在运行（§4 步骤 2），本轮**不启动任何 ROS 沙箱进程**，编译降为低优先级低并发，只做代码与纯单元测试 |
| S8 | 需要用户参与的步骤（改 alias、实机冒烟、基线资源测量、固件/udev/sudoers）一律标 `WAITING_USER`，**跳过**，不阻塞其它任务 |
| S9 | 不删除生产仓库中的任何文件；"隔离 legacy"只在重构分支上移动，并在 PROGRESS 列出清单等用户确认 |
| S10 | 与用户已确认决策冲突、或 PLAN 未覆盖的设计取舍：按 PLAN 精神取最保守（行为不变）的做法，并在 PROGRESS 的"待确认"栏记录，不擅自改变行为 |

---

## 3. 环境

| 项 | 值 |
|---|---|
| 主机 | `ssh -o BatchMode=yes -o ConnectTimeout=8 agx@172.16.22.51`（DHCP 可能漂移，见 §4 步骤 1） |
| 生产仓库 | `~/ros2_ws/src`（git 顶层），分支 `feature/dwb-control-opt`，**只读** |
| 重构工作空间 | `~/ros2_ws_refactor/`，其中 `src/` 是生产仓库的 **git worktree**，分支 `refactor/nav-system` |
| 构建 | `source /opt/ros/humble/setup.bash && cd ~/ros2_ws_refactor && colcon build --symlink-install --metas src/colcon.meta`（**不要** source `~/ros2_ws/install`；`colcon.meta` 让 shim 包沿用生产的 BUILD_TESTING=OFF） |
| 上传改动 | 在本地 scratchpad 编辑文件 → `tar czf - ... \| ssh ... tar xzf - -C ~/ros2_ws_refactor/staging/<轮次>` → 远端 `cp` 到 worktree。`staging/` 已放 `COLCON_IGNORE`（否则 colcon 会把其中的 CMakeLists 当成重复包）。**不要**在 ssh heredoc 里内嵌 Python 生成含 `\n` 的源码——转义会被工具展开成真实换行（第 2 轮踩过） |
| 远端脚本 | 不要对 `source /opt/ros/humble/setup.bash` 开 `set -u`（setup 脚本引用未定义变量）。`ssh host '...'` 单引号脚本里**不能出现英文撇号**（如 `Nav2's`），否则引号提前闭合、剩余文本在本地 shell 执行（第 4 轮踩过，未造成损害）；提交信息一律先写本地文件，再 `ssh host 'cat > 远端文件 && git commit -F 远端文件' < 本地文件` |
| 低负载构建（S7 场景） | 前缀 `nice -n 19 ionice -c3`，加 `--parallel-workers 2`，`MAKEFLAGS=-j2` |
| 测试 | `colcon test --packages-select <pkgs> --event-handlers console_direct+ && colcon test-result --verbose` |
| 沙箱 | `export ROS_DOMAIN_ID=99`；并加载 `~/ros2_ws_refactor/src/david_nav/config/ros_env.conf` 里的 DDS 配置后再覆盖 domain；沙箱进程统一用 `setsid` 启动并记录 PID，结束时只杀这些 PID |
| 版本基线 | Python 3.10.12；ROS 2 Humble（nav2 1.1.20）；pip：websockets 16.1、PyYAML 6.0.3、psutil 5.9.0、pytest 6.2.5、flake8 4.0.1 |
| 提交署名 | 提交信息末尾加 `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>` |

---

## 4. 每轮迭代流程

**时间预算**：单轮约 2 小时有效工作；到点在一个干净的提交边界停下。

1. **连通**：`ssh ... 'hostname; git -C ~/ros2_ws/src rev-parse --short HEAD'`。失败则按记忆中的方法在 `172.16.20.0/22` 找开放 22 端口且家目录含 `wheelhumanoidrobot` 的机器（最多 5 分钟）；仍找不到 → 在 LOG 记录"主机不可达"，结束本轮。
2. **现场检查（只读）**：`pgrep -af 'run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|navigation_executor|dock_navigator|rviz2'`。有任一存在 → 本轮进入 **S7 受限模式**，并在 LOG 记录。
3. **读进度**：打开 `PROGRESS.md`，确认上轮状态；若上轮留下未提交改动（`git -C ~/ros2_ws_refactor/src status`），先判断：能在 15 分钟内补完测试就补完提交，否则 `git stash` 并在 LOG 说明。
4. **选任务**：按 PROGRESS 顺序取第一个 `todo` 且依赖均 `done` 的 `AUTO` 任务；受限模式下跳过需要沙箱 ROS 进程的任务（标记 `[sandbox]`）。
5. **实现**：
   - 对应 PLAN/SPEC 的章节先读一遍再动手。
   - 修改既有行为的任务：**先写失败测试（红）→ 实现（绿）→ 重构**。
   - "搬家"类提交不改逻辑、不改引号风格；风格统一单独提交。
   - 每个提交保持可独立编译与测试；按下一步的变更风险选择实际验证范围。
   - 同主题说明审查合并较大批次；不要为每个摘要重复新增同质门禁。
     复用现有规范检查，新增测试聚焦真实缺陷和有价值的行为边界。
6. **验证（门禁）**：
   - 纯说明、注释或文档：`git diff --check`、受影响文件/包的 lint 与既有规范检查；
     运行源码说明变更核对非 doc AST，检查是否存在 `__doc__`/CLI 消费者。
     不强制重建或全包测试；涉及输出消费者时补相应兼容验证。
   - 局部逻辑变更：先红后绿，运行受影响模块及直接消费者回归和 lint。
     C++、构建/安装/依赖变化需构建相关包及受影响下游；不强制无关包重跑。
   - 跨包接口、启动链路变更或阶段收尾：执行全量构建、测试和结果汇总。
     风险需要时可以提前全量，但不得机械地每轮重跑。
   - 最终 DONE 仍要求全部 AUTO 完成后连续两轮完整验证通过，未降低最终门禁。
     增量结果必须标明范围；不得将混有旧结果的汇总写成新鲜全量通过。
   - `[sandbox]` 任务额外跑沙箱集成测试（非受限模式下）。
   - **生产不受影响检查**：`git -C ~/ros2_ws/src status --porcelain | md5sum` 与本轮开始时一致；`~/.bashrc` 的 md5 与 PROGRESS 记录的一致。不一致 → 立即停止本轮，LOG 写"⚠ 生产环境被改动"并按 §6 通知用户。
7. **提交**：`git -C ~/ros2_ws_refactor/src commit`，信息格式：`<type>(<pkg>): <做了什么> — <为什么>（PLAN §x / SPEC §y / 任务 ID）`。
8. **记录**：更新 PROGRESS（任务状态、commit、测试范围/结果、新增 WAITING_USER / 待确认）；在 LOG 追加本轮摘要（时间、模式、完成任务、提交列表、验证结果、阻塞项、下一步）。未做全量时保留上次全量的明确提交与日期，不冒充当前全仓验证。
9. **结束**：若达到 DONE 条件，按 §6 通知；否则正常结束等待下一轮。

---

## 5. 关键实现约定（摘要，细节以 PLAN/SPEC 为准）

- **行为不变优先**：Phase 0–4 的所有改动必须通过 golden 证明等价（线协议帧、param dump、BT XML、CLI、日志冻结行）。
- **base_link ≠ base_footprint**：两者分键书写，任何代码/文档不得混称"base"（PLAN §2.3）。
- **odom**：档案 `localization.odom_topic: /lightning/loc/odometry`；Nav2 接线 `nav2.odom_topic` 保持现值 `/lightning/odometry`；是否切换是 TODO-O1，**本循环不切换**。
- **脱困**：本循环只做"触发可见"（PLAN §10.3），不改 BT 行为；新策略 TODO-E1 不实现。
- **L7 显示离机、MCAP、import-linter、foxglove**：不做，不装；依赖它们的测试 `pytest.skip` 并注明 TODO ID。
- **进程名兼容**：`david_nav/david_nav/navigation_executor.py` 路径与可执行位保持（PLAN Phase 4 切 alias 前）。
- **dock 日志行冻结**：`[dock]` 前缀的日志文本逐字保持（dock_bench 正则依赖）。
- **包划分**：PLAN §5.3；物理包可暂时合并但 import 规则（§5.4）必须由单测保证。
- **pep257 约定**：新建/迁入包的 `test_pep257.py` 用 `main(argv=['.', 'test', '--convention', 'google'])`；旧包用 `['.', 'test', '--add-ignore', 'D213', '--add-select', 'D212']`（路径参数必须放在选项前面，否则会被当成错误码吞掉）。
- **测试安全**：任何会执行 shell 启动脚本的测试，必须在 PATH 前置假 `sudo`/`nvpmodel`/`jetson_clocks`/`pgrep`（参考 `test_headless_navigation_script.py`），跑测试一律 `ROS_DOMAIN_ID=99`。
- **沙箱经验（第 3 轮）**：参考 `david_nav/test/golden/capture_nav2_param_golden.sh` 的写法——`setsid` 起进程、只按会话 id 收尸；`ros2 param dump` 必须跳过没有参数服务的辅助节点（`*_rclcpp_node`、`transform_listener_impl_*`，否则永久阻塞），每次 dump 加 `timeout` 与重试；整次沙箱 bringup 偶发失败（DDS 服务应答超时），自动化里要做"冷却 30 s + 最多 3 次"重试，且不得清理 `/dev/shm`。
- **tty 读取**：只读串口前先 `stty raw -echo`，否则内核回显会把收到的字节写回下位机。
- **沙箱集成检查（第 5 轮）**：会启动 ROS 进程的检查放在各包 `sandbox_tests/`，文件名**不得**匹配 `test_*.py`（colcon 的 pytest 会递归收集，受限模式轮次也跑 `colcon test`），只经 `sandbox_tests/run_sandbox_checks.sh` 显式运行（拒绝在导航栈运行时启动、强制 `ROS_DOMAIN_ID=99`、可用 `SANDBOX_ROS_ENV_FILE` 加载 DDS 白名单）。flake8/pep257 仍会检查这些文件。非受限轮次在 `[sandbox]` 任务收尾时运行。
- **现场检查别匹配到自己（第 6 轮）**：`pgrep -f` 比对整条命令行，`ssh host '... dock_navigator ...'` 这类命令行本身含被监视的进程名，会被当成"导航栈在运行"。手敲的检查用方括号写法（`pgrep -af '[d]ock_navigator|[b]t_navigator|...'`）；沙箱脚本与 conftest 已排除自己的祖先进程。

---

## 6. 通知用户

以下情况在 LOG 顶部追加醒目条目，并在当轮最终回复中写明（若 PushNotification 工具可用，同时推送一条简短通知）：

- 生产环境被改动（S2 检查失败）
- 某任务连续 2 轮失败或需要设计决策（写入"待确认"）
- 出现新的 `WAITING_USER` 项
- 达到 DONE
