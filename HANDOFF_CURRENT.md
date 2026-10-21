# David 导航重构：当前接手入口

交接日期：2026-09-17（Asia/Shanghai）。本文件描述交接时状态，不是完成报告。

> **2026-09-17 接手更新（Claude Code）**：§3 的安全暂停**已解除**——用户确认该生产 launch 改动是其 09-16 的 cmd_vel_mux 接线改动并要求保留，生产指纹基线更新为 `2b46c35450efb573317140c44f5c1f12`，生产未回滚、未部署、仍只读。§5 的 5 个未提交文件经核验（SHA 与清单一致、三个运行模块非 doc AST 与 HEAD 一致、相关 116 项测试绿）后已提交为 `d5c7f7b`，不再处于未提交状态；本文件与 docs/handoff/20260917/ 保持未跟踪。§8 其余步骤继续按原文执行，持续状态以 codex_handoff_david_nav_refactor/refactor_loop/PROGRESS.md 为准。

## 1. 先读这里

代码在 David 的 `/home/agx/ros2_ws_refactor/src`，分支 `refactor/nav-system`，HEAD **c6199da**。
生产仓库 `/home/agx/ros2_ws/src` 的 HEAD 为 **977e29f**，生产保持只读。
本次只新增交接文档，没有继续实现、提交代码、部署或运行测试。

原自动目标已标记 **blocked，未完成**。直接原因是生产启动文件出现来源未确认的新修改，触发 RUNBOOK §4.6 安全暂停；不是 IMU 缺失，也不是要求每次全量测试。
用户当前要求另一个 agent 接手；这不等于确认该生产修改、部署重构代码或允许机器人运动。

建议阅读顺序：本文件 → [现场约定快照](docs/handoff/20260917/WORKSPACE_AGENTS.md) → [RUNBOOK 快照](docs/handoff/20260917/RUNBOOK.md) → [PROGRESS 快照](docs/handoff/20260917/PROGRESS.md) → [人工审核清单快照](docs/handoff/20260917/REVIEW_PENDING.md)。
任务总方案与执行器规格也随交接放入 `docs/handoff/20260917/`。

原始持续维护文档位于 `/home/agx/codex_handoff_david_nav_refactor/`：

- `refactor_loop/PROGRESS.md`：逐项任务状态真源；表格与顶部最新记录优先，下方很多段落是历史阶段快照。
- `refactor_loop/RUNBOOK.md`、`refactor_loop/LOG.md`：操作规程和完整历史证据。
- `DAVID_NAV_AGENT_HANDOFF_20260916.md`：详细历史交接，虽文件名为 16 日，顶部已更新到 17 日。
- `REVIEW_PENDING.md`、`david_nav_refactor_daily_20260917.md`：人工问题和简短日报。
- `docs/2026-09-14-david-nav-system-refactor-plan.md`、`docs/2026-09-14-david-nav-executor-refactor-spec.md`。

不要按旧 HANDOFF/AGENTS 中“Phase 2 进行中”从头重做。旧记录的“未接入口”“R8 未修”“headless Ctrl+C skip”不是最新结论；当前功能与待验收边界见下文和任务表。
本目录内快照只用于接手，后续不要同时维护两套互相漂移的任务状态。

## 2. 用户要求及不可越过的边界

- **不修改任何 `3` 相关代码、配置、脚本**，包括 Lightning/uslam、雷达、IMU；也不擅自启动、停止、重启 `3`。定位问题记待审核，导航侧可执行工作不因其缺失而全部停下。
- 正式组合由现场先启动 `3`、再启动 `0`。交接时只读检查仍看到定位、底盘、Nav2 进程，按 S7：只做纯测试和低优先级编译，不启动 ROS 沙箱。PID 会变，接手重新检查。
- 不发导航目标或非零速度，包括模拟话题；零速也仅限允许的 domain 99 沙箱。假件/内存测试不等于动作或实机验收。
- 生产 `ros2_ws/{src,build,install}`、`.bashrc`、`/etc`、定位/底盘原工作空间及用户地图、rosbag 只读；不清理 `/dev/shm`，不停止 daemon，不给非本轮拥有的进程发信号，不打开真实串口。
- 不安装 apt/pip/源码依赖，不 push，不改权限设置；需要新权限或部署决定的事项记录待确认。历史“直接授权”不能用于绕过这些明确边界。
- WiFi `wlP1p1s0` 用于 SSH，不在正式 Fast DDS 白名单中，这是有意隔离。不能因诊断看不到节点而删除白名单或换生产 domain。
- 不打印 `.bashrc` 或 headless 脚本中的凭据；不把密码写入交接、提交或日志。
- 本次没有授权并行 agent。交接后由接手 agent 根据用户新指示工作。

## 3. 本次安全暂停：是什么、不是是什么

生产 `git status --porcelain | md5sum` 的旧记录为 `da1ef50b0f796a7d12d71374dc3815bd`，当前为 `2b46c35450efb573317140c44f5c1f12`。
新增状态行是 ` M david_nav/launch/2d_david_nav2_bringup.launch.py`；只读计算去掉该行后恢复旧指纹。
生产 HEAD 未变，`.bashrc` MD5 仍为 `695bd38f807e689f084ed249ae855536`。

该 launch 的 diff 为 13 行新增、1 行删除：

- `output_cmd_vel_topic` 从固定 `/cmd_vel` 改为 `LaunchConfiguration('david_move_output_topic')`。
- 新增同名 launch 参数，默认仍 `/cmd_vel`。
- 注释/参数说明称 alias 0 传 `/cmd_vel_nav_chassis`，使导航接入 `cmd_vel_mux`，位置环 `/cmd_vel_position`（优先级 100）可抢占。

这里只确认源文件改动及其文字意图，**没有证明生产实际接线正确或运行已采用它**。来源未知，不归因于用户或某个 agent；已记录 mtime `2026-09-17 09:28:10 +0200`，不是修改者证明。
状态哈希只检查状态行，不能证明原本已脏文件的内容未变。`.fastdds_david.rHTYWa` 和 `record_track.py` 属于此前已存在的改动，不应清理。

连续三次复核仍无变更确认，因此标记 blocked，停止重复空转。没有回滚、覆盖、stash、改生产基线或继续代码提交。
解除方式：让负责人确认该生产差异是否预期、是否保留及如何处理基线；确认前不要自行接受新基线。若由其他负责人修复，也应先只读复核再继续。
证据：`/home/agx/ros2_ws_refactor/staging/p74_config_contracts_codex/PRODUCTION_CHANGE_PAUSE.md`。

## 4. 已完成的关键工作，不要重做

总体结构已迁移到配置/启动、任务内核、WS 适配器、统一 Nav2 客户端、Dock 技能和旧入口 shim；P6 多项仍 doing 的原因主要是完整 ROS/动作验收未完成，不是这些实现不存在。

- 用户追加的三项要求已在重构分支实现（起始提交 `35cdda7`，后续迁移保留）：dock 预备点完整车身 footprint 静态地图校验；脱困成功后回到规划/路径跟踪，不把恢复完成作为 Goal succeeded；停稳角速度阈值约 **0.05 rad/s**。没有部署生产，也没有实机验收。
- footprint 检查含旋转地图原点、面积覆盖；占用值 `>=50` 拒绝，缺地图/越界/非法输入保守拒绝，未知格 `-1` 沿用旧允许策略。不要把普通 POI 点吸附当完整车身校验。
- 独立 Dock 的 **5°/2 次**与 precise 的 **25°/0 次**策略分别保留；R8 默认目标话题已改 `/dock_goal`（`b767ea9`）。
- `818698b`、`6768d2c`、`75ab8f5` 分别修复 Dock close 重试、TaskRuntime 取消后的 settle 重试、rclpy Future 保存 CancelledError 时等待者可能悬挂。清理失败仍保留资源所有权，不能假报 idle。
- `6ba3e5d` 修复参数 dump 输出名碰撞（`/a/b` 与 `/a__b`、报告保留名），`4adb9d4` 完成采样/监控/转储契约审查。
- `42bd363` 修复 mode override 对绑定容器后代/列表下标别名的绕过；数字字典键仍独立。10 新用例，旧 8 失败/2 通过 → 相关增量 **145 通过**。
- `c6199da` 将 schema 包名/键、绑定键/标记四处正则改 fullmatch，拒绝尾部换行；8 新用例，旧 4 失败/4 通过 → 相关增量 **185 通过**。

后两提交是实际逻辑修复，不能把它们也称为“仅说明、行为不变”。它们之后的未提交说明批才有非 doc AST 等价证据。

## 5. 必须保留的 5 个未提交文件

交接前 `git status --short` 只有下面 5 项；**本次另新增 HANDOFF_CURRENT.md 与 docs/handoff/20260917/ 文档，不属于原说明批**。
原 5 项均已测试，不要 stash、覆盖或按“用户垃圾”删除；也不要 `git add .` 混入交接文档。

```text
63f1ef5f7d35a23abdeb3a0f02c71ce22844493f5cdd528e4f9162619d37accc  nav_bringup/nav_bringup/navcfg/schema.py
317708a4e8439b5942bdf0446b8f9377675918408872ebc6734f8367012c188d  nav_bringup/nav_bringup/navcfg/render.py
24f7e496efb633a5769423a0c72a8d23a13afac1200508888fecf5766ee76c7a  nav_bringup/nav_bringup/doctor/checks.py
00f1decf2e20fdd2bba5ae551b71215f282340930e99135fdc1b67d1a25c19a9  nav_bringup/test/test_config_contract_boundaries.py
28016b424d97466a2ccfc6a15f071a16e28a25a69b4130823636be38c52ea0b2  docs/adr/0013-doctor-observation-history.md
```

内容：schema 19、render 16、doctor 19，共 54 处说明；ADR-0013 原文保留三段历史诊断叙述，不是本轮现场事实。
4 项边界测试覆盖 catalog 默认值嵌套引用、mode 嵌套映射引用、注入绑定字符串不递归展开、doctor 不解析现存 XML 的既有行为。没有顺带强化这些策略。
三运行模块非 doc AST 与 `c6199da` 相同；三段历史文字核对通过。最终增量 **240 passed, 2 warnings in 12.55s**，已终态，无需原样重跑。

证据目录 `/home/agx/ros2_ws_refactor/staging/p74_config_contracts_codex/`：

- `targeted.log`：240 项实际结果；两项警告来自 flake8 的 SelectableGroups 弃用提示。
- `ast_history.log`、`verify_config_docs.py`：AST 与历史文字核对。
- `existing_docs_audit.log`：语言候选扫描，不是完整语义规范证明。
- `config_docs_files.list`：仅该批 5 文件的上传/提交清单。

另两修复证据在 `staging/p74_binding_guard_codex/` 与 `staging/p74_config_names_codex/` 的 `red.log`、`targeted.log`、`commit_result.txt`。
本地稀疏副本 `/tmp/david-config-contracts.7lQbKt` 在控制端 PC，不在 David；接手无需依赖它。David 的工作树和 staging 已保留完整所需内容。不要把稀疏目录整体覆盖仓库。

## 6. 剩余工作与证据缺口

| 项目 | 真实状态 / 下一步 |
| --- | --- |
| 生产差异确认 | 当前实现/提交暂停的直接原因，见 §3；不要反复运行测试试图解除此条件。 |
| P7.4 说明/规范 | 草稿态扫描尚有 239 个无 CJK 候选，主要在 nav_bringup，另有混合语言及语义/类型审查。语言计数不等于完成指标；合并同主题批次，优先真实缺陷，不新增大量同质摘要测试。 |
| 下一组审查 | 可从 navrun 的 keyboard_session、locking、keyboard_channel 或 navcfg 的 live_check/live_ros、recorder/run_event_stream 整组继续；不要重审本批 schema/render/doctor 54 处。 |
| P7.A3 协议 | payload 的完整 JSON Schema 校验、官方 AsyncAPI 元 schema 校验尚未运行。David/PC 缺所需工具依赖的记录在审核清单；S4 禁止安装。字段对应/文档存在不等于完整验证。 |
| C-16 版权 | 实际 ament_copyright 曾报 15 errors / 19 files；15 个 david_cmd 文件缺可靠声明。不是缺依赖，不能靠 skip 宣告合格。只读追溯来源，权利主体/年份不明时需负责人确认，不能按 git 作者推断。 |
| P3.5 自测闭环 | 协调器/观察/报告已实现，prepare-only/startup-only 已有结果；实际 POI + Dock 带目标闭环未运行，S1 禁止自动发目标。 |
| P6.1–P6.4 | 新核心、Nav2/Dock/WS/shim 已实现及纯测试；完整 ROS 生命周期、共享宿主退出与带动作验收未完成。不要重复实现入口或把启动成功当闭环成功。 |
| R6 资源验收 | 只读空闲 CPU 测量工具已实现；没有合格的真实 >=60 秒 idle 测量结果，不能声称达标或优化成功。 |
| USER 项 | P0.U1、P1.U1、P4.U1/U2、P5.U1、P6.U1/U2、P7.U1：资源基线、alias/部署、tty、实机 A/B、驱动启用、odom 接线、legacy/系统/合并决策等仍待人工。 |
| 最终门禁 | 全部 AUTO 完成后再连续两轮完整验证；目前不成立。历史重复全绿不能代替未完成项。 |

其它需保留的兼容/风险记录：G12 外部 CANCELED 的 paused/active 语义、可靠 outbox 仅内存且不保证恰好一次、POI 保存并非多写者事务、日志采集可能部分失败、旧 scan layer 的 current_ 语义、取消清理失败持有 busy、历史 SSH/主机重启根因未知等，详见审核清单。不要一边迁移一边悄悄改变这些策略。
健康状态仍以观察/告警为主；不把缺 IMU 当导航改定位的理由，不自行实现未授权 H1 自动停机策略。
TODO-O1 保持现有 Nav2 `/lightning/odometry` 接线，不能因档案另有 `/lightning/loc/odometry` 就自行切换；`base_link` 与 `base_footprint` 必须区分。

## 7. 如何加速、如何验证

用户最新要求：**不需要每次全量测试，当前过慢，要加速**。已更新 RUNBOOK，旧 AGENTS“每段工作全量”等说法不得覆盖该新要求。

- 纯文档/说明：diff 检查、受影响 lint/既有规范门禁、运行源码非 doc AST 核对；不强制 build/全量。
- 局部逻辑：先失败测试再修复，运行受影响模块与直接消费者回归。C++/构建/安装/依赖改变再构建相关包与下游。
- 跨包接口、启动链路、阶段收尾或最终门禁才做必要全量。源码没有变化且证据完整时不要机械重跑同一批。
- `ready_exec`、`navcfg/cli`、`event_flush` 存在 docstring/help 消费；翻译前先捕获旧帮助并保持兼容。新说明使用中文内容、Google 分节，摘要末尾用英文句点以通过 pep257。
- 开始编辑（含 PC 副本）前确认上一轮测试/build 已终态，避免结果对应混合源码。
- 低负载用 `nice -n 19 ionice -c3`，构建 workers 2、`MAKEFLAGS=-j2`。不要安装缺失依赖或改测试范围“换绿”。

最后一次新鲜全量属于 **5ea1813**：18 包，**3980 tests / 0 errors / 0 failures / 2 skipped**，build 15.4 秒/test 2 分 32 秒。
两项历史 skip 是版权与缺 jsonschema；版权不符合最终“仅缺依赖可跳过”的条件。后续提交只做对应增量，不能称当前 HEAD 已全量通过。
全量证据在 `staging/p74_linked_trigger_codex/`；当前的 145/185/240 分属不同范围，不相加当成独立用例总数。

## 8. 接手操作顺序

1. 连接 `agx@172.16.22.51`，先读适用的 AGENTS 与本文件/快照；确认分支、HEAD、工作树及无遗留测试进程。交接时生产 3+0 仍运行，只做只读检查，不启停它。
2. 对照 §3 核实生产 diff，并获得基线处理确认；未解除则可以解释/整理交接，但不要继续实现、提交或部署。
3. 解除后核对 §5 的五文件 SHA、现有日志和 staged 清单。内容未变无需重跑已完成的 240 项；按风险做必要核验后单独提交这批，交接文件另行处理。不要自动 stash。
4. 按任务表推进尚可自主完成的项；需现场/新依赖/版权判断的事项留待审核，不以一项 WAITING_USER 阻断其它独立工作。
5. 持续更新原 `refactor_loop/PROGRESS.md`、`LOG.md`、审核清单与短日报。最终报告准确区分已实现、已测试、已部署、已实机验收。

正式 ROS 只读诊断必须在同一个远端 shell 加载：

```bash
source /opt/ros/humble/setup.bash
source /home/agx/ros2_ws/install/setup.bash
set -o allexport
source /home/agx/ros2_ws/src/david_nav/config/ros_env.conf
set +o allexport
# 之后才执行已授权的只读 ROS 诊断。
```

重构测试使用 Humble + refactor install，按 RUNBOOK 加载 refactor 的 ros_env 后覆盖 `ROS_DOMAIN_ID=99`；S7 时仍只允许纯测试，不因 domain 99 就启动 ROS。
构建使用新的 shell，仅 source Humble，不继承生产/refactor overlay 的构建前缀；执行 `colcon build --symlink-install --metas src/colcon.meta` 时仍遵守受影响范围与低负载规则。

不要照抄 RUNBOOK 的旧 Claude 联署身份；实际由谁修改就用谁的真实身份。不要修改权限配置以绕过任何限制。

本交接无密码、无新的测试/实机结论。原重构目标保持 blocked；本次只完成交接文档任务。
