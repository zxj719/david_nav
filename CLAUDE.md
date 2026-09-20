# David 导航工作树：给 agent 的操作手册

项目地图见 [README.md](README.md)，架构见 [docs/architecture.md](docs/architecture.md)。
本文只讲**动手前必须知道的事**：硬边界、环境陷阱、验证规矩、不可破坏的设计不变量。
本文不扩张授权，也不能代替现场的安全确认。

## 0. 接手的头十分钟

```bash
ssh -o BatchMode=yes agx@172.16.22.51
cat ~/codex_handoff_david_nav_refactor/refactor_loop/PROGRESS.md | head -40   # 状态真源
tail -60 ~/codex_handoff_david_nav_refactor/refactor_loop/LOG.md             # 最近发生了什么
cat ~/codex_handoff_david_nav_refactor/REVIEW_PENDING.md                     # 需要人判断的事
cd ~/ros2_ws/src && git status --porcelain | md5sum   # 生产指纹，应为 2b46c35450efb573317140c44f5c1f12
md5sum ~/.bashrc                                      # 应为 415c0a8144aa27740205c8cf46d60083
cd ~/ros2_ws_refactor/src && git status --porcelain && git log --oneline -5
ps -eo pid,etimes,cmd | grep -E "[l]ightning|[l]slidar|[w]it_imu|[m]ove_group"  # 现场在跑什么
```

状态只认 `PROGRESS.md`，不认旧交接文档里的阶段描述。生产指纹或 `.bashrc` 对不上先问，
不要自行"修正"——那多半是现场自己的改动。

## 1. 硬边界（不可协商）

- 只在 `~/ros2_ws_refactor/src` 的 `refactor/nav-system` 分支改，**本地提交，永不 push**。
- 生产 `~/ros2_ws/{src,build,install}`、`~/.bashrc`、`/etc`、别人的工作区**只读**，
  除非用户直接点名要改。
- **不碰 `3`**：Lightning/uslam、雷达、IMU 的代码、配置、启停一律不动。定位侧问题只记录。
- **不发导航目标、不发非零速度**，除非用户明确授权本次实机运动。零速与仿真不在此列。
- 测试一律 `ROS_DOMAIN_ID=99`；只有实机验收才用默认 domain 0。
- 不装软件、不改 sudoers/udev/权限、不清共享内存、不停别人的进程或 daemon。
- 脚本里的 sudo 口令不得出现在任何输出、日志或提交里。
- 需要人判断的事（实机 A/B、部署、合并、固件、精度结论）一律留给用户，不自行拍板。

## 2. 环境陷阱（不知道会浪费半天）

**① SSH 上手敲 `ros2` 会"半瞎"。** DDS 白名单不放行 WiFi 网卡，且 CLI daemon 会变陈旧，
表现是 TF 树空、话题 0 发布者、节点列表缺一半——全是假象。任何 `ros2` 命令前：

```bash
set -o allexport; source ~/ros2_ws/src/david_nav/config/ros_env.conf; set +o allexport
source /opt/ros/humble/setup.bash && source ~/ros2_ws_refactor/install/setup.bash
ros2 daemon stop        # 关键；必要时用 --no-daemon 做对照
```

同样地，**导航入口自己也必须加载 `ros_env.conf`**：不加载时各节点通告的网卡集合不对称，
生命周期服务的响应发不回管理器，表现为"卡在第一个节点、RViz 没有代价图"。

**② 反复起停后 `Failed init_port fastrtps_portNNNN`。** 共享内存段被删但仍被定位进程映射。
判别：在定位进程的 `maps` 里该段标 `(deleted)`。**不要**自己删段；恢复要现场重启 `3`。

**③ 现场同时跑着机械臂 moveit 栈。** 它自带一个 `robot_state_publisher`。
项目里"启动前杀 robot_state_publisher"这一项已于 2026-09-20 **整项剔除**（`ab0ac75`），
不要再加回来。重复的 URDF 发布者只由 `navrun doctor` 报告，不自动处理。
启动前用 `navrun up ... --dry-run` 看 `preflight.target_pids`，**非空就停下来问**。

**④ 上位机地址不在本机。** 场地配置写的是 `ws://192.168.123.186:8101`，David 上没有这个地址。
本地联调要临时改 `david_config/sites/daolan/site.yaml` 的 `agent_server.uri` 为
`ws://127.0.0.1:8101`，然后 `colcon build --packages-select david_config`；
**用完 `git checkout` 还原并核对 sha256**（原值 `921d91cb…`）。

**⑤ `~/sim_agent_server.py` 一连上就下发目标**——想做无运动测试**不要用它**，
用 `tools/nomotion_agent.py`（本仓内，所有目标都指向不存在的点位）。

**⑥ 两个 odom 话题不是一回事。** 定位发的是 `/lightning/loc/odometry`，
Nav2 侧保留的键是 `/lightning/odometry`，目前**无发布者**，是已知项 TODO-O1，未获批准切换。
启动日志里的 `localization_odom_missing`、`topic_publishers /odom 0 publishers` 由此而来。

**⑦ 地图的 `mode`。** Lightning 的图用 `px=128` 表示未知；nav 侧必须用自建的
`mode: trinary` 的 yaml 指向同一张 pgm，用 `scale` 会把大片未知区变成致命障碍。

**⑧ 帧。** `frames.nav_base` 是 `base_footprint`，`frames.body` 是 `base_link`，
两者差 0.165 m（URDF 固定关节），POI 采集用 `frames.poi_capture`。三者永不合并。
`david_nav` 里的 `scan_priority_layer` 有硬编码的帧名，改 `robot_base_frame` 必须连带改它。

## 3. 怎么不动车也能验

这是 2026-09-20 用过并留档的方法，可直接复用：

```bash
# 0) 先看预演：清理目标必须为空，否则停下来问
navrun up --mode precise --chassis off --dry-run

# 1) 确认没有任何东西能驱动轮子（应该连话题都不存在）
ros2 topic info /cmd_vel; ros2 topic info /cmd_control_command
ps -eo pid,cmd | grep -E "[w]heeltec|lib/david_cmd/[d]avid_move|[c]hassis_adapter"

# 2) 起栈时不启动驱动（实际调用带 ros2 run nav_bringup 前缀）
navrun up --mode precise --chassis off --record off

# 3) 用只走拒绝/畸形/取消/重连的假上位机
python3 tools/nomotion_agent.py /tmp/ws_transcript.jsonl 8101

# 4) 停止：协作式，不强杀
navrun down
```

事后核对"确实没动"：运行记录里与目标/入位相关的事件数应为 0；
收尾只应有设计中的 `stop_zero_sent`。完整证据模板见
`~/codex_handoff_david_nav_refactor/onsite_acceptance_20260920/README.md`。

## 4. 验证规矩

| 改动类型 | 需要做什么 |
|---|---|
| 纯说明（docstring） | `git diff --check` ＋ `python3 tools/ast_doc_check.py HEAD <文件…>` 证明去掉说明后 AST 与 HEAD 一致 ＋ 受影响包 lint/测试 |
| 改行为 | 先红后绿：先加会失败的测试，再改实现；搬迁代码不得夹带调参 |
| 改配置/schema | `navcfg validate`，并重新生成 `docs/parameters.md`（`navcfg doc --output docs/parameters.md`） |
| 任何提交前 | 受影响**包**的 `python3 -m pytest test -q`（进包目录跑，见 README 的重名说明） |
| 收口 | `colcon build && colcon test && colcon test-result --all`，当前基线 **4129 / 0 / 0 / 0**；再核对生产指纹 |

日常**不要每次全量**（2026-09-17 用户要求），按风险做增量；最终 DONE 才要连续两轮全量。

离线闭环自测（全仿真，不碰真车）：

```bash
ROS_DOMAIN_ID=99 navrun selftest --mode precise --allow-goals --output-dir /tmp/st_precise
ROS_DOMAIN_ID=99 navrun selftest --mode tour --allow-goals --output-dir /tmp/st_tour
```

`--output-dir` 必须是**尚不存在**的目录；不给 `--allow-goals` 就只是启动观察，不是闭环。
入位用例只在 precise 模式生成——生产的导览模式本来就不配置入位节点。

## 5. 不可破坏的设计不变量

- **任务只有一个 owner**：不要再引入第二套任务实现或第二个 Nav2 action client。
  历史上两个执行器抢 action client 的 wait set 直接导致过崩溃。
- 纯逻辑层（`nav_tasks.domain/core/skills`、`nav_executor.protocol`）不得 import
  ROS/YAML/WebSocket；`nav_executor` 只经 `nav_tasks.api` 进任务层，反向依赖禁止。
- `goal.*`（Nav2 到点容差）与 `dock.*`（入位达标阈值）是**两组独立的档案项**，
  2026-09-18 拆开过一次，不要再合并——合并会把已达标的入位判成失败。
- 入位的"停稳"判据只看偏航角速度，**按用户 2026-09-20 决定维持现状**；
  若将来把入位达标收紧到 0.15 m 以内，需要重新评估该判据。
- `[dock]` 冻结日志行、协议金样、AsyncAPI 文档**不得为了让测试变绿而重新生成**。
- 恢复行为的 `global_frame` 必须与局部代价图一致（历史上配成 `map` 导致 Spin/BackUp
  每次 100 ms 内 abort），已有不变量测试守着。
- 清理与取消：取消失败不得假装已释放任务；清理要等自有线程、回调、子进程真正结束，
  不伪报 IDLE。
- Python 公共 API 要类型注解 ＋ Google 风格中文说明，flake8 行宽 99；
  运行代码的说明**摘要行必须是中文**，有全包门禁 `test_every_runtime_summary_is_chinese`
  守着；门禁只证明语言，不证明语义。

## 6. 记录与提交

每轮结束都要更新（它们在 `~/codex_handoff_david_nav_refactor/`，不在本仓）：
`refactor_loop/PROGRESS.md`（状态真源）、`refactor_loop/LOG.md`（追加摘要）、
`REVIEW_PENDING.md`（需要人判断的事）、必要时补一份简短日报。

提交说明写清**改了什么、为什么、以及本次没有证明什么**；署名用当前代理的
`Co-Authored-By:` 行（Claude 用 `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`）。
用 `git add <具体路径>`，不要 `git add docs` 这类宽匹配——2026-09-20 就因此把一批
未跟踪的交接快照意外纳入了版本库。

最后一条：**测试绿不等于验收通过**。实体停车、精度、资源收益、alias 部署，
都必须由现场实测，不能用纯测试结论替代。
