# 旋转过阻尼调优复盘（2026-07-01）

> 目标：让 `position_controller` 原地转**过阻尼**（宁可略欠转留角度让 caster 拖尾补进，
> 也绝不冲过终点）。历时一整轮，最终达成——但**真正的根因在底盘固件，不在位置环**。
> 这份复盘的重点不是"改了哪个参数"，而是**为什么绕了远路、下轮怎么少绕**。

## 0. TL;DR

- **起点误判**：以为旋转超调是"位置环末端整形不够"，连试三法（非对称快减速 /
  抬 rotate_lead / 角度接近减速），**全部失败**。
- **真根因**（逐拍 trace 才看出）：**底盘实际转速 ≈ 2.5× 指令**，且接近终点时
  actual 与 cmd 解耦——底盘当飞轮硬滑 ~50°。位置环命令的任何整形都没用。
- **定位**：`wheeltec_robot_node` 是透传（cmd×1000→STM32；odom÷1000 回来），
  增益在 **STM32 固件**（新底盘换大轮+宽轮距后运动学常数未更新 + 速度环欠阻尼）。
- **固件修（用户做）**：① 蓝牙自动标定 odom（运动学/里程准）；② 速度环阻尼。
- **修完重标定 + 重调位置环** → 旋转超调 **19~22° → 2~6°**，残余 ~2.5°（过阻尼预留），
  ~5s、无抽搐、可复现。**三管齐下才成：固件 odom 标定 + 固件速度环阻尼 + 位置环重调。**
- **一个大坑**：`pkill -f position_controller` 会杀掉**自己的 shell**（命令行含该串），
  造成一连串"启动静默失败"，排查耗时。用 `pkill -f "[p]osition_controller"`。

## 1. 问题与目标

用户要求旋转"过阻尼"：由于 caster 拖尾，终点后机身会继续漂一点；所以宁可**停在终点前
几度**（欠转），让拖尾自然把机身补到终点，**绝不冲过**（冲过再回抽在这台底盘上是一记
猛冲，更糟）。初始症状：`position_controller.yaml` 旋转"速度过大且超调严重"。

## 2. 调优历程（时间线 + 每步的发现）

| 阶段 | 做法 | 结果 / 发现 |
|---|---|---|
| A. 位置环整形（假设超调是控制器自造） | 加 `alpha_decel_max`（提前停真正快速切零，仿 velocity_smoother 的 accel/decel 分离）+ 单测 | 单测能演示机理；**实车"19→11 砍半"是假象** |
| B. 陈旧 pyc 打脸 | 逐拍 trace 发现减速率是 0.8 不是 2.0 | **install `__pycache__`（build 期编译）盖过 symlink 源，跑的是旧 bytecode**；A 的"生效"从未发生，11° 只是逐次方差（真实 11~30°） |
| C. 过阻尼靠调参（抬 rotate_lead + 放宽 yaw_tol） | rotate_lead 0.4→0.65 | **更糟**：欠转过头 → `min_cmd_w=0.55` 反复回抽 9 次、超调 22° |
| D. 逐拍 trace 定根因 | 记录 `/odom` yaw+twist vs `/cmd_vel` | **cmd 0.75 → actual ~1.9rad/s（~2.5×）**；cmd 斜降到 0 的 0.5s 里 actual 岿然不动 → 动量主导、与命令解耦。**位置环整形注定无效** |
| E. 定位到固件 | 读 `wheeltec_robot.cpp` | 节点透传（`angular.z*1000`→STM32；`Odom_Trans`÷1000）；`odom_z_scale=1.0`；`twist_transform` 只管 `/twist_raw` teleop。∴ 增益在 **STM32 固件运动学** |
| F. 固件修（用户） | 蓝牙自动标定 odom + 速度环阻尼 | odom 准；破缸从 ~0.60 降到 ~0.30；actual 可跟随 cmd |
| G. 重标定 + 重调 | 开环 `chassis_characterize` 重扫旋转 + 位置环重调 | **成功**：超调 2~6°、残余 ~2.5°、~5s、可复现 |

## 3. 根因总结（分层，务必记住这个"层次"）

1. **底盘固件层（真根因）**：新底盘换大轮+宽轮距后，STM32 运动学常数未更新 →
   cmd↔轮速↔odom 全偏（角速度 ~2.5×，里程漂 >2m）；且速度环欠阻尼 → actual 冲过
   setpoint 不收敛。**这是超调的物理来源。**
2. **速度区间层**：`w_max/min_cmd_w` 是在**未标定的 plant** 上定的"指令"值，对应的
   actual 与真值脱节；标定后必须重扫重定。
3. **位置环层（最表层）**：末端整形（rotate_lead / approach-decel）**只有在底层
   plant 老实跟随命令后才有意义**。一开始就在这层调，是"在错的抽象层修问题"。

> 教训：**从底往上确认**——先证明"底盘老实执行速度命令"（cmd→actual≈1:1、能收敛），
> 再谈位置环整形。否则再多控制器技巧都被 plant 吃掉。

## 4. 最终配方 + 实测

`config/position_controller.yaml`（标定 + 速度环阻尼后）：

| 参数 | 旧 | 新 | 依据 |
|---|---|---|---|
| `min_cmd_w` | 0.55 | **0.40** | 重扫破缸 ≈0.30 + 裕量；floor 速度低→动量小 |
| `w_max` | 0.75 | **0.60** | 慢转降动量 |
| `alpha_approach` | 0(关) | **0.5** | 角度接近减速；速度环能跟随后**才生效** |
| `rotate_lead` | 0.40 | **0.10** | 滑行~5°，预留~6°停在终点前 |
| `yaw_tol` | 0.02 | **0.05** | 锁小欠转不回抽 |

新增控制基元（core，默认 0=关，含单测，plant 修好后才该开）：
`alpha_decel_max`（角加速/角减速分离）、`alpha_approach`（角度接近减速）。

**实测**（`position_controller_field_test.py --tests rotate`，两轮）：

| 转向 | 超调 | 残余 | 用时 |
|---|---|---|---|
| +90° | 2.2° / 3.5° | ~−2° | 4.5 / 6.2 s |
| −90° | 6.2° / 6.7° | ~+2.7° | 5.1 / 4.9 s |

对比：老基线 19~22°、误标定期 25~40°。

## 5. 下轮如何更高效（方法论清单）

1. **先验证 plant，再调控制器**：任何底盘/固件动过之后，第一步是开环
   `chassis_characterize`——确认 cmd→actual 增益、破缸、能否收敛。别直接调位置环。
2. **改完代码先证明"真生效"再看指标**：david_nav 是 symlink-install，但 install
   `__pycache__` 会跑旧 bytecode。测前 `rm -rf __pycache__` + `colcon build` +
   **实例化节点打印 `n.limits.<字段>`** 或 `ros2 param get` 确认参数真到位。
3. **逐拍 trace > 单次指标**：单个"超调 11°"会骗人（方差 + 假生效）。
   `scratchpad/rotate_trace.py`（记 `/odom` yaw+twist + `/cmd_vel` 每拍）10 秒看穿
   "命令在降、实速不降"这类解耦——这是这轮定根因的关键。
4. **一次只动一个变量 + 记因果**：C 阶段一次动两个（lead+tol）+ 在错 plant 上，
   直接进抽搐，还难归因。
5. **pkill 别自杀**：管理 `position_controller` 用 `pkill -f "[p]osition_controller"`；
   或直接 `kill <pid>`。别在含目标串的 shell 里 `pkill -f 目标串`。
6. **pre-flight 查重**：`pgrep -cf '[p]osition_controller'`；上一个 hung 的后台
   trace 曾持有 goal 污染了整整两次测量。
7. **诚实更正**：A/B 的"19→11"是我基于假生效 + 方差的误报，及时逐拍 trace 才纠正。
   宁可慢，也别把方差当疗效。

## 6. 待改进 / 遗留

- **线速度未在标定后重扫**：标定同样改了直行破缸/增益。跑 POSE/waypoint 前，用
  `chassis_characterize --tests straight` 重定 `min_cmd_v`/`v_max`/`approach_decel`。
- **方向不对称**：−90° 超调（~6.5°）> +90°（~3°），caster 侧向不对称；如需可在核心层
  按转向给 floor/lead 小偏置。
- **POSE 全链未测**：只测了 ROTATE。POSE 的 ALIGN/FINAL_TURN 复用同套转向逻辑应同样受益，
  但 DRIVE（线速度）依赖上面的线速度重标定。
- **残余 ~2.5°**：是过阻尼的预留（设计如此）；若要更准，需固件 break-away 前馈或更细的
  拖尾建模，位置环层到此为止。
- **固件基线归档**：`WHEELTEC_C50X_2025.08.07` 是本次固件源；建议单独版本管理（不入本 ROS 仓，
  已在 `.gitignore` 之外，勿误 `git add .`）。

## 7. 附：工具与坑速查

- 开环表征：`scripts/chassis_characterize.py --tests rotate --no-precondition --w-levels ...`
- 闭环验收：`scripts/position_controller_field_test.py --tests rotate --rotate-deg 90 --no-confirm`
- 逐拍 trace：`scratchpad/rotate_trace.py <deg>`（session 私有，未入仓）
- 部署校验：改 core 后 `rm -rf __pycache__` + `colcon build --symlink-install --packages-select david_nav` + 实例化打印 `n.limits.*`
- 坑：陈旧 pyc（[[project_david_stale_pyc_trap]]）；pkill 自杀；后台 trace hang 持 goal 污染测量。
