# dock_bench — 精准入位（alias 0）固定测试用例与评测框架

测 `dock_navigator` 两段式入位的**最后 0.5 m**：从预备点沿目标航向直线入位那一段。
可靠性覆盖全程，其余四类指标只在阶段2 的时间窗内算。

设计取舍见 `weekly_report/specs/2026-09-09-david-dock-bench-design.md`。
和 `../controller_bench/`（alias 7+8，测 DWB controller）**并列**，互不影响。

## 为什么这个测试是良定义的

预备点 `P = G − 0.5 m × [cos,sin](yaw_G)` 只由目标决定，**与起点无关**。所以不管每轮
从哪儿绕过来，被测的那 0.5 m 几何上完全一致，跨轮次天然可比 —— 这也是本框架敢让
起点松着来（自动导航回去、差几十厘米无所谓）的依据。

## 关键约定

- **真值 = lightning 的 `map->base_footprint` TF**（20 Hz）。`record_track.py` 在
  `/groundtruth/pose` 收不到时自动退回查 TF，所以跑 alias 3 时它记的就是 lightning 位姿。
  - ⚠ **lightning 既是被测系统的定位器、又是唯一的尺子**，所以本框架测的是
    **系统自洽的入位误差**，不是对地面的绝对精度。lightning 整体漂移多少，这套指标
    看不见。`/odom` 位姿照录但**只作诊断，不进任何指标**。
  - slam_gt 和 lightning 都发 `map->odom`，一个 TF 树里 `odom` 不能有两个父节点，
    所以**不能**像 alias 8 那样用 slam_gt 当独立真值。
- **不启动任何被测组件**：alias 3（且已在 rviz 给过初始位姿）和 alias 0 必须先跑起来。
- **失败分类直接取 `dock_navigator` 自己的判词**（被动订阅 `/rosout`），不改被测节点。
- **终点误差由 bench 从 TF 复算**，不采信节点自评；两者的差单列一节，用于发现 TF 时序问题。

## 文件

| 文件 | 作用 |
|---|---|
| `dock_case.yaml` | ★唯一真源：起点 + dock goal + 容差 + 轮次（**现在是占位坐标**） |
| `dock_common.py` | 纯函数：入位几何、判词解析、流畅性统计（可单测，无 ROS 依赖） |
| `test_dock_common.py` | `dock_common` 的单测，`python3 test_dock_common.py` |
| `run_dock_bench.py` | 编排 N 轮：回起点 → 静置 → 发 `/dock_goal` → 等判词 → 落一行 |
| `dock_metrics.py` | 离线汇总 → 五张表（markdown），无 ROS 依赖 |
| `plot_dock_run.py` | 每轮两联图：全局 + **最后 0.5 m 放大**（剩余距离 vs 有符号横向偏差） |
| `validate_dock_case.py` | 首跑前校核：占位坐标拒跑 + goal/预备点/入位直线的 costmap 代价 |
| `run_dock_bench.sh` | 一键入口（**alias 10**） |

复用 `../controller_bench/` 的 `record_track.py`（轨迹录制）、`bench_common.py`
（bias/spread/CEP 终点统计）、`plot_run.py`（地图底）。给 `record_track.py` 加了两个
**可选**参数，不传时行为与改动前完全一致。

## 用法

```bash
# 0. 一次性：把 dock_case.yaml 的占位坐标换成实测值，然后校核
python3 validate_dock_case.py            # 占位坐标会直接拒跑

# 1. 冒烟 5 轮（alias 3 和 alias 0 必须已在跑，且已给过初始位姿）
./run_dock_bench.sh --runs 5 --battery-v 25.9 --note "地面A"

# 2. 正式 20 轮
./run_dock_bench.sh --runs 20 --battery-v 25.9

# 3. 出报告
python3 dock_metrics.py --session dock_20260909_031200 --out /tmp/dock_report.md
```

## 五类指标

| # | 维度 | 窗口 | 要点 |
|---|---|---|---|
| 1 | 可靠性 | 全程 | 成功率（分母剔除 INVALID）、失败分类计数、**Spin 补偿次数分布** |
| 2 | 入位精度 | 终点 | **达标率算在全部有效轮上**（只算成功轮是同义反复）；bias(系统性) vs spread(随机) 分开 |
| 3 | 稳定性 | 终点 | 围绕质心的 std / CEP50 / CEP95 —— 重复性 |
| 4 | 贴合度 | ★阶段2 | vs **理想入位直线**（过 G、方向 yaw_G）的**有符号**横向偏差 + 航向收敛曲线 |
| 5 | 流畅性 | ★阶段2 | 停顿事件 / w 符号反转 / w 抖动 RMS / 最低速度 / 阶段2 耗时 |

第 4 项用**无限长直线**而不是 `P→G` 线段：阶段1 的落点本身有容差，用线段会把阶段1
的误差算到阶段2 头上。横向偏差**带符号**（左正右负），均值显著非零 = 总偏一侧的
系统性误差；只取绝对值的话左右各偏一半会互相抵消成"看起来很准"。

第 5 项的停顿口径与 `analyze_smoothness.py` 一致（`|cmd_v|<0.02` 持续 ≥0.25 s），
保证两套 bench 可比。**末端为收 yaw 而停是设计行为**，所以只统计进入终点容差圈之前
的停顿，圈内的单列一栏。

## 混淆变量（这台车物理敏感）

每轮记 `--battery-v`（低力矩底盘对电压敏感）；固定同一块地面（脚轮摩擦）；
改完 py 先清 `__pycache__`（`run_dock_bench.sh` 会自动清）。

## 已知缺口

- `/scan` 目前没有发布者（`scan_bridge` 未开），阶段2 **没有动态避障**，只剩静态地图。
  `run_dock_bench.sh` 启动时会告警。只在空场地跑。
- `dock_case.yaml` 的 `approach_dist` 必须与 alias 0 launch 里的 `approach_dist` 一致，
  否则 bench 画的理想直线和车实际走的入位段不是一回事。
