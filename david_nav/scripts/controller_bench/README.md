# controller_bench — 导航 controller 固定测试框架

用**一条固定 map 折线**当所有 controller 共享的参考,重复跑 N 次,量三件事:
**可靠性**(成功率+失败分类)、**稳定性**(终点重复性)、**精确性**(终点 vs goal,含 yaw)、
**跟踪**(实际轨迹 vs 参考折线的 cross-track)。

设计取舍见 [../../docs/reports/2026-07-07-nav-fixed-testcase-design.md](../../docs/reports/2026-07-07-nav-fixed-testcase-design.md)。

## 关键约定

- **真值 = `david_slam_gt` 的 `/groundtruth/pose`**(`PoseWithCovarianceStamped`,map 系
  base_footprint,20Hz;来自 slam_toolbox localization 锁图)。统一当绝对真值,忽略定位漂移
  (用户决定)。位置含 yaw。拿不到时 record/run 退回直接查 `map->base` TF(同一位姿)。
- **只测 controller,不测 planner**:planner 对所有 controller 都被绕过,大家吃同一条固定折线。
- **一条折线覆盖 S1-S4**(直行 / 原地转 / L 形 turn-drive-turn / 入位)。见 `reference_path.yaml`。
- **一次 `run_bench.sh` = 一次 run**。每次跑前**物理复位机身到起点(贴地面胶带)+ 重给 slam 初始位姿**,
  否则上一次的漂移污染这一次。重复 N 次拿分布。

## 文件

| 文件 | 作用 |
|---|---|
| `reference_path.yaml` | ★唯一真源:固定 start + waypoints(**占位坐标,建图后换成本地图实测值**)+ 容差 |
| `bench_common.py` | 纯函数:body-frame 变换、cross-track、CEP/bias-spread、循环 yaw 统计(可单测) |
| `record_track.py` | 被动记录器(controller 无关):`map->base` 实际轨迹 + cmd_vel 全链 + action 状态 → `track_<id>.csv` |
| `run_bench.py` | 驱动器:发参考 Path、逐段驱动被测 controller、算终点误差、追加 `runs.csv` |
| `run_bench.sh` | 单次 run 编排:起 recorder → 驱动 → 收尾 |
| `plot_run.py` | ★每 run 自动出图:占据地图底 + 参考折线 + waypoints + **真值轨迹** → `plot_<id>.png` |
| `validate_reference_on_map.py` | 首跑前对活 `/map` 校核参考折线不穿墙(OK/UNKNOWN/BLOCKED) |
| `bench_metrics.py` | 离线汇总 N 次 → 三张指标表(markdown) |

> `run_bench.sh` 跑完会自动:停 recorder → `plot_run.py` 出 `plot_<id>.png`(地图底优先抓活
> `/map`,抓不到退回磁盘 pgm)。`track_<id>.csv` 现在同时录**真值位姿**(actual_x/y/yaw,来自
> `/groundtruth/pose`)和 **odom 位姿**(odom_x/y/yaw)——两者之差 = 定位漂移 vs 控制误差。

## 先决条件(本框架不启动,需已在跑)

1. **`david_slam_gt` localization**(真值来源,见 [../../../david_slam_gt/](../../../david_slam_gt/)):
   ```bash
   # 先建图存图(仅首次 / 换场地):另起 wheeltec + 遥控,把场地转一圈
   ros2 run david_slam_gt launch_slam_gt.sh mapping
   ros2 run david_slam_gt save_map.sh ~/ros2_ws/src/david_slam_gt/maps/david_map
   # 再锁图定位(每次测试):发 /groundtruth/pose
   ros2 run david_slam_gt launch_slam_gt.sh localization ~/ros2_ws/src/david_slam_gt/maps/david_map
   ```
2. wheeltec 底盘驱动(出 `/odom`,给 slam_gt 的 `odom->base`)。
3. 被测 controller 节点(现:`position_controller`,提供 `/position_controller/move` + `set_parameters`)。

> 先确认真值在发:`ros2 topic echo /groundtruth/pose --once`。
> (frame=map,pose=base_footprint;base 帧不一致时设 `export BENCH_BASE_FRAME=<你的>`。)

## 用法

```bash
# 0. 一次性:把 reference_path.yaml 的占位坐标换成你地图里的实测 start/waypoints。

# 1. 跑一次(先复位机身到起点胶带 + RViz 重给初始位姿):
./run_bench.sh S1S4_run01 --battery-v 25.9 --note "地面A"
#   → /tmp/david_nav_logs/bench/track_S1S4_run01.csv  (轨迹)
#   → /tmp/david_nav_logs/bench/runs.csv              (追加一行汇总)

# 2. 重复 N 次(建议冒烟 10,报 CEP95 上 30),每次换 run-id:
./run_bench.sh S1S4_run02 --battery-v 25.7
# ...

# 3. 汇总出报告:
python3 bench_metrics.py --out /tmp/bench_report.md
#   仅看某 controller:  --controller position_controller
```

## 加一个新 controller(如 DWB)

在 `run_bench.py` 里实现一个 `ControllerAdapter` 子类(`drive(runner, ref)` 返回每段状态),
DWB 版把固定折线发成 `Path` 走 `FollowPath`(仍绕过 planner),`--controller` 加选项即可。
recorder / metrics / 参考路径都不用改 —— 这就是"统一框架"的意义。

## 混淆变量(必记,这台车物理敏感)

每次 run 记 `--battery-v`(低力矩底盘对电压敏感);固定同一块地面(caster 摩擦);
改完 python 先清 `__pycache__` + rebuild 再测(陈旧 pyc 坑)。
