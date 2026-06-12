# 设计：让相邻路径点的直线段不穿越膨胀层（inflation-aware sparsifier）

日期：2026-06-11 ・ 状态：设计稿（未实现）
关联代码：`david_nav/waypoint_sparsifier.py`、`david_nav/grid_map_view.py`、
`config/waypoint_follower.yaml`、`config/nav2_params_2d.yaml`（global_costmap）

## 0. 问题一句话

waypoint_follower 把 NavFn 的密集路径稀疏成"相邻点 ≤2m、直线无碰"的 waypoint，
直线检查（LOS）阈值是 99（inscribed）——**只保证连线不撞墙体本身，允许整段穿
膨胀带**。要改成：连线与障碍保持"膨胀级"的距离。

## 1. 现状解剖（为什么现在等于没有膨胀约束）

### 1.1 costmap 实际值分布（published /global_costmap/costmap，0..100）

| 离墙距离 d | 内部 cost | published | 现 LOS(≥99 阻断) |
|---|---|---|---|
| 墙内/贴墙 | 254 | 100 | 阻断 |
| d < 0.30 (inscribed) | 253 | 99 | 阻断 |
| 0.30 ≤ d < 0.40 | 252·e^(-1.5(d-0.30)) ≈ 217 | ~84 | **放行** |
| d ≥ 0.40 (inflation_radius) | 0 | 0 | 放行 |

要点：
- `cost_scaling_factor 1.5` + `inflation_radius 0.40` + 分辨率 0.1m
  → **膨胀梯度带只有 1 格宽**。"costmap 实际值是 0"的观感来源于此：
  99% 的自由区真的是 0，梯度带窄得在 RViz 里几乎看不见。
- 静态层 `trinary_costmap: true`：/lightning/grid_map 进来只有 0/100/-1，
  没有概率灰度——梯度只能靠 inflation_layer 造出来。
- obstacle_layer 当前是瞎的（/scan 被网内 om1 设备污染，时间戳全错被
  Message Filter 丢弃）。**修复 /scan 源是"加入 costmap 实际值"的前提**，
  否则动态障碍永远进不了 costmap（见 §5 前置项）。

### 1.2 为什么"中心线不穿膨胀"现在也不够

LOS 检查的是**中心线**。机器人是 0.84×0.60 的矩形，控制又有滑差：

| 误差源 | 量级 |
|---|---|
| 中间 waypoint 容差 wp_dist_tol | 0.8 m |
| caster 旋转过冲 | 20°~45°（弧长 ~0.2-0.4m）|
| 定位运动期漂移（2026-06-11 实测） | ~0.85 m |

中心线哪怕严格贴着 0.40m 膨胀边走，**实际车身可以越线 0.8m**。
所以本设计必须两条腿：①膨胀带加宽到能吃下控制误差，②LOS 阈值真正用上膨胀值。

## 2. 设计

### 2.1 第一层：把膨胀带做宽（纯参数，nav2_params_2d.yaml）

```yaml
inflation_layer:
  inflation_radius: 0.75        # 0.40 → 0.75：inscribed(0.30) + 控制横向误差预算(~0.45)
  cost_scaling_factor: 2.5      # 1.5 → 2.5：把梯度铺满 0.30~0.75m 带
```

改后 published cost ↔ 离墙距离对照（决策依据表）：

| 离墙 d (m) | published cost |
|---|---|
| <0.30 | 99/100 |
| 0.40 | ~76 |
| 0.50 | ~59 |
| 0.60 | ~46 |
| 0.70 | ~36 |
| ≥0.75 | 0 |

NavFn 吃同一张图：密集路径会被推向走廊中线（代价场变宽了），这本身就把
大部分"贴墙"问题在源头解决——稀疏化只需要"不要毁掉 NavFn 已经做对的事"。

### 2.2 第二层：sparsifier 双阈值 LOS（代码，waypoint_sparsifier.py）

新增参数（waypoint_follower.yaml）：

```yaml
los_cost_strict: 50      # 中间段首选阈值 ≈ 中心线离墙 ≥0.55m
los_cost_threshold: 99   # 不变,作为兜底(=inscribed,可通过的物理底线)
```

`sparsify()` 选点逻辑改为（伪码）：

```
对每个 anchor:
    best_strict = 在 ≤max_segment 内,线上所有格 cost < strict 的最远点
    若存在 best_strict → 取之                     # 干净走廊:远离膨胀
    否则 best_loose = 同样规则但阈值用 99 → 取之   # 窄区(门洞/过道):退回今天的行为
    否则 advance-one fallback(现状保留),但打 WARN  # 兜底段未经检查,要看得见
```

关键性质：
- **开阔区**：段与障碍保持 ~0.55m+，不再切角穿膨胀带。
- **窄区不瘫痪**：门洞处 strict 找不到点，自动退 99——和今天行为一致，
  不会出现"阈值收紧后机器人在走廊里发几百个 0.1m 小段"的退化。
- 退化路径全部有日志（[TRACE] 标 strict/loose/fallback），实车可归因。

### 2.3 末段豁免（必须有，否则贴桌目标永远到不了）

递水/抓取的目标点天然离桌沿 <0.75m——goal 本身就在膨胀带里。规则：

- **最后一段（→goal）与进场段（→approach A）只用 99 阈值**，不用 strict；
- `insert_approach_waypoint` 的两条 line_clear 同样用 99（现状不变）。

即：中间巡航远离膨胀，末端入位允许进入膨胀（物理上可通过即可）。

### 2.4 配套修正：executor 扩圈判定（navigation_goal_resolver）

`is_goal_free` 现在要求 cost==0。膨胀加宽后，贴桌 goal 必然 cost>0 →
会被扩圈挪走最多 1m（fallback_snap_radius）——这就是"明明能到却停在
1m 外"的另一个制造者。改：goal 可达判定放宽为 `cost < 99`（物理可站立），
而不是"完全自由"。

### 2.5 已知弱点声明（本期不修，记录在案）

- Bresenham 单线:斜穿时只查线穿过的格,不查两侧贴角格。0.1m 分辨率下
  对角缝隙可能漏检。后续可换 supercover 线(查线触及的所有格)。
- 膨胀只来自静态图(若 obstacle_layer 继续禁用):运动中冒出来的障碍
  仍然不可见——属于 /scan 修复议题,不在本设计内。

## 3. 不选的方案及理由

| 方案 | 否决理由 |
|---|---|
| 直接把 los_cost_threshold 99→1 | 门洞/窄过道处 NavFn 密集路径本身就带 cost>0,LOS 全挂 → advance-one fallback 每 0.1m 发一个段,且 fallback 段反而未经检查 |
| 只加宽膨胀、不动 sparsifier | chord 切角仍可穿带:LOS 阈值 99 对 1~98 的膨胀值不设防 |
| 代价积分/最小最大代价的连续优化 | 收益对本场景不明显,调参成本高;双阈值已覆盖"开阔走干净、窄区能通过"两个需求 |
| 让 position_controller 沿密集路径跟线 | 推翻"盲走直线段"的既有架构,涉及控制重写,超出本期范围 |

## 4. 验证计划

1. 单测（test_waypoint_sparsifier.py 扩展）：
   - 构造带梯度的 grid：开阔区 strict 选点远离墙；
   - 窄走廊（带内值 60~80）：strict 无解 → 退 99 仍可通过；
   - 末段豁免：goal 在 cost=80 的格子上，最后一段仍可达；
   - fallback 触发时返回值与 WARN 路径。
2. 干跑：固定一张实测 grid_map 快照，对比改前/改后 wps_map 与墙距
   （[TRACE] plan 行直接给出坐标，叠在 RViz 上看）。
3. 实车：同一目标改前/改后各 3 次，统计段轨迹离墙最小距离 + 是否有
   strict→loose 退化日志。

## 5. 前置项（不做这个，本设计白做）

1. **/scan 污染**：om1_ros2_driver_node（网内另一台设备，domain 0）抢占
   /scan,时间戳为设备开机秒数 → obstacle_layer 全丢。要么关 obstacle_layer
   （纯静态图导航,膨胀仍有效）,要么换专用 scan topic / 隔离 om1。
2. 定位动态漂移 ~0.85m（2026-06-11 实测）:误差预算表里它最大。SLAM 侧
   不收敛的话,膨胀加到 0.75 也兜不住 —— 该项决定 inflation_radius 最终取值,
   联调时按实测漂移回填。
