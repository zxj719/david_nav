# ADR-0003: keep snap policy, remove unused static/dynamic distinction

The extracted resolver is a pure search over GridMapView snapshots. ROS
subscription ownership belongs to the ROS adapter, not the search algorithm.
Ring radii, tolerance, candidate order, zero-cost checks, clearance and fallback
behavior are unchanged and compared with the legacy implementation in tests.

After arrival at a snap point, the current policy accepts success whenever the
anchor remains inadmissible, whether the obstacle is static or dynamic. It only
starts the anchor segment if a fresh admissibility check passes. Therefore the
old `is_anchor_static_blocked` helper is not part of the new GoalResolver port or
implementation. It has no caller controlling this policy. The retained legacy
reference class still exists for compatibility tests; runtime shims now dispatch
through the new task manager rather than using it as the motion owner.

Current implementation: [search](../nav_tasks/core/goal_resolution.py),
[POI skill](../nav_tasks/core/handlers/navigate_to.py) and
[wire regression](../../nav_executor/test/test_task_wire_golden.py).
The archived 2026-07-24 distinction below is superseded by the 2026-07-27 decision;
it is retained as source history, not revived as current behavior.

The Dock skill is separate: dock POIs must not use point snapping, and the static
map footprint validation for their preparation pose remains mandatory.

## 背景原文（逐字保留）

下列片段取自标明的不可变提交，保留原标点、缩进和时间。它们不是本轮新的现场测量，
也不是实体停车/到达验收；存在与现状不同的历史描述时，以本 ADR 上方当前决定为准。

### snap-success

来源：`93676a5:david_nav/david_nav/navigation_executor_core.py`。
SHA256：`ba103637fb79ad94f494fdfc16d4c35b5d5291e077156e3eb3b2f2f60c5950d8`。

<!-- history: snap-success -->

```python
                    # 2026-07-27：到达扩圈可行点 = 导航成功，不再区分
                    # "静态占用（永久）/costmap 动态占用（可能散去）"。
                    # 实测旧语义下 anchor 落在膨胀区（静态图自由但 costmap
                    # 代价>0，永远不会变 admissible）时报"仍不可达"，
                    # 服务端只会原地重试同样失败。resolver 连续外扩选出的
                    # 本来就是最近可达点，到点即是该目标的最好结果。
```

### static-distinction

来源：`93676a5:david_nav/david_nav/navigation_goal_resolver.py`。
SHA256：`b92d6f9da8d3393885d7c0ceb5397e805a5c9ab6fe0733bae3fdd486ce3b60dd`。

<!-- history: static-distinction -->

```python
        """anchor 是否被【静态地图】占用（= 永久不可达）。

        2026-07-24 需求：POI 压在静态图障碍里（标在货架/墙上的语义点）时，
        扩圈可行点就是物理上能到的最好位置 —— core 据此把"到达扩圈点"
        判为导航成功；只有"静态图自由、costmap 动态占用"（障碍可能散去）
        才保留"仍不可达"报错。静态图未就绪时返回 False（信息不足，
        维持原报错语义，不误判成功）。
        """
```
