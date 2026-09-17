# ADR-0006: keep Dock's blocking worker under the single task owner

Status: implemented in the refactor runtime; physical acceptance remains separate.

PLAN §9 requires one TaskManager and one Nav2 client. SPEC P6 requires the Dock
algorithm and run_action facade to remain blocking work outside ROS callbacks,
not an asyncio rewrite of its two-stage/settling policy.

We keep DockWorkflow synchronous on an explicitly owned thread. Its blocking
motion port passes each request to the existing loop-confined Nav2Client and waits
on an internal future. No second motion owner or action session is created.
The configured Dock host uses a MultiThreadedExecutor for callbacks, while the
one task loop owns backend state. The worker never spins ROS. The ordinary-only
host retains its existing single-threaded mode; this is not an R6 spin-policy change.

The shared runtime, ROS pose/action ingress and optional WS are connected, as are
the retained executable shims and precise launch (884e4d5). See
[public assembly](../nav_tasks/app/runtime.py),
[worker ownership](../nav_tasks/adapters/ros/dock_worker.py),
[Dock host](../../nav_executor/nav_executor/adapters/ros/host.py),
[outer composition](../../nav_executor/nav_executor/app/runtime.py) and
[lifecycle tests](../../nav_executor/test/test_outer_dock_runtime.py).
This does not say production was deployed or that physical stopping was verified.

Consequences:

- Pure policy retains its old timing, measured-pose verdict and frozen logs.
- Backend state and raw feedback callbacks remain on the TaskManager loop.
- Admission, result and cleanup deadlines are separate; a timeout never means
  motion ownership can be discarded. A cleanup error cannot become an advisory
  stage-two failure followed by an apparently successful measured verdict.
- Cancellation is cooperative. Pose/map callbacks must be bounded and thread-safe.
  A stuck callback keeps the owner busy rather than being detached or forcibly killed.
- Worker shutdown is joined, shielded and retryable; the shared backend lives until
  task/worker cleanup is complete. A successful old cleanup is not run again against
  a newer action. Stop fences prevent the old canceled-Spin/acceptable-pose corner
  from leaking a successful task result.
- Pose/static-map observation, explicit robot/mode configuration, ROS ingress,
  structured events and same-process optional WS are assembled. Domain99 no-goal
  startup/cleanup checks are separate from complete POI/Dock task acceptance.
  Component/fake-I/O tests do not authorize motion or prove physical stopping.

## Current settling and footprint invariants

The whole approach footprint must pass static-map admission before stage one;
the approach is not final arrival. After stage two, the existing finish routine
waits for settling before deciding whether a correction is allowed, and waits
again after each Spin. A timeout may still produce a measured diagnostic pose,
but does not license a correction. Spin success alone is not arrival proof.

`SettleSpec` retains 0.05 rad/s, a 0.4 s window and 0.05 s sampling. The window
includes the last sample at or before its beginning: rounding the first sample
up would shorten a non-integral sampling window. These values and code paths
are unchanged by this documentation move; observed yaw stability is not a
guarantee of zero physical velocity. Robot/mode tolerance and retry policy remain
explicit and separate from helper defaults.

## 背景原文（逐字保留）

下列片段取自标明的不可变提交，保留原标点、缩进和时间。它们不是本轮新的现场测量，
也不是实体停车/到达验收；存在与现状不同的历史描述时，以本 ADR 上方当前决定为准。

### finish-module

来源：`ca42041:nav_tasks/nav_tasks/skills/docking/finish.py`。
SHA256：`018a53625b219738dff2da5306ea5f4934ef0abfbdb62edbdf96d38af0451fa1`。

<!-- history: finish-module -->

```python
"""Measure, judge and optionally correct yaw after phase two, without ROS dependencies.

## 这段逻辑为什么要单独拎出来

2026-09-10 dock_bench 首次冒烟（5 轮，3 轮 YAW_NOT_CONVERGED）实测：原来的实现把
这段逻辑内联在 `dock_navigator.execute_docking` 里，一行 ROS 都绕不开，导致
**唯一的验证手段是开着车跑**。而它的 bug 恰恰是时序性的（在运动中测量），
跑一次要 40s、还看天吃饭。抽出来 + 注入 pose/spin 之后，用一个带惯性的假底盘
就能在毫秒级把这类时序 bug 钉死。

## 修的是什么

原实现在阶段2 的 NavigateToPose 一报 SUCCEEDED 就立刻取 TF 测 eyaw，而此时车仍以
**0.55 rad/s** 在转（末端 RotateToGoal 满速收朝向的尾巴）：

    25.66 阶段2 SUCCEEDED, 入位实测 eyaw=6.8°   ← 运动中的瞬时值，不是终点
    25.67 Spin(-6.8°)
    25.77 Spin#1: SUCCEEDED                     ← 100ms 就"成功"
    25.77 旋转后 eyaw=13.9°                      ← 惯性又转过去 7°
    25.88 Spin#2: ABORTED → 失败 eyaw=19.6°

220ms 内两次补偿把误差从 6.8° 推到 19.6°。**nav2 的 `Spin` 用 `abs(relative_yaw)`
判完成**，对一台还在反向转的车，累计转角绝对值瞬间超过目标量 → 立刻 SUCCEEDED。
所以"车没停稳时 Spin 在原理上就不可能起作用，重试只会更糟"。

修法：**每次测量之前、每次 Spin 之后，都先等车真正停稳**（见 dock_settle）。
没停稳就不下发 Spin —— 宁可如实报 `not_settled`，也不做一次注定使误差变大的补偿。
"""
```

### settle-module

来源：`ca42041:nav_tasks/nav_tasks/skills/docking/settle.py`。
SHA256：`93229136745f4605dc52d9cd336b9c41030e08172d40f01ea02e5b57b778776e`。

<!-- history: settle-module -->

```python
"""Detect continuous settling from pose samples without ROS dependencies.

## 为什么需要它

2026-09-10 dock_bench 首次冒烟实测（5 轮，3 轮 YAW_NOT_CONVERGED）暴露：
`dock_navigator` 在阶段2 的 NavigateToPose 报 SUCCEEDED 的**那一瞬间**就取 TF 测
eyaw 并据此下发 Spin —— 而此时车仍以 **0.55 rad/s** 在转（末端 RotateToGoal 满速
收朝向的尾巴）。于是：

    25.66 阶段2 SUCCEEDED, 入位实测 eyaw=6.8°   ← 运动中的瞬时值
    25.67 Spin(-6.8°)
    25.77 Spin#1: SUCCEEDED                     ← 只用 100ms
    25.77 旋转后 eyaw=13.9°                      ← 惯性又转过去 7°
    25.88 Spin#2: ABORTED → 失败 eyaw=19.6°

整个失败序列 **220ms**，车根本没机会停。两次补偿把误差从 6.8° 推到 19.6°。

nav2 的 `Spin` 行为用 **`abs(relative_yaw)`** 判完成，所以对一台"还在往反方向转"
的车，累计转角的**绝对值**瞬间就超过目标量 → 立刻报 SUCCEEDED。也就是说：
**只要车没停稳，Spin 补偿在原理上就不可能起作用**，重试只会让误差更大。

结论：测量和补偿都必须发生在车真正静止之后。这个模块只回答"停稳了吗"。
"""
```

### settle-spec

来源：`ca42041:nav_tasks/nav_tasks/skills/docking/settle.py`。
SHA256：`7616a0fda47338b1b6380cba2c1b84fd7185451e381b80f8c53e43fb85ec176f`。

<!-- history: settle-spec -->

```python
    """Thresholds for a continuously settled trailing sample window.

    max_rate_rad_s : 窗口内任意相邻样本间的角速度上限。实测静态定位抖动
                     yaw std 仅 0.068°(≈0.0012 rad)，20Hz 下抖动带来的表观角速度
                     约 0.02-0.05 rad/s，所以阈值不能定得比这更严，否则永远等不到。
    window_sec     : 必须**连续**满足上限的时长。只看一个瞬时差值不够 —— 车在
                     换向的过零点上瞬时角速度也接近 0。
    sample_period_sec : 轮询周期。

    Attributes:
        max_rate_rad_s: 相邻样本允许的最大表观偏航角速度（弧度/秒）。
        window_sec: 连续满足角速度限制所需覆盖的窗口（秒）。
        sample_period_sec: 请求位姿样本的轮询周期（秒）。
    """
```

### settle-check

来源：`ca42041:nav_tasks/nav_tasks/skills/docking/settle.py`。
SHA256：`2a8c4443651e67221540ff90c89f7330c93950d9f37dfc7ffa66c2b1a1de41d2`。

<!-- history: settle-check -->

```python
    """Check continuous angular-rate stability across the complete trailing window.

    两条都必须满足，缺一不可：
      ① 尾部窗口**被数据铺满**（时间跨度 >= window_sec）—— 这条专门堵"刚停 100ms
         就宣布停稳"，正是实测里 Spin 秒过的那个坑；
      ② 窗口内每一对相邻样本的角速度都低于上限。
    角度差一律先 wrap，否则车停在 ±180° 附近时 +π/−π 的表示切换会被当成 2π 的
    高速旋转。

    Args:
        samples: Time/yaw samples ordered by ascending time.
        spec: Continuous-window and angular-rate thresholds.

    Returns:
        Whether the trailing window is covered and every measured rate satisfies the threshold.
    """
```

### settle-bridge

来源：`ca42041:nav_tasks/nav_tasks/skills/docking/settle.py`。
SHA256：`c2616c2b13129714f8a02be9f6cd2b9c07eb3ea0572bbd42eec10fff0189da9a`。

<!-- history: settle-bridge -->

```python
    # 取最后一个"不晚于窗口起点"的样本当**桥接样本**，窗口从它开始。
    # ⚠ 不能只取 t >= t_cut 的样本：那样窗口跨度是 Δ×floor(window/Δ)，
    #   只有采样间隔 Δ 整除 window 时才等于 window。真机上 pose_now() 自身有
    #   耗时，Δ 恒略大于 sample_period（0.05 -> 0.051），floor 掉一档，跨度
    #   永远差一点点，于是**永远判不出停稳** —— 2026-09-10 实测每轮都跑满 8s
    #   超时报 NOT_SETTLED，而同一时段录下来的 TF 明明是静止的。
    #   单测当时用完美的 0.05 等间隔（0.4/0.05=8 刚好整除）把这个 bug 放过去了。
```
