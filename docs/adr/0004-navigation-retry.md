# ADR-0004：普通导航单次失败静默重试

状态：已迁入纯 `RetryPolicy`，行为及中文错误文字保持。

## 背景原文（保留）

来源 `93676a5:david_nav/david_nav/nav2_action_bridge.py`：

```python
# 失败消抖(2026-07-24):nav2 偶发单次 ABORTED(控制器抽风/costmap 瞬时
# 占用/启动初期 action server 未就绪)会造成假失败。单次失败不上报 agent,
# 等 DEFAULT_GOAL_RETRY_DELAY_SEC 后重发同一个导航点;连败耗尽重试预算才
# 发 on_error。CANCELED 是主动取消(abort/preempt),不算失败、不重试。
```

## 决定与当前落点

[RetryPolicy](../../nav_tasks/nav_tasks/core/retry.py) 默认额外重试 1 次、间隔 1 秒；
`NavigationBackend.navigate` 只负责一次尝试。只对 FAILED 返回值重试，不吞取消或未知
异常。耗尽报 `导航失败(共尝试{n}次): {detail}`；每段导航独立预算。
`RetryAudit` 保留每次失败及耗尽文字，但中间失败不发 on_error。
取消/抢占在尝试前后和等待边界有防护，不能复活旧目标。

精准 Dock 默认不对整套两段位移静默重试，独立旋转补偿策略不等于本重试策略。
该机制也不是 BT recovery：用户追加的脱困后重新规划/跟踪不得提前 Goal succeeded，
其行为树回归另有覆盖，不由这里把“脱困成功”当作任务到达。

## 证据与后果

`nav_executor/test/test_task_wire_golden.py` G7/G8 验证先失败后成功/耗尽错误字节，
`nav_tasks/test/test_core_primitives.py` 和 `test_navigate_skill.py` 覆盖预算、延迟及取消。
测试使用假时钟/后端，不证明实体恢复能力；未知运行异常由任务 supervisor 处理。
