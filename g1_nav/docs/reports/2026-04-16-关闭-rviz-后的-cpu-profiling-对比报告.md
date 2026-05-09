# 2026-04-16 关闭 RViz 后的 CPU Profiling 对比报告

## 1. 报告目的

本报告用于对比以下两种运行状态下的 CPU 占用差异：

1. 开启 RViz：

```bash
./start_navigation_headless.sh -rse
```

2. 关闭 RViz：

```bash
./start_navigation_headless.sh -se
```

目标是回答两个问题：

1. 之前 CPU 很高到底是不是 RViz 导致的
2. 关闭 RViz 之后，当前导航链路的 CPU 基线是多少，主要由哪些模块构成

对比基线报告：

- [2026-04-16-headless-navigation-cpu-profiling-report.md](/home/unitree/ros2_ws/src/g1_nav/docs/reports/2026-04-16-headless-navigation-cpu-profiling-report.md)

## 2. 当前测试场景

本次现场命令：

```bash
./start_navigation_headless.sh -se
```

含义：

- 不启动 RViz
- 不启用 Realsense scan bridge
- 不启用 frontier exploration
- 保留：
  - `map -> odom` 静态 TF
  - Lightning SLAM
  - Nav2 bringup
  - navigation executor

采样时间：

- 2026-04-16 11:38，Asia/Shanghai

## 3. 关闭 RViz 后的系统级快照

`top` 摘要：

```text
load average: 4.64, 7.04, 5.39
%Cpu(s): 27.4 us, 12.3 sy, 57.5 id
Mem: 15655.6 MiB total, 9213.0 MiB free
Swap: 73363.8 MiB total, 0 used
```

解读：

- 当前系统仍然有一定 CPU 压力，但已经明显不是“高负载卡死”状态
- CPU 空闲比例达到 `57.5%`
- 内存压力很低，交换区未使用
- 这说明关闭 RViz 后，系统从“明显 CPU 受限”回到了“有余量”的状态

## 4. 关闭 RViz 后的主要 CPU 进程

现场 `ps` 快照中的主要进程如下：

| PID | 进程 | CPU 约值 | 说明 |
|---|---|---:|---|
| 9648 | `gnome-shell` | 60.1% | 桌面环境本身仍然有一定开销 |
| 16229 | `run_slam_online` | 39.2% | 当前最高的导航链路相关进程 |
| 16352 | `navigation_executor.py` | 21.5% | 第二高的导航链路相关进程 |
| 16494 | `g1_move` | 14.3% | 运动控制链路 |
| 16458 | `lifecycle_manager_navigation` | 13.7% | Nav2 生命周期管理 |
| 16412 | `bt_navigator` | 12.2% | Nav2 行为树导航 |
| 16384 | `controller_server` | 10.1% | Nav2 控制器 |
| 16393 | `behavior_server` | 10.1% | Nav2 行为服务器 |
| 16388 | `planner_server` | 9.3% | Nav2 全局规划 |
| 16386 | `smoother_server` | 8.2% | Nav2 平滑器 |
| 16490 | `collision_monitor` | 8.1% | 碰撞监控 |
| 16445 | `velocity_smoother` | 7.6% | 速度平滑 |
| 16441 | `waypoint_follower` | 6.9% | waypoint 跟随 |

关键点：

- 当前已经没有 `rviz2` 这个大头
- 现在的 CPU 消耗主要由：
  - `Lightning SLAM`
  - `navigation_executor`
  - `Nav2` 各节点
  - `g1_move`
  共同组成

## 5. 与“开启 RViz”时的直接对比

### 5.1 系统总负载对比

开启 RViz 时：

```text
load average: 10.79, 7.00, 4.71
%Cpu(s): 70.1 us, 7.5 sy, 19.7 id
```

关闭 RViz 后：

```text
load average: 4.64, 7.04, 5.39
%Cpu(s): 27.4 us, 12.3 sy, 57.5 id
```

结论：

- CPU 空闲从 `19.7%` 提升到 `57.5%`
- 用户态 CPU 从 `70.1%` 降到 `27.4%`
- 这说明 RViz 的移除显著降低了整机 CPU 占用

### 5.2 最大热点进程对比

开启 RViz 时：

- `rviz2`：约 `174% ~ 194%`
- `gnome-shell`：约 `48% ~ 217%`
- `run_slam_online`：约 `12% ~ 25%`
- `navigation_executor.py`：约 `12% ~ 18%`

关闭 RViz 后：

- `rviz2`：不存在
- `gnome-shell`：约 `60%`
- `run_slam_online`：约 `39%`
- `navigation_executor.py`：约 `21.5%`

结论：

- 关闭 RViz 后，最大异常热点进程已经消失
- 原本被 RViz 掩盖的正常链路开销开始显现出来：
  - `run_slam_online`
  - `navigation_executor.py`
  - `Nav2`

### 5.3 总体判断

开启 RViz 时的高 CPU，主因是：

- `rviz2` 的异常渲染路径

关闭 RViz 后的 CPU，占用来源变成：

- 正常的导航运行时负载

也就是说：

- “高 CPU” 这个问题，主锅确实是 RViz
- 关闭 RViz 后，剩下的是系统在正常工作时的 CPU 底座开销

## 6. 线程级 Profiling

### 6.1 关闭 RViz 后的 SLAM

`run_slam_online` 线程视图显示：

- 主线程占用较低
- 少量工作线程在 `7% ~ 11%`
- 多个辅助线程在 `2% ~ 3%`

结论：

- SLAM 现在是一个真实且持续的 CPU 消费者
- 但它表现为“多线程分摊的正常计算负载”，不是异常 runaway

### 6.2 关闭 RViz 后的 navigation_executor

`navigation_executor.py` 线程视图显示：

- 主线程约 `18%`
- 其他线程几乎都接近 `0%`

结论：

- executor 当前 CPU 主要集中在主线程
- 这和之前的观察一致，说明它不是多线程并行渲染/计算型负载
- 结合功能看，更像是：
  - WebSocket 消息处理
  - ROS spin
  - TF / odom / Nav2 bridge 事件处理

### 6.3 关闭 RViz 后的 bt_navigator

`bt_navigator` 线程视图显示：

- 主线程约 `6.1%`
- 另一个工作线程约 `3.5%`
- 其他线程很低

结论：

- `bt_navigator` 占用不算异常
- 它属于 Nav2 正常调度的一部分

## 7. 为什么现在 CPU 仍然不低

即使关闭 RViz，CPU 也不是非常低，主要原因是：

1. Lightning SLAM 本身是持续计算型任务
2. Nav2 整套 bringup 已经全部 active：
   - `controller_server`
   - `planner_server`
   - `behavior_server`
   - `bt_navigator`
   - `velocity_smoother`
   - `waypoint_follower`
   - `collision_monitor`
3. `navigation_executor` 仍在和服务器保持 WebSocket 连接，同时维护 ROS 回调
4. `g1_move` 和其子进程也在持续工作

所以当前看到的 CPU 不是“异常空转”，而是：

- 关闭 RViz 后，导航执行链路的真实运行成本

## 8. 根因判断

综合两次 profiling，可以得出比较明确的判断：

### 8.1 开启 RViz 时

CPU 异常偏高的主因是：

- RViz 地图显示触发了渲染兼容问题
- RViz 落入软件渲染路径
- `llvmpipe-*` 线程持续占满多个核

这与前一份报告中的 RViz 日志完全一致：

```text
GLSL link result:
active samplers with a different type refer to the same texture image unit
```

### 8.2 关闭 RViz 后

CPU 负载结构恢复为：

- SLAM
- executor
- Nav2
- g1_move

这说明：

- 之前“CPU 很高”的异常部分已经基本被移除
- 剩余负载属于系统运行本身的计算成本

## 9. 对当前系统的结论

当前 `-se` 模式下：

- 系统整体 CPU 已明显改善
- 没有再出现 RViz 这种单点异常热点
- 剩余热点都能被解释为导航链路的正常运行开销

因此可以认为：

1. 如果目标是稳定运行导航执行链，不依赖 RViz 可视化，`-se` 是更健康的运行方式
2. 如果要恢复 RViz，需要单独处理 RViz 的地图渲染问题，而不是去优化 SLAM / executor / Nav2

## 10. 建议

建议按优先级分两条线看：

### 10.1 运行侧建议

如果当前任务以实际执行为主：

1. 默认使用 `-se`
2. 把 RViz 视为可选调试工具，而不是默认常驻组件

### 10.2 RViz 修复侧建议

如果后续需要恢复 RViz：

1. 先把 `rviz/nav2_go2_view.rviz` 中以下三项默认关闭：
   - `Map`
   - `Global Costmap`
   - `Local Costmap`
2. 重新 profiling
3. 如果 CPU 恢复正常，再逐项打开，定位具体触发项

## 11. 最终结论

本次对比证明：

- 之前的高 CPU 主要不是导航链路本身造成的
- 主要异常来源是 RViz 渲染问题
- 关闭 RViz 后，CPU 负载显著下降，系统回到了“可解释、可接受”的正常导航运行状态

简化成一句话就是：

- `-rse` 的高 CPU 主要是 RViz 问题
- `-se` 下剩余的 CPU 是 SLAM + Nav2 + executor 的正常底座开销
