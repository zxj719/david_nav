# refactor-task reference

## 1. Change ticket template

```markdown
## 变更单 <ID>：<一句话目标>
- 类型：缺陷修复 | 结构简化
- 依据：<现场事故 / 可复现故障 / 明确安全风险；以及现有机制为何不够>
- 范围：允许改 <文件/包>；明确不改 <…>
- 必须保留的行为：<回归测试名 / 现场场景>
- 验收证据：离线 <测试，先红后绿>；现场 <场景，逐次记录>
- 回退点：<提交>；配置/地图/启动方式是否随之回退
```

## 2. Commit message template

```text
<type>(<pkg>): <what changed>（<ticket id>）

删掉/改了什么：…
不变的部分：…
证据：<RED→GREEN 测试名>；<包> <N> 通过；清单比对意外删除 0。
待现场（WAITING_FIELD）：…（没有就省略）

Co-Authored-By: <agent identity>
```

## 3. Isolated workspace on David (agx@172.16.22.51)

```bash
git clone /home/agx/ros2_ws_refactor/src ~/ros2_ws_<name>/src      # never edit the deployed tree
git -C ~/ros2_ws_<name>/src checkout -b <branch> <base-commit>
git -C ~/ros2_ws_<name>/src config user.name "<real author>"        # machine default is a shared account
cd ~/ros2_ws_<name> && source /opt/ros/humble/setup.bash
nice -n 19 ionice -c3 env MAKEFLAGS=-j2 colcon build --symlink-install --metas src/colcon.meta --parallel-workers 2
ROS_DOMAIN_ID=99 colcon test --parallel-workers 2 && colcon test-result --all
```

Build in a shell that sourced only Humble. The colcon "underlay" warning for
`nav2_rotation_shim_controller` is expected (vendored patched controller).

## 4. Sandbox with an explicit workspace

```bash
W=$HOME/ros2_ws_<name>; ENVF=$W/src/david_system_env/config/ros_env.conf
REFACTOR_WS=$W SANDBOX_ROS_ENV_FILE=$ENVF bash $W/src/nav_bringup/sandbox_tests/run_sandbox_checks.sh -q
SANDBOX_INSTALL_SETUP=$W/install/setup.bash SANDBOX_ROS_ENV_FILE=$ENVF bash $W/src/nav_sim/sandbox_tests/run_startup_checks.sh
SANDBOX_INSTALL_SETUP=$W/install/setup.bash SANDBOX_ROS_ENV_FILE=$ENVF bash $W/src/nav_sim/sandbox_tests/run_coordinator_checks.sh
```

The runner refuses to start while a navigation/localization/chassis stack runs.
Pass `--basetemp=<dir>` to keep run directories (pytest prunes all but the last 3 sessions).
The two pre-existing sandbox defects (fixture reading the removed `nav2.scan_topic`;
`nav_sim/_ros.py` not catching `ExternalShutdownException`) were fixed on 2026-10-23 (R4-0):
nav_bringup 73/73, nav_sim startup and coordinator checks green twice. If a sandbox check fails,
reproduce it on a pristine build before blaming a change.

## 5. Test inventory for deletions

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws_<name>/install/setup.bash
export ROS_DOMAIN_ID=99
python3 ~/ros2_ws_<name>/src/.claude/skills/refactor-task/scripts/test_inventory.py collect ~/ros2_ws_<name>/src before.txt   # before deleting
python3 ~/ros2_ws_<name>/src/.claude/skills/refactor-task/scripts/test_inventory.py collect ~/ros2_ws_<name>/src after.txt    # after deleting
python3 ~/ros2_ws_<name>/src/.claude/skills/refactor-task/scripts/test_inventory.py diff before.txt after.txt --allow-file planned_removals.txt
```

`planned_removals.txt`: one test function name or test file name per line (`#` comments).
Exit code 1 lists every removed test that is neither planned nor moved.

## 6. Field sheet checklist

- Restart localization (alias 3); set the initial pose; check `tf2_echo map base_link` is plausible.
- Test spot: no mapped obstacle within footprint + inflation (David: ≥ 1 m sideways, 2.5 m ahead).
- Goals ≥ 3× `xy_goal_tolerance` (David tour: 0.30 m → use ≥ 1.0 m); out-and-back legs.
- Cancel test; a continuous run ≥ 10 min; per-trial table (result, time, moved, tape, heading, error, stop).
- Operator sets the acceptable degradation **before** comparing.
- Example: `weekly_report` W46 second-round sheet; goal tool `send_forward_goal.py` (dry-run, turn-around legs, cancel, divergence guard).
