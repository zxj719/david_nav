#!/usr/bin/env python3
"""alias 0 专用：高精度两段式入位导航（2026-08-31）。

重构分支：以下历史两个入口现合并为一个任务管理器。开启 WS 时，Dock 与 WS
一起在 Nav2 就绪后开放；禁用 WS 时仍提供本地 Dock。已有任一任务管理器则拒绝
此次启动，不自动结束用户进程。生产文件和 alias 没有随此提交切换。

= 现有 2d_david_nav2_bringup.launch.py（传高精度参数快照）
+ dock_navigator（两段式对齐入位编排节点）
+ wheeltec 底盘（原 alias 5，2026-09-08 并入；已经在跑就自动跳过）
+ navigation_executor（alias 4 的 server 模块，2026-09-15 并入；状态上报与 POI 接收
  与 alias 4 相同；dock_poi_ids 里的 POI 导航时走两段式精准入位）

生产导览 alias 4 完全不受影响：本文件不修改任何生产文件，只是换一份
params_file 并额外起节点。

话题分工（关键，别搞混）：
  /dock_goal   -> dock_navigator   两段式入位（rviz 的 "2D Goal Pose" 工具发这个）
  /goal_pose   -> bt_navigator     单段直达（CLI 直接发，用于对照测试）
故意分开两个话题：bt_navigator 自己订阅 /goal_pose，若 dock_navigator 也收同一个
话题，一个目标会同时触发两条链路。

2026-09-08 变更：底盘并入（原来要另开一个终端跑 alias 5）。
  - 起之前先 pgrep 查 wheeltec_robot_node / twist_transform / "launch
    turn_on_wheeltec_robot"，**已在跑就跳过**：重复起会抢串口
    （PortNotOpenedException 复盘见 2026-07-24）并双发 /odom。
  - 自起底盘时，nav2 延后 chassis_startup_delay 秒再拉（对齐
    start_navigation_headless.sh 的 CHASSIS_STARTUP_DELAY_SEC=3）；跳过或
    start_chassis:=false 时不延后。
  - Ctrl-C 只收掉本 launch 自己起的底盘；跳过的那份（你手动的 alias 5）不动。

2026-09-15 变更：server 模块并入（原来只有 alias 4 有）。
  - nav2 就绪（/navigate_to_pose 可用，scripts/wait_nav_ready.py 探测）后才起
    navigation_executor，与 start_navigation_headless.sh 的顺序一致。
  - server 地址默认读 config/navigation_headless.yaml 的 executor.server_uri，点位文件
    默认 ~/semantic_map/named_poses.json，都与 alias 4 相同。
  - dock_poi_ids（默认 POI_017）里的 POI：agent 下发 navigate_to 时交给 dock_navigator
    的 dock_to_pose 动作做两段式精准入位，不做目标扩圈、失败不静默重发；其它 POI 照旧
    直发 bt_navigator（本 launch 的高精度参数）。
  - 已有 navigation_executor 在跑（例如 alias 4 没收干净）时只告警，不起第二个。
  - start_executor:=false 恢复旧行为（纯本地导航，不连 server）。
"""

import os
import shlex
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 判定"底盘已经在跑"的进程特征。三条都查是因为 alias 5 / alias 4 / 手工 ros2 run
# 起出来的进程树形状不同：alias 5 会留下 `ros2 launch turn_on_wheeltec_robot`
# 这个父进程，headless 里是 bash -c 包一层，两个 Node 本身则总在。
CHASSIS_PROC_PATTERNS = (
    'wheeltec_robot_node',
    'twist_transform',
    'launch turn_on_wheeltec_robot',
)

# 同理判定"nav2 已经在跑"。本 launch 不会去杀它（那是用户自己的终端），只告警。
NAV2_PROC_PATTERNS = (
    'nav2_controller/controller_server',
    'nav2_bt_navigator/bt_navigator',
)

# 判定"server 模块已经在跑"。两个 executor 会同时连 agent server、抢同一批指令。
EXECUTOR_PROC_PATTERNS = (
    'navigation_executor.py', 'nav_task_manager',
    'lib/nav_executor/navigation_executor', 'lib/david_nav/dock_navigator',
    'david_nav/dock_navigator.py',
)

DEFAULT_WHEELTEC_WS = '~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2'
DEFAULT_ROS_SETUP = '/opt/ros/humble/setup.bash'


def _pgrep(pattern):
    """pgrep -f 的 pid 列表。无匹配时 pgrep 退出码 1，这里统一返回空表。"""
    try:
        proc = subprocess.run(
            ['pgrep', '-f', '--', pattern],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=5)
    except Exception:  # noqa: BLE001  pgrep 不存在/超时都按"查不到"处理
        return []
    return [int(p) for p in proc.stdout.split() if p.strip().isdigit()]


def _pids_for(patterns):
    pids = set()
    for pat in patterns:
        pids.update(_pgrep(pat))
    pids.discard(os.getpid())
    pids.discard(os.getppid())
    return sorted(pids)


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _source_package_dir():
    """本 launch 所在的源码包目录（install 里的 launch 目录是指向源码的软链）。"""
    return os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def build_executor_script(executor, probe, executor_args, nav_ready_timeout_sec):
    """等 nav2 就绪后再 exec navigation_executor 的 bash 脚本（与 headless 的启动顺序一致）。"""
    timeout = int(float(nav_ready_timeout_sec))
    return (
        'deadline=$((SECONDS + {timeout})); '
        'until python3 {probe} --timeout 5 --quiet; do '
        'if (( SECONDS >= deadline )); then '
        'echo "[precise_nav] nav2 在 {timeout}s 内未就绪，不启动 navigation_executor" >&2; '
        'exit 1; fi; sleep 1; done; '
        'echo "[precise_nav] nav2 已就绪，启动 navigation_executor"; '
        'exec python3 {executor} {args}'
    ).format(
        timeout=timeout,
        probe=shlex.quote(probe),
        executor=shlex.quote(executor),
        args=' '.join(shlex.quote(str(arg)) for arg in executor_args),
    )


def _executor_actions(context):
    if not _as_bool(LaunchConfiguration('start_executor').perform(context)):
        return [LogInfo(msg='[precise_nav] start_executor:=false —— 不起 server 模块'
                            '（不连 agent server）')]
    running = _pids_for(EXECUTOR_PROC_PATTERNS)
    if running:
        return [LogInfo(msg=(
            '[precise_nav] ⚠ 已有 navigation_executor 在跑 (pid=%s)，不再起第二个 —— '
            '两个执行器会同时连 agent server、抢同一批指令。先收掉旧的再重启本 launch。'
            % ','.join(str(p) for p in running)))]

    source_dir = _source_package_dir()
    executor = os.path.join(source_dir, 'david_nav', 'navigation_executor.py')
    probe = os.path.join(source_dir, 'scripts', 'wait_nav_ready.py')
    for path in (executor, probe):
        if not os.path.isfile(path):
            return [LogInfo(msg='[precise_nav] ⚠ 找不到 %s，不起 server 模块' % path)]

    server_uri = LaunchConfiguration('server_uri').perform(context).strip()
    reconnect_delay = 3.0
    try:
        from david_nav.headless_config import build_executor_settings, load_headless_config
        settings = build_executor_settings(load_headless_config(
            os.path.join(source_dir, 'config', 'navigation_headless.yaml')))
        server_uri = server_uri or settings['server_uri']
        reconnect_delay = settings['reconnect_delay']
    except Exception as exc:  # noqa: BLE001  配置读不到时允许用 server_uri 参数兜底
        if not server_uri:
            return [LogInfo(msg='[precise_nav] ⚠ 读不到 navigation_headless.yaml（%s）'
                                '且未传 server_uri，不起 server 模块' % exc)]
    if not server_uri:
        return [LogInfo(msg='[precise_nav] ⚠ navigation_headless.yaml 没有 '
                            'executor.server_uri，不起 server 模块')]

    pose_file = os.path.expanduser(LaunchConfiguration('pose_file').perform(context))
    dock_ids = [poi.strip() for poi in
                LaunchConfiguration('dock_poi_ids').perform(context).split(',') if poi.strip()]
    args = ['--server-uri', server_uri, '--reconnect-delay', '%g' % reconnect_delay,
            '--pose-file', pose_file]
    for poi_id in dock_ids:
        args += ['--dock-poi-id', poi_id]
    args += ['--dock-config-file', _dock_policy(), '--websocket', '--ros-args']
    for key, value in _dock_parameters(context).items():
        args += ['-p', '%s:=%s' % (key, value)]
    timeout = LaunchConfiguration('nav_ready_timeout').perform(context)
    return [
        LogInfo(msg='[precise_nav] server 模块：%s，点位文件 %s，精准入位 POI：%s'
                    '（nav2 就绪后启动）' % (server_uri, pose_file, ','.join(dock_ids) or '无')),
        ExecuteProcess(cmd=['bash', '-c', build_executor_script(executor, probe, args, timeout)],
                       name='navigation_executor', output='screen'),
    ]


def _dock_policy():
    return os.path.join(get_package_share_directory('david_config'), 'dock', 'standalone.yaml')


def _dock_parameters(context):
    return {
        'goal_topic': LaunchConfiguration('dock_goal_topic').perform(context),
        'approach_dist': float(LaunchConfiguration('approach_dist').perform(context)),
        'xy_ok': float(LaunchConfiguration('xy_ok').perform(context)),
        'yaw_ok': float(LaunchConfiguration('yaw_ok').perform(context)),
        'max_yaw_retries': int(LaunchConfiguration('max_yaw_retries').perform(context)),
        'robot_base_frame': 'base_footprint', 'global_frame': 'map',
        'action_name': 'dock_to_pose',
    }


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('david_nav')
    actions = []
    running = _pids_for(EXECUTOR_PROC_PATTERNS)
    if running:
        return [LogInfo(msg='[precise_nav] 已有任务管理器 (pid=%s)，拒绝重复启动；'
                            '请先检查旧进程，不自动清理。' % ','.join(map(str, running)))]
    websocket = _as_bool(LaunchConfiguration('start_executor').perform(context))
    manager_actions = _executor_actions(context) if websocket else [Node(
        package='nav_executor', executable='nav_task_manager', name='dock_navigator',
        output='screen', arguments=['--dock-config-file', _dock_policy(), '--no-websocket'],
        parameters=[_dock_parameters(context)])]
    if websocket and not any(isinstance(action, ExecuteProcess) for action in manager_actions):
        # A missing/occupied WS root must not leave a second local motion owner behind.
        return manager_actions

    # ---------- 重复启动：nav2 ----------
    # 两套 nav2 会有两个 controller_server 抢 /cmd_vel_nav、两个 lifecycle_manager
    # 抢 bond，症状是车抽搐或反复 abort。不自动杀（那是用户另一个终端的前台进程），
    # 只把话说清楚。
    stale_nav = _pids_for(NAV2_PROC_PATTERNS)
    if stale_nav:
        actions.append(LogInfo(msg=(
            '[precise_nav] ⚠⚠ 已经有一套 nav2 在跑 (pid=%s)。再起一套会两个 '
            'controller_server 抢 /cmd_vel_nav。请 Ctrl-C 掉这次启动，先收掉旧的那份。'
            % ','.join(str(p) for p in stale_nav))))

    # ---------- 底盘（原 alias 5） ----------
    start_chassis = _as_bool(LaunchConfiguration('start_chassis').perform(context))
    delay = float(LaunchConfiguration('chassis_startup_delay').perform(context))
    running_chassis = _pids_for(CHASSIS_PROC_PATTERNS)

    if not start_chassis:
        actions.append(LogInfo(msg='[precise_nav] start_chassis:=false —— 底盘不管，'
                                   '需要你自己另起 alias 5'))
        delay = 0.0
    elif running_chassis:
        actions.append(LogInfo(msg=(
            '[precise_nav] 底盘已在运行 (pid=%s)，跳过自起 —— 重复起会抢串口'
            '(PortNotOpenedException) 并双发 /odom。Ctrl-C 本 launch 不会动它。'
            % ','.join(str(p) for p in running_chassis))))
        delay = 0.0
    else:
        ws = os.path.expanduser(os.environ.get('WHEELTEC_WS_DIR', DEFAULT_WHEELTEC_WS))
        setup = os.path.join(ws, 'install', 'setup.bash')
        ros_setup = os.environ.get('DAVID_ROS_SETUP_FILE', DEFAULT_ROS_SETUP)
        if not os.path.isfile(setup):
            actions.append(LogInfo(msg=(
                '[precise_nav] ⚠ 找不到底盘工作空间 %s，跳过自起。'
                '设 WHEELTEC_WS_DIR 或自己跑 alias 5。' % setup)))
            delay = 0.0
        else:
            cmd = 'source %s && source %s && exec ros2 launch %s %s' % (
                shlex.quote(ros_setup), shlex.quote(setup),
                'turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.launch.py')
            actions.append(LogInfo(msg='[precise_nav] 自起 wheeltec 底盘（原 alias 5），'
                                       '%.1fs 后再拉 nav2' % delay))
            actions.append(ExecuteProcess(
                cmd=['bash', '-c', cmd], name='wheeltec_chassis', output='screen'))

    # ---------- nav2 + dock_navigator ----------
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', '2d_david_nav2_bringup.launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'cmd_shaping': 'false',
        }.items(),
    )

    core = [bringup] + ([] if websocket else manager_actions)
    if delay > 0.0:
        actions.append(TimerAction(period=delay, actions=core))
    else:
        actions.extend(core)

    # ---------- server 模块（alias 4 的 navigation_executor，2026-09-15 并入） ----------
    # 它自己等 nav2 就绪才 exec executor，不需要跟着底盘延时。
    if websocket:
        actions.extend(manager_actions)
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('david_nav')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params_2d_precise.yaml'),
            description='高精度参数快照。与生产 nav2_params_2d.yaml 的差异见该文件头。',
        ),
        DeclareLaunchArgument(
            'dock_goal_topic',
            default_value='/dock_goal',
            description='dock_navigator 的输入话题。刻意不用 /goal_pose —— '
                        'bt_navigator 自己订阅 /goal_pose，同名会双链路抢同一目标。',
        ),
        DeclareLaunchArgument(
            'approach_dist', default_value='0.5',
            description='预备点沿目标航向后退的距离(m)。',
        ),
        DeclareLaunchArgument(
            'xy_ok', default_value='0.10',
            description='入位判定的位置误差阈值(m)，与 general_goal_checker 一致。',
        ),
        DeclareLaunchArgument(
            'yaw_ok', default_value='0.4363',
            description='入位判定的朝向误差阈值(rad)，必须与 general_goal_checker 一致。'
                        '★2026-09-10 方案C: 0.087(5°) -> 0.4363(25°)。5° 受底盘死区限制'
                        '物理上达不到（最小可执行原地转角约 13.6°）；实测 |eyaw| 均值 11.6°、'
                        '最大 22.0°。底盘死区前馈落地后再收紧回 5°。',
        ),
        DeclareLaunchArgument(
            'max_yaw_retries', default_value='0',
            description='入位后朝向超差时的纯旋转补偿次数上限。'
                        '★2026-09-10 由 2 改为 0：实测 Spin 补偿是净负收益。'
                        'david_move 把任何纯原地旋转指令抬到 min_stationary_angular_z'
                        '=0.55rad/s（死区补偿），底盘停下来惰走约 13.6°，而 yaw 容差只有 5°，'
                        '于是每次补偿都过冲反向：命令 5.9°实际转 13.6°、命令 7.7°实际转 32.9°。'
                        'dock_bench 三版对比：不补偿 |eyaw| 均值 10.9°，补偿两次反而 16.3°。'
                        '等底盘死区前馈（见 project_david_motor_control_improvements P0）'
                        '落地、最小可控转角降到 5° 以内之后，再把它改回 2。',
        ),
        DeclareLaunchArgument(
            'start_chassis', default_value='true',
            description='是否自起 wheeltec 底盘（原 alias 5）。已检测到底盘在跑时'
                        '无论本开关都会跳过。false = 完全不管底盘。',
        ),
        DeclareLaunchArgument(
            'chassis_startup_delay', default_value='3.0',
            description='自起底盘后等多少秒再拉 nav2（对齐 headless 的 '
                        'CHASSIS_STARTUP_DELAY_SEC）。跳过自起时不生效。',
        ),
        DeclareLaunchArgument(
            'start_executor', default_value='true',
            description='是否起 navigation_executor（alias 4 的 server 模块：连 agent server、'
                        '状态上报、POI 接收）。false = 旧行为，纯本地导航。',
        ),
        DeclareLaunchArgument(
            'server_uri', default_value='',
            description='agent server 地址；留空 = 读 config/navigation_headless.yaml 的 '
                        'executor.server_uri（与 alias 4 相同）。',
        ),
        DeclareLaunchArgument(
            'pose_file', default_value='~/semantic_map/named_poses.json',
            description='executor 启动时导入 POI 库的点位文件（与 alias 4 相同）。',
        ),
        DeclareLaunchArgument(
            'dock_poi_ids', default_value='POI_017',
            description='逗号分隔。这些 POI 的 navigate_to 走 dock_navigator 两段式精准入位'
                        '（dock_to_pose 动作），不做目标扩圈、失败不静默重发；空 = 全部直发 nav2。',
        ),
        DeclareLaunchArgument(
            'nav_ready_timeout', default_value='120',
            description='等 nav2 就绪（/navigate_to_pose 可用）的最长秒数，超时不起 executor'
                        '（对齐 headless 的 nav2_wait_timeout_sec）。',
        ),
        OpaqueFunction(function=_setup),
    ])
