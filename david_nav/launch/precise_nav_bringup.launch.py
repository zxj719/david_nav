#!/usr/bin/env python3
"""alias 0 专用：高精度两段式入位导航（2026-08-31）。

= 现有 2d_david_nav2_bringup.launch.py（传高精度参数快照）
+ dock_navigator（两段式对齐入位编排节点）
+ wheeltec 底盘（原 alias 5，2026-09-08 并入；已经在跑就自动跳过）

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


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('david_nav')
    actions = []

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

    dock_navigator = Node(
        package='david_nav',
        executable='dock_navigator',
        name='dock_navigator',
        output='screen',
        parameters=[{
            'goal_topic': LaunchConfiguration('dock_goal_topic'),
            'approach_dist': LaunchConfiguration('approach_dist'),
            'xy_ok': LaunchConfiguration('xy_ok'),
            'yaw_ok': LaunchConfiguration('yaw_ok'),
            'max_yaw_retries': LaunchConfiguration('max_yaw_retries'),
            'robot_base_frame': 'base_footprint',
            'global_frame': 'map',
        }],
    )

    if delay > 0.0:
        actions.append(TimerAction(period=delay, actions=[bringup, dock_navigator]))
    else:
        actions.extend([bringup, dock_navigator])
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
        OpaqueFunction(function=_setup),
    ])
