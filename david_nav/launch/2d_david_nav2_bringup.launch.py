#!/usr/bin/env python3
"""2D Nav2 bringup for Unitree David with external Lightning SLAM topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_david_nav = get_package_share_directory('david_nav')
    urdf_file = os.path.join(pkg_david_nav, 'urdf', 'david.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_robot_state = LaunchConfiguration('publish_robot_state')
    params_file = LaunchConfiguration('params_file')
    enable_exploration = LaunchConfiguration('enable_exploration')
    explorer_params_file = LaunchConfiguration('explorer_params_file')
    collision_monitor_params_file = LaunchConfiguration('collision_monitor_params_file')
    enable_scan_bridge = LaunchConfiguration('enable_scan_bridge')
    enable_collision_monitor = LaunchConfiguration('enable_collision_monitor')
    enable_map_warmup_spin = LaunchConfiguration('enable_map_warmup_spin')
    map_warmup_min_live_area_m2 = LaunchConfiguration('map_warmup_min_live_area_m2')
    map_warmup_angular_speed = LaunchConfiguration('map_warmup_angular_speed')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml')
    log_level = LaunchConfiguration('log_level')
    map_topic = LaunchConfiguration('map_topic')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    enable_map_server = LaunchConfiguration('enable_map_server')
    # 2026-07-20: STM32 固件重调位置环/速度环后，软件侧速度整形可整体停用
    # （cmd_shaping:=false）：velocity_smoother 转 OPEN_LOOP 直通、david_move
    # 死区抬升归零（只剩限幅+看门狗）。默认 on = 原行为。
    cmd_shaping = LaunchConfiguration('cmd_shaping')
    scan_observation_active = enable_scan_bridge
    configured_nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_bt_xml_filename': nav_to_pose_bt_xml,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
            'local_costmap.local_costmap.ros__parameters.scan_priority_layer.require_live_scan':
                scan_observation_active,
            # 地图来源可切：本 launch 内的 map_server(/map) 或 slam_gt(/map)。
            # 2026-08-28 起不再默认订阅 lightning 的 /lightning/grid_map。
            'global_costmap.global_costmap.ros__parameters.static_layer.map_topic':
                map_topic,
            'local_costmap.local_costmap.ros__parameters.static_layer.map_topic':
                map_topic,
        },
        convert_types=True,
    )
    configured_nav2_params_file = ParameterFile(
        configured_nav2_params,
        allow_substs=True,
    )

    _core_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    # map_server 必须排在最前：lifecycle_manager 按列表顺序 configure/activate，
    # 让地图先就位再激活 costmap，避免 global_costmap 在 Activating 里空等。
    lifecycle_nodes_with_map = ['map_server'] + _core_lifecycle_nodes
    lifecycle_nodes = _core_lifecycle_nodes
    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings,
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings,
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings,
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_nav2_params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
        condition=IfCondition(cmd_shaping),
    )

    # cmd_shaping:=false 变体：同名节点（lifecycle manager 无感知），参数覆盖成
    # 直通 —— OPEN_LOOP（不再闭环轮速计）、加速度限幅放开、20Hz 对齐 DWB
    # （原 10Hz 平滑会额外引入 ~100ms 时延）。限速 max_velocity 仍由 yaml 生效。
    velocity_smoother_passthrough_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_nav2_params_file, {
            'feedback': 'OPEN_LOOP',
            'smoothing_frequency': 20.0,
            'max_accel': [99.0, 0.0, 99.0],
            'max_decel': [-99.0, 0.0, -99.0],
        }],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
        condition=UnlessCondition(cmd_shaping),
    )

    # 2026-08-28: 导航侧不再订阅 lightning 的 /lightning/grid_map，改由
    # nav2_map_server 从磁盘一次性读入静态地图并 latch 到 /map。
    # 原因：lightning 以 10Hz x 770KB 重发该图（内容与 map.pgm 逐字节相同：
    # 948*813=770724 = OccupancyGrid.data 长度 = map.pgm 去掉 PGM 头的字节数），
    # 是 /tf 投递被挤住的主因之一。详见 docs 的 TF 投递延迟分析。
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': use_sim_time,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=nav2_remappings,
        condition=IfCondition(enable_map_server),
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes_with_map,
        }],
        condition=IfCondition(enable_map_server),
    )

    # enable_map_server:=false 变体（bench / slam_gt 环境用）：同名节点，
    # node_names 里不含 map_server（地图由 slam_gt 自己发 /map）。
    nav2_lifecycle_manager_no_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes,
        }],
        condition=UnlessCondition(enable_map_server),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', urdf_file]),
                value_type=str,
            ),
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(publish_robot_state),
    )

    # 2026-06-18: 删除 base->base_link 静态别名。SLAM 改进后直接发
    # map->odom->base->base_link(base->base_link 含 Rz(-90deg)+z=-0.275,
    # base_link = 正向本体帧)。本节点曾发 base->base_link 恒等,会和 SLAM
    # 抢同一条边(恒等 vs -90deg)→ base_link 朝向闪烁/裂树。base_link 的父
    # 现在唯一由 SLAM 提供。URDF(RSP)只发 base_link 的子帧。

    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/lightning/loc/aligned_scan'),
            ('scan', '/scan'),
        ],
        parameters=[{
            # target_frame = base_footprint。2026-06-18 起 base_link 已被 SLAM
            # 转成正向帧(与 base_footprint 同朝向),但仍用 base_footprint:它在
            # 地面 z=0,配合下面 min_height=0.10 滤地面;base_link 高于地面。
            # ±π/2 角度过滤在 base_footprint 系下精确对应机身正前方半圆。
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.1,
            # 2026-05-18: min_height 抬到 0.10m。base_footprint 在地面 (z=0)，
            # 改完后只看 ≥10cm 高的点 ⇒ 滤掉地面反射、机身底部自影、低洼小物。
            # 副作用：高度 < 10cm 的障碍（线缆、小台阶、矮挡板）会被忽略。
            'min_height': 0.10,
            'max_height': 1.0,
            # 前向 180° (-π/2 .. +π/2)，在 base_footprint 系下精确对应机身正前方半圆。
            'angle_min': -1.5708,
            'angle_max': 1.5708,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 12.0,
            'use_inf': True,
        }],
        condition=IfCondition(enable_scan_bridge),
    )

    map_warmup_spin_node = Node(
        package='david_nav',
        executable='map_warmup_spin',
        name='map_warmup_spin',
        output='screen',
        parameters=[{
            'map_topic': '/lightning/grid_map',
            'live_map_ready_topic': '/map_live_ready',
            'cmd_vel_nav_topic': '/cmd_vel_nav',
            'resume_topic': 'explore/resume',
            'spin_angular_speed': map_warmup_angular_speed,
            'min_live_map_area_m2': map_warmup_min_live_area_m2,
        }],
        condition=IfCondition(enable_map_warmup_spin),
    )

    frontier_explorer = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='david_nav',
                executable='frontier_explorer_cpp',
                name='frontier_explorer',
                output='screen',
                parameters=[explorer_params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(enable_exploration),
            ),
        ],
    )

    # 2026-05-20: delay RViz start by `rviz_delay_sec` so nav2 lifecycle
    # has time to fully Activate before RViz starts subscribing. Without
    # this, RViz subscribes to /global_costmap/costmap / /lightning/grid_map
    # before they're publishing (lifecycle still Configuring) — the
    # subscription connects but no data ever arrives in the early window,
    # and depending on QoS it can stay empty until RViz is restarted.
    # Default 12 s is conservative: nav2 lifecycle typically completes
    # in 6-10 s on this hardware.
    rviz_delay_sec = LaunchConfiguration('rviz_delay_sec')
    rviz_inner_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    rviz_node = TimerAction(
        period=rviz_delay_sec,
        actions=[rviz_inner_node],
        condition=IfCondition(use_rviz),
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_monitor_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_collision_monitor),
    )

    collision_monitor_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
        condition=IfCondition(enable_collision_monitor),
    )

    # Plan D (2026-05-19): david_move publishes to /cmd_vel_raw, then
    # cmd_vel_separator hard-switches between pure-linear and pure-angular
    # before forwarding to /cmd_vel (consumed by wheeltec_robot_node).
    # See docs/report/2026-05-18-nav2-left-drift-david-move-deadzone.md for
    # the chain before this change.
    david_move_node = Node(
        package='david_cmd',
        executable='david_move',
        name='david_move',
        output='screen',
        parameters=[{
            # 2026-06-06 (branch feature/disable-cmd-vel-separator):
            # cmd_vel_separator is disabled on this branch, so david_move must
            # publish straight to /cmd_vel (consumed by wheeltec_robot_node)
            # instead of /cmd_vel_raw. This is the pre-Plan-D chain:
            #   velocity_smoother -> david_move -> /cmd_vel -> wheeltec.
            # The original Plan D wiring ('/cmd_vel_raw' + separator) is kept
            # on feature/david_main.
            'output_cmd_vel_topic': '/cmd_vel',
            # 2026-05-20: when collision_monitor is disabled via launch arg,
            # /cmd_vel_safe is never published. Point david_move directly
            # at /cmd_vel_smoothed (velocity_smoother's output) so the chain
            # doesn't break. PythonExpression returns the topic string.
            'input_cmd_vel_topic': PythonExpression([
                "'cmd_vel_safe' if '",
                enable_collision_monitor,
                "' == 'true' else 'cmd_vel_smoothed'",
            ]),
            # 2026-05-20: 与 DWB / smoother 同步从 0.70 降到 0.50（实跑过冲）。
            # 2026-06-08: 再降到 0.20。换电机/轮 + 修正轮速计(原值小 3×)后，实测
            # 线速度 0.2 已让 70kg 上身明显晃动，0.2 = 物理上限，与 DWB/smoother 同步。
            'max_linear_x': 0.20,
            # 2026-06-08 死区实验标定（裸破启 worst-case 脚轮, PI=Kp300/Ki800）：
            # 线 ≈0.04 m/s、角 ≈0.15 rad/s（破启=抽动阈值，非可持续运动）。
            # 线速度 lift 默认 0.20 → 0.15：实测命令 0.12 已能起步，0.15≈4×破启，
            # 给慢速接近留细度同时远离破启防 stutter。
            # ⚠ 现场需确认最慢档不抖再固化；不行就回 0.20。
            # 2026-07-20: cmd_shaping:=false 时抬升归零（STM32 新位置环/速度环
            # 自行处理死区，软件抬升会造成双重补偿）。david_move 保留 = 限幅
            # + 2s 超时看门狗。
            'min_linear_x': ParameterValue(PythonExpression([
                "0.15 if '", cmd_shaping, "' == 'true' else 0.0",
            ]), value_type=float),
            # 原地旋转破静摩擦 kick floor：实测角破启 0.15，但近破启纯抖(stick-slip)；
            # PI 阶跃实验显示 0.3 rad/s 仍 stuck-then-lurch，故保持 0.35 不下调。
            # 2026-07-24: 不再随 cmd_shaping 归零。实测大角度原地旋转在新固件下
            # 仍会静止卡死需人推 —— 纯旋转（无线速度）保留抬升；滚动态
            # （v、w 同时非零）david_move 本身走 min_angular_z=0 不抬升，
            # 正是"原地用旧参数、行进用新参数"的分工。
            # 2026-07-27: 0.35 → 0.55。复盘 115741 run：原地 ±0.35 卡死 60s
            # （3×progress fail）。0.35 是老链路的值——老链路能转靠 separator
            # binary 把指令踢到端点；无 separator 后真实破启值 = 2026-06 实测
            # 的 0.55。RotationShim 因 lightning TF 滞后 ~0.3s 常态性回退 DWB，
            # 回退路径必须自足。max_angular_z 同步 0.5→0.6（否则 clamp 先于
            # lift，0.55 的 floor 会被钳回 0.5）。
            # 2026-08-28: 由写死改为 launch 参数（本文件被 alias 4 生产栈与 alias 7
            # bench 共用，写死会让 bench 的调参外溢到生产）。默认仍是 0.55 =
            # 生产行为不变；controller_bench 用 min_stationary_angular_z:=0.35
            # 压末端旋转过冲（0.55 硬停惯性带出 ~15°）。
            'min_stationary_angular_z': ParameterValue(
                LaunchConfiguration('min_stationary_angular_z'), value_type=float),
            'max_angular_z': 0.6,
        }],
    )

    cmd_vel_separator_node = Node(
        package='david_nav',
        executable='cmd_vel_separator',
        name='cmd_vel_separator',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel_raw',
            'output_topic': '/cmd_vel',
            'odom_topic': '/odom',
            # Normalisation references. Mirror velocity_smoother /
            # DWB max speeds so the mutex decision uses the same scale.
            'v_max_ref': 0.50,
            'w_max_ref': 0.50,
            'hysteresis_margin': 0.10,
            'min_norm_threshold': 0.05,
            'timeout_sec': 0.5,
            'publish_hz': 20.0,
            # ---- Plan D: active brake on input-dead transitions ----
            # When DWB stops asking for motion but odom still shows
            # significant velocity, output a proportional reverse cmd
            # to dump the PI integrator wind-up. This is what kills the
            # 0.3-0.7 m terminal overshoot from binary 0.7 output.
            # max_brake_w bumped to 0.50 — unit test showed 0.30 was not
            # enough to overcome the angular PI wind-up.
            'brake_enabled': True,
            'k_brake': 1.0,
            'max_brake_v': 0.30,
            'max_brake_w': 0.50,
            'brake_release_v': 0.05,
            'brake_release_w': 0.10,
            'brake_timeout': 2.0,
            # 2026-05-20: angle brake — 切角阈值 45° → 30° (caster overshoot
            # 实测在 nav 跑动时比 45° 余量小)
            'angle_brake_threshold_deg': 30.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz. Requires a valid graphical display.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_david_nav, 'rviz', 'nav2_go2_view.rviz'),
            description='RViz config file',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='true',
            description='Launch robot_state_publisher for the RViz RobotModel display.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_david_nav, 'config', 'nav2_params_2d.yaml'),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=os.path.join(
                pkg_david_nav, 'behavior_trees',
                'navigate_to_pose_w_david_embedded_recovery.xml'),
            description='Behavior tree XML used by bt_navigator for NavigateToPose.',
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_xml',
            default_value=os.path.join(
                pkg_david_nav, 'behavior_trees',
                'navigate_through_poses_w_david_embedded_recovery.xml'),
            description='Behavior tree XML used by bt_navigator for NavigateThroughPoses.',
        ),
        DeclareLaunchArgument(
            'enable_exploration',
            default_value='false',
            description='Launch frontier explorer after Nav2 bringup',
        ),
        DeclareLaunchArgument(
            'enable_collision_monitor',
            default_value='false',
            description=(
                'Launch nav2 collision_monitor + its lifecycle manager. '
                'When false, david_move reads /cmd_vel_smoothed directly and '
                'no obstacle-gating happens. Set false to unblock in-place '
                'rotation while caster-related issues are being debugged.'
            ),
        ),
        DeclareLaunchArgument(
            'min_stationary_angular_z',
            default_value='0.55',
            description=(
                'david_move 纯原地转(linear.x==0)时的角速度地板 rad/s。'
                '0.55 = 实测破启值(2026-07-27 复盘: 0.35 曾原地卡死 60s)。'
                'controller_bench 传 0.35 以压末端旋转过冲。'
            ),
        ),
        DeclareLaunchArgument(
            'cmd_shaping',
            default_value='true',
            description=(
                'Software cmd_vel shaping between DWB and the chassis. '
                'true (default) = velocity_smoother CLOSED_LOOP + david_move '
                'deadzone lift (min_linear_x 0.15 / min_stationary_angular_z '
                '0.35). false = smoother OPEN_LOOP pass-through @20Hz + lifts '
                'zeroed - use after the 2026-07-20 STM32 position/velocity '
                'loop retune so the firmware handles deadzones itself.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_delay_sec',
            default_value='12.0',
            description=(
                'Seconds to wait after launch before starting RViz. Lets '
                'nav2 lifecycle fully Activate first so all costmap / plan '
                'topics are publishing by the time RViz subscribes. '
                'Set to 0 for immediate start (old behaviour).'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=os.path.join(
                pkg_david_nav, 'config', 'frontier_explorer_params.yaml'),
            description='Frontier explorer parameters file',
        ),
        DeclareLaunchArgument(
            'collision_monitor_params_file',
            default_value=os.path.join(
                pkg_david_nav, 'config', 'collision_monitor_params.yaml'),
            description='Collision monitor parameters file',
        ),
        DeclareLaunchArgument(
            'enable_scan_bridge',
            default_value='false',
            description='Launch pointcloud_to_laserscan to publish /scan from lidar.',
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=os.path.join(
                pkg_david_nav, 'config', 'maps', 'daolan_map_nav.yaml'),
            description='nav2_map_server 读取的静态地图 yaml。默认那份的 image 直接'
                        '指向 lightning 地图目录的 map.pgm（路径由 '
                        'start_loc_david001.sh 的 ROOT + David001_c16.yaml 的 '
                        'map_path 解析得到），只是把 mode 从 scale 改成 trinary —— '
                        'lightning 的 map.yaml 用 mode:scale，会让 51.82%% 的未知'
                        '栅格(px=128)被 static_layer 判成致命障碍。详见该文件注释。',
        ),
        DeclareLaunchArgument(
            'enable_map_server',
            default_value='true',
            description='起 nav2_map_server 从磁盘读静态地图并 latch 发布 /map。'
                        'slam_gt / bench 环境自己发 /map，必须传 false 避免抢话题。',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='costmap static_layer 订阅的地图话题。默认 /map（由本 launch '
                        '内的 map_server 从磁盘 latch 发布）。2026-08-28 前默认是 '
                        '/lightning/grid_map，已停用。',
        ),
        DeclareLaunchArgument(
            'enable_map_warmup_spin',
            default_value='false',
            description='Rotate in place until the first live SLAM map grows to a usable size.',
        ),
        DeclareLaunchArgument(
            'map_warmup_min_live_area_m2',
            default_value='12.0',
            description='Minimum live /map area before exploration starts.',
        ),
        DeclareLaunchArgument(
            'map_warmup_angular_speed',
            default_value='0.45',
            description='Angular speed used while warming up the live SLAM map.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 log level',
        ),
        robot_state_publisher_node,
        pointcloud_to_scan_node,
        controller_server_node,
        smoother_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        velocity_smoother_passthrough_node,
        map_server_node,
        nav2_lifecycle_manager,
        nav2_lifecycle_manager_no_map,
        map_warmup_spin_node,
        frontier_explorer,
        rviz_node,
        collision_monitor_node,
        collision_monitor_lifecycle_manager,
        david_move_node,
        # 2026-06-06 (branch feature/disable-cmd-vel-separator): separator
        # temporarily bypassed to test the plain nav2 chain against the
        # current nav2_params_2d.yaml. david_move now feeds /cmd_vel directly
        # (see output_cmd_vel_topic above). Re-enable by uncommenting and
        # reverting david_move's output topic to '/cmd_vel_raw'.
        # cmd_vel_separator_node,
    ])
