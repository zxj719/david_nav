"""dead_reckoning_bringup — 仅起 dead_reckoning_executor 节点。

start_waypoint_nav_headless.sh 当前默认走这套:不起 nav2 planner / waypoint
follower / position_controller / cmd_vel_mux / cmd_vel_separator,server 发
NavigateToPose goal 进来后由 dead_reckoning_executor 跑固定开环序列
(后退 1m → 转 90°,前进段默认关掉,只看 /odom)。

下游依赖:wheeltec 底盘已在跑(收 /cmd_vel + 发 /odom)。

参数走 node 内默认(见 david_nav/dead_reckoning_executor.py 顶部说明);
要覆盖时 `ros2 launch` 后 `--ros-args -p reverse_distance:=...` 透传也行。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="david_nav",
            executable="dead_reckoning_executor",
            name="dead_reckoning_executor",
            output="screen",
        ),
    ])
