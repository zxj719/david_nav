# nav_executor

Navigation WebSocket/ROS ingress and the single process composition root for
`nav_tasks`. The installed `nav_task_manager` entry owns ordinary POI tasks and,
when a complete `--dock-config-file` is supplied, Dock requests. `--no-websocket`
selects ROS-only ingress.

```text
WS / ROS ingress → nav_tasks.api → TaskManager → POI/Dock skills → Nav2 backend
                                  ↘ task events → outbox → WS
```

Only `nav_tasks.api` may be imported from the task package. Protocol modules are
pure and importing them starts no ROS context, thread or network connection.
Configuration precedence and provenance are described in
[configuration](docs/configuration.md); ownership and shutdown order are in the
[outer runtime](docs/outer-runtime.md).

The runtime owns a private ROS context and seals ingress before draining tasks,
workers and callbacks. A cleanup failure remains explicit; software cleanup is
not proof that the physical robot stopped.

WebSockets 16.1 is selected in Python metadata. No dependency is installed by
the build itself. The protocol contract is documented in
[WS v1](../docs/protocol/nav_ws_v1.md) and its AsyncAPI schema.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
colcon build --symlink-install --packages-select nav_executor
source install/setup.bash
set -o allexport
source src/david_system_env/config/ros_env.conf
set +o allexport
export ROS_DOMAIN_ID=99
colcon test --packages-select nav_executor
colcon test-result --test-result-base build/nav_executor --verbose
```
