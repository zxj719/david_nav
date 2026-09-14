# Third-party sources

This directory holds code that is not developed in this repository. Nothing here
is modified; changes belong in the packages that consume it.

## serial

| Item | Value |
|---|---|
| What | C++ serial port library (ROS 2 port of wjwwood/serial) |
| Upstream | https://github.com/wjwwood/serial, ROS 2 fork https://github.com/iwelch82/serial-ros2 |
| Version | `package.xml` 1.2.1 |
| License | MIT (declared in `package.xml`; the copied tree contains no separate LICENSE file) |
| Copied from | `agx@david:~/scout_ws/src/serial-ros2` on 2026-09-14 (excluding `.git/`, whose metadata was corrupt: `fatal: bad object HEAD`) |
| Why vendored | `turn_on_wheeltec_robot` linked against this package through the `~/scout_ws` underlay, an implicit cross-workspace dependency (PLAN §8.1 W7). `wheeltec_driver` will depend on this copy instead |
| Build | Built as the colcon package `serial`; its upstream tests are commented out in `CMakeLists.txt` |

## wheeltec_original

| Item | Value |
|---|---|
| What | Unmodified snapshot of the Wheeltec ROS 2 chassis package `turn_on_wheeltec_robot` |
| Copied from | `agx@david:~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/src/turn_on_wheeltec_robot` on 2026-09-14 (that workspace is not under version control) |
| Why kept | Reference and rollback baseline for extracting `wheeltec_driver` (PLAN §8, PROGRESS P0.10 / P3.7) |
| Build | Not built: `COLCON_IGNORE` is present |
| Golden data | `golden/uplink_status_frames_2026-09-14.txt`: real 24-byte status frames read from the controller (PROGRESS P0.8) |
