# G1 Nav Launch Layering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split the current `g1_nav` launch stack into clean navigation, exploration, and standalone Realsense obstacle layers, while renaming the default RViz config from `nav2_go2_view.rviz` to `nav2_g1_view.rviz`.

**Architecture:** Keep `launch/2d_g1_nav2_bringup.launch.py` focused on generic `slam + nav2`, move frontier explorer and map warmup ownership into `launch/g1_auto_explore.launch.py`, and turn `launch/realsense_depth_to_scan.launch.py` into a standalone composition launch that starts both `realsense2_camera/launch/rs_launch.py` and `depthimage_to_laserscan_node`. Lock the launch ownership boundaries down with pytest before editing the launch files, then verify the final structure with the existing `realsense_nav2_integration_test` target.

**Tech Stack:** ROS 2 Humble launch API, Python launch files, RViz, `pytest`, `ament_cmake_pytest`, `colcon`

**Execution Note:** The current `/home/unitree/ros2_ws/src/g1_nav` worktree already has local edits in files this feature will touch. Execute this plan in a fresh worktree from current `master`, or carefully review each target file diff before staging so unrelated user changes are preserved.

---

## File Structure

### Modified Files

- `launch/2d_g1_nav2_bringup.launch.py`
  Remove frontier exploration, map warmup, and Realsense startup responsibilities so this file is only the reusable navigation bringup.
- `launch/g1_auto_explore.launch.py`
  Keep the include of the base bringup, but directly own the frontier explorer timer, map warmup node, and the renamed RViz config default.
- `launch/realsense_depth_to_scan.launch.py`
  Include `realsense2_camera/launch/rs_launch.py`, expose the curated RealSense launch arguments, and keep the depth-to-scan bridge node local to this file.
- `rviz/nav2_go2_view.rviz`
  Rename this file to `rviz/nav2_g1_view.rviz` without changing its contents.
- `test/test_realsense_nav2_integration.py`
  Update the launch-structure assertions to match the new layering and add coverage for the standalone Realsense camera include.

## Task 1: Write Failing Tests For Bringup And Auto-Explore Ownership

**Files:**
- Modify: `test/test_realsense_nav2_integration.py`
- Test: `test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Replace the old bringup/auto-explore ownership assertions with red tests for the new layering**

```python
def _iter_entities(entities):
    for entity in entities:
        yield entity
        if type(entity).__name__ == 'TimerAction':
            yield from _iter_entities(entity._TimerAction__actions)


def _all_entities(launch_description):
    return list(_iter_entities(launch_description.entities))


def test_bringup_uses_g1_rviz_config_by_default():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_rviz_defaults',
    )
    launch_description = module.generate_launch_description()
    argument = _declared_launch_arguments(launch_description)['rviz_config']

    assert _text_value(argument.default_value).endswith(
        '/g1_nav/rviz/nav2_g1_view.rviz'
    )


def test_bringup_does_not_declare_exploration_or_warmup_arguments():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_without_explore_args',
    )
    launch_description = module.generate_launch_description()
    argument_names = set(_declared_launch_arguments(launch_description))

    removed_arguments = {
        'enable_exploration',
        'explorer_params_file',
        'enable_realsense_scan_bridge',
        'enable_map_warmup_spin',
        'map_warmup_min_live_area_m2',
        'map_warmup_angular_speed',
    }

    assert removed_arguments.isdisjoint(argument_names)


def test_bringup_does_not_include_realsense_or_exploration_entities():
    module = _load_launch_module(
        'launch/2d_g1_nav2_bringup.launch.py',
        'g1_nav_bringup_launch_structure',
    )
    launch_description = module.generate_launch_description()
    entities = _all_entities(launch_description)

    realsense_includes = [
        entity
        for entity in entities
        if type(entity).__name__ == 'IncludeLaunchDescription'
        and _include_source_location(entity).endswith(
            '/g1_nav/launch/realsense_depth_to_scan.launch.py'
        )
    ]
    g1_nav_nodes = [
        entity
        for entity in entities
        if type(entity).__name__ == 'Node'
        and entity._Node__package == 'g1_nav'
    ]

    assert realsense_includes == []
    assert g1_nav_nodes == []


def test_auto_explore_uses_g1_rviz_config_by_default():
    module = _load_launch_module(
        'launch/g1_auto_explore.launch.py',
        'g1_nav_auto_explore_launch_rviz_defaults',
    )
    launch_description = module.generate_launch_description()
    argument = _declared_launch_arguments(launch_description)['rviz_config']

    assert _text_value(argument.default_value).endswith(
        '/g1_nav/rviz/nav2_g1_view.rviz'
    )


def test_auto_explore_keeps_explorer_and_warmup_out_of_bringup_arguments():
    module = _load_launch_module(
        'launch/g1_auto_explore.launch.py',
        'g1_nav_auto_explore_launch_include_args',
    )
    launch_description = module.generate_launch_description()
    include_action = next(
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'IncludeLaunchDescription'
        and _include_source_location(entity).endswith(
            '/g1_nav/launch/2d_g1_nav2_bringup.launch.py'
        )
    )
    launch_arguments = dict(include_action.launch_arguments)

    removed_arguments = {
        'explorer_params_file',
        'enable_realsense_scan_bridge',
        'enable_map_warmup_spin',
        'map_warmup_min_live_area_m2',
        'map_warmup_angular_speed',
    }

    for name in removed_arguments:
        assert name not in launch_arguments


def test_auto_explore_creates_frontier_and_warmup_nodes_locally():
    module = _load_launch_module(
        'launch/g1_auto_explore.launch.py',
        'g1_nav_auto_explore_launch_local_nodes',
    )
    launch_description = module.generate_launch_description()
    entities = _all_entities(launch_description)
    g1_nav_nodes = [
        entity
        for entity in entities
        if type(entity).__name__ == 'Node'
        and entity._Node__package == 'g1_nav'
    ]

    assert {
        (node._Node__node_executable, node._Node__node_name)
        for node in g1_nav_nodes
    } == {
        ('frontier_explorer_cpp', 'frontier_explorer'),
        ('map_warmup_spin', 'map_warmup_spin'),
    }
```

- [ ] **Step 2: Run the focused pytest file and confirm the new tests fail for the old launch structure**

Run: `python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py`

Expected: FAIL. The failures should show that:
- the default RViz filename is still `nav2_go2_view.rviz`
- `2d_g1_nav2_bringup.launch.py` still declares exploration / warmup / Realsense arguments
- `g1_auto_explore.launch.py` still forwards exploration and Realsense ownership into the base bringup

- [ ] **Step 3: Commit the red ownership tests**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add test/test_realsense_nav2_integration.py
git commit -m "test: cover launch ownership boundaries"
```

## Task 2: Refactor Bringup And Auto-Explore, Then Rename The RViz Config

**Files:**
- Modify: `launch/2d_g1_nav2_bringup.launch.py`
- Modify: `launch/g1_auto_explore.launch.py`
- Move: `rviz/nav2_go2_view.rviz` -> `rviz/nav2_g1_view.rviz`
- Test: `test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Strip exploration, warmup, and Realsense startup out of `launch/2d_g1_nav2_bringup.launch.py`**

```python
#!/usr/bin/env python3
"""2D Nav2 bringup for Unitree G1 with external Lightning SLAM topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_g1_nav = get_package_share_directory('g1_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_g1_sim = get_package_share_directory('g1_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_robot_state = LaunchConfiguration('publish_robot_state')
    params_file = LaunchConfiguration('params_file')
    enable_scan_bridge = LaunchConfiguration('enable_scan_bridge')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml')
    configured_nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_bt_xml_filename': nav_to_pose_bt_xml,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
        },
        convert_types=True,
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_nav2_params,
            'autostart': 'true',
        }.items(),
    )

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g1_sim, 'launch', 'g1_urdf2tf.launch.py')
        ),
        condition=IfCondition(publish_robot_state),
    )

    base_alias_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base_link_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'base', 'base_link'],
        output='screen',
    )

    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/utlidar/cloud_livox_mid360'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base',
            'transform_tolerance': 0.1,
            'min_height': -0.3,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 12.0,
            'use_inf': True,
        }],
        condition=IfCondition(enable_scan_bridge),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    g1_move_node = Node(
        package='g1_cmd',
        executable='g1_move',
        name='g1_move',
        output='screen',
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
            default_value=os.path.join(pkg_g1_nav, 'rviz', 'nav2_g1_view.rviz'),
            description='RViz config file',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='true',
            description='Launch robot_state_publisher for the RViz RobotModel display.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_g1_nav, 'config', 'nav2_params_2d.yaml'),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=os.path.join(
                pkg_g1_nav, 'behavior_trees',
                'navigate_to_pose_w_g1_embedded_recovery.xml'),
            description='Behavior tree XML used by bt_navigator for NavigateToPose.',
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_xml',
            default_value=os.path.join(
                pkg_g1_nav, 'behavior_trees',
                'navigate_through_poses_w_g1_embedded_recovery.xml'),
            description='Behavior tree XML used by bt_navigator for NavigateThroughPoses.',
        ),
        DeclareLaunchArgument(
            'enable_scan_bridge',
            default_value='false',
            description='Launch pointcloud_to_laserscan to publish /scan from lidar.',
        ),
        robot_state_launch,
        base_alias_tf_node,
        pointcloud_to_scan_node,
        nav2_launch,
        rviz_node,
        g1_move_node,
    ])
```

- [ ] **Step 2: Move frontier explorer and warmup ownership into `launch/g1_auto_explore.launch.py`**

```python
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_g1_nav = get_package_share_directory('g1_nav')
    explore_nav_to_pose_bt_xml = os.path.join(
        pkg_g1_nav, 'behavior_trees', 'navigate_to_pose_w_g1_explore_fail_fast.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_robot_state = LaunchConfiguration('publish_robot_state')
    params_file = LaunchConfiguration('params_file')
    explorer_params_file = LaunchConfiguration('explorer_params_file')
    enable_scan_bridge = LaunchConfiguration('enable_scan_bridge')
    enable_map_warmup_spin = LaunchConfiguration('enable_map_warmup_spin')
    map_warmup_min_live_area_m2 = LaunchConfiguration('map_warmup_min_live_area_m2')
    map_warmup_angular_speed = LaunchConfiguration('map_warmup_angular_speed')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_g1_nav, 'launch', '2d_g1_nav2_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'rviz_config': rviz_config,
            'publish_robot_state': publish_robot_state,
            'params_file': params_file,
            'nav_to_pose_bt_xml': explore_nav_to_pose_bt_xml,
            'enable_scan_bridge': enable_scan_bridge,
        }.items(),
    )

    map_warmup_spin_node = Node(
        package='g1_nav',
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
                package='g1_nav',
                executable='frontier_explorer_cpp',
                name='frontier_explorer',
                output='screen',
                parameters=[explorer_params_file, {'use_sim_time': use_sim_time}],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz together with Nav2 and frontier exploration.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_g1_nav, 'rviz', 'nav2_g1_view.rviz'),
            description='RViz config file',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='true',
            description='Publish robot_description so RViz can show the robot model.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_g1_nav, 'config', 'nav2_params_2d.yaml'),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=os.path.join(
                pkg_g1_nav, 'config', 'frontier_explorer_params.yaml'),
            description='Frontier explorer parameters file',
        ),
        DeclareLaunchArgument(
            'enable_scan_bridge',
            default_value='false',
            description='Launch pointcloud_to_laserscan to publish /scan from lidar.',
        ),
        DeclareLaunchArgument(
            'enable_map_warmup_spin',
            default_value='false',
            description='Rotate in place until the first live SLAM map grows to a usable size.',
        ),
        DeclareLaunchArgument(
            'map_warmup_min_live_area_m2',
            default_value='12.0',
            description='Minimum live /map area before exploration resumes.',
        ),
        DeclareLaunchArgument(
            'map_warmup_angular_speed',
            default_value='0.45',
            description='Angular speed used while warming up the live SLAM map.',
        ),
        bringup_launch,
        map_warmup_spin_node,
        frontier_explorer,
    ])
```

- [ ] **Step 3: Rename the RViz config file without changing its contents**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
mv rviz/nav2_go2_view.rviz rviz/nav2_g1_view.rviz
```

- [ ] **Step 4: Re-run the focused pytest file and confirm the bringup/auto-explore tests go green**

Run: `python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py`

Expected: PASS for the bringup and auto-explore ownership tests added in Task 1. The Realsense launch tests that already existed should still pass because `launch/realsense_depth_to_scan.launch.py` has not changed yet.

- [ ] **Step 5: Commit the launch layering refactor and RViz rename**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add launch/2d_g1_nav2_bringup.launch.py launch/g1_auto_explore.launch.py rviz/nav2_g1_view.rviz rviz/nav2_go2_view.rviz
git commit -m "refactor: split nav bringup and auto explore"
```

## Task 3: Write Failing Tests For Standalone Realsense Camera Startup

**Files:**
- Modify: `test/test_realsense_nav2_integration.py`
- Test: `test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Add red tests that require `launch/realsense_depth_to_scan.launch.py` to include `rs_launch.py`**

```python
def test_realsense_depth_to_scan_launch_declares_camera_arguments():
    module = _load_launch_module(
        'launch/realsense_depth_to_scan.launch.py',
        'g1_nav_realsense_depth_to_scan_launch_camera_args',
    )
    launch_description = module.generate_launch_description()
    arguments = _declared_launch_arguments(launch_description)

    expected_defaults = {
        'camera_name': 'camera',
        'camera_namespace': 'camera',
        'serial_no': "''",
        'config_file': "''",
        'log_level': 'info',
        'output': 'screen',
        'enable_color': 'true',
        'enable_depth': 'true',
        'align_depth.enable': 'false',
        'pointcloud.enable': 'false',
        'publish_tf': 'true',
        'tf_publish_rate': '0.0',
    }

    assert set(expected_defaults).issubset(arguments)

    for name, expected in expected_defaults.items():
        assert _text_value(arguments[name].default_value) == expected


def test_realsense_depth_to_scan_launch_includes_realsense_camera_launch():
    module = _load_launch_module(
        'launch/realsense_depth_to_scan.launch.py',
        'g1_nav_realsense_depth_to_scan_launch_camera_include',
    )
    launch_description = module.generate_launch_description()
    matching_includes = [
        entity
        for entity in launch_description.entities
        if type(entity).__name__ == 'IncludeLaunchDescription'
        and _include_source_location(entity).endswith(
            '/realsense2_camera/launch/rs_launch.py'
        )
    ]

    assert len(matching_includes) == 1

    include_action = matching_includes[0]
    launch_arguments = dict(include_action.launch_arguments)

    assert set(launch_arguments) == {
        'camera_name',
        'camera_namespace',
        'serial_no',
        'config_file',
        'log_level',
        'output',
        'enable_color',
        'enable_depth',
        'align_depth.enable',
        'pointcloud.enable',
        'publish_tf',
        'tf_publish_rate',
    }
    assert _text_value(launch_arguments['camera_name']) == 'camera_name'
    assert _text_value(launch_arguments['camera_namespace']) == 'camera_namespace'
    assert _text_value(launch_arguments['serial_no']) == 'serial_no'
    assert _text_value(launch_arguments['config_file']) == 'config_file'
    assert _text_value(launch_arguments['log_level']) == 'log_level'
    assert _text_value(launch_arguments['output']) == 'output'
    assert _text_value(launch_arguments['enable_color']) == 'enable_color'
    assert _text_value(launch_arguments['enable_depth']) == 'enable_depth'
    assert _text_value(launch_arguments['align_depth.enable']) == 'align_depth.enable'
    assert _text_value(launch_arguments['pointcloud.enable']) == 'pointcloud.enable'
    assert _text_value(launch_arguments['publish_tf']) == 'publish_tf'
    assert _text_value(launch_arguments['tf_publish_rate']) == 'tf_publish_rate'
```

- [ ] **Step 2: Run only the new Realsense tests and confirm they fail before implementation**

Run: `python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py -k "camera_arguments or camera_launch"`

Expected: FAIL because `launch/realsense_depth_to_scan.launch.py` does not yet declare the RealSense camera arguments and does not yet include `/realsense2_camera/launch/rs_launch.py`.

- [ ] **Step 3: Commit the red Realsense launch tests**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add test/test_realsense_nav2_integration.py
git commit -m "test: cover standalone realsense launch include"
```

## Task 4: Implement The Standalone Realsense Camera Plus Depth-To-Scan Launch

**Files:**
- Modify: `launch/realsense_depth_to_scan.launch.py`
- Test: `test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Replace `launch/realsense_depth_to_scan.launch.py` with a composition launch that includes `rs_launch.py`**

```python
#!/usr/bin/env python3
"""Standalone Realsense camera plus depth-to-LaserScan bridge for Nav2 local obstacle use."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_realsense2_camera = get_package_share_directory('realsense2_camera')

    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_name = LaunchConfiguration('camera_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    serial_no = LaunchConfiguration('serial_no')
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    output = LaunchConfiguration('output')
    enable_color = LaunchConfiguration('enable_color')
    enable_depth = LaunchConfiguration('enable_depth')
    align_depth_enable = LaunchConfiguration('align_depth.enable')
    pointcloud_enable = LaunchConfiguration('pointcloud.enable')
    publish_tf = LaunchConfiguration('publish_tf')
    tf_publish_rate = LaunchConfiguration('tf_publish_rate')
    realsense_depth_image_topic = LaunchConfiguration('realsense_depth_image_topic')
    realsense_depth_camera_info_topic = LaunchConfiguration('realsense_depth_camera_info_topic')
    realsense_scan_topic = LaunchConfiguration('realsense_scan_topic')
    realsense_scan_output_frame = LaunchConfiguration('realsense_scan_output_frame')
    realsense_scan_time = LaunchConfiguration('realsense_scan_time')
    realsense_scan_range_min = LaunchConfiguration('realsense_scan_range_min')
    realsense_scan_range_max = LaunchConfiguration('realsense_scan_range_max')
    realsense_scan_height = LaunchConfiguration('realsense_scan_height')

    realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense2_camera, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_namespace': camera_namespace,
            'serial_no': serial_no,
            'config_file': config_file,
            'log_level': log_level,
            'output': output,
            'enable_color': enable_color,
            'enable_depth': enable_depth,
            'align_depth.enable': align_depth_enable,
            'pointcloud.enable': pointcloud_enable,
            'publish_tf': publish_tf,
            'tf_publish_rate': tf_publish_rate,
        }.items(),
    )

    realsense_depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='realsense_depth_to_scan',
        output='screen',
        remappings=[
            ('depth', realsense_depth_image_topic),
            ('depth_camera_info', realsense_depth_camera_info_topic),
            ('scan', realsense_scan_topic),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': realsense_scan_output_frame,
            'scan_time': realsense_scan_time,
            'range_min': realsense_scan_range_min,
            'range_max': realsense_scan_range_max,
            'scan_height': realsense_scan_height,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='RealSense camera name passed to rs_launch.py.',
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
            description='RealSense camera namespace passed to rs_launch.py.',
        ),
        DeclareLaunchArgument(
            'serial_no',
            default_value="''",
            description='Optional RealSense serial number selector.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value="''",
            description='Optional RealSense YAML config passed to rs_launch.py.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='RealSense node log level.',
        ),
        DeclareLaunchArgument(
            'output',
            default_value='screen',
            description='RealSense node output policy.',
        ),
        DeclareLaunchArgument(
            'enable_color',
            default_value='true',
            description='Enable the RealSense color stream.',
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='true',
            description='Enable the RealSense depth stream.',
        ),
        DeclareLaunchArgument(
            'align_depth.enable',
            default_value='false',
            description='Enable RealSense depth alignment.',
        ),
        DeclareLaunchArgument(
            'pointcloud.enable',
            default_value='false',
            description='Enable the RealSense pointcloud output.',
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Allow rs_launch.py to publish camera TF.',
        ),
        DeclareLaunchArgument(
            'tf_publish_rate',
            default_value='0.0',
            description='RealSense TF publish rate.',
        ),
        DeclareLaunchArgument(
            'realsense_depth_image_topic',
            default_value='/camera/camera/depth/image_rect_raw',
            description='Depth image topic consumed by depthimage_to_laserscan.',
        ),
        DeclareLaunchArgument(
            'realsense_depth_camera_info_topic',
            default_value='/camera/camera/depth/camera_info',
            description='Depth camera info topic consumed by depthimage_to_laserscan.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_topic',
            default_value='/scan',
            description='LaserScan topic published by the Realsense depth bridge.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_output_frame',
            default_value='camera_depth_frame',
            description='Frame id assigned to the LaserScan generated from Realsense depth.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_time',
            default_value='0.2',
            description='scan_time parameter passed to depthimage_to_laserscan.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_range_min',
            default_value='0.2',
            description='Minimum valid range used when converting Realsense depth to LaserScan.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_range_max',
            default_value='2.5',
            description='Maximum valid range used when converting Realsense depth to LaserScan.',
        ),
        DeclareLaunchArgument(
            'realsense_scan_height',
            default_value='3',
            description='Number of depth-image rows fused into the output LaserScan.',
        ),
        realsense_camera_launch,
        realsense_depth_to_scan_node,
    ])
```

- [ ] **Step 2: Re-run the full launch integration pytest file and confirm it is all green**

Run: `python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py`

Expected: PASS. The new camera-argument and `rs_launch.py` include tests should pass, along with the earlier ownership and bridge-node tests.

- [ ] **Step 3: Commit the standalone Realsense launch implementation**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add launch/realsense_depth_to_scan.launch.py
git commit -m "feat: compose standalone realsense obstacle launch"
```

## Task 5: Run Package-Level Verification And Land The Final Change Set

**Files:**
- Verify: `launch/2d_g1_nav2_bringup.launch.py`
- Verify: `launch/g1_auto_explore.launch.py`
- Verify: `launch/realsense_depth_to_scan.launch.py`
- Verify: `rviz/nav2_g1_view.rviz`
- Verify: `test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Run the package-level pytest target through colcon**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R realsense_nav2_integration_test`

Expected: PASS for `realsense_nav2_integration_test`.

- [ ] **Step 2: Check the workspace diff and make sure the final file set matches the plan**

Run: `cd /home/unitree/ros2_ws/src/g1_nav && git status --short`

Expected: only the intended launch, RViz rename, and test files are staged or modified for this feature; unrelated user changes remain untouched.

- [ ] **Step 3: Create the final feature commit**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add launch/2d_g1_nav2_bringup.launch.py launch/g1_auto_explore.launch.py launch/realsense_depth_to_scan.launch.py rviz/nav2_g1_view.rviz rviz/nav2_go2_view.rviz test/test_realsense_nav2_integration.py
git commit -m "feat: layer g1 navigation launches"
```
