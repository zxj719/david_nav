# G1 Embedded Obstacle Escape and Differential Nav Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove nominal `y` motion from Nav2 path tracking and add a directional embedded-obstacle recovery that stays active until G1 is clear of lethal local-costmap overlap.

**Architecture:** First lock the desired Nav2 config and BT wiring in pytest. Then add a focused `EscapeObstacle` helper API plus gtests for direction choice, clear-observation tracking, and zero-stop commands. After that, implement the ROS-facing `BT::StatefulActionNode` plugin and register it in `bt_navigator`. Finally wire the plugin into both embedded-recovery XML trees, disable lateral navigation velocity generation, and run targeted verification.

**Tech Stack:** ROS 2 Humble, Nav2 BT, BehaviorTree.CPP v3, rclcpp, tf2_ros, pytest, gtest, colcon

---

### Task 1: Lock the Nav2 Config and BT Contract in Pytest

**Files:**
- Modify: `test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Add failing pytest coverage for the new contract**

```python
import xml.etree.ElementTree as ET


def _named_sequence(relative_path: str, sequence_name: str):
    root = ET.parse(REPO_ROOT / relative_path).getroot()
    return next(
        node
        for node in root.iter('Sequence')
        if node.attrib.get('name') == sequence_name
    )


def test_bt_navigator_loads_escape_obstacle_plugin():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    plugin_names = config['bt_navigator']['ros__parameters']['plugin_lib_names']

    assert 'g1_escape_obstacle_action_bt_node' in plugin_names


def test_follow_path_disables_lateral_navigation_velocity():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    follow_path = config['controller_server']['ros__parameters']['FollowPath']
    velocity_smoother = config['velocity_smoother']['ros__parameters']

    assert follow_path['max_vel_y'] == 0.0
    assert follow_path['vy_samples'] == 1
    assert velocity_smoother['max_velocity'][1] == 0.0
    assert velocity_smoother['min_velocity'][1] == 0.0


def test_navigate_to_pose_embedded_recovery_uses_escape_obstacle():
    sequence = _named_sequence(
        'behavior_trees/navigate_to_pose_w_g1_embedded_recovery.xml',
        'EmbeddedRecoveryLevel1',
    )

    assert [child.tag for child in sequence] == [
        'CancelControl',
        'EscapeObstacle',
        'ClearEntireCostmap',
        'ClearEntireCostmap',
    ]
    assert sequence[1].attrib == {
        'clear_confirmations': '3',
        'cmd_vel_topic': 'cmd_vel',
        'costmap_topic': 'local_costmap/costmap_raw',
        'footprint_topic': 'local_costmap/published_footprint',
        'robot_base_frame': 'base',
        'longitudinal_speed': '0.25',
        'lateral_speed': '0.20',
        'lethal_cost_threshold': '254',
        'transform_tolerance': '0.2',
    }


def test_navigate_through_poses_embedded_recovery_uses_escape_obstacle():
    sequence = _named_sequence(
        'behavior_trees/navigate_through_poses_w_g1_embedded_recovery.xml',
        'EmbeddedRecoveryLevel1',
    )

    assert [child.tag for child in sequence] == [
        'CancelControl',
        'EscapeObstacle',
        'ClearEntireCostmap',
        'ClearEntireCostmap',
    ]
```

- [ ] **Step 2: Run the regression file and watch it fail for the new behavior**

Run:

```bash
env ROS_LOG_DIR=/tmp/g1_nav_ros_log python3 -m pytest /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py -q
```

Expected: FAIL because the new BT plugin name, differential-drive constraints, and `EscapeObstacle`
XML wiring do not exist yet.

### Task 2: Lock Escape Direction and Clear-State Rules in GTest

**Files:**
- Modify: `CMakeLists.txt`
- Create: `test/escape_obstacle_action_test.cpp`

- [ ] **Step 1: Add a failing unit-test target for the escape helper API**

```cpp
#include <gtest/gtest.h>

#include "g1_nav/escape_obstacle_action.hpp"

TEST(EscapeObstacleAction, FrontOverlapCommandsNegativeX)
{
  const std::vector<g1_nav::EscapeOverlapPoint> overlaps{{0.18, 0.02}, {0.15, -0.03}};
  const auto command = g1_nav::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, -0.25);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
}

TEST(EscapeObstacleAction, RearOverlapCommandsPositiveX)
{
  const std::vector<g1_nav::EscapeOverlapPoint> overlaps{{-0.17, 0.01}, {-0.14, -0.02}};
  const auto command = g1_nav::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.25);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
}

TEST(EscapeObstacleAction, LeftOverlapCommandsNegativeY)
{
  const std::vector<g1_nav::EscapeOverlapPoint> overlaps{{0.02, 0.16}, {-0.01, 0.12}};
  const auto command = g1_nav::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, -0.20);
}

TEST(EscapeObstacleAction, RightOverlapCommandsPositiveY)
{
  const std::vector<g1_nav::EscapeOverlapPoint> overlaps{{0.03, -0.18}, {-0.02, -0.11}};
  const auto command = g1_nav::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.20);
}

TEST(EscapeObstacleAction, ClearTrackerNeedsThreeConsecutiveClearObservations)
{
  g1_nav::ClearObservationTracker tracker(3);

  EXPECT_FALSE(tracker.update(true));
  EXPECT_FALSE(tracker.update(false));
  EXPECT_FALSE(tracker.update(false));
  EXPECT_TRUE(tracker.update(false));
}

TEST(EscapeObstacleAction, StopTwistZeroesAllComponents)
{
  const auto twist = g1_nav::make_stop_twist();

  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}
```

- [ ] **Step 2: Register the new test target**

```cmake
  ament_add_gtest(
    escape_obstacle_action_test
    test/escape_obstacle_action_test.cpp
  )
  target_compile_features(escape_obstacle_action_test PUBLIC cxx_std_17)
  target_include_directories(escape_obstacle_action_test PUBLIC include)
```

- [ ] **Step 3: Run the new unit target and confirm the RED state**

Run:

```bash
colcon --log-base /tmp/g1_nav_log build --packages-select g1_nav --build-base /tmp/g1_nav_build --install-base /tmp/g1_nav_install --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
colcon --log-base /tmp/g1_nav_log test --packages-select g1_nav --build-base /tmp/g1_nav_build --install-base /tmp/g1_nav_install --event-handlers console_direct+ --ctest-args -R escape_obstacle_action_test
```

Expected: FAIL because `g1_nav/escape_obstacle_action.hpp` and the helper API do not exist yet.

### Task 3: Implement the Pure Escape Helper API Until the GTests Pass

**Files:**
- Create: `include/g1_nav/escape_obstacle_action.hpp`
- Create: `src/escape_obstacle_action.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add the helper structs, tracker, and BT-action declaration**

```cpp
#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace g1_nav
{

struct EscapeOverlapPoint
{
  double x{0.0};
  double y{0.0};
};

struct EscapeCommand
{
  double vx{0.0};
  double vy{0.0};
  bool valid{false};
};

EscapeCommand choose_escape_command(
  const std::vector<EscapeOverlapPoint> & overlaps,
  double longitudinal_speed,
  double lateral_speed);

geometry_msgs::msg::Twist make_stop_twist();

class ClearObservationTracker
{
public:
  explicit ClearObservationTracker(int required_clear_observations);
  bool update(bool embedded);
  void reset();

private:
  int required_clear_observations_;
  int consecutive_clear_observations_{0};
};

class EscapeObstacleAction : public BT::StatefulActionNode
{
public:
  EscapeObstacleAction(const std::string & name, const BT::NodeConfiguration & config);
  ~EscapeObstacleAction() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  void on_costmap_received(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void on_footprint_received(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  bool evaluate_escape_command(EscapeCommand & command, bool & embedded);
  std::vector<EscapeOverlapPoint> collect_overlap_points_in_base_frame(
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::PolygonStamped & footprint) const;
  std::vector<Point2D> get_footprint_in_costmap_frame(
    const geometry_msgs::msg::PolygonStamped & footprint,
    const std::string & target_frame) const;
  bool cell_is_lethal(const nav2_msgs::msg::Costmap & costmap, std::size_t index) const;
  bool point_inside_polygon(
    double x,
    double y,
    const std::vector<Point2D> & polygon) const;
  bool point_on_segment(
    double px,
    double py,
    const Point2D & start,
    const Point2D & end) const;
  void publish_command(const EscapeCommand & command);
  void publish_stop_command();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  mutable std::mutex data_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;
  geometry_msgs::msg::PolygonStamped::SharedPtr latest_footprint_;

  std::string costmap_topic_;
  std::string footprint_topic_;
  std::string cmd_vel_topic_;
  std::string robot_base_frame_;
  uint8_t lethal_cost_threshold_{254};
  double transform_tolerance_{0.2};
  double longitudinal_speed_{0.25};
  double lateral_speed_{0.20};
  ClearObservationTracker clear_tracker_{3};
};

}  // namespace g1_nav
```

- [ ] **Step 2: Implement the pure helper functions first**

```cpp
g1_nav::EscapeCommand g1_nav::choose_escape_command(
  const std::vector<EscapeOverlapPoint> & overlaps,
  double longitudinal_speed,
  double lateral_speed)
{
  if (overlaps.empty()) {
    return {};
  }

  double positive_x = 0.0;
  double negative_x = 0.0;
  double positive_y = 0.0;
  double negative_y = 0.0;

  for (const auto & point : overlaps) {
    if (point.x >= 0.0) {
      positive_x += std::abs(point.x);
    } else {
      negative_x += std::abs(point.x);
    }

    if (point.y >= 0.0) {
      positive_y += std::abs(point.y);
    } else {
      negative_y += std::abs(point.y);
    }
  }

  const double longitudinal_pressure = positive_x + negative_x;
  const double lateral_pressure = positive_y + negative_y;

  if (longitudinal_pressure >= lateral_pressure) {
    return positive_x >= negative_x ?
      EscapeCommand{-longitudinal_speed, 0.0, true} :
      EscapeCommand{longitudinal_speed, 0.0, true};
  }

  return positive_y >= negative_y ?
    EscapeCommand{0.0, -lateral_speed, true} :
    EscapeCommand{0.0, lateral_speed, true};
}

geometry_msgs::msg::Twist g1_nav::make_stop_twist()
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.angular.z = 0.0;
  return twist;
}

g1_nav::ClearObservationTracker::ClearObservationTracker(int required_clear_observations)
: required_clear_observations_(std::max(1, required_clear_observations))
{
}

bool g1_nav::ClearObservationTracker::update(bool embedded)
{
  if (embedded) {
    consecutive_clear_observations_ = 0;
    return false;
  }

  consecutive_clear_observations_ += 1;
  return consecutive_clear_observations_ >= required_clear_observations_;
}

void g1_nav::ClearObservationTracker::reset()
{
  consecutive_clear_observations_ = 0;
}
```

- [ ] **Step 3: Build the unit test against the new implementation**

```cmake
  ament_add_gtest(
    escape_obstacle_action_test
    test/escape_obstacle_action_test.cpp
    src/escape_obstacle_action.cpp
  )
  target_compile_features(escape_obstacle_action_test PUBLIC cxx_std_17)
  target_include_directories(escape_obstacle_action_test PUBLIC include)
  ament_target_dependencies(
    escape_obstacle_action_test
    behaviortree_cpp_v3
    geometry_msgs
    nav2_costmap_2d
    nav2_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
    tf2_ros
  )
```

- [ ] **Step 4: Re-run the focused unit target until it turns green**

Run:

```bash
colcon --log-base /tmp/g1_nav_log build --packages-select g1_nav --build-base /tmp/g1_nav_build --install-base /tmp/g1_nav_install --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
colcon --log-base /tmp/g1_nav_log test --packages-select g1_nav --build-base /tmp/g1_nav_build --install-base /tmp/g1_nav_install --event-handlers console_direct+ --ctest-args -R escape_obstacle_action_test
```

Expected: PASS for `escape_obstacle_action_test`.

### Task 4: Implement the ROS-Facing `EscapeObstacle` BT Plugin and Install It

**Files:**
- Modify: `include/g1_nav/escape_obstacle_action.hpp`
- Modify: `src/escape_obstacle_action.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Finish the BT node internals using the same subscription pattern as `RobotEmbeddedInObstacleCondition`**

```cpp
BT::NodeStatus g1_nav::EscapeObstacleAction::onStart()
{
  clear_tracker_.reset();
  return onRunning();
}

BT::NodeStatus g1_nav::EscapeObstacleAction::onRunning()
{
  EscapeCommand command;
  bool embedded = false;

  if (!evaluate_escape_command(command, embedded)) {
    publish_stop_command();
    return BT::NodeStatus::FAILURE;
  }

  if (!embedded) {
    if (clear_tracker_.update(false)) {
      publish_stop_command();
      return BT::NodeStatus::SUCCESS;
    }

    publish_stop_command();
    return BT::NodeStatus::RUNNING;
  }

  clear_tracker_.update(true);
  publish_command(command);
  return BT::NodeStatus::RUNNING;
}

void g1_nav::EscapeObstacleAction::onHalted()
{
  publish_stop_command();
}
```

- [ ] **Step 2: Reuse the existing footprint-overlap scan to derive overlap points in `base` frame**

```cpp
std::vector<g1_nav::EscapeOverlapPoint>
g1_nav::EscapeObstacleAction::collect_overlap_points_in_base_frame(
  const nav2_msgs::msg::Costmap & costmap,
  const geometry_msgs::msg::PolygonStamped & footprint) const
{
  const auto polygon = get_footprint_in_costmap_frame(footprint, costmap.header.frame_id);
  std::vector<EscapeOverlapPoint> overlap_points;
  if (polygon.size() < 3) {
    return overlap_points;
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & point : polygon) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  const double origin_x = costmap.metadata.origin.position.x;
  const double origin_y = costmap.metadata.origin.position.y;
  const double resolution = costmap.metadata.resolution;
  const int size_x = static_cast<int>(costmap.metadata.size_x);
  const int size_y = static_cast<int>(costmap.metadata.size_y);

  int min_mx = static_cast<int>(std::floor((min_x - origin_x) / resolution));
  int min_my = static_cast<int>(std::floor((min_y - origin_y) / resolution));
  int max_mx = static_cast<int>(std::floor((max_x - origin_x) / resolution));
  int max_my = static_cast<int>(std::floor((max_y - origin_y) / resolution));

  min_mx = std::max(0, min_mx);
  min_my = std::max(0, min_my);
  max_mx = std::min(size_x - 1, max_mx);
  max_my = std::min(size_y - 1, max_my);

  const auto transform = tf_buffer_->lookupTransform(
    robot_base_frame_,
    costmap.header.frame_id,
    tf2::TimePointZero,
    tf2::durationFromSec(transform_tolerance_));

  tf2::Quaternion rotation;
  tf2::fromMsg(transform.transform.rotation, rotation);
  const double yaw = tf2::getYaw(rotation);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double tx = transform.transform.translation.x;
  const double ty = transform.transform.translation.y;

  for (int my = min_my; my <= max_my; ++my) {
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      const double cell_center_x = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
      const double cell_center_y = origin_y + (static_cast<double>(my) + 0.5) * resolution;
      if (!point_inside_polygon(cell_center_x, cell_center_y, polygon)) {
        continue;
      }

      const std::size_t index =
        static_cast<std::size_t>(my) * costmap.metadata.size_x + static_cast<std::size_t>(mx);
      if (!cell_is_lethal(costmap, index)) {
        continue;
      }

      overlap_points.push_back({
        tx + cosine * cell_center_x - sine * cell_center_y,
        ty + sine * cell_center_x + cosine * cell_center_y
      });
    }
  }

  return overlap_points;
}

bool g1_nav::EscapeObstacleAction::evaluate_escape_command(
  EscapeCommand & command,
  bool & embedded)
{
  nav2_msgs::msg::Costmap::SharedPtr costmap;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    costmap = latest_costmap_;
    footprint = latest_footprint_;
  }

  if (!costmap || !footprint) {
    return false;
  }

  const auto overlap_points = collect_overlap_points_in_base_frame(*costmap, *footprint);
  embedded = !overlap_points.empty();
  command = choose_escape_command(
    overlap_points,
    longitudinal_speed_,
    lateral_speed_);
  return true;
}
```

- [ ] **Step 3: Register and install the shared BT plugin**

```cmake
add_library(g1_escape_obstacle_action_bt_node SHARED
  src/escape_obstacle_action.cpp
)
target_compile_features(g1_escape_obstacle_action_bt_node PUBLIC cxx_std_17)
target_compile_definitions(
  g1_escape_obstacle_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT
)
target_include_directories(
  g1_escape_obstacle_action_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  g1_escape_obstacle_action_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  nav2_costmap_2d
  nav2_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  g1_escape_obstacle_action_bt_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)
```

- [ ] **Step 4: Export the BT registration symbol**

```cpp
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<g1_nav::EscapeObstacleAction>("EscapeObstacle");
}
```

### Task 5: Wire the Plugin Into Nav2 Params and Both Embedded-Recovery Trees

**Files:**
- Modify: `config/nav2_params_2d.yaml`
- Modify: `behavior_trees/navigate_to_pose_w_g1_embedded_recovery.xml`
- Modify: `behavior_trees/navigate_through_poses_w_g1_embedded_recovery.xml`

- [ ] **Step 1: Register the plugin and disable nominal `y` navigation velocity**

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
    - nav2_distance_traveled_condition_bt_node
    - g1_robot_embedded_in_obstacle_condition_bt_node
    - g1_escape_obstacle_action_bt_node
    - nav2_single_trigger_bt_node

controller_server:
  ros__parameters:
    FollowPath:
      min_vel_y: 0.0
      max_vel_y: 0.0
      vy_samples: 1

velocity_smoother:
  ros__parameters:
    max_velocity: [0.4, 0.0, 1.0]
    min_velocity: [-0.4, 0.0, -1.0]
```

- [ ] **Step 2: Replace `EmbeddedRecoveryLevel1` in `navigate_to_pose`**

```xml
<Sequence name="EmbeddedRecoveryLevel1">
  <CancelControl name="CancelControl-EmbeddedEscape"/>
  <EscapeObstacle
    clear_confirmations="3"
    cmd_vel_topic="cmd_vel"
    costmap_topic="local_costmap/costmap_raw"
    footprint_topic="local_costmap/published_footprint"
    robot_base_frame="base"
    longitudinal_speed="0.25"
    lateral_speed="0.20"
    lethal_cost_threshold="254"
    transform_tolerance="0.2"/>
  <ClearEntireCostmap
    name="EmbeddedClearLocalCostmap-Level1"
    service_name="local_costmap/clear_entirely_local_costmap"/>
  <ClearEntireCostmap
    name="EmbeddedClearGlobalCostmap-Level1"
    service_name="global_costmap/clear_entirely_global_costmap"/>
</Sequence>
```

- [ ] **Step 3: Mirror the same sequence in `navigate_through_poses`**

```xml
<Sequence name="EmbeddedRecoveryLevel1">
  <CancelControl name="CancelControl-EmbeddedEscape"/>
  <EscapeObstacle
    clear_confirmations="3"
    cmd_vel_topic="cmd_vel"
    costmap_topic="local_costmap/costmap_raw"
    footprint_topic="local_costmap/published_footprint"
    robot_base_frame="base"
    longitudinal_speed="0.25"
    lateral_speed="0.20"
    lethal_cost_threshold="254"
    transform_tolerance="0.2"/>
  <ClearEntireCostmap
    name="EmbeddedClearLocalCostmap-Level1"
    service_name="local_costmap/clear_entirely_local_costmap"/>
  <ClearEntireCostmap
    name="EmbeddedClearGlobalCostmap-Level1"
    service_name="global_costmap/clear_entirely_global_costmap"/>
</Sequence>
```

- [ ] **Step 4: Re-run the pytest contract file**

Run:

```bash
env ROS_LOG_DIR=/tmp/g1_nav_ros_log python3 -m pytest /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py -q
```

Expected: PASS

### Task 6: Verify the Package Build and the Targeted Test Set

**Files:**
- Re-run modified files only

- [ ] **Step 1: Build `g1_nav` cleanly in an isolated temp build tree**

Run:

```bash
colcon --log-base /tmp/g1_nav_log build --packages-select g1_nav --build-base /tmp/g1_nav_build --install-base /tmp/g1_nav_install --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
```

Expected: build exits with code `0`.

- [ ] **Step 2: Run the focused regression tests**

Run:

```bash
colcon --log-base /tmp/g1_nav_log test --packages-select g1_nav --build-base /tmp/g1_nav_build --install-base /tmp/g1_nav_install --event-handlers console_direct+ --ctest-args -R 'escape_obstacle_action_test|frontier_navigation_result_policy_test|frontier_goal_selector_test'
```

Expected: all three gtests pass.

- [ ] **Step 3: Re-run the Python integration suite and inspect summarized results**

Run:

```bash
env ROS_LOG_DIR=/tmp/g1_nav_ros_log python3 -m pytest /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py -q
colcon --log-base /tmp/g1_nav_log test-result --test-result-base /tmp/g1_nav_build
```

Expected:

- `test_realsense_nav2_integration.py` passes
- `escape_obstacle_action_test` is listed as `passed`
- no new failures appear in the targeted `g1_nav` regression set
