# Scan Priority Local Costmap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a custom Nav2 `scan_priority_layer` that injects Realsense `/scan` ground-aware obstacle evidence into the front near-field of `local_costmap` without using scan data to clear or lower cost.

**Architecture:** Add a C++ `nav2_costmap_2d::CostmapLayer` plugin that subscribes to raw `/scan`, transforms beams into `base`, computes the expected floor-intersection distance for each front-sector beam, and writes `LETHAL_OBSTACLE` only when the beam lands significantly earlier than the expected ground baseline. Floor-compatible beams remain classification-only and do not clear or lower existing cost. Keep `global_costmap` unchanged, wire the new layer into `local_costmap`, and cover the behavior with pure-geometry gtests plus YAML/plugin integration tests.

**Tech Stack:** ROS 2 Humble, Nav2 `nav2_costmap_2d`, C++17, pluginlib, tf2, sensor_msgs, geometry_msgs, visualization_msgs, `ament_cmake_gtest`, `ament_cmake_pytest`

---

## File Structure

### New Files

- `include/g1_nav/scan_priority_geometry.hpp`
  Pure helper types and functions for beam-to-ground intersection, beam classification, and contiguous obstacle-run detection. Keep this independent from Nav2 plugin lifecycle so it can be tested directly.
- `src/scan_priority_geometry.cpp`
  Implementation of the pure geometric helpers declared above.
- `include/g1_nav/scan_priority_layer.hpp`
  Nav2 layer class declaration, ROS subscriptions, parameter declarations, debug marker publisher, and cached scan state.
- `src/scan_priority_layer.cpp`
  Plugin lifecycle, `/scan` subscription, TF use, costmap writes, debug markers, and failure-handling logic.
- `costmap_plugins.xml`
  pluginlib export manifest for `g1_nav::ScanPriorityLayer`.
- `test/scan_priority_geometry_test.cpp`
  gtests for floor intersection math, obstacle classification, NaN handling, and contiguous-beam threshold behavior.

### Modified Files

- `CMakeLists.txt`
  Add the new library, plugin export, dependencies, and gtest target.
- `package.xml`
  Add runtime/build dependencies required by the plugin export and scan handling.
- `config/nav2_params_2d.yaml`
  Insert `scan_priority_layer` into `local_costmap.plugins` and define initial parameters.
- `test/test_realsense_nav2_integration.py`
  Add YAML assertions for the new local plugin and its parameters.

## Task 1: Add Pure Geometry Tests First

**Files:**
- Create: `test/scan_priority_geometry_test.cpp`
- Test: `test/scan_priority_geometry_test.cpp`

- [ ] **Step 1: Write the failing geometry tests**

```cpp
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "g1_nav/scan_priority_geometry.hpp"

namespace
{

using g1_nav::BeamDecision;
using g1_nav::BeamSample;
using g1_nav::CameraRay;
using g1_nav::GroundIntersection;

TEST(ScanPriorityGeometryTest, intersectsForwardDownwardRayWithGroundPlane)
{
  CameraRay ray;
  ray.origin_x = 0.0;
  ray.origin_y = 0.0;
  ray.origin_z = 0.5;
  ray.dir_x = 1.0;
  ray.dir_y = 0.0;
  ray.dir_z = -1.0;

  const auto hit = g1_nav::intersect_ray_with_ground(ray, 0.0);

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->range_m, std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(hit->hit_x, 0.5, 1e-6);
  EXPECT_NEAR(hit->hit_y, 0.0, 1e-6);
}

TEST(ScanPriorityGeometryTest, returnsNulloptWhenRayNeverHitsGround)
{
  CameraRay ray;
  ray.origin_x = 0.0;
  ray.origin_y = 0.0;
  ray.origin_z = 0.5;
  ray.dir_x = 1.0;
  ray.dir_y = 0.0;
  ray.dir_z = 0.1;

  EXPECT_FALSE(g1_nav::intersect_ray_with_ground(ray, 0.0).has_value());
}

TEST(ScanPriorityGeometryTest, classifiesEarlierBeamAsObstacle)
{
  BeamSample sample;
  sample.range_m = 1.2;
  sample.valid = true;

  const auto decision = g1_nav::classify_beam_against_ground(
    sample,
    /*ground_range_m=*/1.6,
    /*obstacle_margin_m=*/0.15);

  EXPECT_EQ(decision, BeamDecision::kObstacle);
}

TEST(ScanPriorityGeometryTest, classifiesNearGroundMatchAsFloorCompatible)
{
  BeamSample sample;
  sample.range_m = 1.52;
  sample.valid = true;

  const auto decision = g1_nav::classify_beam_against_ground(
    sample,
    /*ground_range_m=*/1.6,
    /*obstacle_margin_m=*/0.15);

  EXPECT_EQ(decision, BeamDecision::kFloorCompatible);
}

TEST(ScanPriorityGeometryTest, treatsInvalidBeamAsUnknown)
{
  BeamSample sample;
  sample.range_m = std::numeric_limits<double>::quiet_NaN();
  sample.valid = false;

  const auto decision = g1_nav::classify_beam_against_ground(
    sample,
    /*ground_range_m=*/1.6,
    /*obstacle_margin_m=*/0.15);

  EXPECT_EQ(decision, BeamDecision::kUnknown);
}

TEST(ScanPriorityGeometryTest, suppressesShortObstacleRunsBelowThreshold)
{
  const std::vector<BeamDecision> decisions = {
    BeamDecision::kFloorCompatible,
    BeamDecision::kObstacle,
    BeamDecision::kObstacle,
    BeamDecision::kFloorCompatible,
    BeamDecision::kObstacle,
    BeamDecision::kFloorCompatible,
  };

  const auto runs = g1_nav::find_obstacle_runs(decisions, /*min_contiguous_beams=*/2);

  ASSERT_EQ(runs.size(), 1u);
  EXPECT_EQ(runs[0].start_index, 1u);
  EXPECT_EQ(runs[0].end_index, 2u);
}

}  // namespace
```

- [ ] **Step 2: Run the new gtest target name before implementation**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R scan_priority_geometry_test`

Expected: the focused test invocation reports that `scan_priority_geometry_test` is not present yet (for example `No tests were found!!!` or an equivalent “package has not been built / target not registered yet” message), which confirms the red path before any production helper exists.

- [ ] **Step 3: Add the test target to `CMakeLists.txt` so the failure is real**

```cmake
  ament_add_gtest(
    scan_priority_geometry_test
    test/scan_priority_geometry_test.cpp
  )
  target_compile_features(scan_priority_geometry_test PUBLIC cxx_std_17)
  target_include_directories(scan_priority_geometry_test PUBLIC include)
```

- [ ] **Step 4: Rebuild so the new target is generated and confirm the build stays red**

Run: `cd /home/unitree/ros2_ws && colcon build --packages-select g1_nav`

Expected: FAIL during compile or link because the geometry production artifacts are still missing. The first hard red should now come from the missing helper header and/or definitions, for example:
- missing `include/g1_nav/scan_priority_geometry.hpp`
- or missing symbols for `intersect_ray_with_ground`, `classify_beam_against_ground`, and `find_obstacle_runs`

Any of those outcomes is acceptable here as long as the build is red for the not-yet-implemented geometry helper without breaking package configuration.

- [ ] **Step 5: Commit the red test scaffold**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add CMakeLists.txt test/scan_priority_geometry_test.cpp
git commit -m "test: add scan priority geometry coverage"
```

## Task 2: Implement the Pure Geometry Helpers

**Files:**
- Create: `include/g1_nav/scan_priority_geometry.hpp`
- Create: `src/scan_priority_geometry.cpp`
- Modify: `CMakeLists.txt`
- Test: `test/scan_priority_geometry_test.cpp`

- [ ] **Step 1: Write the geometry helper declarations**

```cpp
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace g1_nav
{

struct CameraRay
{
  double origin_x;
  double origin_y;
  double origin_z;
  double dir_x;
  double dir_y;
  double dir_z;
};

struct GroundIntersection
{
  double range_m;
  double hit_x;
  double hit_y;
};

struct BeamSample
{
  double range_m;
  bool valid;
};

enum class BeamDecision
{
  kUnknown,
  kFloorCompatible,
  kObstacle,
};

struct ObstacleRun
{
  std::size_t start_index;
  std::size_t end_index;
};

std::optional<GroundIntersection> intersect_ray_with_ground(
  const CameraRay & ray,
  double ground_plane_z_in_base);

BeamDecision classify_beam_against_ground(
  const BeamSample & sample,
  double ground_range_m,
  double obstacle_margin_m);

std::vector<ObstacleRun> find_obstacle_runs(
  const std::vector<BeamDecision> & decisions,
  std::size_t min_contiguous_beams);

}  // namespace g1_nav
```

- [ ] **Step 2: Implement the minimal math in `src/scan_priority_geometry.cpp`**

```cpp
#include "g1_nav/scan_priority_geometry.hpp"

#include <cmath>

namespace g1_nav
{

std::optional<GroundIntersection> intersect_ray_with_ground(
  const CameraRay & ray,
  double ground_plane_z_in_base)
{
  const double dz = ray.dir_z;
  if (std::abs(dz) < 1e-9) {
    return std::nullopt;
  }

  const double t = (ground_plane_z_in_base - ray.origin_z) / dz;
  if (t <= 0.0) {
    return std::nullopt;
  }

  GroundIntersection hit;
  hit.hit_x = ray.origin_x + t * ray.dir_x;
  hit.hit_y = ray.origin_y + t * ray.dir_y;
  const double hit_z = ray.origin_z + t * ray.dir_z;
  if (std::abs(hit_z - ground_plane_z_in_base) > 1e-6) {
    return std::nullopt;
  }

  hit.range_m = std::sqrt(
    std::pow(t * ray.dir_x, 2.0) +
    std::pow(t * ray.dir_y, 2.0) +
    std::pow(t * ray.dir_z, 2.0));
  return hit;
}

BeamDecision classify_beam_against_ground(
  const BeamSample & sample,
  double ground_range_m,
  double obstacle_margin_m)
{
  if (!sample.valid || !std::isfinite(sample.range_m) || !std::isfinite(ground_range_m)) {
    return BeamDecision::kUnknown;
  }

  if (sample.range_m < ground_range_m - obstacle_margin_m) {
    return BeamDecision::kObstacle;
  }

  return BeamDecision::kFloorCompatible;
}

std::vector<ObstacleRun> find_obstacle_runs(
  const std::vector<BeamDecision> & decisions,
  std::size_t min_contiguous_beams)
{
  std::vector<ObstacleRun> runs;
  std::size_t start = 0;
  bool in_run = false;

  for (std::size_t i = 0; i < decisions.size(); ++i) {
    if (decisions[i] == BeamDecision::kObstacle) {
      if (!in_run) {
        start = i;
        in_run = true;
      }
      continue;
    }

    if (in_run && (i - start) >= min_contiguous_beams) {
      runs.push_back({start, i - 1});
    }
    in_run = false;
  }

  if (in_run && (decisions.size() - start) >= min_contiguous_beams) {
    runs.push_back({start, decisions.size() - 1});
  }

  return runs;
}

}  // namespace g1_nav
```

- [ ] **Step 3: Extend the gtest target to compile the real geometry implementation**

```cmake
  ament_add_gtest(
    scan_priority_geometry_test
    test/scan_priority_geometry_test.cpp
    src/scan_priority_geometry.cpp
  )
  target_compile_features(scan_priority_geometry_test PUBLIC cxx_std_17)
  target_include_directories(scan_priority_geometry_test PUBLIC include)
```

- [ ] **Step 4: Run the geometry gtests**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R scan_priority_geometry_test`

Expected: PASS for `scan_priority_geometry_test`.

- [ ] **Step 5: Run the exact gtest result output**

Run: `cd /home/unitree/ros2_ws && colcon test-result --verbose | rg "scan_priority_geometry_test|passed"`

Expected: one `scan_priority_geometry_test` entry marked `passed`.

- [ ] **Step 6: Commit the green geometry helpers**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add include/g1_nav/scan_priority_geometry.hpp src/scan_priority_geometry.cpp CMakeLists.txt test/scan_priority_geometry_test.cpp
git commit -m "feat: add scan priority geometry helpers"
```

## Task 3: Add Failing Integration Assertions for the New Layer

**Files:**
- Modify: `test/test_realsense_nav2_integration.py`
- Test: `test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Extend the YAML integration test with the new local plugin expectation**

```python
def test_local_costmap_uses_scan_priority_layer():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    local_costmap = config['local_costmap']['local_costmap']['ros__parameters']

    assert local_costmap['plugins'] == [
        'static_layer',
        'scan_priority_layer',
        'inflation_layer',
    ]

    layer = local_costmap['scan_priority_layer']
    assert layer['plugin'] == 'g1_nav::ScanPriorityLayer'
    assert layer['scan_topic'] == '/scan'
    assert layer['sector_min_angle'] == -0.55
    assert layer['sector_max_angle'] == 0.55
    assert layer['max_range'] == 2.5
    assert layer['ground_plane_z_in_base'] == 0.0
    assert layer['obstacle_margin_m'] == 0.12
    assert layer['min_contiguous_beams'] == 3
    assert layer['debug_markers_enabled'] is False
```

- [ ] **Step 2: Run the focused pytest and confirm it fails**

Run: `python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py::test_local_costmap_uses_scan_priority_layer`

Expected: FAIL because `scan_priority_layer` is not present in `config/nav2_params_2d.yaml`.

- [ ] **Step 3: Add a plugin-export assertion in the same pytest file**

```python
def test_package_exports_scan_priority_costmap_plugin_xml():
    package_xml = (REPO_ROOT / 'package.xml').read_text()
    assert 'pluginlib' in package_xml

    plugin_xml = (REPO_ROOT / 'costmap_plugins.xml').read_text()
    assert 'g1_nav::ScanPriorityLayer' in plugin_xml
    assert 'nav2_costmap_2d::Layer' in plugin_xml
```

- [ ] **Step 4: Run the focused pytest for the export assertion**

Run: `python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py::test_package_exports_scan_priority_costmap_plugin_xml`

Expected: FAIL because `costmap_plugins.xml` and plugin export wiring do not exist yet.

- [ ] **Step 5: Commit the red integration assertions**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add test/test_realsense_nav2_integration.py
git commit -m "test: expect scan priority local layer"
```

## Task 4: Implement the Plugin Skeleton and Export It

**Files:**
- Create: `include/g1_nav/scan_priority_layer.hpp`
- Create: `src/scan_priority_layer.cpp`
- Create: `costmap_plugins.xml`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`
- Test: `test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Add missing package dependencies**

```xml
  <depend>pluginlib</depend>
  <depend>rclcpp_lifecycle</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
```

Note: `sensor_msgs` is already present in the current `package.xml`; keep it only once in the final file.

- [ ] **Step 2: Add the plugin XML manifest**

```xml
<class_libraries>
  <library path="g1_nav_scan_priority_layer">
    <class type="g1_nav::ScanPriorityLayer" base_class_type="nav2_costmap_2d::Layer">
      <description>
        Front-sector local costmap layer that compares scan beams against a fixed ground baseline.
      </description>
    </class>
  </library>
</class_libraries>
```

- [ ] **Step 3: Add the layer declaration**

```cpp
#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "g1_nav/scan_priority_geometry.hpp"

namespace g1_nav
{

class ScanPriorityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ScanPriorityLayer();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void recomputeLayer(
    const sensor_msgs::msg::LaserScan & scan,
    double robot_x, double robot_y, double robot_yaw);
  std::pair<double, double> basePointToGlobal(
    double x_in_base, double y_in_base,
    double robot_x, double robot_y, double robot_yaw) const;
  void publishDebugMarkers() const;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

  mutable std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  rclcpp::Time latest_scan_stamp_{0, 0, RCL_ROS_TIME};

  std::string scan_topic_{"/scan"};
  double sector_min_angle_{-0.55};
  double sector_max_angle_{0.55};
  double max_range_{2.5};
  double ground_plane_z_in_base_{0.0};
  double obstacle_margin_m_{0.12};
  int min_contiguous_beams_{3};
  bool debug_markers_enabled_{false};
  double scan_timeout_s_{0.5};
};

}  // namespace g1_nav
```

- [ ] **Step 4: Add a minimal plugin implementation that compiles and exports**

```cpp
#include "g1_nav/scan_priority_layer.hpp"

#include <cmath>
#include <functional>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

namespace g1_nav
{

ScanPriorityLayer::ScanPriorityLayer()
{
  enabled_ = true;
  current_ = false;
}

void ScanPriorityLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("scan_topic", rclcpp::ParameterValue(std::string("/scan")));
  declareParameter("sector_min_angle", rclcpp::ParameterValue(-0.55));
  declareParameter("sector_max_angle", rclcpp::ParameterValue(0.55));
  declareParameter("max_range", rclcpp::ParameterValue(2.5));
  declareParameter("ground_plane_z_in_base", rclcpp::ParameterValue(0.0));
  declareParameter("obstacle_margin_m", rclcpp::ParameterValue(0.12));
  declareParameter("min_contiguous_beams", rclcpp::ParameterValue(3));
  declareParameter("debug_markers_enabled", rclcpp::ParameterValue(false));

  scan_topic_ = node->get_parameter(getFullName("scan_topic")).as_string();
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScanPriorityLayer::scanCallback, this, std::placeholders::_1));

  matchSize();
  current_ = true;
}

void ScanPriorityLayer::activate() {}
void ScanPriorityLayer::deactivate() {}
void ScanPriorityLayer::reset() { matchSize(); current_ = false; }
bool ScanPriorityLayer::isClearable() { return false; }

void ScanPriorityLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  useExtraBounds(min_x, min_y, max_x, max_y);
}

void ScanPriorityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

void ScanPriorityLayer::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_stamp_ = rclcpp::Time(msg->header.stamp);
  latest_scan_ = std::move(msg);
}

void ScanPriorityLayer::recomputeLayer(
  const sensor_msgs::msg::LaserScan &,
  double, double, double) {}
std::pair<double, double> ScanPriorityLayer::basePointToGlobal(
  double, double, double, double, double) const
{
  return {0.0, 0.0};
}
void ScanPriorityLayer::publishDebugMarkers() const {}

}  // namespace g1_nav

PLUGINLIB_EXPORT_CLASS(g1_nav::ScanPriorityLayer, nav2_costmap_2d::Layer)
```

- [ ] **Step 5: Wire the library and plugin export in `CMakeLists.txt`**

```cmake
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(g1_nav_scan_priority_layer SHARED
  src/scan_priority_layer.cpp
  src/scan_priority_geometry.cpp
)
target_compile_features(g1_nav_scan_priority_layer PUBLIC cxx_std_17)
target_include_directories(g1_nav_scan_priority_layer PUBLIC include)
ament_target_dependencies(
  g1_nav_scan_priority_layer
  geometry_msgs
  nav2_costmap_2d
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(FILES costmap_plugins.xml DESTINATION share/${PROJECT_NAME})
install(TARGETS g1_nav_scan_priority_layer
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)
```

- [ ] **Step 6: Run the focused pytest export assertions**

Run: `python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py::test_package_exports_scan_priority_costmap_plugin_xml`

Expected: PASS.

- [ ] **Step 7: Run a package build**

Run: `cd /home/unitree/ros2_ws && colcon build --packages-select g1_nav`

Expected: PASS with `Finished <<< g1_nav`.

- [ ] **Step 8: Commit the exported plugin skeleton**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add CMakeLists.txt package.xml costmap_plugins.xml include/g1_nav/scan_priority_layer.hpp src/scan_priority_layer.cpp
git commit -m "feat: export scan priority costmap layer"
```

## Task 5: Implement Front-Sector Ground-Aware Writes

**Files:**
- Modify: `include/g1_nav/scan_priority_layer.hpp`
- Modify: `src/scan_priority_layer.cpp`
- Test: `test/scan_priority_geometry_test.cpp`

- [ ] **Step 1: Add a cached beam record type and the `base`-to-global helper**

```cpp
struct ClassifiedBeam
{
  std::size_t scan_index;
  double angle_rad;
  double measured_range_m;
  double ground_range_m;
  BeamDecision decision;
  bool has_ground_hit;
  bool has_measured_hit;
  double ground_hit_x_in_base;
  double ground_hit_y_in_base;
  double measured_hit_x_in_base;
  double measured_hit_y_in_base;
};

std::vector<ClassifiedBeam> classified_beams_;

std::pair<double, double> ScanPriorityLayer::basePointToGlobal(
  double x_in_base, double y_in_base,
  double robot_x, double robot_y, double robot_yaw) const
{
  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  return {
    robot_x + c * x_in_base - s * y_in_base,
    robot_y + s * x_in_base + c * y_in_base
  };
}
```

- [ ] **Step 2: Implement scan staleness handling in `updateBounds()`**

```cpp
void ScanPriorityLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::fill(costmap_, costmap_ + getSizeInCellsX() * getSizeInCellsY(), nav2_costmap_2d::NO_INFORMATION);

  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
  }

  const auto node = node_.lock();
  if (!node || !scan) {
    current_ = false;
    touch(robot_x - max_range_, robot_y - max_range_, min_x, min_y, max_x, max_y);
    touch(robot_x + max_range_, robot_y + max_range_, min_x, min_y, max_x, max_y);
    return;
  }

  if ((node->now() - rclcpp::Time(scan->header.stamp)).seconds() > scan_timeout_s_) {
    current_ = false;
    touch(robot_x - max_range_, robot_y - max_range_, min_x, min_y, max_x, max_y);
    touch(robot_x + max_range_, robot_y + max_range_, min_x, min_y, max_x, max_y);
    return;
  }

  recomputeLayer(*scan, robot_x, robot_y, robot_yaw);
  touch(robot_x - max_range_, robot_y - max_range_, min_x, min_y, max_x, max_y);
  touch(robot_x + max_range_, robot_y + max_range_, min_x, min_y, max_x, max_y);
  current_ = true;
}
```

- [ ] **Step 3: Implement per-beam classification in `recomputeLayer()`**

```cpp
void ScanPriorityLayer::recomputeLayer(
  const sensor_msgs::msg::LaserScan & scan,
  double, double, double)
{
  classified_beams_.clear();

  geometry_msgs::msg::TransformStamped tf_msg =
    tf_->lookupTransform("base", scan.header.frame_id, scan.header.stamp, tf2::durationFromSec(0.1));

  tf2::Transform scan_to_base;
  tf2::fromMsg(tf_msg.transform, scan_to_base);

  std::vector<BeamDecision> decisions;
  decisions.reserve(scan.ranges.size());

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (angle < sector_min_angle_ || angle > sector_max_angle_) {
      continue;
    }

    tf2::Vector3 beam_dir_scan(std::cos(angle), std::sin(angle), 0.0);
    tf2::Vector3 beam_dir_base = scan_to_base.getBasis() * beam_dir_scan.normalized();
    tf2::Vector3 origin_base = scan_to_base.getOrigin();

    CameraRay ray{
      origin_base.x(), origin_base.y(), origin_base.z(),
      beam_dir_base.x(), beam_dir_base.y(), beam_dir_base.z()
    };

    ClassifiedBeam beam{};
    beam.scan_index = i;
    beam.angle_rad = angle;
    beam.measured_range_m = scan.ranges[i];
    beam.ground_range_m = std::numeric_limits<double>::quiet_NaN();
    beam.decision = BeamDecision::kUnknown;
    beam.has_ground_hit = false;
    beam.has_measured_hit = false;

    const auto ground_hit = intersect_ray_with_ground(ray, ground_plane_z_in_base_);
    if (ground_hit && ground_hit->range_m <= max_range_) {
      beam.has_ground_hit = true;
      beam.ground_range_m = ground_hit->range_m;
      beam.ground_hit_x_in_base = ground_hit->hit_x;
      beam.ground_hit_y_in_base = ground_hit->hit_y;
    }

    BeamSample sample{
      beam.measured_range_m,
      std::isfinite(beam.measured_range_m) &&
      beam.measured_range_m >= scan.range_min &&
      beam.measured_range_m <= max_range_
    };
    if (sample.valid) {
      const tf2::Vector3 measured_hit_base = origin_base + beam.measured_range_m * beam_dir_base;
      beam.has_measured_hit = true;
      beam.measured_hit_x_in_base = measured_hit_base.x();
      beam.measured_hit_y_in_base = measured_hit_base.y();
    }

    if (beam.has_ground_hit) {
      beam.decision = classify_beam_against_ground(
        sample, beam.ground_range_m, obstacle_margin_m_);
    }

    classified_beams_.push_back(beam);
    decisions.push_back(beam.decision);
  }

  const auto runs = find_obstacle_runs(decisions, static_cast<std::size_t>(min_contiguous_beams_));
  // Convert only obstacle runs into costmap writes in the next step.
}
```

- [ ] **Step 4: Write scan-priority obstacle cells into the local layer using positive evidence only**

```cpp
  for (const auto & run : runs) {
    for (std::size_t i = run.start_index; i <= run.end_index; ++i) {
      const auto & beam = classified_beams_[i];
      if (!beam.has_measured_hit) {
        continue;
      }

      const auto [wx, wy] = basePointToGlobal(
        beam.measured_hit_x_in_base, beam.measured_hit_y_in_base,
        robot_x, robot_y, robot_yaw);
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!worldToMap(wx, wy, mx, my)) {
        continue;
      }

      setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
```

- [ ] **Step 5: Run the geometry gtests after the real implementation**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R scan_priority_geometry_test`

Expected: PASS.

- [ ] **Step 6: Build the package again**

Run: `cd /home/unitree/ros2_ws && colcon build --packages-select g1_nav`

Expected: PASS.

- [ ] **Step 7: Commit the real layer logic**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add include/g1_nav/scan_priority_layer.hpp src/scan_priority_layer.cpp
git commit -m "feat: classify front scan beams against floor baseline"
```

## Task 6: Wire the Layer into Nav2 Configuration

**Files:**
- Modify: `config/nav2_params_2d.yaml`
- Modify: `test/test_realsense_nav2_integration.py`
- Test: `test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Insert the new local layer into `nav2_params_2d.yaml`**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["static_layer", "scan_priority_layer", "inflation_layer"]
      scan_priority_layer:
        plugin: "g1_nav::ScanPriorityLayer"
        enabled: true
        scan_topic: /scan
        sector_min_angle: -0.55
        sector_max_angle: 0.55
        max_range: 2.5
        ground_plane_z_in_base: 0.0
        obstacle_margin_m: 0.12
        min_contiguous_beams: 3
        debug_markers_enabled: false
```

- [ ] **Step 2: Keep the existing `obstacle_layer` block out of the active plugin list**

```yaml
      # obstacle_layer:
      #   plugin: "nav2_costmap_2d::ObstacleLayer"
      #   enabled: True
      #   observation_sources: scan
```

Use comments only if you want to preserve the previous tuning for reference; do not leave `obstacle_layer` in the active `plugins` array.

- [ ] **Step 3: Run the focused pytest for the YAML assertion**

Run: `python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py::test_local_costmap_uses_scan_priority_layer`

Expected: PASS.

- [ ] **Step 4: Run the full integration pytest file**

Run: `python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py`

Expected: PASS with all tests green.

- [ ] **Step 5: Commit the Nav2 config wiring**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add config/nav2_params_2d.yaml test/test_realsense_nav2_integration.py
git commit -m "feat: wire scan priority layer into local costmap"
```

## Task 7: Add Debug Marker Publishing and Runtime Safeguards

**Files:**
- Modify: `include/g1_nav/scan_priority_layer.hpp`
- Modify: `src/scan_priority_layer.cpp`
- Test: `test/scan_priority_geometry_test.cpp`

- [ ] **Step 1: Add a debug marker publisher only when enabled**

```cpp
if (debug_markers_enabled_) {
  debug_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    getFullName("debug_markers"), rclcpp::QoS(1).transient_local());
}
```

- [ ] **Step 2: Publish obstacle and floor baseline markers**

```cpp
void ScanPriorityLayer::publishDebugMarkers() const
{
  if (!debug_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker obstacle_marker;
  obstacle_marker.header.frame_id = "base";
  obstacle_marker.header.stamp = latest_scan_stamp_;
  obstacle_marker.ns = "scan_priority_obstacles";
  obstacle_marker.type = visualization_msgs::msg::Marker::POINTS;
  obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
  obstacle_marker.scale.x = 0.05;
  obstacle_marker.scale.y = 0.05;
  obstacle_marker.color.r = 1.0;
  obstacle_marker.color.a = 1.0;

  visualization_msgs::msg::Marker floor_marker = obstacle_marker;
  floor_marker.ns = "scan_priority_floor";
  floor_marker.color.r = 0.0;
  floor_marker.color.g = 1.0;

  for (const auto & beam : classified_beams_) {
    if (beam.decision == BeamDecision::kObstacle && beam.has_measured_hit) {
      geometry_msgs::msg::Point p;
      p.x = beam.measured_hit_x_in_base;
      p.y = beam.measured_hit_y_in_base;
      obstacle_marker.points.push_back(p);
    }
    if (beam.decision == BeamDecision::kFloorCompatible && beam.has_ground_hit) {
      geometry_msgs::msg::Point p;
      p.x = beam.ground_hit_x_in_base;
      p.y = beam.ground_hit_y_in_base;
      floor_marker.points.push_back(p);
    }
  }

  array.markers.push_back(obstacle_marker);
  array.markers.push_back(floor_marker);
  debug_pub_->publish(array);
}
```

- [ ] **Step 3: Mark the layer non-current on stale scan / TF failure**

```cpp
catch (const tf2::TransformException & ex) {
  RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "scan_priority_layer TF failure: %s", ex.what());
  current_ = false;
  return;
}
```

- [ ] **Step 4: Re-run the package build and tests**

Run:

```bash
cd /home/unitree/ros2_ws
colcon build --packages-select g1_nav
colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R "scan_priority_geometry_test|realsense_nav2_integration_test"
```

Expected: PASS for build, `scan_priority_geometry_test`, and `realsense_nav2_integration_test`.

- [ ] **Step 5: Commit the observability and safeguard changes**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add include/g1_nav/scan_priority_layer.hpp src/scan_priority_layer.cpp
git commit -m "feat: add scan priority layer debug markers"
```

## Task 8: Perform Runtime Bringup Validation

**Files:**
- Modify: `docs/reports/2026-04-07-frontier-costmap-search-log.md`

- [ ] **Step 1: Start the updated stack**

Run:

```bash
cd /home/unitree/ros2_ws
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

In a second terminal:

```bash
cd /home/unitree/ros2_ws
source install/setup.bash
ros2 launch g1_nav 2d_g1_nav2_bringup.launch.py use_rviz:=true enable_realsense_scan_bridge:=true
```

Expected: `/scan`, `/local_costmap/costmap`, and `/local_costmap/published_footprint` exist.

- [ ] **Step 2: Confirm the custom layer is loaded**

Run:

```bash
ros2 param get /local_costmap/local_costmap plugins
ros2 topic list | rg "scan_priority"
```

Expected: `plugins` includes `scan_priority_layer`; optional debug marker topic appears when enabled.

- [ ] **Step 3: Verify no positive-evidence insertion on flat floor**

Run:

```bash
ros2 topic echo /local_costmap/costmap --once
```

Expected: when the camera sees only floor, the scan-priority layer does not inject new local lethal cells; any remaining occupancy still comes from the existing static/map layers.

- [ ] **Step 4: Verify hard obstacle insertion**

Procedure: place a box or leg-height obstacle in the front sector before the expected floor-intersection line.

Run:

```bash
ros2 topic echo /scan --once
ros2 topic echo /local_costmap/costmap --once
```

Expected: the obstacle angle now produces a shorter beam than the floor baseline, and the corresponding near-field cells in local costmap become lethal.

- [ ] **Step 5: Record the runtime results**

Add a `## Scan Priority Layer Validation` section to `docs/reports/2026-04-07-frontier-costmap-search-log.md` that records:

- the exact bringup command used
- the observed flat-floor non-obstacle result from Step 3
- the observed obstacle-insertion result from Step 4
- any residual TF, scan-rate, or false-positive issues

- [ ] **Step 6: Commit the validation note**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add docs/reports/2026-04-07-frontier-costmap-search-log.md
git commit -m "docs: record scan priority layer validation"
```

## Self-Review

- Spec coverage:
  - custom local plugin: Task 4 and Task 5
  - front-sector ground baseline classification: Task 2 and Task 5
  - local-only override with unchanged global costmap: Task 6
  - fail-safe stale scan / TF behavior: Task 7
  - debug observability: Task 7
  - automated verification and runtime validation: Task 1, Task 2, Task 3, Task 6, Task 7, Task 8
- Placeholder scan:
  - removed all `TBD`/`TODO` placeholders
  - removed the runtime report template placeholders and replaced them with explicit recording requirements
  - every code-changing step includes concrete code or exact YAML/XML content
- Type consistency:
  - helper names used consistently across tests, declarations, and implementation snippets:
    `intersect_ray_with_ground`, `classify_beam_against_ground`, `find_obstacle_runs`, `ScanPriorityLayer`
  - beam-run indexing now stays aligned because `classified_beams_` and the `decisions` vector are built over the same in-sector beam sequence
  - cost writes now convert `base` points into local-costmap world coordinates before calling `worldToMap()`
  - scan-priority semantics now use scan as positive obstacle evidence only; floor-compatible beams do not clear or lower existing cost
