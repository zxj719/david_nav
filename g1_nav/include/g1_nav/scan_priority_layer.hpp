#pragma once

#include <mutex>
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
  struct ClassifiedBeam
  {
    std::size_t scan_index{0};
    double angle_rad{0.0};
    double measured_range_m{0.0};
    double ground_range_m{0.0};
    BeamDecision decision{BeamDecision::kUnknown};
    bool has_ground_hit{false};
    bool has_measured_hit{false};
    double ground_hit_x_in_base{0.0};
    double ground_hit_y_in_base{0.0};
    double measured_hit_x_in_base{0.0};
    double measured_hit_y_in_base{0.0};
  };

  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void recomputeLayer(
    const sensor_msgs::msg::LaserScan & scan,
    double robot_x, double robot_y, double robot_yaw);
  std::pair<double, double> basePointToGlobal(
    double x_in_base, double y_in_base,
    double robot_x, double robot_y, double robot_yaw) const;
  void publishDebugMarkers() const;
  void clearLayer();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

  mutable std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  rclcpp::Time latest_scan_stamp_{0, 0, RCL_ROS_TIME};

  std::vector<ClassifiedBeam> classified_beams_;

  std::string scan_topic_{"/scan"};
  double sector_min_angle_{-0.55};
  double sector_max_angle_{0.55};
  double max_range_{2.5};
  double ground_plane_z_in_base_{0.0};
  double obstacle_margin_m_{0.12};
  int min_contiguous_beams_{3};
  bool debug_markers_enabled_{false};
  double scan_timeout_s_{0.5};
  bool require_live_scan_{true};
};

}  // namespace g1_nav
