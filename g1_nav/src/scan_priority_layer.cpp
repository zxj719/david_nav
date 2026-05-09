#include "g1_nav/scan_priority_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "geometry_msgs/msg/point.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

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
  if (!node) {
    throw std::runtime_error("ScanPriorityLayer node handle expired during initialization");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("scan_topic", rclcpp::ParameterValue(std::string("/scan")));
  declareParameter("sector_min_angle", rclcpp::ParameterValue(-0.55));
  declareParameter("sector_max_angle", rclcpp::ParameterValue(0.55));
  declareParameter("max_range", rclcpp::ParameterValue(2.5));
  declareParameter("ground_plane_z_in_base", rclcpp::ParameterValue(0.0));
  declareParameter("obstacle_margin_m", rclcpp::ParameterValue(0.12));
  declareParameter("min_contiguous_beams", rclcpp::ParameterValue(3));
  declareParameter("debug_markers_enabled", rclcpp::ParameterValue(false));
  declareParameter("scan_timeout_s", rclcpp::ParameterValue(0.5));
  declareParameter("require_live_scan", rclcpp::ParameterValue(true));

  enabled_ = node->get_parameter(getFullName("enabled")).as_bool();
  scan_topic_ = node->get_parameter(getFullName("scan_topic")).as_string();
  sector_min_angle_ = node->get_parameter(getFullName("sector_min_angle")).as_double();
  sector_max_angle_ = node->get_parameter(getFullName("sector_max_angle")).as_double();
  max_range_ = node->get_parameter(getFullName("max_range")).as_double();
  ground_plane_z_in_base_ = node->get_parameter(
    getFullName("ground_plane_z_in_base")).as_double();
  obstacle_margin_m_ = node->get_parameter(getFullName("obstacle_margin_m")).as_double();
  min_contiguous_beams_ = node->get_parameter(
    getFullName("min_contiguous_beams")).as_int();
  debug_markers_enabled_ = node->get_parameter(
    getFullName("debug_markers_enabled")).as_bool();
  scan_timeout_s_ = node->get_parameter(getFullName("scan_timeout_s")).as_double();
  require_live_scan_ = node->get_parameter(getFullName("require_live_scan")).as_bool();

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScanPriorityLayer::scanCallback, this, std::placeholders::_1));

  if (debug_markers_enabled_) {
    debug_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      getFullName("debug_markers"), rclcpp::QoS(1).transient_local());
  }

  matchSize();
  clearLayer();
  current_ = !enabled_ || !require_live_scan_;
}

void ScanPriorityLayer::activate()
{
  if (debug_pub_) {
    debug_pub_->on_activate();
  }
}

void ScanPriorityLayer::deactivate()
{
  if (debug_pub_) {
    debug_pub_->on_deactivate();
  }
}

void ScanPriorityLayer::reset()
{
  matchSize();
  clearLayer();
  current_ = !enabled_ || !require_live_scan_;
}

bool ScanPriorityLayer::isClearable()
{
  return false;
}

void ScanPriorityLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  clearLayer();

  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
  }

  auto node = node_.lock();
  touch(robot_x - max_range_, robot_y - max_range_, min_x, min_y, max_x, max_y);
  touch(robot_x + max_range_, robot_y + max_range_, min_x, min_y, max_x, max_y);

  if (!enabled_) {
    current_ = true;
    return;
  }

  if (!node || !scan) {
    current_ = !require_live_scan_;
    return;
  }

  if ((node->now() - rclcpp::Time(scan->header.stamp)).seconds() > scan_timeout_s_) {
    current_ = !require_live_scan_;
    return;
  }

  recomputeLayer(*scan, robot_x, robot_y, robot_yaw);
  current_ = true;
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
  const sensor_msgs::msg::LaserScan & scan,
  double robot_x, double robot_y, double robot_yaw)
{
  clearLayer();
  classified_beams_.clear();

  std::vector<BeamDecision> decisions;
  decisions.reserve(scan.ranges.size());

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_->lookupTransform(
      "base",
      scan.header.frame_id,
      scan.header.stamp,
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        2000,
        "scan_priority_layer TF failure: %s",
        ex.what());
    }
    current_ = false;
    return;
  }

  tf2::Transform scan_to_base;
  tf2::fromMsg(tf_msg.transform, scan_to_base);

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

    ClassifiedBeam beam;
    beam.scan_index = i;
    beam.angle_rad = angle;
    beam.measured_range_m = scan.ranges[i];
    beam.ground_range_m = std::numeric_limits<double>::quiet_NaN();

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
      const tf2::Vector3 measured_hit_base = origin_base + beam_dir_base * beam.measured_range_m;
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

  const auto runs = find_obstacle_runs(
    decisions,
    static_cast<std::size_t>(std::max(min_contiguous_beams_, 1)));

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

  publishDebugMarkers();
}

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

void ScanPriorityLayer::publishDebugMarkers() const
{
  if (!debug_pub_ || !debug_pub_->is_activated()) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;

  visualization_msgs::msg::Marker obstacle_marker;
  obstacle_marker.header.frame_id = "base";
  obstacle_marker.header.stamp = latest_scan_stamp_;
  obstacle_marker.ns = "scan_priority_obstacles";
  obstacle_marker.id = 0;
  obstacle_marker.type = visualization_msgs::msg::Marker::POINTS;
  obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
  obstacle_marker.scale.x = 0.05;
  obstacle_marker.scale.y = 0.05;
  obstacle_marker.color.r = 1.0;
  obstacle_marker.color.a = 1.0;

  visualization_msgs::msg::Marker floor_marker = obstacle_marker;
  floor_marker.ns = "scan_priority_floor";
  floor_marker.id = 1;
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

void ScanPriorityLayer::clearLayer()
{
  const auto size_x = getSizeInCellsX();
  const auto size_y = getSizeInCellsY();
  std::fill(costmap_, costmap_ + size_x * size_y, nav2_costmap_2d::NO_INFORMATION);
}

}  // namespace g1_nav

PLUGINLIB_EXPORT_CLASS(g1_nav::ScanPriorityLayer, nav2_costmap_2d::Layer)
