#include "g1_nav/robot_embedded_in_obstacle_condition.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace g1_nav
{

namespace
{

constexpr double kEpsilon = 1e-6;

}  // namespace

RobotEmbeddedInObstacleCondition::RobotEmbeddedInObstacleCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  getInput("costmap_topic", costmap_topic_);
  if (costmap_topic_.empty()) {
    costmap_topic_ = "local_costmap/costmap_raw";
  }

  getInput("footprint_topic", footprint_topic_);
  if (footprint_topic_.empty()) {
    footprint_topic_ = "local_costmap/published_footprint";
  }

  getInput("confirmations_required", confirmations_required_);
  confirmations_required_ = std::max(1, confirmations_required_);

  int lethal_cost_threshold = static_cast<int>(lethal_cost_threshold_);
  getInput("lethal_cost_threshold", lethal_cost_threshold);
  lethal_cost_threshold_ = static_cast<uint8_t>(std::clamp(lethal_cost_threshold, 0, 255));

  getInput("transform_tolerance", transform_tolerance_);
  transform_tolerance_ = std::max(0.0, transform_tolerance_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto footprint_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

  rclcpp::SubscriptionOptions costmap_options;
  costmap_options.callback_group = callback_group_;
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic_,
    costmap_qos,
    std::bind(&RobotEmbeddedInObstacleCondition::onCostmapReceived, this, std::placeholders::_1),
    costmap_options);

  rclcpp::SubscriptionOptions footprint_options;
  footprint_options.callback_group = callback_group_;
  footprint_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic_,
    footprint_qos,
    std::bind(
      &RobotEmbeddedInObstacleCondition::onFootprintReceived,
      this,
      std::placeholders::_1),
    footprint_options);

  callback_group_executor_thread_ = std::thread([this]() {
      callback_group_executor_.spin();
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "RobotEmbeddedInObstacle BT node listening to %s and %s (confirmations=%d, lethal>=%u)",
    costmap_topic_.c_str(),
    footprint_topic_.c_str(),
    confirmations_required_,
    static_cast<unsigned int>(lethal_cost_threshold_));
}

RobotEmbeddedInObstacleCondition::~RobotEmbeddedInObstacleCondition()
{
  callback_group_executor_.cancel();
  if (callback_group_executor_thread_.joinable()) {
    callback_group_executor_thread_.join();
  }
}

BT::PortsList RobotEmbeddedInObstacleCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "costmap_topic",
      "local_costmap/costmap_raw",
      "Local costmap topic used for embedded detection"),
    BT::InputPort<std::string>(
      "footprint_topic",
      "local_costmap/published_footprint",
      "Published footprint topic used for embedded detection"),
    BT::InputPort<int>(
      "confirmations_required",
      3,
      "Number of consecutive positive detections before returning SUCCESS"),
    BT::InputPort<int>(
      "lethal_cost_threshold",
      nav2_costmap_2d::LETHAL_OBSTACLE,
      "Minimum cell cost treated as a lethal obstacle"),
    BT::InputPort<double>(
      "transform_tolerance",
      0.2,
      "Transform lookup timeout in seconds")
  };
}

BT::NodeStatus RobotEmbeddedInObstacleCondition::tick()
{
  nav2_msgs::msg::Costmap::SharedPtr costmap;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint;
  uint64_t costmap_generation = 0;
  uint64_t footprint_generation = 0;
  bool fresh_observation = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    costmap = latest_costmap_;
    footprint = latest_footprint_;
    costmap_generation = costmap_generation_;
    footprint_generation = footprint_generation_;
    fresh_observation = hasFreshObservationLocked(costmap_generation, footprint_generation);
  }

  if (!costmap || !footprint) {
    consecutive_hits_ = 0;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "RobotEmbeddedInObstacle waiting for both costmap and footprint messages");
    return BT::NodeStatus::FAILURE;
  }

  bool embedded = false;
  try {
    embedded = isRobotEmbedded(*costmap, *footprint);
  } catch (const std::exception & ex) {
    consecutive_hits_ = 0;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "RobotEmbeddedInObstacle skipped detection: %s",
      ex.what());
    return BT::NodeStatus::FAILURE;
  }

  if (embedded) {
    if (fresh_observation) {
      consecutive_hits_ = std::min(confirmations_required_, consecutive_hits_ + 1);
    }
  } else {
    consecutive_hits_ = 0;
  }

  if (embedded && consecutive_hits_ >= confirmations_required_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "RobotEmbeddedInObstacle confirmed (%d/%d) on local costmap",
      consecutive_hits_,
      confirmations_required_);
    return BT::NodeStatus::SUCCESS;
  }

  if (embedded && fresh_observation) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "RobotEmbeddedInObstacle observed (%d/%d)",
      consecutive_hits_,
      confirmations_required_);
  }

  return BT::NodeStatus::FAILURE;
}

void RobotEmbeddedInObstacleCondition::onCostmapReceived(
  const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_costmap_ = msg;
  ++costmap_generation_;
}

void RobotEmbeddedInObstacleCondition::onFootprintReceived(
  const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_footprint_ = msg;
  ++footprint_generation_;
}

bool RobotEmbeddedInObstacleCondition::isRobotEmbedded(
  const nav2_msgs::msg::Costmap & costmap,
  const geometry_msgs::msg::PolygonStamped & footprint) const
{
  if (
    costmap.metadata.size_x == 0 || costmap.metadata.size_y == 0 ||
    costmap.metadata.resolution <= 0.0)
  {
    return false;
  }

  if (costmap.data.empty()) {
    return false;
  }

  const auto polygon = getFootprintInCostmapFrame(footprint, costmap.header.frame_id);
  if (polygon.size() < 3) {
    return false;
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

  if (max_mx < 0 || max_my < 0 || min_mx >= size_x || min_my >= size_y) {
    return false;
  }

  min_mx = std::max(0, min_mx);
  min_my = std::max(0, min_my);
  max_mx = std::min(size_x - 1, max_mx);
  max_my = std::min(size_y - 1, max_my);

  for (int my = min_my; my <= max_my; ++my) {
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      const double cell_center_x = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
      const double cell_center_y = origin_y + (static_cast<double>(my) + 0.5) * resolution;
      if (!pointInsidePolygon(cell_center_x, cell_center_y, polygon)) {
        continue;
      }

      const std::size_t index =
        static_cast<std::size_t>(my) * costmap.metadata.size_x + static_cast<std::size_t>(mx);
      if (index < costmap.data.size() && cellIsLethal(costmap, index)) {
        return true;
      }
    }
  }

  return false;
}

std::vector<RobotEmbeddedInObstacleCondition::Point2D>
RobotEmbeddedInObstacleCondition::getFootprintInCostmapFrame(
  const geometry_msgs::msg::PolygonStamped & footprint,
  const std::string & target_frame) const
{
  std::vector<Point2D> polygon;
  polygon.reserve(footprint.polygon.points.size());

  if (footprint.polygon.points.empty()) {
    return polygon;
  }

  if (footprint.header.frame_id.empty() || footprint.header.frame_id == target_frame) {
    for (const auto & point : footprint.polygon.points) {
      polygon.push_back({point.x, point.y});
    }
    return polygon;
  }

  const auto transform = tf_buffer_->lookupTransform(
    target_frame,
    footprint.header.frame_id,
    tf2::TimePointZero,
    tf2::durationFromSec(transform_tolerance_));

  tf2::Quaternion rotation;
  tf2::fromMsg(transform.transform.rotation, rotation);
  const double yaw = tf2::getYaw(rotation);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double tx = transform.transform.translation.x;
  const double ty = transform.transform.translation.y;

  for (const auto & point : footprint.polygon.points) {
    polygon.push_back({
      tx + cosine * point.x - sine * point.y,
      ty + sine * point.x + cosine * point.y});
  }

  return polygon;
}

bool RobotEmbeddedInObstacleCondition::cellIsLethal(
  const nav2_msgs::msg::Costmap & costmap,
  std::size_t index) const
{
  const auto cost = costmap.data[index];
  return cost != nav2_costmap_2d::NO_INFORMATION && cost >= lethal_cost_threshold_;
}

bool RobotEmbeddedInObstacleCondition::pointInsidePolygon(
  double x,
  double y,
  const std::vector<Point2D> & polygon) const
{
  bool inside = false;
  const std::size_t point_count = polygon.size();

  for (std::size_t i = 0, j = point_count - 1; i < point_count; j = i++) {
    const auto & current = polygon[i];
    const auto & previous = polygon[j];

    if (pointOnSegment(x, y, previous, current)) {
      return true;
    }

    const bool intersects =
      ((current.y > y) != (previous.y > y)) &&
      (x < (previous.x - current.x) * (y - current.y) / ((previous.y - current.y) + kEpsilon) +
      current.x);

    if (intersects) {
      inside = !inside;
    }
  }

  return inside;
}

bool RobotEmbeddedInObstacleCondition::pointOnSegment(
  double px,
  double py,
  const Point2D & start,
  const Point2D & end) const
{
  const double cross =
    (px - start.x) * (end.y - start.y) - (py - start.y) * (end.x - start.x);
  if (std::fabs(cross) > kEpsilon) {
    return false;
  }

  const double dot =
    (px - start.x) * (end.x - start.x) + (py - start.y) * (end.y - start.y);
  if (dot < -kEpsilon) {
    return false;
  }

  const double squared_length =
    (end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y);
  return dot <= squared_length + kEpsilon;
}

bool RobotEmbeddedInObstacleCondition::hasFreshObservationLocked(
  uint64_t costmap_generation,
  uint64_t footprint_generation)
{
  const bool fresh =
    costmap_generation != last_evaluated_costmap_generation_ ||
    footprint_generation != last_evaluated_footprint_generation_;

  last_evaluated_costmap_generation_ = costmap_generation;
  last_evaluated_footprint_generation_ = footprint_generation;
  return fresh;
}

}  // namespace g1_nav

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<g1_nav::RobotEmbeddedInObstacleCondition>("RobotEmbeddedInObstacle");
}
