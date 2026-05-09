#include "g1_nav/escape_obstacle_action.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/exceptions.h"
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

EscapeCommand choose_escape_command(
  const std::vector<EscapeOverlapPoint> & overlaps,
  double longitudinal_speed,
  double lateral_speed)
{
  double pressure_pos_x = 0.0;
  double pressure_neg_x = 0.0;
  double pressure_pos_y = 0.0;
  double pressure_neg_y = 0.0;

  for (const auto & point : overlaps) {
    if (point.x > 0.0) {
      pressure_pos_x += std::abs(point.x);
    } else if (point.x < 0.0) {
      pressure_neg_x += std::abs(point.x);
    }

    if (point.y > 0.0) {
      pressure_pos_y += std::abs(point.y);
    } else if (point.y < 0.0) {
      pressure_neg_y += std::abs(point.y);
    }
  }

  const double x_axis_pressure = pressure_pos_x + pressure_neg_x;
  const double y_axis_pressure = pressure_pos_y + pressure_neg_y;

  EscapeCommand command{0.0, 0.0, false};
  if (x_axis_pressure <= 0.0 && y_axis_pressure <= 0.0) {
    return command;
  }

  command.valid = true;
  if (x_axis_pressure >= y_axis_pressure) {
    command.vx = (pressure_pos_x >= pressure_neg_x) ?
      -std::abs(longitudinal_speed) :
      std::abs(longitudinal_speed);
  } else {
    command.vy = (pressure_pos_y >= pressure_neg_y) ?
      -std::abs(lateral_speed) :
      std::abs(lateral_speed);
  }

  return command;
}

geometry_msgs::msg::Twist make_stop_twist()
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  return twist;
}

ClearObservationTracker::ClearObservationTracker(int required_clear_observations)
: required_clear_observations_(std::max(1, required_clear_observations))
{
}

bool ClearObservationTracker::update(bool embedded)
{
  if (embedded) {
    clear_observations_ = 0;
    return false;
  }

  clear_observations_ = std::min(
    required_clear_observations_,
    clear_observations_ + 1);
  return clear_observations_ >= required_clear_observations_;
}

void ClearObservationTracker::reset()
{
  clear_observations_ = 0;
}

bool ObservationGenerationTracker::update(
  uint64_t costmap_generation,
  uint64_t footprint_generation)
{
  const bool fresh_observation =
    costmap_generation != last_costmap_generation_ ||
    footprint_generation != last_footprint_generation_;

  last_costmap_generation_ = costmap_generation;
  last_footprint_generation_ = footprint_generation;
  return fresh_observation;
}

void ObservationGenerationTracker::reset()
{
  last_costmap_generation_ = 0;
  last_footprint_generation_ = 0;
}

EscapeObstacleAction::EscapeObstacleAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(action_name, conf)
{
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  if (!node_) {
    throw BT::RuntimeError("EscapeObstacle requires a valid blackboard 'node'");
  }
  if (!tf_buffer_) {
    throw BT::RuntimeError("EscapeObstacle requires a valid blackboard 'tf_buffer'");
  }

  getInput("costmap_topic", costmap_topic_);
  if (costmap_topic_.empty()) {
    costmap_topic_ = "local_costmap/costmap_raw";
  }

  getInput("footprint_topic", footprint_topic_);
  if (footprint_topic_.empty()) {
    footprint_topic_ = "local_costmap/published_footprint";
  }

  getInput("cmd_vel_topic", cmd_vel_topic_);
  if (cmd_vel_topic_.empty()) {
    cmd_vel_topic_ = "cmd_vel";
  }

  getInput("robot_base_frame", robot_base_frame_);
  if (robot_base_frame_.empty()) {
    robot_base_frame_ = "base";
  }

  int clear_confirmations = 3;
  getInput("clear_confirmations", clear_confirmations);
  clear_tracker_ = ClearObservationTracker(clear_confirmations);

  int lethal_cost_threshold = static_cast<int>(lethal_cost_threshold_);
  getInput("lethal_cost_threshold", lethal_cost_threshold);
  lethal_cost_threshold_ = static_cast<uint8_t>(std::clamp(lethal_cost_threshold, 0, 255));

  getInput("transform_tolerance", transform_tolerance_);
  transform_tolerance_ = std::max(0.0, transform_tolerance_);

  getInput("longitudinal_speed", longitudinal_speed_);
  longitudinal_speed_ = std::abs(longitudinal_speed_);

  getInput("lateral_speed", lateral_speed_);
  lateral_speed_ = std::abs(lateral_speed_);

  getInput("max_escape_duration", max_escape_duration_);
  max_escape_duration_ = std::max(0.0, max_escape_duration_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  auto footprint_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  rclcpp::SubscriptionOptions costmap_options;
  costmap_options.callback_group = callback_group_;
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic_,
    costmap_qos,
    std::bind(&EscapeObstacleAction::on_costmap_received, this, std::placeholders::_1),
    costmap_options);

  rclcpp::SubscriptionOptions footprint_options;
  footprint_options.callback_group = callback_group_;
  footprint_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic_,
    footprint_qos,
    std::bind(&EscapeObstacleAction::on_footprint_received, this, std::placeholders::_1),
    footprint_options);

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  callback_group_executor_thread_ = std::thread([this]() {
      callback_group_executor_.spin();
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "EscapeObstacle BT node listening to %s and %s, publishing %s",
    costmap_topic_.c_str(),
    footprint_topic_.c_str(),
    cmd_vel_topic_.c_str());
}

EscapeObstacleAction::~EscapeObstacleAction()
{
  callback_group_executor_.cancel();
  if (callback_group_executor_thread_.joinable()) {
    callback_group_executor_thread_.join();
  }
}

BT::PortsList EscapeObstacleAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "costmap_topic",
      "local_costmap/costmap_raw",
      "Local costmap topic used to detect embedded obstacles"),
    BT::InputPort<std::string>(
      "footprint_topic",
      "local_costmap/published_footprint",
      "Published footprint topic used to detect embedded obstacles"),
    BT::InputPort<std::string>(
      "cmd_vel_topic",
      "cmd_vel",
      "Velocity command topic used to publish escape motions"),
    BT::InputPort<std::string>(
      "robot_base_frame",
      "base",
      "Robot base frame used to express overlap points"),
    BT::InputPort<int>(
      "clear_confirmations",
      3,
      "Number of consecutive clear observations required before succeeding"),
    BT::InputPort<int>(
      "lethal_cost_threshold",
      nav2_costmap_2d::LETHAL_OBSTACLE,
      "Minimum cell cost treated as a lethal obstacle"),
    BT::InputPort<double>(
      "transform_tolerance",
      0.2,
      "Transform lookup timeout in seconds"),
    BT::InputPort<double>(
      "longitudinal_speed",
      0.25,
      "Absolute longitudinal escape speed in meters per second"),
    BT::InputPort<double>(
      "lateral_speed",
      0.20,
      "Absolute lateral escape speed in meters per second"),
    BT::InputPort<double>(
      "max_escape_duration",
      4.0,
      "Maximum time in seconds to keep this escape stage running before failing over")
  };
}

BT::NodeStatus EscapeObstacleAction::onStart()
{
  clear_tracker_.reset();
  observation_tracker_.reset();
  recovery_start_time_ = node_->get_clock()->now();
  return onRunning();
}

BT::NodeStatus EscapeObstacleAction::onRunning()
{
  EscapeCommand command;
  bool embedded = false;
  bool fresh_observation = false;

  if (!evaluate_escape_command(command, embedded, fresh_observation)) {
    publish_stop_command();
    return BT::NodeStatus::FAILURE;
  }

  if (!embedded) {
    if (fresh_observation && clear_tracker_.update(false)) {
      publish_stop_command();
      return BT::NodeStatus::SUCCESS;
    }

    publish_stop_command();
    return BT::NodeStatus::RUNNING;
  }

  if (!command.valid) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "EscapeObstacle found embedded overlap but no valid escape command");
    publish_stop_command();
    return BT::NodeStatus::FAILURE;
  }

  if (
    max_escape_duration_ > 0.0 &&
    (node_->get_clock()->now() - recovery_start_time_).seconds() >= max_escape_duration_)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "EscapeObstacle exceeded max escape duration of %.2f s",
      max_escape_duration_);
    publish_stop_command();
    return BT::NodeStatus::FAILURE;
  }

  if (fresh_observation) {
    clear_tracker_.update(true);
  }
  publish_command(command);
  return BT::NodeStatus::RUNNING;
}

void EscapeObstacleAction::onHalted()
{
  publish_stop_command();
}

void EscapeObstacleAction::on_costmap_received(
  const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_costmap_ = msg;
  ++costmap_generation_;
}

void EscapeObstacleAction::on_footprint_received(
  const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_footprint_ = msg;
  ++footprint_generation_;
}

bool EscapeObstacleAction::evaluate_escape_command(
  EscapeCommand & command,
  bool & embedded,
  bool & fresh_observation)
{
  nav2_msgs::msg::Costmap::SharedPtr costmap;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint;
  uint64_t costmap_generation = 0;
  uint64_t footprint_generation = 0;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    costmap = latest_costmap_;
    footprint = latest_footprint_;
    costmap_generation = costmap_generation_;
    footprint_generation = footprint_generation_;
  }

  if (!costmap || !footprint) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "EscapeObstacle waiting for both costmap and footprint messages");
    return false;
  }

  fresh_observation = observation_tracker_.update(costmap_generation, footprint_generation);

  try {
    const auto overlap_points = collect_overlap_points_in_base_frame(*costmap, *footprint);
    embedded = !overlap_points.empty();
    command = choose_escape_command(overlap_points, longitudinal_speed_, lateral_speed_);
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "EscapeObstacle skipped evaluation: %s",
      ex.what());
    return false;
  }
}

std::vector<EscapeOverlapPoint> EscapeObstacleAction::collect_overlap_points_in_base_frame(
  const nav2_msgs::msg::Costmap & costmap,
  const geometry_msgs::msg::PolygonStamped & footprint) const
{
  std::vector<EscapeOverlapPoint> overlap_points;
  if (
    costmap.metadata.size_x == 0 || costmap.metadata.size_y == 0 ||
    costmap.metadata.resolution <= 0.0 || costmap.data.empty())
  {
    return overlap_points;
  }

  const auto polygon = get_footprint_in_costmap_frame(footprint, costmap.header.frame_id);
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

  if (max_mx < 0 || max_my < 0 || min_mx >= size_x || min_my >= size_y) {
    return overlap_points;
  }

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

  overlap_points.reserve(static_cast<std::size_t>(max_mx - min_mx + 1) * (max_my - min_my + 1));

  for (int my = min_my; my <= max_my; ++my) {
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      const double cell_center_x = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
      const double cell_center_y = origin_y + (static_cast<double>(my) + 0.5) * resolution;
      if (!point_inside_polygon(cell_center_x, cell_center_y, polygon)) {
        continue;
      }

      const std::size_t index =
        static_cast<std::size_t>(my) * costmap.metadata.size_x + static_cast<std::size_t>(mx);
      if (index >= costmap.data.size() || !cell_is_lethal(costmap, index)) {
        continue;
      }

      overlap_points.push_back({
          tx + cosine * cell_center_x - sine * cell_center_y,
          ty + sine * cell_center_x + cosine * cell_center_y});
    }
  }

  return overlap_points;
}

std::vector<EscapeObstacleAction::Point2D> EscapeObstacleAction::get_footprint_in_costmap_frame(
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

bool EscapeObstacleAction::cell_is_lethal(
  const nav2_msgs::msg::Costmap & costmap,
  std::size_t index) const
{
  const auto cost = costmap.data[index];
  return cost != nav2_costmap_2d::NO_INFORMATION && cost >= lethal_cost_threshold_;
}

bool EscapeObstacleAction::point_inside_polygon(
  double x,
  double y,
  const std::vector<Point2D> & polygon) const
{
  bool inside = false;
  const std::size_t point_count = polygon.size();

  for (std::size_t i = 0, j = point_count - 1; i < point_count; j = i++) {
    const auto & current = polygon[i];
    const auto & previous = polygon[j];

    if (point_on_segment(x, y, previous, current)) {
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

bool EscapeObstacleAction::point_on_segment(
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

void EscapeObstacleAction::publish_command(const EscapeCommand & command)
{
  geometry_msgs::msg::Twist twist = make_stop_twist();
  twist.linear.x = command.vx;
  twist.linear.y = command.vy;
  cmd_vel_pub_->publish(twist);
}

void EscapeObstacleAction::publish_stop_command()
{
  cmd_vel_pub_->publish(make_stop_twist());
}

}  // namespace g1_nav

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<g1_nav::EscapeObstacleAction>("EscapeObstacle");
}
