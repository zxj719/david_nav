#ifndef G1_NAV__ESCAPE_OBSTACLE_ACTION_HPP_
#define G1_NAV__ESCAPE_OBSTACLE_ACTION_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
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
  int required_clear_observations_{1};
  int clear_observations_{0};
};

class ObservationGenerationTracker
{
public:
  bool update(uint64_t costmap_generation, uint64_t footprint_generation);
  void reset();

private:
  uint64_t last_costmap_generation_{0};
  uint64_t last_footprint_generation_{0};
};

class EscapeObstacleAction : public BT::StatefulActionNode
{
public:
  EscapeObstacleAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);
  ~EscapeObstacleAction() override;

  static BT::PortsList providedPorts();

protected:
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

  bool evaluate_escape_command(
    EscapeCommand & command,
    bool & embedded,
    bool & fresh_observation);
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
  uint64_t costmap_generation_{0};
  uint64_t footprint_generation_{0};

  std::string costmap_topic_;
  std::string footprint_topic_;
  std::string cmd_vel_topic_;
  std::string robot_base_frame_;
  uint8_t lethal_cost_threshold_{254};
  double transform_tolerance_{0.2};
  double longitudinal_speed_{0.25};
  double lateral_speed_{0.20};
  double max_escape_duration_{4.0};
  ClearObservationTracker clear_tracker_{3};
  ObservationGenerationTracker observation_tracker_;
  rclcpp::Time recovery_start_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace g1_nav

#endif  // G1_NAV__ESCAPE_OBSTACLE_ACTION_HPP_
