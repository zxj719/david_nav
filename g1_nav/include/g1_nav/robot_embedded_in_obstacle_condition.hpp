#ifndef G1_NAV__ROBOT_EMBEDDED_IN_OBSTACLE_CONDITION_HPP_
#define G1_NAV__ROBOT_EMBEDDED_IN_OBSTACLE_CONDITION_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace g1_nav
{

class RobotEmbeddedInObstacleCondition : public BT::ConditionNode
{
public:
  RobotEmbeddedInObstacleCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ~RobotEmbeddedInObstacleCondition() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  struct Point2D
  {
    double x;
    double y;
  };

  void onCostmapReceived(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void onFootprintReceived(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  bool isRobotEmbedded(
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::PolygonStamped & footprint) const;
  std::vector<Point2D> getFootprintInCostmapFrame(
    const geometry_msgs::msg::PolygonStamped & footprint,
    const std::string & target_frame) const;
  bool cellIsLethal(const nav2_msgs::msg::Costmap & costmap, std::size_t index) const;
  bool pointInsidePolygon(
    double x,
    double y,
    const std::vector<Point2D> & polygon) const;
  bool pointOnSegment(
    double px,
    double py,
    const Point2D & start,
    const Point2D & end) const;
  bool hasFreshObservationLocked(uint64_t costmap_generation, uint64_t footprint_generation);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;

  mutable std::mutex data_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;
  geometry_msgs::msg::PolygonStamped::SharedPtr latest_footprint_;
  uint64_t costmap_generation_{0};
  uint64_t footprint_generation_{0};
  uint64_t last_evaluated_costmap_generation_{0};
  uint64_t last_evaluated_footprint_generation_{0};

  std::string costmap_topic_;
  std::string footprint_topic_;
  int confirmations_required_{3};
  uint8_t lethal_cost_threshold_{254};
  double transform_tolerance_{0.2};
  int consecutive_hits_{0};
};

}  // namespace g1_nav

#endif  // G1_NAV__ROBOT_EMBEDDED_IN_OBSTACLE_CONDITION_HPP_
