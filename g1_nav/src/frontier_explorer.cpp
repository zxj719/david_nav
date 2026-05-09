#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "g1_nav/exploration_types.hpp"
#include "g1_nav/frontier_blacklist_policy.hpp"
#include "g1_nav/frontier_completion_policy.hpp"
#include "g1_nav/frontier_goal_selector.hpp"
#include "g1_nav/frontier_navigation_result_policy.hpp"
#include "g1_nav/frontier_search.hpp"

namespace
{

struct BlacklistedPoint
{
  double x{0.0};
  double y{0.0};
};

struct CompletedPoint
{
  double x{0.0};
  double y{0.0};
};

struct PendingNavigationGoal
{
  std::optional<std::pair<double, double>> frontier_xy;
  std::pair<double, double> goal_xy;
  std::optional<double> yaw;
  bool returning_to_initial_pose{false};
};

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quat;
  quat.z = std::sin(0.5 * yaw);
  quat.w = std::cos(0.5 * yaw);
  return quat;
}

std::string normalize_frame_id(const std::string & frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quat)
{
  const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double distance_xy(
  const std::pair<double, double> & a,
  const std::pair<double, double> & b)
{
  return std::hypot(a.first - b.first, a.second - b.second);
}

geometry_msgs::msg::Point make_marker_point(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

}  // namespace

class FrontierExplorerNode : public rclcpp::Node
{
public:
  using Frontier = g1_nav::Frontier;
  using FrontierTarget = g1_nav::FrontierTarget;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavTo = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  FrontierExplorerNode()
  : rclcpp::Node("frontier_explorer")
  {
    declare_parameter("planner_frequency", 0.2);
    declare_parameter("min_frontier_size", 20);
    declare_parameter("search_free_threshold", 50);
    declare_parameter("costmap_search_threshold", 20);
    declare_parameter("robot_base_frame", "base");
    declare_parameter("odom_frame", "odom");
    declare_parameter("odom_topic", "/lightning/odometry");
    declare_parameter("global_frame", "map");
    declare_parameter("map_topic", "/lightning/grid_map");
    declare_parameter("global_costmap_topic", "/global_costmap/costmap");
    declare_parameter("live_map_ready_topic", "/map_live_ready");
    declare_parameter("require_live_map", true);
    declare_parameter("transform_tolerance", 2.0);
    declare_parameter("blacklist_radius", 0.5);
    declare_parameter("progress_timeout", 30.0);
    declare_parameter("visualize", true);
    declare_parameter("frontier_update_radius", 0.0);
    declare_parameter("return_to_init", false);
    declare_parameter("frontier_snap_radius", 1.0);
    declare_parameter("frontier_fallback_snap_radius", 1.0);
    declare_parameter("goal_clearance_radius", 0.35);
    declare_parameter("goal_update_min_distance", 0.4);
    declare_parameter("frontier_distance_weight", 1.0);
    declare_parameter("frontier_size_weight", 0.0);

    planner_freq_ = std::max(0.05, get_parameter("planner_frequency").as_double());
    frontier_search_config_.min_frontier_size = std::max(
      1, static_cast<int>(get_parameter("min_frontier_size").as_int()));
    frontier_search_config_.search_free_threshold = std::clamp(
      static_cast<int>(get_parameter("search_free_threshold").as_int()), 0, 100);
    frontier_search_config_.costmap_search_threshold = std::clamp(
      static_cast<int>(get_parameter("costmap_search_threshold").as_int()), 0, 100);
    frontier_search_config_.frontier_update_radius = std::max(
      0.0, get_parameter("frontier_update_radius").as_double());
    frontier_search_config_.frontier_distance_weight = std::max(
      0.0, get_parameter("frontier_distance_weight").as_double());
    frontier_search_config_.frontier_size_weight = std::max(
      0.0, get_parameter("frontier_size_weight").as_double());
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    global_costmap_topic_ = get_parameter("global_costmap_topic").as_string();
    live_map_ready_topic_ = get_parameter("live_map_ready_topic").as_string();
    require_live_map_ = get_parameter("require_live_map").as_bool();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();
    blacklist_radius_ = std::max(0.05, get_parameter("blacklist_radius").as_double());
    progress_timeout_ = std::max(1.0, get_parameter("progress_timeout").as_double());
    visualize_ = get_parameter("visualize").as_bool();
    return_to_init_ = get_parameter("return_to_init").as_bool();
    frontier_snap_radius_ = std::max(
      0.1, get_parameter("frontier_snap_radius").as_double());
    frontier_fallback_snap_radius_ = std::max(
      frontier_snap_radius_, get_parameter("frontier_fallback_snap_radius").as_double());
    goal_clearance_radius_ = std::max(
      0.0, get_parameter("goal_clearance_radius").as_double());
    goal_update_min_distance_ = std::max(
      0.05, get_parameter("goal_update_min_distance").as_double());

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    rclcpp::QoS tf_qos(rclcpp::KeepLast(100));
    tf_qos.best_effort();
    tf_qos.durability_volatile();

    rclcpp::QoS tf_static_qos(rclcpp::KeepLast(100));
    tf_static_qos.reliable();
    tf_static_qos.transient_local();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&FrontierExplorerNode::map_callback, this, std::placeholders::_1));
    global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      global_costmap_topic_, map_qos,
      std::bind(&FrontierExplorerNode::global_costmap_callback, this, std::placeholders::_1));
    live_map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      live_map_ready_topic_, map_qos,
      std::bind(&FrontierExplorerNode::live_map_ready_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&FrontierExplorerNode::odom_callback, this, std::placeholders::_1));
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf_qos,
      std::bind(&FrontierExplorerNode::tf_callback, this, std::placeholders::_1));
    tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf_static_qos,
      std::bind(&FrontierExplorerNode::tf_callback, this, std::placeholders::_1));
    resume_sub_ = create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&FrontierExplorerNode::resume_callback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    if (visualize_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "explore/frontiers", 10);
    }

    const auto timer_period =
      std::chrono::duration<double>(1.0 / planner_freq_);
    plan_timer_ = create_wall_timer(
      timer_period, std::bind(&FrontierExplorerNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Frontier explorer ready: map_topic=%s global_costmap_topic=%s odom_topic=%s snap_radius=%.2fm fallback_snap=%.2fm clearance=%.2fm",
      map_topic_.c_str(), global_costmap_topic_.c_str(), odom_topic_.c_str(),
      frontier_snap_radius_, frontier_fallback_snap_radius_, goal_clearance_radius_);
  }

private:
  void timer_callback()
  {
    auto robot_xy = get_robot_position();
    if (robot_xy && !initial_pose_) {
      initial_pose_ = robot_xy;
    }

    if (navigating_ && robot_xy) {
      check_progress(*robot_xy);
    }

    if (!exploring_) {
      return;
    }

    if (require_live_map_ && !live_map_ready_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for live map readiness before frontier planning");
      return;
    }

    if (!map_msg_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for map data on %s", map_topic_.c_str());
      return;
    }

    if (!global_costmap_msg_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for global costmap data on %s", global_costmap_topic_.c_str());
      return;
    }

    if (!robot_xy) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for robot pose (%s -> %s -> %s)",
        global_frame_.c_str(), odom_frame_.c_str(), robot_base_frame_.c_str());
      return;
    }

    const auto grid = make_grid_view();
    const auto global_costmap = make_global_costmap_view();
    if (!grid.valid() || !global_costmap.valid()) {
      return;
    }

    std::vector<double> dummy_dist_map(map_msg_->data.size(), 0.0);
    const auto observed_frontiers =
      g1_nav::FrontierSearch::search(
      grid, global_costmap, *robot_xy, dummy_dist_map, frontier_search_config_);

    auto active_frontiers = filter_frontiers(observed_frontiers);
    if (g1_nav::should_recover_blacklist(active_frontiers.size(), blacklisted_.size())) {
      RCLCPP_INFO(
        get_logger(),
        "No available frontier after blacklist filtering, clearing blacklist to retry");
      blacklisted_.clear();
      active_frontiers = filter_frontiers(observed_frontiers);
    }

    g1_nav::FrontierGoalSelectorConfig selector_config;
    selector_config.snap_radius = frontier_snap_radius_;
    selector_config.fallback_snap_radius = frontier_fallback_snap_radius_;
    selector_config.goal_clearance_radius = goal_clearance_radius_;

    std::vector<FrontierTarget> targets;
    targets.reserve(active_frontiers.size());
    std::vector<Frontier> rejected_frontiers;
    rejected_frontiers.reserve(active_frontiers.size());
    for (const auto & frontier : active_frontiers) {
      const auto target = g1_nav::select_frontier_target(
        grid, global_costmap, frontier, *robot_xy, selector_config);
      if (!target) {
        rejected_frontiers.push_back(frontier);
        continue;
      }
      targets.push_back(*target);
    }

    g1_nav::sort_frontier_targets_for_selection(
      targets,
      *robot_xy,
      frontier_search_config_.frontier_distance_weight,
      frontier_search_config_.frontier_size_weight);
    targets = g1_nav::suppress_frontier_targets_by_radius(targets, frontier_snap_radius_);

    for (const auto & frontier : rejected_frontiers) {
      const bool suppressed_by_live_target = std::any_of(
        targets.begin(), targets.end(),
        [&frontier, this](const FrontierTarget & target) {
          return std::hypot(
            frontier.centroid_x - target.anchor_xy.first,
            frontier.centroid_y - target.anchor_xy.second) <= frontier_snap_radius_;
        });
      if (suppressed_by_live_target) {
        continue;
      }

      blacklist_point(
        frontier.centroid_x,
        frontier.centroid_y,
        "No admissible navigation goal on the snap ring");
    }

    // Targets are built from the full current frontier set each cycle.
    // Pick the highest-ranked target that is not already reached.
    std::optional<FrontierTarget> selected_target;
    for (const auto & target : targets) {
      if (g1_nav::should_complete_candidate_frontier_by_proximity(
          current_frontier_,
          target.anchor_xy,
          *robot_xy,
          frontier_reached_radius(),
          frontier_completion_match_radius()))
      {
        complete_frontier(target.anchor_xy.first, target.anchor_xy.second);
        continue;
      }

      selected_target = target;
      break;
    }

    if (visualize_) {
      if (observed_frontiers.empty() && targets.empty()) {
        clear_frontier_markers();
      } else {
        publish_frontier_markers(grid, observed_frontiers, targets, selected_target);
      }
    }

    if (!g1_nav::should_accept_new_frontier_goal_while_navigating(navigating_)) {
      return;
    }

    if (selected_target) {
      const auto & target = *selected_target;
      current_frontier_ = target.anchor_xy;
      current_frontier_retry_used_ = false;
      navigate_to(
        target.goal_xy.first,
        target.goal_xy.second,
        compute_goal_yaw(*robot_xy, target.goal_xy));
      return;
    }

    if (return_to_init_ && initial_pose_ && !returned_to_initial_pose_) {
      const double dist_home = distance_xy(*robot_xy, *initial_pose_);
      if (dist_home > 0.3) {
        if (navigating_) {
          request_navigation_preempt(
            std::nullopt, *initial_pose_, std::nullopt, true);
        } else {
          current_frontier_.reset();
          returning_to_initial_pose_ = true;
          navigate_to(initial_pose_->first, initial_pose_->second);
        }
        return;
      }
      returned_to_initial_pose_ = true;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No reachable frontier is currently available");
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = msg;
    if (!live_map_ready_) {
      live_map_ready_ = true;
    }
  }

  void global_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    global_costmap_msg_ = msg;
  }

  void live_map_ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    live_map_ready_ = msg->data;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto frame_id = normalize_frame_id(msg->header.frame_id);
    if (!frame_id.empty() && frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring odom pose in unexpected frame '%s'; expected '%s'",
        frame_id.c_str(), odom_frame_.c_str());
      return;
    }

    odom_pose_ = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
    odom_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
    odom_stamp_ = msg->header.stamp;
    update_robot_pose_cache();
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & transform : msg->transforms) {
      const auto parent = normalize_frame_id(transform.header.frame_id);
      const auto child = normalize_frame_id(transform.child_frame_id);
      if (parent == global_frame_ && child == odom_frame_) {
        store_map_to_odom(
          transform.transform.translation.x,
          transform.transform.translation.y,
          yaw_from_quaternion(transform.transform.rotation),
          transform.header.stamp);
      } else if (parent == odom_frame_ && child == global_frame_) {
        const double tx = transform.transform.translation.x;
        const double ty = transform.transform.translation.y;
        const double yaw = yaw_from_quaternion(transform.transform.rotation);
        const double cos_y = std::cos(yaw);
        const double sin_y = std::sin(yaw);
        store_map_to_odom(
          -(cos_y * tx + sin_y * ty),
          sin_y * tx - cos_y * ty,
          -yaw,
          transform.header.stamp);
      }
    }
  }

  void resume_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    exploring_ = msg->data;
    if (!exploring_) {
      RCLCPP_INFO(get_logger(), "Exploration paused");
      return;
    }
    RCLCPP_INFO(get_logger(), "Exploration resumed");
  }

  void nav_feedback_callback(const geometry_msgs::msg::PoseStamped & current_pose)
  {
    const auto frame_id = normalize_frame_id(current_pose.header.frame_id);
    if (!frame_id.empty() && frame_id != global_frame_) {
      return;
    }
    robot_pose_ = std::make_pair(
      current_pose.pose.position.x,
      current_pose.pose.position.y);
    robot_yaw_ = yaw_from_quaternion(current_pose.pose.orientation);
    robot_pose_stamp_ = current_pose.header.stamp;
  }

  void navigate_to(double x, double y, std::optional<double> yaw = std::nullopt)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    if (yaw) {
      goal.pose.pose.orientation = yaw_to_quaternion(*yaw);
    } else {
      goal.pose.pose.orientation.w = 1.0;
    }

    ++goal_seq_;
    const int seq = goal_seq_;

    typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, seq](GoalHandleNavTo::SharedPtr handle) {
        goal_response_callback(handle, seq);
      };
    options.feedback_callback =
      [this](
      GoalHandleNavTo::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        nav_feedback_callback(feedback->current_pose);
      };
    options.result_callback =
      [this, seq](const GoalHandleNavTo::WrappedResult & result) {
        navigation_result_callback(result, seq);
      };

    nav_client_->async_send_goal(goal, options);

    current_goal_ = std::make_pair(x, y);
    navigating_ = true;
    cancel_requested_ = false;
    progress_timeout_cancel_requested_ = false;
    last_progress_time_ = now();
    last_robot_pos_ = get_robot_position();

    RCLCPP_INFO(get_logger(), "Sending frontier goal: (%.2f, %.2f)", x, y);
  }

  void request_navigation_preempt(
    const std::optional<std::pair<double, double>> & frontier_xy,
    const std::pair<double, double> & goal_xy,
    std::optional<double> yaw,
    bool returning_to_initial_pose)
  {
    pending_goal_ = PendingNavigationGoal{frontier_xy, goal_xy, yaw, returning_to_initial_pose};

    if (preempt_requested_ && cancel_requested_) {
      return;
    }

    if (!navigating_) {
      dispatch_pending_goal();
      return;
    }

    preempt_requested_ = true;
    cancel_current_goal();
  }

  void dispatch_pending_goal()
  {
    if (!pending_goal_) {
      return;
    }

    const auto pending_goal = *pending_goal_;
    pending_goal_.reset();
    current_frontier_ = pending_goal.frontier_xy;
    current_frontier_retry_used_ = false;
    returning_to_initial_pose_ = pending_goal.returning_to_initial_pose;
    navigate_to(
      pending_goal.goal_xy.first,
      pending_goal.goal_xy.second,
      pending_goal.yaw);
  }

  void goal_response_callback(GoalHandleNavTo::SharedPtr handle, int seq)
  {
    if (seq != goal_seq_) {
      return;
    }

    if (!handle) {
      RCLCPP_WARN(get_logger(), "Goal rejected by Nav2");
      blacklist_current_frontier("Goal rejected by Nav2");
      clear_navigation_state();
      return;
    }

    nav_goal_handle_ = handle;
    RCLCPP_INFO(get_logger(), "Goal accepted by Nav2");
    if (preempt_requested_ && pending_goal_) {
      cancel_current_goal();
    }
  }

  void navigation_result_callback(const GoalHandleNavTo::WrappedResult & result, int seq)
  {
    if (seq != goal_seq_) {
      return;
    }

    const bool handoff_in_progress = preempt_requested_ && pending_goal_.has_value();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      if (returning_to_initial_pose_) {
        returned_to_initial_pose_ = true;
        RCLCPP_INFO(get_logger(), "Returned to the initial pose");
      } else if (current_frontier_) {
        complete_frontier(current_frontier_->first, current_frontier_->second);
        RCLCPP_INFO(get_logger(), "Frontier goal reached");
      } else {
        RCLCPP_INFO(get_logger(), "Navigation succeeded");
      }
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      if (g1_nav::should_blacklist_on_navigation_abort(
          preempt_requested_, pending_goal_.has_value()))
      {
        if (retry_current_frontier("Navigation aborted by Nav2")) {
          return;
        }
        RCLCPP_WARN(get_logger(), "Navigation aborted by Nav2");
        blacklist_current_frontier("Navigation aborted by Nav2");
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Navigation aborted while handing off to a new goal; keeping current frontier unblacklisted");
      }
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      if (preempt_requested_) {
        RCLCPP_INFO(get_logger(), "Navigation canceled for goal handoff");
      } else if (progress_timeout_cancel_requested_) {
        if (retry_current_frontier("Navigation canceled after progress timeout")) {
          return;
        }
        RCLCPP_WARN(get_logger(), "Navigation canceled after progress timeout");
        blacklist_current_frontier("No progress timeout");
      } else {
        RCLCPP_INFO(get_logger(), "Navigation canceled");
      }
    } else {
      RCLCPP_WARN(
        get_logger(), "Navigation ended with result code %d",
        static_cast<int>(result.code));
    }

    const bool should_dispatch_pending = handoff_in_progress;
    clear_navigation_state();
    if (should_dispatch_pending) {
      dispatch_pending_goal();
    }
  }

  void check_progress(const std::pair<double, double> & robot_xy)
  {
    if (!last_progress_time_) {
      last_progress_time_ = now();
    }

    if (!last_robot_pos_) {
      last_robot_pos_ = robot_xy;
      return;
    }

    if (distance_xy(robot_xy, *last_robot_pos_) > 0.15) {
      last_robot_pos_ = robot_xy;
      last_progress_time_ = now();
      return;
    }

    if (cancel_requested_ || !last_progress_time_) {
      return;
    }

    const double elapsed = (now() - *last_progress_time_).seconds();
    if (elapsed <= progress_timeout_) {
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "No progress for %.0fs - canceling the current frontier goal",
      elapsed);
    progress_timeout_cancel_requested_ = true;
    cancel_current_goal();
  }

  bool retry_current_frontier(const std::string & reason)
  {
    if (!current_frontier_) {
      return false;
    }

    const auto robot_xy = get_robot_position();
    if (!robot_xy) {
      return false;
    }

    const auto retry_target = build_retry_target_for_current_frontier(*robot_xy);
    if (!g1_nav::should_retry_same_frontier_after_failure(
        current_frontier_.has_value(),
        current_frontier_retry_used_,
        retry_target.has_value()))
    {
      return false;
    }

    RCLCPP_WARN(
      get_logger(),
      "%s; retrying the current frontier once",
      reason.c_str());

    const auto target = *retry_target;
    const auto yaw = compute_goal_yaw(*robot_xy, target.goal_xy);

    clear_navigation_state();
    current_frontier_retry_used_ = true;
    current_frontier_ = target.anchor_xy;
    navigate_to(target.goal_xy.first, target.goal_xy.second, yaw);
    return true;
  }

  bool should_preempt_navigation(
    const std::pair<double, double> & frontier_xy,
    const std::pair<double, double> & goal_xy) const
  {
    if (!navigating_) {
      return true;
    }

    if (cancel_requested_ && !preempt_requested_) {
      return false;
    }

    if (returning_to_initial_pose_ || !current_frontier_ || !current_goal_) {
      return true;
    }

    return distance_xy(frontier_xy, *current_frontier_) > goal_update_min_distance_ ||
           distance_xy(goal_xy, *current_goal_) > goal_update_min_distance_;
  }

  void cancel_current_goal()
  {
    if (!nav_goal_handle_) {
      if (!preempt_requested_) {
        clear_navigation_state();
      }
      return;
    }
    if (cancel_requested_) {
      return;
    }
    cancel_requested_ = true;
    nav_client_->async_cancel_goal(nav_goal_handle_);
  }

  void clear_navigation_state()
  {
    navigating_ = false;
    cancel_requested_ = false;
    progress_timeout_cancel_requested_ = false;
    preempt_requested_ = false;
    current_frontier_retry_used_ = false;
    nav_goal_handle_.reset();
    current_goal_.reset();
    current_frontier_.reset();
    returning_to_initial_pose_ = false;
    last_progress_time_.reset();
    last_robot_pos_.reset();
  }

  std::optional<FrontierTarget> build_retry_target_for_current_frontier(
    const std::pair<double, double> & robot_xy) const
  {
    if (!current_frontier_) {
      return std::nullopt;
    }

    const auto grid = make_grid_view();
    const auto global_costmap = make_global_costmap_view();
    if (!grid.valid() || !global_costmap.valid()) {
      return std::nullopt;
    }

    Frontier retry_frontier;
    retry_frontier.size = 1;
    retry_frontier.centroid_x = current_frontier_->first;
    retry_frontier.centroid_y = current_frontier_->second;
    retry_frontier.min_distance = distance_xy(robot_xy, *current_frontier_);
    retry_frontier.heuristic_distance = retry_frontier.min_distance;

    g1_nav::FrontierGoalSelectorConfig selector_config;
    selector_config.snap_radius = frontier_snap_radius_;
    selector_config.fallback_snap_radius = frontier_fallback_snap_radius_;
    selector_config.goal_clearance_radius = goal_clearance_radius_;

    return g1_nav::select_frontier_target(
      grid, global_costmap, retry_frontier, robot_xy, selector_config);
  }

  g1_nav::GridMapView make_grid_view() const
  {
    g1_nav::GridMapView grid;
    if (!map_msg_) {
      return grid;
    }
    grid.cells = &map_msg_->data;
    grid.width = static_cast<int>(map_msg_->info.width);
    grid.height = static_cast<int>(map_msg_->info.height);
    grid.resolution = map_msg_->info.resolution;
    grid.origin_x = map_msg_->info.origin.position.x;
    grid.origin_y = map_msg_->info.origin.position.y;
    return grid;
  }

  g1_nav::GridMapView make_global_costmap_view() const
  {
    g1_nav::GridMapView grid;
    if (!global_costmap_msg_) {
      return grid;
    }
    grid.cells = &global_costmap_msg_->data;
    grid.width = static_cast<int>(global_costmap_msg_->info.width);
    grid.height = static_cast<int>(global_costmap_msg_->info.height);
    grid.resolution = global_costmap_msg_->info.resolution;
    grid.origin_x = global_costmap_msg_->info.origin.position.x;
    grid.origin_y = global_costmap_msg_->info.origin.position.y;
    return grid;
  }

  void store_map_to_odom(
    double tx,
    double ty,
    double yaw,
    const builtin_interfaces::msg::Time & stamp)
  {
    map_to_odom_xy_ = std::make_pair(tx, ty);
    map_to_odom_yaw_ = yaw;
    map_to_odom_stamp_ = stamp;
    update_robot_pose_cache();
  }

  void update_robot_pose_cache()
  {
    if (!map_to_odom_xy_ || !map_to_odom_yaw_ || !odom_pose_) {
      return;
    }

    const double cos_y = std::cos(*map_to_odom_yaw_);
    const double sin_y = std::sin(*map_to_odom_yaw_);
    robot_pose_ = std::make_pair(
      map_to_odom_xy_->first + cos_y * odom_pose_->first - sin_y * odom_pose_->second,
      map_to_odom_xy_->second + sin_y * odom_pose_->first + cos_y * odom_pose_->second);
    if (odom_yaw_) {
      robot_yaw_ = *map_to_odom_yaw_ + *odom_yaw_;
    }
    robot_pose_stamp_ = odom_stamp_;
  }

  std::optional<std::pair<double, double>> get_robot_position() const
  {
    return robot_pose_;
  }

  std::optional<double> compute_goal_yaw(
    const std::pair<double, double> & robot_xy,
    const std::pair<double, double> & goal_xy) const
  {
    const double dx = goal_xy.first - robot_xy.first;
    const double dy = goal_xy.second - robot_xy.second;
    if (std::hypot(dx, dy) < 1e-6) {
      return std::nullopt;
    }
    return std::atan2(dy, dx);
  }

  double frontier_reached_radius() const
  {
    return std::max(0.35, 2.0 * blacklist_radius_);
  }

  double frontier_completion_match_radius() const
  {
    // Completion history should only absorb small centroid drift for the
    // same frontier anchor. Using the full reached radius here would hide
    // legitimately new nearby frontiers after the robot arrives.
    return std::max(0.10, 0.5 * frontier_snap_radius_);
  }

  std::vector<Frontier> filter_frontiers(const std::vector<Frontier> & snapped_frontiers) const
  {
    std::vector<Frontier> active_frontiers;
    active_frontiers.reserve(snapped_frontiers.size());
    for (const auto & frontier : snapped_frontiers) {
      if (is_blacklisted(frontier.centroid_x, frontier.centroid_y) ||
        is_completed(frontier.centroid_x, frontier.centroid_y))
      {
        continue;
      }
      active_frontiers.push_back(frontier);
    }
    return active_frontiers;
  }

  void blacklist_current_frontier(const std::string & reason)
  {
    if (!current_frontier_) {
      return;
    }
    blacklist_point(current_frontier_->first, current_frontier_->second, reason);
  }

  void blacklist_point(double x, double y, const std::string & reason)
  {
    if (is_blacklisted(x, y)) {
      return;
    }
    blacklisted_.push_back({x, y});
    RCLCPP_WARN(
      get_logger(),
      "Blacklisting frontier (%.2f, %.2f): %s",
      x, y, reason.c_str());
  }

  void complete_frontier(double x, double y)
  {
    if (is_completed(x, y)) {
      return;
    }
    completed_.push_back({x, y});
  }

  bool is_blacklisted(double x, double y) const
  {
    for (const auto & item : blacklisted_) {
      if (std::hypot(item.x - x, item.y - y) <= blacklist_radius_) {
        return true;
      }
    }
    return false;
  }

  bool is_completed(double x, double y) const
  {
    const std::pair<double, double> frontier_xy{x, y};
    for (const auto & item : completed_) {
      if (g1_nav::is_same_frontier_anchor(
          frontier_xy,
          std::pair<double, double>{item.x, item.y},
          frontier_completion_match_radius()))
      {
        return true;
      }
    }
    return false;
  }

  void clear_frontier_markers()
  {
    if (!marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    marker_pub_->publish(markers);
  }

  void publish_frontier_markers(
    const g1_nav::GridMapView & grid,
    const std::vector<Frontier> & observed_frontiers,
    const std::vector<FrontierTarget> & targets,
    const std::optional<FrontierTarget> & chosen)
  {
    if (!marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);

    const auto stamp = now();
    int marker_id = 1;

    visualization_msgs::msg::Marker frontier_cells_marker;
    frontier_cells_marker.header.frame_id = global_frame_;
    frontier_cells_marker.header.stamp = stamp;
    frontier_cells_marker.ns = "frontier_cells";
    frontier_cells_marker.id = marker_id++;
    frontier_cells_marker.type = visualization_msgs::msg::Marker::POINTS;
    frontier_cells_marker.action = visualization_msgs::msg::Marker::ADD;
    frontier_cells_marker.pose.orientation.w = 1.0;
    frontier_cells_marker.scale.x = std::max(0.03, grid.resolution * 0.45);
    frontier_cells_marker.scale.y = std::max(0.03, grid.resolution * 0.45);
    frontier_cells_marker.color.r = 1.0;
    frontier_cells_marker.color.g = 0.6;
    frontier_cells_marker.color.b = 0.0;
    frontier_cells_marker.color.a = 0.85;
    frontier_cells_marker.lifetime = rclcpp::Duration::from_seconds(5.0);

    visualization_msgs::msg::Marker frontier_centroids_marker;
    frontier_centroids_marker.header.frame_id = global_frame_;
    frontier_centroids_marker.header.stamp = stamp;
    frontier_centroids_marker.ns = "frontier_centroids";
    frontier_centroids_marker.id = marker_id++;
    frontier_centroids_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    frontier_centroids_marker.action = visualization_msgs::msg::Marker::ADD;
    frontier_centroids_marker.pose.orientation.w = 1.0;
    frontier_centroids_marker.scale.x = 0.12;
    frontier_centroids_marker.scale.y = 0.12;
    frontier_centroids_marker.scale.z = 0.12;
    frontier_centroids_marker.color.r = 1.0;
    frontier_centroids_marker.color.g = 0.15;
    frontier_centroids_marker.color.b = 0.15;
    frontier_centroids_marker.color.a = 0.9;
    frontier_centroids_marker.lifetime = rclcpp::Duration::from_seconds(5.0);

    for (const auto & frontier : observed_frontiers) {
      for (const auto & [cell_x, cell_y] : frontier.cells) {
        const auto [wx, wy] = grid.cell_center_world(cell_x, cell_y);
        frontier_cells_marker.points.push_back(make_marker_point(wx, wy, 0.02));
      }
      frontier_centroids_marker.points.push_back(
        make_marker_point(frontier.centroid_x, frontier.centroid_y, 0.08));
    }

    markers.markers.push_back(frontier_cells_marker);
    markers.markers.push_back(frontier_centroids_marker);

    for (const auto & target : targets) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = stamp;
      marker.ns = "frontier_goals";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = target.goal_xy.first;
      marker.pose.position.y = target.goal_xy.second;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.18;
      marker.scale.y = 0.18;
      marker.scale.z = 0.18;

      const bool selected = chosen.has_value() &&
        std::hypot(
        target.goal_xy.first - chosen->goal_xy.first,
        target.goal_xy.second - chosen->goal_xy.second) <= 1e-4;
      if (selected) {
        marker.color.g = 1.0;
      } else {
        marker.color.r = 0.2;
        marker.color.g = 0.4;
        marker.color.b = 1.0;
      }
      marker.color.a = 0.85;
      marker.lifetime = rclcpp::Duration::from_seconds(5.0);
      markers.markers.push_back(marker);
    }

    marker_pub_->publish(markers);
  }

  g1_nav::FrontierSearchConfig frontier_search_config_;

  double planner_freq_{0.2};
  std::string robot_base_frame_{"base"};
  std::string odom_frame_{"odom"};
  std::string odom_topic_{"/lightning/odometry"};
  std::string global_frame_{"map"};
  std::string map_topic_{"/lightning/grid_map"};
  std::string global_costmap_topic_{"/global_costmap/costmap"};
  std::string live_map_ready_topic_{"/map_live_ready"};
  bool require_live_map_{true};
  double transform_tolerance_{2.0};
  double blacklist_radius_{0.5};
  double progress_timeout_{30.0};
  bool visualize_{true};
  bool return_to_init_{false};
  double frontier_snap_radius_{1.0};
  double frontier_fallback_snap_radius_{1.0};
  double goal_clearance_radius_{0.35};
  double goal_update_min_distance_{0.4};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr live_map_ready_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNavTo::SharedPtr nav_goal_handle_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  nav_msgs::msg::OccupancyGrid::SharedPtr global_costmap_msg_;
  bool live_map_ready_{false};
  bool exploring_{true};
  bool navigating_{false};
  bool cancel_requested_{false};
  bool progress_timeout_cancel_requested_{false};
  bool returning_to_initial_pose_{false};
  bool returned_to_initial_pose_{false};
  bool preempt_requested_{false};
  bool current_frontier_retry_used_{false};
  int goal_seq_{0};

  std::optional<std::pair<double, double>> odom_pose_;
  std::optional<double> odom_yaw_;
  builtin_interfaces::msg::Time odom_stamp_;
  std::optional<std::pair<double, double>> map_to_odom_xy_;
  std::optional<double> map_to_odom_yaw_;
  builtin_interfaces::msg::Time map_to_odom_stamp_;
  std::optional<std::pair<double, double>> robot_pose_;
  std::optional<double> robot_yaw_;
  builtin_interfaces::msg::Time robot_pose_stamp_;
  std::optional<std::pair<double, double>> current_frontier_;
  std::optional<std::pair<double, double>> current_goal_;
  std::optional<PendingNavigationGoal> pending_goal_;
  std::optional<std::pair<double, double>> last_robot_pos_;
  std::optional<rclcpp::Time> last_progress_time_;
  std::optional<std::pair<double, double>> initial_pose_;

  std::vector<BlacklistedPoint> blacklisted_;
  std::vector<CompletedPoint> completed_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierExplorerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
