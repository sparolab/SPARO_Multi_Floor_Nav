#ifndef MULTI_FLOOR_NAV2_BT__PATROL_WAYPOINTS_ACTION_HPP_
#define MULTI_FLOOR_NAV2_BT__PATROL_WAYPOINTS_ACTION_HPP_

#include <string>
#include <vector>
#include <queue>
#include <memory>
#include <atomic>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sparo_navigation_core/config_loader.hpp"

namespace sparo_navigation_core
{

/**
 * @brief Stateful BT Action Node for patrolling waypoints using NavigateToPose action
 */
class PatrolWaypointsAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  PatrolWaypointsAction(
    const std::string& name,
    const BT::NodeConfiguration& conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Floor number to patrol"),
      BT::InputPort<std::string>("waypoint_type", "patrol", "Type: patrol or stair_descent"),
      BT::InputPort<bool>("loop", true, "Loop waypoints continuously"),
      BT::InputPort<double>("goal_tolerance", 0.5, "Distance tolerance for waypoint reached")
    };
  }

  void initialize(rclcpp::Node::SharedPtr node);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void loadWaypoints(const std::string& floor);
  void publishWaypointMarkers();
  bool sendGoal();
  bool sendGoalToPose(const geometry_msgs::msg::Pose& pose, const std::string& description);
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool isWaypointReached(double tolerance);

  // Intermediate waypoint functions for smooth turns (legged robot support)
  std::vector<geometry_msgs::msg::Pose> generateIntermediateWaypoints(
    const geometry_msgs::msg::Pose& current,
    const geometry_msgs::msg::Pose& target);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  std::vector<Waypoint> waypoints_;
  size_t current_waypoint_index_;
  std::string current_floor_;
  bool waypoints_loaded_;
  double goal_tolerance_;

  geometry_msgs::msg::Pose current_pose_;
  bool received_pose_;

  GoalHandleNav::SharedPtr goal_handle_;
  bool goal_sent_;

  // Intermediate waypoint state for smooth turns
  std::queue<geometry_msgs::msg::Pose> intermediate_queue_;
  geometry_msgs::msg::Pose current_target_pose_;

  // Goal result tracking
  std::atomic<bool> goal_succeeded_{false};
};

}  // namespace sparo_navigation_core

#endif  // MULTI_FLOOR_NAV2_BT__PATROL_WAYPOINTS_ACTION_HPP_
