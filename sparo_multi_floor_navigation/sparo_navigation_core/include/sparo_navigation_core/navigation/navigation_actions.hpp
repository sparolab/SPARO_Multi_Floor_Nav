#ifndef MULTI_FLOOR_NAV2_BT__NAVIGATION_ACTIONS_HPP_
#define MULTI_FLOOR_NAV2_BT__NAVIGATION_ACTIONS_HPP_

#include <string>
#include <vector>
#include <queue>
#include <memory>
#include <atomic>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sparo_navigation_core/config_loader.hpp"

namespace sparo_navigation_core
{

/**
 * @brief BT Action Node to navigate to elevator front using Nav2
 */
class NavigateToElevatorFront : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToElevatorFront(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Current floor number"),
      BT::InputPort<double>("tolerance", 0.5, "Distance tolerance")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool sendGoal();
  bool isGoalReached(double tolerance);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose goal_pose_;
  bool received_pose_;
  double tolerance_;

  GoalHandleNav::SharedPtr goal_handle_;
  bool goal_sent_;
};

/**
 * @brief BT Action Node to enter elevator using Nav2
 */
class EnterElevator : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  EnterElevator(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Current floor number"),
      BT::InputPort<double>("tolerance", 0.3, "Distance tolerance")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool sendGoal();
  bool isGoalReached(double tolerance);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose goal_pose_;
  bool received_pose_;
  double tolerance_;

  GoalHandleNav::SharedPtr goal_handle_;
  bool goal_sent_;
};

/**
 * @brief BT Action Node to exit elevator using Nav2 backup behavior
 */
class ExitElevator : public BT::StatefulActionNode
{
public:
  using BackUp = nav2_msgs::action::BackUp;
  using GoalHandleBackUp = rclcpp_action::ClientGoalHandle<BackUp>;

  ExitElevator(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Target floor number"),
      BT::InputPort<double>("backup_dist", 2.5, "Distance to backup in meters"),
      BT::InputPort<double>("backup_speed", 0.2, "Backup speed in m/s")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_;

  GoalHandleBackUp::SharedPtr goal_handle_;
  std::atomic<bool> goal_completed_{false};
  std::atomic<bool> goal_succeeded_{false};

  double backup_dist_;
  double backup_speed_;
};

/**
 * @brief BT Action Node to turn 180 degrees inside elevator using Nav2
 */
class TurnInElevator : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  TurnInElevator(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("num_waypoints", 8, "Number of intermediate waypoints for turning"),
      BT::InputPort<int>("min_waypoints_to_complete", 5, "Minimum waypoints to reach before considering turn complete")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool sendGoalToPose(const geometry_msgs::msg::Pose& pose);
  void generateTurnWaypoints(int num_points);
  bool sendNextWaypoint();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose start_pose_;
  bool received_pose_;

  GoalHandleNav::SharedPtr goal_handle_;
  std::queue<geometry_msgs::msg::Pose> turn_queue_;
  geometry_msgs::msg::Pose current_target_;
  double tolerance_;

  // State tracking for goal completion
  std::atomic<bool> goal_completed_{false};
  std::atomic<bool> goal_succeeded_{false};
  bool waiting_for_result_{false};

  // Waypoint completion tracking
  int waypoints_reached_{0};
  int min_waypoints_to_complete_{5};
  int total_waypoints_{0};
};

/**
 * @brief BT Action Node to navigate to stairs front using Nav2
 */
class NavigateToStairs : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToStairs(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Current floor number"),
      BT::InputPort<double>("tolerance", 0.5, "Distance tolerance")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool sendGoal();
  bool isGoalReached(double tolerance);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose goal_pose_;
  bool received_pose_;
  double tolerance_;

  GoalHandleNav::SharedPtr goal_handle_;
  bool goal_sent_;
};

}  // namespace sparo_navigation_core

#endif  // MULTI_FLOOR_NAV2_BT__NAVIGATION_ACTIONS_HPP_
