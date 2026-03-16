#ifndef SPARO_NAVIGATION_CORE__NAVIGATE_TO_INTRUDER_ACTION_HPP_
#define SPARO_NAVIGATION_CORE__NAVIGATE_TO_INTRUDER_ACTION_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace sparo_navigation_core
{

/**
 * @brief BT Action Node to navigate to intruder position
 * Sends a goal to Nav2 navigate_to_pose action server
 */
class NavigateToIntruderAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToIntruderAction(
    const std::string& name,
    const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("position_x", "Intruder X position"),
      BT::InputPort<double>("position_y", "Intruder Y position"),
      BT::InputPort<double>("position_z", 0.0, "Intruder Z position"),
      BT::InputPort<double>("yaw", 0.0, "Target yaw orientation")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandleNav::SharedPtr goal_handle_;
  bool goal_sent_;
};

}  // namespace sparo_navigation_core

#endif  // SPARO_NAVIGATION_CORE__NAVIGATE_TO_INTRUDER_ACTION_HPP_
