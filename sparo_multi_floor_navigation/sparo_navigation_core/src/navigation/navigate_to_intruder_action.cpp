#include "sparo_navigation_core/navigation/navigate_to_intruder_action.hpp"
#include <cmath>

namespace sparo_navigation_core
{

NavigateToIntruderAction::NavigateToIntruderAction(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config),
  goal_sent_(false)
{
}

void NavigateToIntruderAction::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  RCLCPP_INFO(node_->get_logger(), "[NavigateToIntruder] Initialized");
}

BT::NodeStatus NavigateToIntruderAction::onStart()
{
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToIntruder"), "Node not initialized!");
    return BT::NodeStatus::FAILURE;
  }

  double x, y, z = 0.0, yaw = 0.0;

  if (!getInput("position_x", x) || !getInput("position_y", y)) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToIntruder] Missing position_x or position_y");
    return BT::NodeStatus::FAILURE;
  }

  getInput("position_z", z);
  getInput("yaw", yaw);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToIntruder] NavigateToPose action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // Create goal
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = z;

  // Set orientation from yaw
  goal.pose.pose.orientation.x = 0.0;
  goal.pose.pose.orientation.y = 0.0;
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

  RCLCPP_INFO(node_->get_logger(),
              "[NavigateToIntruder] Navigating to intruder position: (%.2f, %.2f, %.2f)",
              x, y, z);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr& goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "[NavigateToIntruder] Goal rejected by server");
      } else {
        RCLCPP_INFO(node_->get_logger(), "[NavigateToIntruder] Goal accepted by server");
        goal_handle_ = goal_handle;
      }
    };

  action_client_->async_send_goal(goal, send_goal_options);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToIntruderAction::onRunning()
{
  if (!goal_sent_ || !goal_handle_) {
    return BT::NodeStatus::RUNNING;
  }

  auto status = goal_handle_->get_status();

  if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "[NavigateToIntruder] Reached intruder position");
    goal_sent_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
           status == rclcpp_action::GoalStatus::STATUS_CANCELED) {
    RCLCPP_WARN(node_->get_logger(), "[NavigateToIntruder] Navigation failed or canceled");
    goal_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void NavigateToIntruderAction::onHalted()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "[NavigateToIntruder] Canceling goal");
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_sent_ = false;
}

}  // namespace sparo_navigation_core

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sparo_navigation_core::NavigateToIntruderAction>("NavigateToIntruder");
}
