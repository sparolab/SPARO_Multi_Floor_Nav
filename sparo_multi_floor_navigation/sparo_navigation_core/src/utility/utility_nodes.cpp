#include "sparo_navigation_core/utility/utility_nodes.hpp"

namespace sparo_navigation_core
{

// ============== LogMessage ==============
LogMessage::LogMessage(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

void LogMessage::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
}

BT::NodeStatus LogMessage::tick()
{
  std::string message;
  if (!getInput<std::string>("message", message)) {
    return BT::NodeStatus::FAILURE;
  }

  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "[BT] %s", message.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("LogMessage"), "[BT] %s", message.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

// ============== IsSystemInitialized ==============
IsSystemInitialized::IsSystemInitialized(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config),
    initialized_(false)
{
}

void IsSystemInitialized::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  // Check parameter for initialization state (like original version)
  if (!node_->has_parameter("system_initialized")) {
    initialized_ = node_->declare_parameter("system_initialized", false);
  } else {
    node_->get_parameter("system_initialized", initialized_);
  }
}

BT::NodeStatus IsSystemInitialized::tick()
{
  // Re-check parameter each tick
  if (node_) {
    node_->get_parameter("system_initialized", initialized_);
  }

  if (initialized_) {
    RCLCPP_INFO(rclcpp::get_logger("IsSystemInitialized"), "System is initialized");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("IsSystemInitialized"), "System is NOT initialized");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace sparo_navigation_core
