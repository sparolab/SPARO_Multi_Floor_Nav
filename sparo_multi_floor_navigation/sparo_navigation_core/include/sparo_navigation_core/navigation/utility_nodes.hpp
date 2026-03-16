#ifndef MULTI_FLOOR_NAV2_BT__UTILITY_NODES_HPP_
#define MULTI_FLOOR_NAV2_BT__UTILITY_NODES_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sparo_navigation_core
{

/**
 * @brief Log a message
 */
class LogMessage : public BT::SyncActionNode
{
public:
  LogMessage(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("message", "Message to log")
    };
  }

  BT::NodeStatus tick() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Check if system is initialized (AMCL has pose)
 */
class IsSystemInitialized : public BT::ConditionNode
{
public:
  IsSystemInitialized(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
  bool initialized_;
};

}  // namespace sparo_navigation_core

#endif  // MULTI_FLOOR_NAV2_BT__UTILITY_NODES_HPP_
