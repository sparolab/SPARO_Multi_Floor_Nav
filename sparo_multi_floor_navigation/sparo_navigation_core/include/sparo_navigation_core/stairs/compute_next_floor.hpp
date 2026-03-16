#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace sparo_navigation_core
{

class ComputeNextFloor : public BT::SyncActionNode
{
public:
  ComputeNextFloor(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
  {
    static int instance_count = 0;
    node_ = std::make_shared<rclcpp::Node>("compute_next_floor_" + std::to_string(instance_count++));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("current_floor"),
      BT::InputPort<std::string>("direction"),   
      BT::OutputPort<int>("next_floor")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace sparo_navigation_core
