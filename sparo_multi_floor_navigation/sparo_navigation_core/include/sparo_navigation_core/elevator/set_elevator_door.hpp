#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "sparo_navigation_core/elevator/elevator_client.hpp"
#include <yaml-cpp/yaml.h>

namespace sparo_navigation_core {

class SetElevatorDoor : public BT::SyncActionNode {
public:
  SetElevatorDoor(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config),
    node_([](){
      static int instance_count = 0;
      return std::make_shared<rclcpp::Node>("set_elevator_door_" + std::to_string(instance_count++));
    }()),
    client_(node_) {}

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("elevator_yaml"),
      BT::InputPort<std::string>("elevator_ns"),
      BT::InputPort<bool>("open")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  ElevatorClient client_;
};

} // namespace sparo_navigation_core
