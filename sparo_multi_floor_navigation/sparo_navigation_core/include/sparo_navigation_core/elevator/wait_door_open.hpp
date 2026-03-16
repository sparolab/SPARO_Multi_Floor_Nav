#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include "sparo_navigation_core/elevator/elevator_client.hpp"

namespace sparo_navigation_core {

class WaitDoorOpen : public BT::StatefulActionNode {
public:
  WaitDoorOpen(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("elevator_yaml"),
      BT::InputPort<std::string>("elevator_ns"),
      BT::InputPort<bool>("want_open"),
      BT::InputPort<double>("open_threshold"), 
      BT::InputPort<double>("timeout")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool loadYaml(const std::string& path, std::string& ns_out);

  rclcpp::Node::SharedPtr node_;
  ElevatorClient client_;

  double threshold_{0.5};     // |door_a| + |door_b|
  bool   want_open_{true};
  double timeout_{20.0};
  double settle_sec_{0.5};    // YAML: door_settle_sec

  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point settle_t_;
  bool settle_started_{false};
  std::chrono::steady_clock::time_point last_log_;
};

} // namespace sparo_navigation_core
