#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>  
#include <yaml-cpp/yaml.h>
#include "sparo_navigation_core/elevator/elevator_client.hpp"

namespace sparo_navigation_core {

struct ElevatorConfig {
  std::string ns;
  std::vector<double> floors;
  double tol = 0.03;
  double settle = 0.8;
};

class CallElevatorToFloor : public BT::StatefulActionNode {
public:
  CallElevatorToFloor(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("elevator_yaml"),
      BT::InputPort<std::string>("elevator_ns"),
      BT::InputPort<int>("floor_idx"),
      BT::InputPort<double>("timeout")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  ElevatorClient client_;
  ElevatorConfig cfg_;

  int target_floor_{0};
  double target_z_{0.0};
  double timeout_{20.0};

  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point settle_t_;
  std::chrono::steady_clock::time_point last_log_;
  bool settle_started_{false};

  std::chrono::steady_clock::time_point warmup_deadline_;

  bool loadConfig(const std::string& yaml_path);
};

} // namespace sparo_navigation_core
