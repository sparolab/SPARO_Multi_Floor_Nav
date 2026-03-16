#ifndef MULTI_FLOOR_NAV2_BT__ELEVATOR_ACTIONS_HPP_
#define MULTI_FLOOR_NAV2_BT__ELEVATOR_ACTIONS_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace sparo_navigation_core
{

/**
 * @brief BT Action Node to call elevator to specific floor
 */
class CallElevator : public BT::SyncActionNode
{
public:
  CallElevator(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("floor_index", "Floor index to call elevator to")
    };
  }

  BT::NodeStatus tick() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr elevator_floor_pub_;
};

/**
 * @brief BT Action Node to open/close elevator door
 */
class OpenElevatorDoor : public BT::SyncActionNode
{
public:
  OpenElevatorDoor(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("open", "true to open door, false to close")
    };
  }

  BT::NodeStatus tick() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr elevator_door_pub_;
};

/**
 * @brief BT Action Node to wait for elevator arrival at target height
 */
class WaitElevatorArrival : public BT::StatefulActionNode
{
public:
  WaitElevatorArrival(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("target_z", "Target Z height"),
      BT::InputPort<double>("tolerance", 0.1, "Distance tolerance"),
      BT::InputPort<double>("timeout", 30.0, "Timeout in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  void elevatorCallback(const std_msgs::msg::Float64::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr elevator_sub_;
  double current_elevator_z_;
  bool received_data_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace sparo_navigation_core

#endif  // MULTI_FLOOR_NAV2_BT__ELEVATOR_ACTIONS_HPP_
