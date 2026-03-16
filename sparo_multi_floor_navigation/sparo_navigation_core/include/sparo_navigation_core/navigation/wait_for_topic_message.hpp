#ifndef SPARO_NAVIGATION_CORE__WAIT_FOR_TOPIC_MESSAGE_HPP_
#define SPARO_NAVIGATION_CORE__WAIT_FOR_TOPIC_MESSAGE_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace sparo_navigation_core
{

/**
 * @brief Wait for a message on a topic and extract elevator info
 */
class WaitForTopicMessage : public BT::StatefulActionNode
{
public:
  WaitForTopicMessage(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/floor_change_complete", "Topic to wait for"),
      BT::InputPort<double>("timeout", 300.0, "Timeout in seconds"),
      BT::OutputPort<std::string>("elevator_info", "Which elevator was used (lift1/lift2/stair)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void messageCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Time start_time_;
  bool message_received_;
  std::string elevator_info_;
};

}  // namespace sparo_navigation_core

#endif  // SPARO_NAVIGATION_CORE__WAIT_FOR_TOPIC_MESSAGE_HPP_
