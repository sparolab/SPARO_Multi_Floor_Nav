#include "sparo_navigation_core/navigation/wait_for_topic_message.hpp"

namespace sparo_navigation_core
{

WaitForTopicMessage::WaitForTopicMessage(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), message_received_(false)
{
  static int instance_count = 0;
  node_ = std::make_shared<rclcpp::Node>("wait_for_topic_message_" + std::to_string(instance_count++));
}

void WaitForTopicMessage::messageCallback(const std_msgs::msg::String::SharedPtr msg)
{
  message_received_ = true;
  elevator_info_ = msg->data;
  RCLCPP_INFO(node_->get_logger(), "[WaitForTopicMessage] Message received: %s", msg->data.c_str());
}

BT::NodeStatus WaitForTopicMessage::onStart()
{
  std::string topic = "/floor_change_complete";
  getInput("topic", topic);

  message_received_ = false;
  start_time_ = node_->now();

  // Create subscription
  sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic, 10,
    std::bind(&WaitForTopicMessage::messageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "[WaitForTopicMessage] Waiting for message on %s", topic.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForTopicMessage::onRunning()
{
  double timeout = 300.0;
  getInput("timeout", timeout);

  // Spin to process callbacks
  rclcpp::spin_some(node_);

  if (message_received_) {
    RCLCPP_WARN(node_->get_logger(), "[WaitForTopicMessage] Setting output elevator_used=%s", elevator_info_.c_str());
    setOutput("elevator_info", elevator_info_);
    RCLCPP_INFO(node_->get_logger(), "[WaitForTopicMessage] Message received, SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  if ((node_->now() - start_time_).seconds() > timeout) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitForTopicMessage] Timeout after %.1fs", timeout);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForTopicMessage::onHalted()
{
  sub_.reset();
  RCLCPP_INFO(node_->get_logger(), "[WaitForTopicMessage] Halted");
}

}  // namespace sparo_navigation_core

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sparo_navigation_core::WaitForTopicMessage>("WaitForTopicMessage");
}
