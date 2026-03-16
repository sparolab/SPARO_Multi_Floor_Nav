#include "sparo_navigation_core/navigation/publish_floor_request.hpp"

namespace sparo_navigation_core
{

PublishFloorRequest::PublishFloorRequest(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

void PublishFloorRequest::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
}

BT::NodeStatus PublishFloorRequest::tick()
{
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishFloorRequest"), "Node not initialized!");
    return BT::NodeStatus::FAILURE;
  }

  std::string topic = "/elevator/floor_request";
  getInput("topic", topic);

  int target_floor;
  if (!getInput("target_floor", target_floor)) {
    RCLCPP_ERROR(node_->get_logger(), "[PublishFloorRequest] Missing target_floor");
    return BT::NodeStatus::FAILURE;
  }

  // Create publisher if needed or topic changed
  if (!pub_ || last_topic_ != topic) {
    pub_ = node_->create_publisher<std_msgs::msg::Int32>(topic, 10);
    last_topic_ = topic;
    // Wait for subscriber
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std_msgs::msg::Int32 msg;
  msg.data = target_floor;
  pub_->publish(msg);

  RCLCPP_INFO(node_->get_logger(), "[PublishFloorRequest] Published target_floor=%d to %s",
              target_floor, topic.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sparo_navigation_core

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sparo_navigation_core::PublishFloorRequest>("PublishFloorRequest");
}
