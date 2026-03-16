#ifndef SPARO_NAVIGATION_CORE__PUBLISH_FLOOR_REQUEST_HPP_
#define SPARO_NAVIGATION_CORE__PUBLISH_FLOOR_REQUEST_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace sparo_navigation_core
{

/**
 * @brief Publish floor request to elevator_bt or stair_bt
 */
class PublishFloorRequest : public BT::SyncActionNode
{
public:
  PublishFloorRequest(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/elevator/floor_request", "Topic to publish (elevator or stairs)"),
      BT::InputPort<int>("target_floor", "Target floor index (0=L1, 1=L2, 2=L3)")
    };
  }

  BT::NodeStatus tick() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  std::string last_topic_;
};

}  // namespace sparo_navigation_core

#endif  // SPARO_NAVIGATION_CORE__PUBLISH_FLOOR_REQUEST_HPP_
