#ifndef MULTI_FLOOR_NAV2_BT__SET_INITIAL_POSE_ACTION_HPP_
#define MULTI_FLOOR_NAV2_BT__SET_INITIAL_POSE_ACTION_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sparo_navigation_core/config_loader.hpp"

namespace sparo_navigation_core
{

/**
 * @brief BT Action Node to set initial pose from floor config
 */
class SetInitialPoseAction : public BT::SyncActionNode
{
public:
  SetInitialPoseAction(
    const std::string& name,
    const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Floor number for initial pose"),
      BT::InputPort<std::string>("pose_type", "default", "Type of initial pose (or \"auto\" to use arrival_method)"),
      BT::InputPort<std::string>("arrival_method", "", "How robot arrived"),
      BT::InputPort<std::string>("elevator_info", "", "Elevator used (/lift1, /lift2, or stair)")
    };
  }

  BT::NodeStatus tick() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

}  // namespace sparo_navigation_core

#endif  // MULTI_FLOOR_NAV2_BT__SET_INITIAL_POSE_ACTION_HPP_
