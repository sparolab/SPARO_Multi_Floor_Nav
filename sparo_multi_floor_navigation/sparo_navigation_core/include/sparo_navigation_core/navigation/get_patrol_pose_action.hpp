#ifndef SPARO_NAVIGATION_CORE__GET_PATROL_POSE_ACTION_HPP_
#define SPARO_NAVIGATION_CORE__GET_PATROL_POSE_ACTION_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sparo_navigation_core/config_loader.hpp"

namespace sparo_navigation_core
{

/**
 * @brief BT Action Node to get a specific pose from patrol YAML file
 * Reads initial_pose from patrol_L*.yaml and outputs it to blackboard
 */
class GetPatrolPoseAction : public BT::SyncActionNode
{
public:
  GetPatrolPoseAction(
    const std::string& name,
    const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Floor number"),
      BT::InputPort<std::string>("pose_type", "default", "Type of pose (default, elevator, elevator_lift1, elevator_lift2, stair, etc.)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Output pose")
    };
  }

  BT::NodeStatus tick() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace sparo_navigation_core

#endif  // SPARO_NAVIGATION_CORE__GET_PATROL_POSE_ACTION_HPP_
