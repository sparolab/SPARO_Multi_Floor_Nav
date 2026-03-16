#include "sparo_navigation_core/navigation/get_patrol_pose_action.hpp"

namespace sparo_navigation_core
{

GetPatrolPoseAction::GetPatrolPoseAction(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

void GetPatrolPoseAction::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
}

BT::NodeStatus GetPatrolPoseAction::tick()
{
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("GetPatrolPose"), "ROS2 node not set!");
    return BT::NodeStatus::FAILURE;
  }

  std::string floor;
  if (!getInput("floor", floor)) {
    RCLCPP_ERROR(node_->get_logger(), "[GetPatrolPose] Missing floor input");
    return BT::NodeStatus::FAILURE;
  }

  std::string pose_type = "default";
  getInput("pose_type", pose_type);

  if (!g_config_loader) {
    RCLCPP_ERROR(node_->get_logger(), "[GetPatrolPose] Config loader not initialized");
    return BT::NodeStatus::FAILURE;
  }

  // Get pose from patrol YAML
  geometry_msgs::msg::Pose pose;
  std::string patrol_config_dir = "/home/test_ws/src/sparo_multi_floor_navigation/sparo_navigation_bringup/config/navigation";

  if (!g_config_loader->getInitialPoseFromPatrol(floor, pose_type, patrol_config_dir, pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[GetPatrolPose] Failed to get pose.%s for floor %s",
                 pose_type.c_str(), floor.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Create PoseStamped output
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = node_->now();
  goal_pose.pose = pose;

  // Set output port
  setOutput("goal_pose", goal_pose);

  RCLCPP_INFO(node_->get_logger(),
              "[GetPatrolPose] Got pose.%s for floor %s: (%.2f, %.2f)",
              pose_type.c_str(), floor.c_str(),
              pose.position.x, pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sparo_navigation_core

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sparo_navigation_core::GetPatrolPoseAction>("GetPatrolPose");
}
