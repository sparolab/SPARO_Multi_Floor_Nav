#include "sparo_navigation_core/navigation/set_initial_pose_action.hpp"
#include <thread>
#include <chrono>

namespace sparo_navigation_core
{

SetInitialPoseAction::SetInitialPoseAction(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

void SetInitialPoseAction::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  // Don't create publisher here - create it in tick() like the original version
}

BT::NodeStatus SetInitialPoseAction::tick()
{
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("SetInitialPose"), "ROS2 node not set!");
    return BT::NodeStatus::FAILURE;
  }

  std::string floor;
  if (!getInput("floor", floor)) {
    RCLCPP_ERROR(node_->get_logger(), "[SetInitialPose] Missing floor input");
    return BT::NodeStatus::FAILURE;
  }

  // Get pose_type (default to "default" if not provided)
  std::string pose_type = "default";
  getInput("pose_type", pose_type);

  // If pose_type is "auto", determine from elevator_info
  if (pose_type == "auto") {
    std::string elevator_info;
    RCLCPP_WARN(node_->get_logger(), "[SetInitialPose] pose_type is 'auto', checking elevator_info...");
    if (getInput("elevator_info", elevator_info)) {
      RCLCPP_WARN(node_->get_logger(), "[SetInitialPose] Got elevator_info: '%s'", elevator_info.c_str());
      if (elevator_info == "/lift1") {
        pose_type = "elevator_lift1_inside";
      } else if (elevator_info == "/lift2") {
        pose_type = "elevator_lift2_inside";
      } else if (elevator_info == "stair") {
        pose_type = "stair";
      } else {
        pose_type = "default";
      }
      RCLCPP_INFO(node_->get_logger(), "[SetInitialPose] Auto-selected pose_type=%s based on elevator_info=%s",
                  pose_type.c_str(), elevator_info.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[SetInitialPose] pose_type='auto' but elevator_info not available! Using default.");
      pose_type = "default";
    }
  }

  if (!g_config_loader) {
    RCLCPP_ERROR(node_->get_logger(), "[SetInitialPose] Config loader not initialized");
    return BT::NodeStatus::FAILURE;
  }

  // Get initial pose from patrol YAML
  geometry_msgs::msg::Pose initial_pose;
  std::string patrol_config_dir = "/home/test_ws/src/sparo_multi_floor_navigation/sparo_navigation_bringup/config/navigation";

  if (!g_config_loader->getInitialPoseFromPatrol(floor, pose_type, patrol_config_dir, initial_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[SetInitialPose] Failed to get initial_pose.%s for floor %s",
                 pose_type.c_str(), floor.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Create publisher if not exists
  if (!pose_pub_) {
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
  }

  // Wait for map to be fully loaded (especially after SwitchMap)
  RCLCPP_INFO(node_->get_logger(), "[SetInitialPose] Waiting 2s for map to load...");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Publish initial pose multiple times for AMCL convergence
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = node_->now();
    pose_msg.pose.pose = initial_pose;

    // Force z=0 for 2D navigation (AMCL doesn't use z)
    pose_msg.pose.pose.position.z = 0.0;

    // Set covariance
    pose_msg.pose.covariance[0] = 0.25;   // x
    pose_msg.pose.covariance[7] = 0.25;   // y
    pose_msg.pose.covariance[35] = 0.07;  // yaw

    pose_pub_->publish(pose_msg);

    RCLCPP_INFO(node_->get_logger(),
                "[SetInitialPose] Published initial_pose.%s for floor %s (attempt %d/5): (%.2f, %.2f)",
                pose_type.c_str(), floor.c_str(), i + 1,
                initial_pose.position.x,
                initial_pose.position.y);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sparo_navigation_core

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sparo_navigation_core::SetInitialPoseAction>("SetInitialPose");
}
