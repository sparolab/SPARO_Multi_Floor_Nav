#include "sparo_navigation_core/navigation/wait_for_floor_change.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace sparo_navigation_core
{

WaitForFloorChange::WaitForFloorChange(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  static int instance_count = 0;
  node_ = std::make_shared<rclcpp::Node>("wait_for_floor_change_" + std::to_string(instance_count++));
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus WaitForFloorChange::onStart()
{
  start_time_ = node_->now();
  RCLCPP_INFO(node_->get_logger(), "[WaitForFloorChange] Waiting for floor change...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForFloorChange::onRunning()
{
  int target_floor;
  if (!getInput("target_floor", target_floor)) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitForFloorChange] Missing target_floor");
    return BT::NodeStatus::FAILURE;
  }

  double timeout = 180.0;
  getInput("timeout", timeout);

  if ((node_->now() - start_time_).seconds() > timeout) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitForFloorChange] Timeout waiting for floor change");
    return BT::NodeStatus::FAILURE;
  }

  std::string map_frame = "map";
  std::string base_frame = "base_link";
  getInput("map_frame", map_frame);
  getInput("base_frame", base_frame);

  double f0 = 0.0, f1 = 5.0, f2 = 10.0;
  getInput("floor0_z", f0);
  getInput("floor1_z", f1);
  getInput("floor2_z", f2);

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[WaitForFloorChange] Waiting for TF: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }

  double z = tf.transform.translation.z;

  // Determine current floor
  std::vector<double> floors = {f0, f1, f2};
  double threshold = 2.0;  // Within 2m of floor height

  int current_floor = -1;
  for (size_t i = 0; i < floors.size(); ++i) {
    if (std::abs(z - floors[i]) < threshold) {
      current_floor = static_cast<int>(i);
      break;
    }
  }

  if (current_floor == target_floor) {
    RCLCPP_INFO(node_->get_logger(), "[WaitForFloorChange] Reached target floor %d (z=%.2f)",
                target_floor, z);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                       "[WaitForFloorChange] Current z=%.2f, waiting for floor %d...",
                       z, target_floor);

  return BT::NodeStatus::RUNNING;
}

void WaitForFloorChange::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[WaitForFloorChange] Halted");
}

}  // namespace sparo_navigation_core

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sparo_navigation_core::WaitForFloorChange>("WaitForFloorChange");
}
