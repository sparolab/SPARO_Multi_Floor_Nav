#include "sparo_navigation_core/elevator/wait_robot_inside_cabin.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace sparo_navigation_core {

WaitRobotInsideCabin::WaitRobotInsideCabin(const std::string& name,
                                           const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  static int instance_count = 0;
  node_ = std::make_shared<rclcpp::Node>("wait_robot_inside_cabin_" + std::to_string(instance_count++));
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
}

BT::NodeStatus WaitRobotInsideCabin::onStart()
{
  std::string yaml_path;
  if (!getInput("elevator_yaml", yaml_path)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WaitRobotInsideCabin] missing elevator_yaml");
    return BT::NodeStatus::FAILURE;
  }

  getInput("world_frame", world_frame_);
  getInput("base_frame",  base_frame_);
  getInput("timeout", timeout_);

  if (!load_zones_config(yaml_path, zones_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WaitRobotInsideCabin] failed to load zones from %s",
                 yaml_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  t0_ = std::chrono::steady_clock::now();
  last_log_ = t0_;

  RCLCPP_INFO(node_->get_logger(),
              "[WaitRobotInsideCabin] world_frame=%s, base_frame=%s, "
              "cabin[%.2f,%.2f]x[%.2f,%.2f]",
              world_frame_.c_str(), base_frame_.c_str(),
              zones_.cabin_min_x, zones_.cabin_max_x,
              zones_.cabin_min_y, zones_.cabin_max_y);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitRobotInsideCabin::onRunning()
{
  exec_->spin_some(std::chrono::milliseconds(0));
  auto now = std::chrono::steady_clock::now();

  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform(
        world_frame_, base_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[WaitRobotInsideCabin] TF lookup failed: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }

  const double x = tf.transform.translation.x;
  const double y = tf.transform.translation.y;

  // Extract yaw from quaternion
  const auto& q = tf.transform.rotation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  // Expected yaw: opposite of entrance direction (turned around inside cabin)
  // entrance_yaw points INTO cabin, robot should face OUT (opposite direction)
  double expected_yaw = zones_.entrance_yaw + M_PI;

  // Normalize to [-pi, pi]
  while (expected_yaw > M_PI) expected_yaw -= 2.0 * M_PI;
  while (expected_yaw < -M_PI) expected_yaw += 2.0 * M_PI;

  // Normalize yaw difference to [-pi, pi]
  double yaw_diff = std::fmod(yaw - expected_yaw + M_PI, 2.0 * M_PI);
  if (yaw_diff < 0) yaw_diff += 2.0 * M_PI;
  yaw_diff -= M_PI;

  const double yaw_tolerance = 0.4;  // ~23 degrees tolerance
  const double xy_tolerance = 0.1;   // 10cm position tolerance

  const bool inside_position =
      (x >= zones_.cabin_min_x - xy_tolerance && x <= zones_.cabin_max_x + xy_tolerance &&
       y >= zones_.cabin_min_y - xy_tolerance && y <= zones_.cabin_max_y + xy_tolerance);

  const bool correct_orientation = std::abs(yaw_diff) < yaw_tolerance;

  if (inside_position && correct_orientation) {
    RCLCPP_INFO(node_->get_logger(),
                "[WaitRobotInsideCabin] robot inside cabin at (%.3f, %.3f), yaw=%.2f (diff=%.2f rad)",
                x, y, yaw, yaw_diff);
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = std::chrono::duration<double>(now - t0_).count();
  if (elapsed > timeout_) {
    RCLCPP_WARN(node_->get_logger(),
                "[WaitRobotInsideCabin] timeout (%.1f s), x=%.3f,y=%.3f",
                elapsed, x, y);
    return BT::NodeStatus::FAILURE;
  }

  if (std::chrono::duration<double>(now - last_log_).count() > 1.0) {
    RCLCPP_INFO(node_->get_logger(),
                "[WaitRobotInsideCabin] waiting, pos=(%.3f,%.3f) in_pos=%d, "
                "yaw=%.2f expected=%.2f diff=%.2f in_yaw=%d",
                x, y, inside_position, yaw, expected_yaw, yaw_diff, correct_orientation);
    last_log_ = now;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitRobotInsideCabin::onHalted()
{
}

} // namespace sparo_navigation_core
