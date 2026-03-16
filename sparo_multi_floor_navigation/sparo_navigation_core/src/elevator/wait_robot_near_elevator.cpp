#include "sparo_navigation_core/elevator/wait_robot_near_elevator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace sparo_navigation_core {

WaitRobotNearElevator::WaitRobotNearElevator(const std::string& name,
                                             const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  static int instance_count = 0;
  node_ = std::make_shared<rclcpp::Node>("wait_robot_near_elevator_" + std::to_string(instance_count++));
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
}

BT::NodeStatus WaitRobotNearElevator::onStart()
{
  std::string yaml_path;
  if (!getInput("elevator_yaml", yaml_path)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WaitRobotNearElevator] missing elevator_yaml");
    return BT::NodeStatus::FAILURE;
  }

  getInput("world_frame", world_frame_);
  getInput("base_frame",  base_frame_);
  getInput("timeout", timeout_);

  if (!load_zones_config(yaml_path, zones_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WaitRobotNearElevator] failed to load zones from %s",
                 yaml_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  t0_ = std::chrono::steady_clock::now();
  last_log_ = t0_;

  RCLCPP_INFO(node_->get_logger(),
              "[WaitRobotNearElevator] world_frame=%s, base_frame=%s, "
              "zone[%.2f,%.2f]x[%.2f,%.2f], front_offset=%.2f",
              world_frame_.c_str(), base_frame_.c_str(),
              zones_.zone_min_x, zones_.zone_max_x,
              zones_.zone_min_y, zones_.zone_max_y,
              zones_.front_offset);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitRobotNearElevator::onRunning()
{    
  RCLCPP_INFO(node_->get_logger(),
                "[WaitRobotNearElevator] onRunning");
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
    // TF 아직 준비 안 됐으면 계속 대기
    RCLCPP_DEBUG(node_->get_logger(),
                 "[WaitRobotNearElevator] TF lookup failed: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }

  const double x = tf.transform.translation.x;
  const double y = tf.transform.translation.y;

  const auto& q = tf.transform.rotation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  // base_link → front point (앞부분)
  const double dx = zones_.front_offset; // 로봇 좌표계 +x 방향 앞 (확실하지 않음)
  const double dy = 0.0;

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double front_x = x + cos_yaw * dx - sin_yaw * dy;
  const double front_y = y + sin_yaw * dx + cos_yaw * dy;

  // Add tolerance for easier arrival detection
  const double xy_tolerance = 0.7;  // 60cm tolerance on zone boundaries

  const bool in_zone =
      (front_x >= zones_.zone_min_x - xy_tolerance && front_x <= zones_.zone_max_x + xy_tolerance &&
       front_y >= zones_.zone_min_y - xy_tolerance && front_y <= zones_.zone_max_y + xy_tolerance);

  if (in_zone) {
    RCLCPP_INFO(node_->get_logger(),
                "[WaitRobotNearElevator] front in zone (%.3f, %.3f) with tolerance %.2fm",
                front_x, front_y, xy_tolerance);
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = std::chrono::duration<double>(now - t0_).count();
  if (elapsed > timeout_) {
    RCLCPP_WARN(node_->get_logger(),
                "[WaitRobotNearElevator] timeout (%.1f s), "
                "front=(%.3f,%.3f) still outside zone",
                elapsed, front_x, front_y);
    return BT::NodeStatus::FAILURE;
  }

  if (std::chrono::duration<double>(now - last_log_).count() > 1.0) {
    RCLCPP_INFO(node_->get_logger(),
                "[WaitRobotNearElevator] waiting, front=(%.3f,%.3f), "
                "zone[%.2f,%.2f]x[%.2f,%.2f]",
                front_x, front_y,
                zones_.zone_min_x, zones_.zone_max_x,
                zones_.zone_min_y, zones_.zone_max_y);
    last_log_ = now;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitRobotNearElevator::onHalted()
{
}

} // namespace sparo_navigation_core
