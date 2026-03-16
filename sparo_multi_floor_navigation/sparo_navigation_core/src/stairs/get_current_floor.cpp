#include "sparo_navigation_core/stairs/get_current_floor.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace sparo_navigation_core
{

GetCurrentFloor::GetCurrentFloor(
    const std::string & name,
    const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  static int instance_count = 0;
  node_ = std::make_shared<rclcpp::Node>("get_current_floor_" + std::to_string(instance_count++));
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  // Subscribe to odom
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      robot_pose_ = msg->pose.pose;
      have_robot_pose_ = true;
    });
}

BT::PortsList GetCurrentFloor::providedPorts()
{
  return {
    BT::InputPort<std::string>("map_frame", "map", "Map frame"),
    BT::InputPort<std::string>("base_frame", "base_link", "Base frame"),
    BT::InputPort<double>("floor0_z", 0.0, "Floor 0 height"),
    BT::InputPort<double>("floor1_z", 5.0, "Floor 1 height"),
    BT::InputPort<double>("floor2_z", 10.0, "Floor 2 height"),
    BT::OutputPort<int>("current_floor", "Current floor index (0, 1, 2)")
  };
}

BT::NodeStatus GetCurrentFloor::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[GetCurrentFloor] onStart");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetCurrentFloor::onRunning()
{
  using namespace std::chrono_literals;

  // Spin executor to process odom callbacks
  exec_->spin_some(0ms);

  std::string map_frame = "map";
  std::string base_frame = "base_link";
  getInput<std::string>("map_frame", map_frame);
  getInput<std::string>("base_frame", base_frame);

  double f0 = 0.0, f1 = 5.0, f2 = 10.0;
  getInput<double>("floor0_z", f0);
  getInput<double>("floor1_z", f1);
  getInput<double>("floor2_z", f2);
  // Get z from odom
  double z;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!have_robot_pose_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[GetCurrentFloor] No odom received yet");
      return BT::NodeStatus::RUNNING;
    }
    z = robot_pose_.position.z;
  }


  double b01 = 0.5 * (f0 + f1);
  double b12 = 0.5 * (f1 + f2);

  int floor = 0;
  if (z < f1) {
    floor = 0;
  } else if (z < f2) {
    floor = 1;
  } else {
    floor = 2;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[GetCurrentFloor] z=%.3f -> floor=%d", z, floor);

  setOutput<int>("current_floor", floor);
  return BT::NodeStatus::SUCCESS;
}

void GetCurrentFloor::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[GetCurrentFloor] halted");
}

}  // namespace sparo_navigation_core
