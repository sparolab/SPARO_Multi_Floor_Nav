#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <yaml-cpp/yaml.h>
#include <mutex>

namespace sparo_navigation_core
{

class SelectNearestElevator : public BT::SyncActionNode
{
public:
  SelectNearestElevator(const std::string & name,
                        const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  struct EntranceInfo
  {
    std::string yaml_path;
    std::string ns;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    bool valid{false};
  };

  bool loadEntrance(const std::string & yaml_path,
                    int floor_idx,
                    EntranceInfo & out_info);

  rclcpp::Node::SharedPtr node_;
  // Robot pose from /amcl_pose
  geometry_msgs::msg::Pose robot_pose_;
  bool have_robot_pose_{false};
  std::mutex pose_mutex_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  // executor for subscriptions
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;

  // /lift1/cabin_z, /lift2/cabin_z
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_lift1_z_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_lift2_z_;

  double lift1_cabin_z_;
  double lift2_cabin_z_;
  bool   have_lift1_z_;
  bool   have_lift2_z_;
  std::mutex z_mutex_;
};

}  // namespace sparo_navigation_core
