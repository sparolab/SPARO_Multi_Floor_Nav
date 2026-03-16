#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <cmath>
#include "sparo_navigation_core/config_loader.hpp"

namespace sparo_navigation_core
{

/**
 * StairNavigationAction - Nav2 없이 cmd_vel로 계단 waypoints 이동
 *
 * AMCL pose (map frame) 기반으로 현재 위치 추적하고, 목표 waypoint까지 직진
 */
class StairNavigationAction : public BT::StatefulActionNode
{
public:
  StairNavigationAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Floor number"),
      BT::InputPort<double>("linear_speed", 0.3, "Linear speed (m/s)"),
      BT::InputPort<double>("angular_speed", 0.5, "Angular speed (rad/s)"),
      BT::InputPort<double>("goal_tolerance", 0.5, "Distance tolerance to waypoint")
    };
  }

  void initialize(rclcpp::Node::SharedPtr node);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
  double normalizeAngle(double angle);
  void stopRobot();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr nav2_cancel_client_;

  std::vector<Waypoint> waypoints_;
  size_t current_waypoint_index_;
  bool waypoints_loaded_;

  geometry_msgs::msg::Pose current_pose_;
  bool pose_received_;

  double linear_speed_;
  double angular_speed_;
  double goal_tolerance_;
};

}  // namespace sparo_navigation_core
