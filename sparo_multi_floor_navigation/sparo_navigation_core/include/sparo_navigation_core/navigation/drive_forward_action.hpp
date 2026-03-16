#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <cmath>

namespace sparo_navigation_core
{

/**
 * DriveForward - cmd_vel로 지정된 방향으로 회전 후 직진
 *
 * Nav2 goal을 취소하고, 지정된 heading으로 회전 후 odom 기반으로 전진
 */
class DriveForward : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  DriveForward(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 5.0, "Distance to drive forward (m)"),
      BT::InputPort<double>("speed", 0.3, "Linear speed (m/s)"),
      BT::InputPort<double>("heading", -999.0, "Target heading in radians (-999 = use current heading)"),
      BT::InputPort<double>("angular_speed", 0.5, "Angular speed for rotation (rad/s)")
    };
  }

  void initialize(rclcpp::Node::SharedPtr node);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void stopRobot();
  void cancelNav2Goals();
  double normalizeAngle(double angle);
  double getYawFromQuaternion(double x, double y, double z, double w);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;

  double target_distance_;
  double speed_;
  double target_heading_;
  double angular_speed_;
  double start_x_;
  double start_y_;
  double current_x_;
  double current_y_;
  double current_yaw_;
  bool odom_received_;
  bool started_;
  bool nav2_cancelled_;
  bool rotation_complete_;
};

}  // namespace sparo_navigation_core
