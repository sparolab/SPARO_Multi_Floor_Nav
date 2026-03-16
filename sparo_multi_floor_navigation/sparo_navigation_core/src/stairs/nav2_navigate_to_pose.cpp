#include "sparo_navigation_core/stairs/nav2_navigate_to_pose.hpp"

#include <chrono>
#include <cmath>

namespace sparo_navigation_core
{

Nav2NavigateToPose::Nav2NavigateToPose(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  logger_(rclcpp::get_logger("Nav2NavigateToPose"))
{
  node_ = rclcpp::Node::make_shared("nav2_navigate_to_pose_bt_node");

  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "navigate_to_pose");

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus Nav2NavigateToPose::onStart()
{
  if (!getInput("entry_pose", goal_pose_))
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose] missing input [entry_pose]");
    return BT::NodeStatus::FAILURE;
  }

  getInput<std::string>("base_frame", base_frame_);
  getInput<double>("success_radius", success_radius_);
  getInput<double>("yaw_tolerance", yaw_tolerance_);
  getInput<double>("timeout", timeout_sec_);

  if (success_radius_ < 0.0) success_radius_ = std::abs(success_radius_);
  if (timeout_sec_ <= 0.0) timeout_sec_ = 120.0;

  if (goal_pose_.header.frame_id.empty())
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose] entry_pose.header.frame_id is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose] navigate_to_pose action server not available");
    return BT::NodeStatus::FAILURE;
  }


  goal_sent_ = false;
  current_goal_handle_.reset();
  goal_start_time_ = node_->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Nav2NavigateToPose::onRunning()
{
  if ((node_->now() - goal_start_time_).seconds() > timeout_sec_)
  {
    RCLCPP_WARN(logger_, "[Nav2NavigateToPose] timeout %.1f s", timeout_sec_);
    if (goal_sent_ && current_goal_handle_)
    {
      action_client_->async_cancel_goal(current_goal_handle_);
    }
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped tf;
  try
  {
    const std::string frame = goal_pose_.header.frame_id;
    tf = tf_buffer_->lookupTransform(frame, base_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "[Nav2NavigateToPose] waiting TF (%s->%s): %s",
      goal_pose_.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;
  }

  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;
  const double rz = tf.transform.translation.z;

  if (!goal_sent_)
  {

    goal_pose_.pose.position.z = rz;

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal_pose_;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    auto future_goal_handle = action_client_->async_send_goal(nav_goal, send_goal_options);

    auto ret = rclcpp::spin_until_future_complete(
      node_, future_goal_handle, std::chrono::seconds(2));

    if (ret != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        logger_,
        "[Nav2NavigateToPose] Failed to send goal (ret=%d)", static_cast<int>(ret));
      return BT::NodeStatus::FAILURE;
    }

    current_goal_handle_ = future_goal_handle.get();
    if (!current_goal_handle_)
    {
      RCLCPP_ERROR(logger_, "[Nav2NavigateToPose] Goal was rejected by server");
      return BT::NodeStatus::FAILURE;
    }

    goal_sent_ = true;

    RCLCPP_INFO(
      logger_,
      "[Nav2NavigateToPose] Goal sent (frame=%s, x=%.3f, y=%.3f, z(current)=%.3f), "
      "success_radius=%.2f timeout=%.1f base_frame=%s",
      goal_pose_.header.frame_id.c_str(),
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y,
      goal_pose_.pose.position.z,
      success_radius_, timeout_sec_, base_frame_.c_str());

    return BT::NodeStatus::RUNNING;
  }

  const double gx = goal_pose_.pose.position.x;
  const double gy = goal_pose_.pose.position.y;

  const double dist = std::hypot(gx - rx, gy - ry);

  bool position_ok = (dist < success_radius_);
  bool orientation_ok = true;

  // Check yaw if yaw_tolerance >= 0
  if (yaw_tolerance_ >= 0.0) {
    // Get current yaw from TF
    double current_yaw = std::atan2(
      2.0 * (tf.transform.rotation.w * tf.transform.rotation.z +
             tf.transform.rotation.x * tf.transform.rotation.y),
      1.0 - 2.0 * (tf.transform.rotation.y * tf.transform.rotation.y +
                   tf.transform.rotation.z * tf.transform.rotation.z));

    // Get goal yaw
    double goal_yaw = std::atan2(
      2.0 * (goal_pose_.pose.orientation.w * goal_pose_.pose.orientation.z +
             goal_pose_.pose.orientation.x * goal_pose_.pose.orientation.y),
      1.0 - 2.0 * (goal_pose_.pose.orientation.y * goal_pose_.pose.orientation.y +
                   goal_pose_.pose.orientation.z * goal_pose_.pose.orientation.z));

    double yaw_error = std::abs(goal_yaw - current_yaw);
    // Normalize to [0, pi]
    while (yaw_error > M_PI) yaw_error = 2.0 * M_PI - yaw_error;

    orientation_ok = (yaw_error < yaw_tolerance_);
  }

  if (position_ok && orientation_ok)
  {
    RCLCPP_INFO(
      logger_,
      "[Nav2NavigateToPose] Goal reached (dist=%.3f < %.3f), cancel and SUCCESS",
      dist, success_radius_);

    action_client_->async_cancel_goal(current_goal_handle_);
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  auto result_future = action_client_->async_get_result(current_goal_handle_);
  auto ret = rclcpp::spin_until_future_complete(
    node_, result_future, std::chrono::milliseconds(10));

  if (ret == rclcpp::FutureReturnCode::TIMEOUT)
  {
    return BT::NodeStatus::RUNNING;
  }

  if (ret != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger_,
      "[Nav2NavigateToPose] Failed to get result (ret=%d)", static_cast<int>(ret));
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  auto wrapped_result = result_future.get();
  auto code = wrapped_result.code;

  RCLCPP_INFO(
    logger_,
    "[Nav2NavigateToPose] navigation finished with code %d",
    static_cast<int>(code));

  if (code == rclcpp_action::ResultCode::CANCELED)
  {
    RCLCPP_WARN(logger_, "[Nav2NavigateToPose] navigation CANCELED");
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  goal_sent_ = false;
  current_goal_handle_.reset();
  return BT::NodeStatus::SUCCESS;
}

void Nav2NavigateToPose::onHalted()
{
  if (goal_sent_ && current_goal_handle_)
  {
    RCLCPP_INFO(logger_, "[Nav2NavigateToPose] Halted, cancel goal");
    action_client_->async_cancel_goal(current_goal_handle_);
  }
  goal_sent_ = false;
  current_goal_handle_.reset();
}

}  // namespace sparo_navigation_core
