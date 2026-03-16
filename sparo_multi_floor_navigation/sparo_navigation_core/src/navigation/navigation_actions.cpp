#include "sparo_navigation_core/navigation/navigation_actions.hpp"
#include <cmath>
#include <thread>

namespace sparo_navigation_core
{

// Helper function
static double calculateDistance(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2)
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

// Helper: get yaw from quaternion
static double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// Helper: create quaternion from yaw
static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

// Helper: normalize angle to [-pi, pi]
static double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// ============== NavigateToElevatorFront ==============
NavigateToElevatorFront::NavigateToElevatorFront(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config),
    received_pose_(false),
    tolerance_(0.5),
    goal_sent_(false)
{
}

void NavigateToElevatorFront::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&NavigateToElevatorFront::poseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "[NavigateToElevatorFront] Initialized");
}

void NavigateToElevatorFront::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  received_pose_ = true;
}

bool NavigateToElevatorFront::sendGoal()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToElevatorFront] Action server not available");
    return false;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->now();
  goal.pose.pose = goal_pose_;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr& goal_handle) {
      if (goal_handle) {
        goal_handle_ = goal_handle;
        RCLCPP_INFO(node_->get_logger(), "[NavigateToElevatorFront] Goal accepted");
      }
    };

  action_client_->async_send_goal(goal, send_goal_options);
  goal_sent_ = true;
  return true;
}

bool NavigateToElevatorFront::isGoalReached(double tolerance)
{
  if (!received_pose_) return false;
  return calculateDistance(current_pose_, goal_pose_) <= tolerance;
}

BT::NodeStatus NavigateToElevatorFront::onStart()
{
  if (!node_) return BT::NodeStatus::FAILURE;

  std::string floor;
  if (!getInput<std::string>("floor", floor)) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToElevatorFront] Missing input [floor]");
    return BT::NodeStatus::FAILURE;
  }

  getInput<double>("tolerance", tolerance_);

  if (!g_config_loader) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToElevatorFront] Config loader not initialized");
    return BT::NodeStatus::FAILURE;
  }

  FloorConfig floor_config;
  if (!g_config_loader->getFloorConfig(floor, floor_config)) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToElevatorFront] Failed to get config for floor %s", floor.c_str());
    return BT::NodeStatus::FAILURE;
  }

  goal_pose_ = floor_config.elevator_front;

  RCLCPP_INFO(node_->get_logger(), "[NavigateToElevatorFront] Going to elevator front on floor %s (%.2f, %.2f)",
              floor.c_str(), goal_pose_.position.x, goal_pose_.position.y);

  if (!sendGoal()) return BT::NodeStatus::FAILURE;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToElevatorFront::onRunning()
{
  if (isGoalReached(tolerance_)) {
    RCLCPP_INFO(node_->get_logger(), "[NavigateToElevatorFront] Reached elevator front");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void NavigateToElevatorFront::onHalted()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
  RCLCPP_WARN(node_->get_logger(), "[NavigateToElevatorFront] Halted");
}

// ============== EnterElevator ==============
EnterElevator::EnterElevator(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config),
    received_pose_(false),
    tolerance_(0.3),
    goal_sent_(false)
{
}

void EnterElevator::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&EnterElevator::poseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "[EnterElevator] Initialized");
}

void EnterElevator::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  received_pose_ = true;
}

bool EnterElevator::sendGoal()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "[EnterElevator] Action server not available");
    return false;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->now();
  goal.pose.pose = goal_pose_;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr& goal_handle) {
      if (goal_handle) {
        goal_handle_ = goal_handle;
        RCLCPP_INFO(node_->get_logger(), "[EnterElevator] Goal accepted");
      }
    };

  action_client_->async_send_goal(goal, send_goal_options);
  goal_sent_ = true;
  return true;
}

bool EnterElevator::isGoalReached(double tolerance)
{
  if (!received_pose_) return false;
  return calculateDistance(current_pose_, goal_pose_) <= tolerance;
}

BT::NodeStatus EnterElevator::onStart()
{
  if (!node_) return BT::NodeStatus::FAILURE;

  std::string floor;
  if (!getInput<std::string>("floor", floor)) {
    RCLCPP_ERROR(node_->get_logger(), "[EnterElevator] Missing input [floor]");
    return BT::NodeStatus::FAILURE;
  }

  getInput<double>("tolerance", tolerance_);

  if (!g_config_loader) {
    RCLCPP_ERROR(node_->get_logger(), "[EnterElevator] Config loader not initialized");
    return BT::NodeStatus::FAILURE;
  }

  FloorConfig floor_config;
  if (!g_config_loader->getFloorConfig(floor, floor_config)) {
    RCLCPP_ERROR(node_->get_logger(), "[EnterElevator] Failed to get config for floor %s", floor.c_str());
    return BT::NodeStatus::FAILURE;
  }

  goal_pose_ = floor_config.elevator_inside;

  RCLCPP_INFO(node_->get_logger(), "[EnterElevator] Entering elevator (%.2f, %.2f)",
              goal_pose_.position.x, goal_pose_.position.y);

  if (!sendGoal()) return BT::NodeStatus::FAILURE;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus EnterElevator::onRunning()
{
  if (isGoalReached(tolerance_)) {
    // Don't cancel - just return SUCCESS
    RCLCPP_INFO(node_->get_logger(), "[EnterElevator] Entered elevator");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void EnterElevator::onHalted()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
  RCLCPP_WARN(node_->get_logger(), "[EnterElevator] Halted");
}

// ============== ExitElevator (using backup behavior) ==============
ExitElevator::ExitElevator(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config),
    goal_completed_(false),
    goal_succeeded_(false),
    backup_dist_(2.5),
    backup_speed_(0.2)
{
}

void ExitElevator::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  backup_client_ = rclcpp_action::create_client<BackUp>(node_, "backup");
  RCLCPP_INFO(node_->get_logger(), "[ExitElevator] Initialized with backup behavior");
}

BT::NodeStatus ExitElevator::onStart()
{
  if (!node_) return BT::NodeStatus::FAILURE;

  std::string floor;
  getInput<std::string>("floor", floor);
  getInput<double>("backup_dist", backup_dist_);
  getInput<double>("backup_speed", backup_speed_);

  // Reset state
  goal_completed_ = false;
  goal_succeeded_ = false;
  goal_handle_.reset();

  RCLCPP_INFO(node_->get_logger(), "[ExitElevator] Exiting elevator on floor %s using backup (dist=%.2f, speed=%.2f)",
              floor.c_str(), backup_dist_, backup_speed_);

  if (!backup_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "[ExitElevator] Backup action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // Create backup goal
  BackUp::Goal goal;
  goal.target.x = -backup_dist_;  // Negative = backward
  goal.target.y = 0.0;
  goal.target.z = 0.0;
  goal.speed = backup_speed_;
  goal.time_allowance.sec = 30;  // 30 seconds timeout

  auto send_goal_options = rclcpp_action::Client<BackUp>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandleBackUp::SharedPtr& goal_handle) {
      if (goal_handle) {
        goal_handle_ = goal_handle;
        RCLCPP_INFO(node_->get_logger(), "[ExitElevator] Backup goal accepted");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "[ExitElevator] Backup goal rejected");
        goal_completed_ = true;
        goal_succeeded_ = false;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleBackUp::WrappedResult& result) {
      goal_completed_ = true;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          goal_succeeded_ = true;
          RCLCPP_INFO(node_->get_logger(), "[ExitElevator] Backup SUCCEEDED");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          goal_succeeded_ = false;
          RCLCPP_WARN(node_->get_logger(), "[ExitElevator] Backup ABORTED");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          goal_succeeded_ = false;
          RCLCPP_WARN(node_->get_logger(), "[ExitElevator] Backup CANCELED");
          break;
        default:
          goal_succeeded_ = false;
          RCLCPP_WARN(node_->get_logger(), "[ExitElevator] Backup unknown result");
          break;
      }
    };

  backup_client_->async_send_goal(goal, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExitElevator::onRunning()
{
  if (goal_completed_) {
    if (goal_succeeded_) {
      RCLCPP_INFO(node_->get_logger(), "[ExitElevator] Successfully exited elevator");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[ExitElevator] Backup failed, but considering as done");
      // Even if backup didn't complete fully, we'll consider it done
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void ExitElevator::onHalted()
{
  if (goal_handle_) {
    backup_client_->async_cancel_goal(goal_handle_);
  }
  goal_completed_ = false;
  goal_succeeded_ = false;
  RCLCPP_WARN(node_->get_logger(), "[ExitElevator] Halted");
}

// ============== TurnInElevator ==============
TurnInElevator::TurnInElevator(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config),
    received_pose_(false),
    tolerance_(0.5),
    goal_completed_(false),
    goal_succeeded_(false),
    waiting_for_result_(false)
{
}

void TurnInElevator::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&TurnInElevator::poseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Initialized");
}

void TurnInElevator::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  received_pose_ = true;
}

void TurnInElevator::generateTurnWaypoints(int num_points)
{
  // Clear queue
  while (!turn_queue_.empty()) turn_queue_.pop();

  if (!received_pose_) return;

  double start_yaw = getYawFromQuaternion(start_pose_.orientation);

  // Circle center is 0.35m in front of the robot
  // Robot starts at the back of the circle and moves along the arc
  // 6 waypoints: 30, 60, 90, 120, 150, 180 degrees around the circle

  double radius = 0.35;
  double angle_step = M_PI / 6.0;  // 30 degrees per waypoint
  int num_waypoints = 6;  // Fixed 6 waypoints for 180 degree turn

  RCLCPP_INFO(node_->get_logger(),
              "[TurnInElevator] Start pose: (%.2f, %.2f), yaw=%.1f deg",
              start_pose_.position.x, start_pose_.position.y, start_yaw * 180.0 / M_PI);

  // Center of circle is 0.35m in front of robot
  double center_x = start_pose_.position.x + radius * std::cos(start_yaw);
  double center_y = start_pose_.position.y + radius * std::sin(start_yaw);

  RCLCPP_INFO(node_->get_logger(),
              "[TurnInElevator] Circle center: (%.2f, %.2f), radius: %.2f",
              center_x, center_y, radius);

  // Generate 6 waypoints: 30, 60, 90, 120, 150, 180 degrees
  // Robot starts at back of circle (180 deg from center's perspective)
  // and moves counterclockwise (to the left)
  for (int i = 1; i <= num_waypoints; ++i) {
    double turn_angle = angle_step * i;  // 30, 60, 90, 120, 150, 180 degrees

    // Robot's new yaw (turning left)
    double current_yaw = start_yaw + turn_angle;

    // Position on circle: robot starts at angle (start_yaw + PI) from center
    // and moves counterclockwise
    double circle_angle = start_yaw + M_PI - turn_angle;

    geometry_msgs::msg::Pose wp;
    wp.position.x = center_x + radius * std::cos(circle_angle);
    wp.position.y = center_y + radius * std::sin(circle_angle);
    wp.position.z = 0.0;
    wp.orientation = quaternionFromYaw(current_yaw);

    turn_queue_.push(wp);
    RCLCPP_INFO(node_->get_logger(),
                "[TurnInElevator] Waypoint %d/%d: (%.2f, %.2f), yaw=%.1f deg",
                i, num_waypoints, wp.position.x, wp.position.y, current_yaw * 180.0 / M_PI);
  }
}

bool TurnInElevator::sendGoalToPose(const geometry_msgs::msg::Pose& pose)
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "[TurnInElevator] Action server not available");
    return false;
  }

  // Reset state before sending new goal
  goal_completed_ = false;
  goal_succeeded_ = false;
  waiting_for_result_ = true;
  goal_handle_.reset();

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->now();
  goal.pose.pose = pose;

  RCLCPP_INFO(node_->get_logger(),
              "[TurnInElevator] Sending goal to (%.2f, %.2f)",
              pose.position.x, pose.position.y);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr& goal_handle) {
      if (goal_handle) {
        goal_handle_ = goal_handle;
        RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Goal accepted");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "[TurnInElevator] Goal rejected");
        goal_completed_ = true;
        goal_succeeded_ = false;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult& result) {
      goal_completed_ = true;
      waiting_for_result_ = false;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          goal_succeeded_ = true;
          RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Goal SUCCEEDED");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          goal_succeeded_ = false;
          RCLCPP_WARN(node_->get_logger(), "[TurnInElevator] Goal ABORTED");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          goal_succeeded_ = false;
          RCLCPP_WARN(node_->get_logger(), "[TurnInElevator] Goal CANCELED");
          break;
        default:
          goal_succeeded_ = false;
          RCLCPP_WARN(node_->get_logger(), "[TurnInElevator] Goal unknown result");
          break;
      }
    };

  action_client_->async_send_goal(goal, send_goal_options);
  return true;
}

bool TurnInElevator::sendNextWaypoint()
{
  if (turn_queue_.empty()) {
    return false;
  }

  current_target_ = turn_queue_.front();
  turn_queue_.pop();

  RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Sending next waypoint, %zu remaining in queue",
              turn_queue_.size());

  return sendGoalToPose(current_target_);
}

BT::NodeStatus TurnInElevator::onStart()
{
  RCLCPP_INFO(rclcpp::get_logger("TurnInElevator"), "[TurnInElevator] onStart() called");

  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("TurnInElevator"), "[TurnInElevator] node_ is null!");
    return BT::NodeStatus::FAILURE;
  }

  // Wait for pose if not received yet
  if (!received_pose_) {
    RCLCPP_WARN(node_->get_logger(), "[TurnInElevator] No pose received, waiting up to 3 seconds...");
    auto start_time = std::chrono::steady_clock::now();
    while (!received_pose_) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= 3) {
        RCLCPP_ERROR(node_->get_logger(), "[TurnInElevator] Timeout waiting for pose!");
        return BT::NodeStatus::FAILURE;
      }
    }
    RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Pose received!");
  }

  int num_waypoints = 8;
  getInput("num_waypoints", num_waypoints);
  getInput("min_waypoints_to_complete", min_waypoints_to_complete_);

  // Reset state
  goal_completed_ = false;
  goal_succeeded_ = false;
  waiting_for_result_ = false;
  waypoints_reached_ = 0;
  total_waypoints_ = num_waypoints + 1;  // +1 for final waypoint

  start_pose_ = current_pose_;

  RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Starting 180 degree turn with %d waypoints (complete after %d) at (%.2f, %.2f)",
              num_waypoints, min_waypoints_to_complete_, start_pose_.position.x, start_pose_.position.y);

  generateTurnWaypoints(num_waypoints);

  if (turn_queue_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[TurnInElevator] Failed to generate waypoints");
    return BT::NodeStatus::FAILURE;
  }

  // Send first waypoint
  if (!sendNextWaypoint()) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnInElevator::onRunning()
{
  if (!received_pose_) return BT::NodeStatus::RUNNING;

  // Check distance to current target
  double dx = current_pose_.position.x - current_target_.position.x;
  double dy = current_pose_.position.y - current_target_.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // Check angle difference to current target
  double current_yaw = getYawFromQuaternion(current_pose_.orientation);
  double target_yaw = getYawFromQuaternion(current_target_.orientation);
  double angle_diff = std::abs(normalizeAngle(current_yaw - target_yaw));

  // Tolerances for elevator turn
  double angle_tolerance = 0.35;  // ~20 degrees

  // Both distance AND angle must be within tolerance
  bool position_ok = (distance <= tolerance_);
  bool angle_ok = (angle_diff <= angle_tolerance);
  bool waypoint_reached = position_ok && angle_ok;

  // Check if we've reached minimum waypoints to complete
  if (waypoints_reached_ >= min_waypoints_to_complete_) {
    RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Turn complete! Reached %d/%d waypoints (min: %d)",
                waypoints_reached_, total_waypoints_, min_waypoints_to_complete_);
    if (goal_handle_) {
      action_client_->async_cancel_goal(goal_handle_);
    }
    return BT::NodeStatus::SUCCESS;
  }

  // If we've reached current waypoint, count it and move to next
  if (waypoint_reached && !goal_completed_) {
    waypoints_reached_++;
    RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Waypoint %d/%d reached (dist=%.2f, angle=%.1f deg)",
                waypoints_reached_, total_waypoints_, distance, angle_diff * 180.0 / M_PI);

    // Check if we've reached minimum waypoints
    if (waypoints_reached_ >= min_waypoints_to_complete_) {
      RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Turn complete! Minimum waypoints reached.");
      if (goal_handle_) {
        action_client_->async_cancel_goal(goal_handle_);
      }
      return BT::NodeStatus::SUCCESS;
    }

    // Move to next waypoint if available
    if (!turn_queue_.empty()) {
      if (goal_handle_) {
        action_client_->async_cancel_goal(goal_handle_);
      }
      if (!sendNextWaypoint()) {
        RCLCPP_ERROR(node_->get_logger(), "[TurnInElevator] Failed to send next waypoint");
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  // Check if current goal completed via result callback
  if (goal_completed_) {
    if (goal_succeeded_) {
      // Only count waypoint if goal actually succeeded
      waypoints_reached_++;
      RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Goal SUCCEEDED, waypoints reached: %d/%d",
                  waypoints_reached_, total_waypoints_);

      // Check if we've reached minimum waypoints
      if (waypoints_reached_ >= min_waypoints_to_complete_) {
        RCLCPP_INFO(node_->get_logger(), "[TurnInElevator] Turn complete! Minimum waypoints reached.");
        return BT::NodeStatus::SUCCESS;
      }

      // Send next waypoint if available
      if (!turn_queue_.empty()) {
        if (!sendNextWaypoint()) {
          RCLCPP_ERROR(node_->get_logger(), "[TurnInElevator] Failed to send next waypoint");
          return BT::NodeStatus::FAILURE;
        }
      }
    } else {
      // Goal failed/aborted/canceled - DO NOT count, just try next waypoint
      RCLCPP_WARN(node_->get_logger(), "[TurnInElevator] Goal FAILED/ABORTED, NOT counting. Trying next waypoint...");

      if (turn_queue_.empty()) {
        // No more waypoints to try
        RCLCPP_WARN(node_->get_logger(), "[TurnInElevator] No more waypoints, reached %d/%d",
                    waypoints_reached_, min_waypoints_to_complete_);
        // Only succeed if we reached minimum
        if (waypoints_reached_ >= min_waypoints_to_complete_) {
          return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
      }

      if (!sendNextWaypoint()) {
        RCLCPP_ERROR(node_->get_logger(), "[TurnInElevator] Failed to send next waypoint after failure");
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  return BT::NodeStatus::RUNNING;
}

void TurnInElevator::onHalted()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_completed_ = false;
  goal_succeeded_ = false;
  waiting_for_result_ = false;
  RCLCPP_WARN(node_->get_logger(), "[TurnInElevator] Halted");
}

// ============== NavigateToStairs ==============
NavigateToStairs::NavigateToStairs(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config),
    received_pose_(false),
    tolerance_(0.5),
    goal_sent_(false)
{
}

void NavigateToStairs::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&NavigateToStairs::poseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "[NavigateToStairs] Initialized");
}

void NavigateToStairs::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  received_pose_ = true;
}

bool NavigateToStairs::sendGoal()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToStairs] Action server not available");
    return false;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->now();
  goal.pose.pose = goal_pose_;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr& goal_handle) {
      if (goal_handle) {
        goal_handle_ = goal_handle;
        RCLCPP_INFO(node_->get_logger(), "[NavigateToStairs] Goal accepted");
      }
    };

  action_client_->async_send_goal(goal, send_goal_options);
  goal_sent_ = true;
  return true;
}

bool NavigateToStairs::isGoalReached(double tolerance)
{
  if (!received_pose_) return false;
  return calculateDistance(current_pose_, goal_pose_) <= tolerance;
}

BT::NodeStatus NavigateToStairs::onStart()
{
  if (!node_) return BT::NodeStatus::FAILURE;

  std::string floor;
  if (!getInput<std::string>("floor", floor)) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToStairs] Missing input [floor]");
    return BT::NodeStatus::FAILURE;
  }

  getInput<double>("tolerance", tolerance_);

  if (!g_config_loader) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToStairs] Config loader not initialized");
    return BT::NodeStatus::FAILURE;
  }

  FloorConfig floor_config;
  if (!g_config_loader->getFloorConfig(floor, floor_config)) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToStairs] Failed to get config for floor %s", floor.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!floor_config.stairs.has_stairs) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToStairs] No stairs configured for floor %s", floor.c_str());
    return BT::NodeStatus::FAILURE;
  }

  goal_pose_ = floor_config.stairs.front;

  RCLCPP_INFO(node_->get_logger(), "[NavigateToStairs] Going to stairs front on floor %s (%.2f, %.2f)",
              floor.c_str(), goal_pose_.position.x, goal_pose_.position.y);

  if (!sendGoal()) return BT::NodeStatus::FAILURE;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToStairs::onRunning()
{
  if (isGoalReached(tolerance_)) {
    RCLCPP_INFO(node_->get_logger(), "[NavigateToStairs] Reached stairs front");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void NavigateToStairs::onHalted()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
  RCLCPP_WARN(node_->get_logger(), "[NavigateToStairs] Halted");
}

}  // namespace sparo_navigation_core
