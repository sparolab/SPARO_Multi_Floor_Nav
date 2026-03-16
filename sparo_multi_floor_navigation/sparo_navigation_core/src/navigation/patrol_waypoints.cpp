#include "sparo_navigation_core/navigation/patrol_waypoints.hpp"
#include <cmath>

namespace sparo_navigation_core
{

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

PatrolWaypointsAction::PatrolWaypointsAction(
  const std::string& name,
  const BT::NodeConfiguration& conf)
: BT::StatefulActionNode(name, conf),
  current_waypoint_index_(0),
  waypoints_loaded_(false),
  goal_tolerance_(0.5),
  received_pose_(false),
  goal_sent_(false)
{
}

void PatrolWaypointsAction::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoint_markers", 10);

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&PatrolWaypointsAction::poseCallback, this, std::placeholders::_1));

  // Intruder alert subscription
  intruder_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/intruder_alert", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(intruder_mutex_);
      intruder_alert_ = *msg;
      has_intruder_alert_ = true;
      RCLCPP_WARN(node_->get_logger(), "[PatrolWaypoints] INTRUDER ALERT! Floor=%s, pos=(%.2f, %.2f)",
        msg->header.frame_id.c_str(), msg->pose.position.x, msg->pose.position.y);
    });

  // Intruder marker publisher
  intruder_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/intruder_marker", 10);
  
  // Floor request publisher (for triggering elevator/stairs)

  RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Initialized with intruder detection");
}

void PatrolWaypointsAction::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  received_pose_ = true;
}

void PatrolWaypointsAction::publishIntruderMarker(const geometry_msgs::msg::PoseStamped& alert)
{
  if (!intruder_marker_pub_ || !received_pose_) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_->now();
  marker.ns = "intruder_alert";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Position above robot (+2m in X direction)
  marker.pose.position.x = current_pose_.position.x + 2.0;
  marker.pose.position.y = current_pose_.position.y;
  marker.pose.position.z = current_pose_.position.z + 1.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.z = 1.0;

  // Red color
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.text = "INTRUDER DETECTED!";
  marker.lifetime = rclcpp::Duration::from_seconds(10.0);

  intruder_marker_pub_->publish(marker);
}

bool PatrolWaypointsAction::isWaypointReached(double tolerance)
{
  if (!received_pose_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                         "[PatrolWaypoints] No pose received yet");
    return false;
  }

  if (current_waypoint_index_ >= waypoints_.size()) {
    return false;
  }

  const auto& wp = waypoints_[current_waypoint_index_];
  double dx = current_pose_.position.x - wp.pose.position.x;
  double dy = current_pose_.position.y - wp.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "[PatrolWaypoints] Distance to waypoint %zu: %.2f (tolerance: %.2f)",
                       current_waypoint_index_ + 1, distance, tolerance);

  return distance <= tolerance;
}

void PatrolWaypointsAction::loadWaypoints(const std::string& floor)
{
  if (!g_config_loader) {
    RCLCPP_ERROR(node_->get_logger(), "Config loader not initialized!");
    return;
  }

  // Get waypoint type
  std::string waypoint_type = "patrol";
  getInput("waypoint_type", waypoint_type);

  if (waypoint_type == "patrol") {
    // Load patrol waypoints from separate patrol_L*.yaml file
    std::string patrol_config_dir;
    if (!getInput("patrol_config_dir", patrol_config_dir)) {
      RCLCPP_WARN(node_->get_logger(), "[PatrolWaypoints] No patrol_config_dir specified, using default");
      patrol_config_dir = "/home/test_ws/src/sparo_multi_floor_navigation/sparo_navigation_bringup/config/navigation";
    }

    if (!g_config_loader->loadPatrolWaypoints(floor, patrol_config_dir)) {
      RCLCPP_ERROR(node_->get_logger(), "[PatrolWaypoints] Failed to load patrol waypoints for floor %s", floor.c_str());
      return;
    }
  }

  FloorConfig floor_config;
  if (!g_config_loader->getFloorConfig(floor, floor_config)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get config for floor %s", floor.c_str());
    return;
  }

  if (waypoint_type == "stair_descent") {
    waypoints_ = floor_config.stair_descent_waypoints;
    RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Loaded %zu STAIR DESCENT waypoints for floor %s",
                waypoints_.size(), floor.c_str());
  } else {
    waypoints_ = floor_config.patrol_waypoints;
    RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Loaded %zu PATROL waypoints for floor %s",
                waypoints_.size(), floor.c_str());

    // Sort waypoints by distance from current pose (if available)
    if (received_pose_ && !waypoints_.empty()) {
      RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Sorting waypoints by distance from current pose (%.2f, %.2f)",
                  current_pose_.position.x, current_pose_.position.y);
      waypoints_ = g_config_loader->sortWaypointsByDistance(waypoints_, current_pose_);
      RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Starting from closest waypoint: (%.2f, %.2f)",
                  waypoints_[0].pose.position.x, waypoints_[0].pose.position.y);
    } else {
      RCLCPP_WARN(node_->get_logger(), "[PatrolWaypoints] No pose received yet, using waypoints in original order");
    }
  }

  current_floor_ = floor;
  waypoints_loaded_ = true;

  publishWaypointMarkers();
}

void PatrolWaypointsAction::publishWaypointMarkers()
{
  if (!marker_pub_ || waypoints_.empty()) return;

  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const auto& wp = waypoints_[i];

    // Determine color based on progress
    float r, g, b;
    if (i < current_waypoint_index_) {
      // Completed - green
      r = 0.0; g = 1.0; b = 0.0;
    } else if (i == current_waypoint_index_) {
      // Current - yellow
      r = 1.0; g = 1.0; b = 0.0;
    } else {
      // Pending - blue
      r = 0.0; g = 0.5; b = 1.0;
    }

    // Sphere marker
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "map";
    sphere.header.stamp = node_->now();
    sphere.ns = "patrol_waypoints_sphere";
    sphere.id = static_cast<int>(i);
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = wp.pose;
    sphere.pose.position.z = 1.0;  // Display at 0.5m height
    sphere.scale.x = 0.5;
    sphere.scale.y = 0.5;
    sphere.scale.z = 0.5;
    sphere.color.r = r; sphere.color.g = g; sphere.color.b = b;
    sphere.color.a = 0.6;
    marker_array.markers.push_back(sphere);
  }

  marker_pub_->publish(marker_array);
}

// Generate intermediate waypoints based on turn angle
// 60-90 deg: 1 point, 90-120 deg: 2 points, 120+ deg: 3 points
std::vector<geometry_msgs::msg::Pose> PatrolWaypointsAction::generateIntermediateWaypoints(
  const geometry_msgs::msg::Pose& current,
  const geometry_msgs::msg::Pose& target)
{
  std::vector<geometry_msgs::msg::Pose> intermediates;

  double current_yaw = getYawFromQuaternion(current.orientation);
  double dx = target.position.x - current.position.x;
  double dy = target.position.y - current.position.y;
  double target_direction = std::atan2(dy, dx);
  double angle_diff = normalizeAngle(target_direction - current_yaw);
  double abs_angle_diff = std::abs(angle_diff);
  double distance = std::sqrt(dx * dx + dy * dy);

  // 140도(2.44rad) 넘으면 왼쪽(반시계방향)으로 돌기
  if (abs_angle_diff > 2.44 && angle_diff < 0) {
    angle_diff += 2.0 * M_PI;
    abs_angle_diff = angle_diff;
  }

  // Threshold: ~90 degrees
  const double TURN_THRESHOLD = 1.57;

  if (abs_angle_diff <= TURN_THRESHOLD) {
    return intermediates;  // No intermediate needed
  }

  // Determine number of points based on angle
  int num_points;
  if (abs_angle_diff <= 1.57) {        // 60-90 degrees
    num_points = 1;
  } else if (abs_angle_diff <= 2.09) { // 90-120 degrees
    num_points = 2;
  } else {                              // 120+ degrees
    num_points = 3;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[PatrolWaypoints] Turn %.1f degrees, adding %d intermediate waypoint(s)",
              abs_angle_diff * 180.0 / M_PI, num_points);

  double step_angle = angle_diff / (num_points + 1);
  double max_step = 0.5;
  double step_distance = std::min(max_step, distance / (num_points + 1));

  geometry_msgs::msg::Pose prev_pose = current;

  for (int i = 1; i <= num_points; ++i) {
    double intermediate_yaw = current_yaw + step_angle * i;

    geometry_msgs::msg::Pose intermediate;
    intermediate.position.x = prev_pose.position.x + step_distance * std::cos(intermediate_yaw);
    intermediate.position.y = prev_pose.position.y + step_distance * std::sin(intermediate_yaw);
    intermediate.position.z = 0.0;
    intermediate.orientation = quaternionFromYaw(intermediate_yaw);

    intermediates.push_back(intermediate);
    prev_pose = intermediate;

    RCLCPP_INFO(node_->get_logger(),
                "[PatrolWaypoints] Intermediate %d/%d at (%.2f, %.2f)",
                i, num_points, intermediate.position.x, intermediate.position.y);
  }

  return intermediates;
}

bool PatrolWaypointsAction::sendGoalToPose(const geometry_msgs::msg::Pose& pose, const std::string& description)
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "[PatrolWaypoints] NavigateToPose action server not available");
    return false;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->now();
  goal.pose.pose = pose;

  // Force z=0 for 2D navigation
  goal.pose.pose.position.z = 0.0;

  RCLCPP_INFO(node_->get_logger(),
              "[PatrolWaypoints] Navigating to %s (%.2f, %.2f)",
              description.c_str(), pose.position.x, pose.position.y);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr& goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "[PatrolWaypoints] Goal was rejected by server");
      } else {
        RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Goal accepted by server");
        goal_handle_ = goal_handle;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult& result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Nav2 goal succeeded");
        goal_succeeded_.store(true);
      }
    };

  action_client_->async_send_goal(goal, send_goal_options);
  goal_sent_ = true;

  return true;
}

bool PatrolWaypointsAction::sendGoal()
{
  RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] sendGoal() called, index=%zu, waypoints_.size()=%zu",
              current_waypoint_index_, waypoints_.size());

  if (current_waypoint_index_ >= waypoints_.size()) {
    RCLCPP_WARN(node_->get_logger(), "[PatrolWaypoints] sendGoal() index out of range!");
    return false;
  }

  const auto& wp = waypoints_[current_waypoint_index_];
  RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Target waypoint: %s at (%.2f, %.2f)",
              wp.name.c_str(), wp.pose.position.x, wp.pose.position.y);

  // Direct navigation without intermediate waypoints
  while (!intermediate_queue_.empty()) intermediate_queue_.pop();
  intermediate_queue_.push(wp.pose);

  // Send first waypoint in queue
  if (!intermediate_queue_.empty()) {
    auto next_pose = intermediate_queue_.front();
    intermediate_queue_.pop();
    current_target_pose_ = next_pose;

    bool is_final = intermediate_queue_.empty();
    std::string desc = is_final ?
      ("waypoint " + std::to_string(current_waypoint_index_ + 1) + "/" + std::to_string(waypoints_.size()) + ": " + wp.name) :
      "intermediate waypoint";

    return sendGoalToPose(next_pose, desc);
  }

  return false;
}

BT::NodeStatus PatrolWaypointsAction::onStart()
{
  if (!node_) {
    return BT::NodeStatus::FAILURE;
  }

  // Load waypoints on first start
  if (!waypoints_loaded_) {
    std::string floor;
    if (!getInput("floor", floor)) {
      RCLCPP_ERROR(node_->get_logger(), "[PatrolWaypoints] Missing floor input");
      return BT::NodeStatus::FAILURE;
    }
    loadWaypoints(floor);
    current_waypoint_index_ = 0;

    // Use threshold from FloorConfig (loaded from patrol YAML)
    if (g_config_loader) {
      FloorConfig floor_config;
      if (g_config_loader->getFloorConfig(floor, floor_config)) {
        goal_tolerance_ = floor_config.position_tolerance;
        RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Using position_tolerance from patrol YAML: %.2f",
                    goal_tolerance_);
      }
    }
  }

  // Allow BT to override tolerance if specified
  // double bt_tolerance;
  // if (getInput("goal_tolerance", bt_tolerance)) {
  //   goal_tolerance_ = bt_tolerance;
  // }

  if (waypoints_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[PatrolWaypoints] No waypoints to patrol");
    return BT::NodeStatus::FAILURE;
  }

  // Clear goal_succeeded_ before starting patrol to avoid stale success from previous run
  goal_succeeded_.store(false);

  // Send first goal
  if (!sendGoal()) {
    return BT::NodeStatus::FAILURE;
  }

  publishWaypointMarkers();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PatrolWaypointsAction::onRunning()
{
  if (!received_pose_) return BT::NodeStatus::RUNNING;

  // Check for intruder alert
  {
    std::lock_guard<std::mutex> lock(intruder_mutex_);
    if (has_intruder_alert_) {
      RCLCPP_WARN(node_->get_logger(),
        "[PatrolWaypoints] !!! INTRUDER DETECTED IN onRunning !!! Stopping patrol.");

      // Publish intruder marker
      publishIntruderMarker(intruder_alert_);
      
      // Clear waypoint markers
      if (marker_pub_) {
        visualization_msgs::msg::MarkerArray clear_markers;
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_markers.markers.push_back(delete_marker);
        marker_pub_->publish(clear_markers);
        RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Cleared waypoint markers");
      }

      // Set output ports for BT to handle
      setOutput("intruder_detected", true);
      setOutput("intruder_floor", intruder_alert_.header.frame_id);
      setOutput("intruder_pose", intruder_alert_);
      

      // Check stop_on_intruder flag
      bool stop_on_intruder = false;
      getInput("stop_on_intruder", stop_on_intruder);

      // Reset state
      has_intruder_alert_ = false;
      waypoints_loaded_ = false;
      current_waypoint_index_ = 0;

      if (stop_on_intruder) {
        RCLCPP_WARN(node_->get_logger(), "[PatrolWaypoints] Returning FAILURE to stop BT");
        return BT::NodeStatus::FAILURE;  // Stop BT
      } else {
        RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Returning SUCCESS to continue BT");
        return BT::NodeStatus::SUCCESS;  // Continue BT
      }
    }
  }

  // Check current distance to current target
  double dx = current_pose_.position.x - current_target_pose_.position.x;
  double dy = current_pose_.position.y - current_target_pose_.position.y;
  double distance_to_current = std::sqrt(dx * dx + dy * dy);

  // Check if there are more waypoints in queue (intermediate or final)
  // Only use distance-based check (ignore Nav2 goal success)
  bool close_enough = (distance_to_current <= goal_tolerance_);

  if (close_enough) {
    if (!intermediate_queue_.empty()) {
      RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Reached intermediate (dist=%.2fm), %zu more in queue",
                  distance_to_current, intermediate_queue_.size());

      // Cancel current goal if still active
      if (goal_handle_) {
        auto status = goal_handle_->get_status();
        if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
            status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
          action_client_->async_cancel_goal(goal_handle_);
        }
        goal_handle_.reset();
      }

      auto next_pose = intermediate_queue_.front();
      intermediate_queue_.pop();
      current_target_pose_ = next_pose;

      bool is_final = intermediate_queue_.empty();
      const auto& wp = waypoints_[current_waypoint_index_];
      std::string desc = is_final ?
        ("waypoint " + std::to_string(current_waypoint_index_ + 1) + "/" + std::to_string(waypoints_.size()) + ": " + wp.name) :
        "intermediate waypoint";

      if (!sendGoalToPose(next_pose, desc)) {
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  // Check if final waypoint reached (distance-based only)
  bool waypoint_reached = isWaypointReached(goal_tolerance_);
  bool queue_empty = intermediate_queue_.empty();

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[PatrolWaypoints] State check: dist_to_target=%.2f, waypoint_reached=%d, queue_empty=%d, index=%zu/%zu",
    distance_to_current, waypoint_reached, queue_empty, current_waypoint_index_, waypoints_.size());

  if (waypoint_reached && queue_empty) {
    RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Reached waypoint %zu/%zu (dist=%.2fm)",
                current_waypoint_index_ + 1, waypoints_.size(), distance_to_current);

    // Cancel current goal if still active
    if (goal_handle_) {
      auto status = goal_handle_->get_status();
      if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
          status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
        RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Canceling active goal");
        action_client_->async_cancel_goal(goal_handle_);
      }
      goal_handle_.reset();
    }

    // Check if we should stop at this waypoint
    int stop_at_index = -1;
    getInput("stop_at_waypoint_index", stop_at_index);

    if (stop_at_index >= 0 && static_cast<size_t>(stop_at_index) == current_waypoint_index_) {
      RCLCPP_INFO(node_->get_logger(),
                  "[PatrolWaypoints] Reached stop_at_waypoint_index %d. Patrol stopping.",
                  stop_at_index);
      waypoints_loaded_ = false;
      current_waypoint_index_ = 0;
      return BT::NodeStatus::SUCCESS;
    }

    // Move to next waypoint
    current_waypoint_index_++;

    if (current_waypoint_index_ >= waypoints_.size()) {
      // All waypoints completed - update markers to show all green
      publishWaypointMarkers();

      RCLCPP_INFO(node_->get_logger(),
                  "[PatrolWaypoints] Patrol complete! Visited all %zu waypoints", waypoints_.size());
      waypoints_loaded_ = false;
      current_waypoint_index_ = 0;
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Moving to next waypoint %zu/%zu",
                current_waypoint_index_ + 1, waypoints_.size());

    if (!sendGoal()) {
      RCLCPP_ERROR(node_->get_logger(), "[PatrolWaypoints] sendGoal() failed!");
      return BT::NodeStatus::FAILURE;
    }

    publishWaypointMarkers();
  }

  return BT::NodeStatus::RUNNING;
}

void PatrolWaypointsAction::onHalted()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "[PatrolWaypoints] Canceling current goal");
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_sent_ = false;
  goal_succeeded_.store(false);
  // Clear intermediate queue
  while (!intermediate_queue_.empty()) intermediate_queue_.pop();
}

}  // namespace sparo_navigation_core
