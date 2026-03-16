#include "sparo_navigation_core/elevator/check_elevator_accessible.hpp"
#include "sparo_navigation_core/config_loader.hpp"
#include <cmath>
#include <yaml-cpp/yaml.h>

namespace sparo_navigation_core
{

CheckElevatorAccessible::CheckElevatorAccessible(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

void CheckElevatorAccessible::initialize(rclcpp::Node::SharedPtr node)
{
  // Create own node and executor (like SelectNearestElevator)
  static int instance_count = 0;
  node_ = std::make_shared<rclcpp::Node>("check_elevator_accessible_" + std::to_string(instance_count++));
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  // Global costmap subscription
  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap", rclcpp::QoS(10).best_effort(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      costmap_data_.assign(msg->data.begin(), msg->data.end());
      costmap_resolution_ = msg->info.resolution;
      costmap_origin_x_ = msg->info.origin.position.x;
      costmap_origin_y_ = msg->info.origin.position.y;
      costmap_width_ = msg->info.width;
      costmap_height_ = msg->info.height;
      costmap_received_ = true;
    });

  // Marker publisher for RViz status display
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "/elevator_status_marker", 10);

  RCLCPP_INFO(node_->get_logger(),
    "[CheckElevatorAccessible] Initialized with own executor");
}

void CheckElevatorAccessible::publishStatusMarker(
  const std::string& text, bool accessible, double x, double y)
{
  if (!marker_pub_) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_->now();
  marker.ns = "elevator_status";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 2.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.z = 0.8;

  if (accessible) {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  } else {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  marker.color.a = 1.0;

  marker.text = text;
  marker.lifetime = rclcpp::Duration::from_seconds(10.0);

  marker_pub_->publish(marker);
}

int8_t CheckElevatorAccessible::getCostAt(double x, double y)
{
  if (!costmap_received_ || costmap_data_.empty()) {
    return -1;
  }

  int cell_x = static_cast<int>((x - costmap_origin_x_) / costmap_resolution_);
  int cell_y = static_cast<int>((y - costmap_origin_y_) / costmap_resolution_);

  if (cell_x < 0 || cell_x >= static_cast<int>(costmap_width_) ||
      cell_y < 0 || cell_y >= static_cast<int>(costmap_height_)) {
    return -1;
  }

  size_t index = static_cast<size_t>(cell_y) * costmap_width_ + static_cast<size_t>(cell_x);
  return costmap_data_[index];
}

BT::NodeStatus CheckElevatorAccessible::tick()
{
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckElevatorAccessible"), "Node not initialized!");
    return BT::NodeStatus::FAILURE;
  }

  using namespace std::chrono_literals;

  // Spin own executor to process costmap callbacks
  exec_->spin_some(0ms);

  std::string floor;
  getInput("floor", floor);

  std::string elevator_ns;
  getInput("elevator_ns", elevator_ns);

  // Wait briefly for costmap to update after door opens
  RCLCPP_INFO(node_->get_logger(),
    "[CheckElevatorAccessible] Waiting 2s for fresh costmap...");

  for (int i = 0; i < 20; i++) {
    exec_->spin_some(0ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(node_->get_logger(),
    "[CheckElevatorAccessible] Checking elevator accessibility for floor %s", floor.c_str());

  if (!g_config_loader) {
    RCLCPP_ERROR(node_->get_logger(), "[CheckElevatorAccessible] Config loader not available!");
    return BT::NodeStatus::FAILURE;
  }

  FloorConfig floor_config;
  if (!g_config_loader->getFloorConfig(floor, floor_config)) {
    RCLCPP_ERROR(node_->get_logger(),
      "[CheckElevatorAccessible] Floor %s config not found!", floor.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!costmap_received_) {
    RCLCPP_WARN(node_->get_logger(),
      "[CheckElevatorAccessible] No costmap received yet, assuming accessible");
    return BT::NodeStatus::SUCCESS;
  }

  // Log costmap info for debugging
  RCLCPP_INFO(node_->get_logger(),
    "[CheckElevatorAccessible] Costmap: origin=(%.2f, %.2f), res=%.2f, size=%ux%u",
    costmap_origin_x_, costmap_origin_y_, costmap_resolution_, costmap_width_, costmap_height_);

  // Determine which elevator (lift1 or lift2)
  std::string lift_name = "lift1";
  if (elevator_ns == "/lift2") {
    lift_name = "lift2";
  }

  RCLCPP_INFO(node_->get_logger(), "[CheckElevatorAccessible] Checking %s cabin area", lift_name.c_str());

  // Load elevator YAML to get cabin zone
  std::string elevator_yaml;
  if (lift_name == "lift1") {
    elevator_yaml = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/config/elevator/elevator_lift1.yaml";
  } else {
    elevator_yaml = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/config/elevator/elevator_lift2.yaml";
  }

  // Parse YAML to get cabin bounds
  double cabin_min_x, cabin_min_y, cabin_max_x, cabin_max_y;
  try {
    YAML::Node y = YAML::LoadFile(elevator_yaml);
    if (!y["zones"] || !y["zones"]["cabin_min"] || !y["zones"]["cabin_max"]) {
      RCLCPP_ERROR(node_->get_logger(), "[CheckElevatorAccessible] Missing cabin zones in %s", elevator_yaml.c_str());
      return BT::NodeStatus::SUCCESS;  // Assume accessible if config missing
    }

    auto cabin_min = y["zones"]["cabin_min"];
    auto cabin_max = y["zones"]["cabin_max"];
    cabin_min_x = cabin_min[0].as<double>();
    cabin_min_y = cabin_min[1].as<double>();
    cabin_max_x = cabin_max[0].as<double>();
    cabin_max_y = cabin_max[1].as<double>();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node_->get_logger(), "[CheckElevatorAccessible] Failed to read %s: %s", elevator_yaml.c_str(), ex.what());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[CheckElevatorAccessible] Cabin area: [%.2f,%.2f] x [%.2f,%.2f]",
    cabin_min_x, cabin_max_x, cabin_min_y, cabin_max_y);

  // Sample cabin area with a grid
  const double SAMPLE_RESOLUTION = 0.2;  // Check every 20cm
  const int8_t OBSTACLE_THRESHOLD = 50;

  int total_samples = 0;
  int blocked_count = 0;
  int8_t max_cost = 0;
  int sum_cost = 0;

  for (double x = cabin_min_x; x <= cabin_max_x; x += SAMPLE_RESOLUTION) {
    for (double y = cabin_min_y; y <= cabin_max_y; y += SAMPLE_RESOLUTION) {
      int8_t cost = getCostAt(x, y);

      if (cost >= 0) {  // Valid cell
        total_samples++;
        sum_cost += cost;

        if (cost > max_cost) {
          max_cost = cost;
        }

        if (cost > OBSTACLE_THRESHOLD) {
          blocked_count++;
          RCLCPP_DEBUG(node_->get_logger(),
            "[CheckElevatorAccessible] Blocked cell at (%.2f, %.2f): cost=%d",
            x, y, cost);
        }
      }
    }
  }

  double avg_cost = total_samples > 0 ? (double)sum_cost / total_samples : 0.0;
  double occupancy_ratio = total_samples > 0 ? (double)blocked_count / total_samples : 0.0;

  RCLCPP_INFO(node_->get_logger(),
    "[CheckElevatorAccessible] Cabin occupancy: blocked=%d/%d (%.1f%%), max_cost=%d, avg_cost=%.1f",
    blocked_count, total_samples, occupancy_ratio * 100.0, max_cost, avg_cost);

  double elevator_center_x = (cabin_min_x + cabin_max_x) / 2.0;
  double elevator_center_y = (cabin_min_y + cabin_max_y) / 2.0;
  double marker_x = elevator_center_x + 5.0;
  double marker_y = elevator_center_y + 5.0;

  // If 70% or more of cabin area is blocked, consider elevator inaccessible
  if (occupancy_ratio >= 0.70) {
    RCLCPP_WARN(node_->get_logger(),
      "[CheckElevatorAccessible] Cabin BLOCKED! %.1f%% occupied (max_cost=%d)",
      occupancy_ratio * 100.0, max_cost);

    std::string status_text = "ELEVATOR BLOCKED\n(" + std::to_string(static_cast<int>(occupancy_ratio * 100)) + "% occupied)";
    publishStatusMarker(status_text, false, marker_x, marker_y);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[CheckElevatorAccessible] Elevator ACCESSIBLE! max_cost=%d", max_cost);

  std::string status_text = "ELEVATOR OK\n(Path clear)";
  publishStatusMarker(status_text, true, marker_x, marker_y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sparo_navigation_core
