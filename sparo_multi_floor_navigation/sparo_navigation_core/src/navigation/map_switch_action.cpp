#include "sparo_navigation_core/navigation/map_switch_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <chrono>

namespace sparo_navigation_core
{

SwitchMap::SwitchMap(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config),
    service_called_(false)
{
}

void SwitchMap::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  map_load_client_ = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoint_markers", 10);
  clear_markers_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/clear_waypoint_markers", 10);
  RCLCPP_INFO(node_->get_logger(), "[SwitchMap] Initialized");
}

BT::NodeStatus SwitchMap::onStart()
{
  if (!node_) return BT::NodeStatus::FAILURE;

  std::string floor;
  if (!getInput<std::string>("floor", floor)) {
    RCLCPP_ERROR(node_->get_logger(), "[SwitchMap] Missing required input [floor]");
    return BT::NodeStatus::FAILURE;
  }

  if (!map_load_client_->wait_for_service(std::chrono::seconds(15))) {
    RCLCPP_WARN(node_->get_logger(), "[SwitchMap] LoadMap service not available after 15s");
    return BT::NodeStatus::FAILURE;
  }

  FloorConfig floor_config;
  if (!g_config_loader || !g_config_loader->getFloorConfig(floor, floor_config)) {
    RCLCPP_ERROR(node_->get_logger(), "[SwitchMap] Failed to get config for floor %s", floor.c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::string maps_dir = ament_index_cpp::get_package_share_directory("sparo_navigation_bringup") + "/maps/";
  std::string map_path = maps_dir + floor_config.map_file;

  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = map_path;

  future_result_ = map_load_client_->async_send_request(request);
  service_called_ = true;

  RCLCPP_INFO(node_->get_logger(), "[SwitchMap] Switching to floor %s map: %s",
              floor.c_str(), map_path.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SwitchMap::onRunning()
{
  if (!service_called_) {
    return BT::NodeStatus::RUNNING;
  }

  auto wait_result = future_result_.wait_for(std::chrono::milliseconds(100));

  if (wait_result == std::future_status::ready) {
    auto result = future_result_.get();

    if (result->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[SwitchMap] Successfully switched map");
      clearWaypointMarkers();
      service_called_ = false;
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[SwitchMap] Failed to load map (result=%d)", result->result);
      service_called_ = false;
      return BT::NodeStatus::FAILURE;
    }
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "[SwitchMap] Still waiting for map service response...");
  }

  return BT::NodeStatus::RUNNING;
}

void SwitchMap::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[SwitchMap] Halted");
  service_called_ = false;
}

void SwitchMap::clearWaypointMarkers()
{
  clear_markers_pub_->publish(std_msgs::msg::Empty());

  visualization_msgs::msg::MarkerArray delete_array;
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = "map";
  delete_marker.header.stamp = node_->now();
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_array.markers.push_back(delete_marker);
  marker_pub_->publish(delete_array);

  RCLCPP_INFO(node_->get_logger(), "[SwitchMap] Cleared all waypoint markers");

  std::string floor;
  if (!getInput<std::string>("floor", floor)) return;

  FloorConfig floor_config;
  if (!g_config_loader || !g_config_loader->getFloorConfig(floor, floor_config)) return;

  if (floor_config.patrol_waypoints.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[SwitchMap] No waypoints for floor %s", floor.c_str());
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  for (size_t i = 0; i < floor_config.patrol_waypoints.size(); ++i) {
    const auto& wp = floor_config.patrol_waypoints[i];

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "map";
    sphere.header.stamp = node_->now();
    sphere.ns = "patrol_waypoints_sphere";
    sphere.id = static_cast<int>(i);
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = wp.pose;
    sphere.scale.x = 0.5;
    sphere.scale.y = 0.5;
    sphere.scale.z = 0.5;
    sphere.color.r = 0.0;
    sphere.color.g = 0.5;
    sphere.color.b = 1.0;
    sphere.color.a = 0.6;
    marker_array.markers.push_back(sphere);
  }

  marker_pub_->publish(marker_array);
  RCLCPP_INFO(node_->get_logger(), "[SwitchMap] Published %zu waypoint markers for floor %s",
              floor_config.patrol_waypoints.size(), floor.c_str());
}

}  // namespace sparo_navigation_core
