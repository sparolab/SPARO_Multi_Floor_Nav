#include "sparo_navigation_core/config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <algorithm>

namespace sparo_navigation_core
{

std::shared_ptr<ConfigLoader> g_config_loader = nullptr;

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

// Helper: calculate 2D distance between two poses
static double calculateDistance(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2)
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

ConfigLoader::ConfigLoader()
  : logger_(rclcpp::get_logger("ConfigLoader"))
{
}

bool ConfigLoader::loadConfig(const std::string& config_file_path)
{
  RCLCPP_INFO(logger_, "Loading floors configuration from: %s", config_file_path.c_str());

  try {
    YAML::Node config = YAML::LoadFile(config_file_path);

    if (!config["floors"]) {
      RCLCPP_ERROR(logger_, "No 'floors' section found in config file");
      return false;
    }

    // Load floor configurations (simplified - only metadata)
    for (const auto& floor_node : config["floors"]) {
      std::string floor_num = floor_node.first.as<std::string>();
      YAML::Node floor_data = floor_node.second;

      FloorConfig floor_config;
      floor_config.floor_number = floor_num;
      floor_config.floor_index = floor_data["floor_index"].as<int>();
      floor_config.height = floor_data["height"].as<double>();
      floor_config.map_file = floor_data["map_file"].as<std::string>();
      floor_config.floor_z = floor_config.height;

      // Store floor height mapping
      floor_heights_[floor_config.floor_index] = floor_config.height;

      // Note: All poses now loaded from patrol_L*.yaml via loadPatrolWaypoints()
      // and getInitialPoseFromPatrol()

      floor_configs_[floor_num] = floor_config;
      RCLCPP_INFO(logger_, "Loaded floor %s metadata: index=%d, height=%.1f, map=%s",
                  floor_num.c_str(), floor_config.floor_index,
                  floor_config.height, floor_config.map_file.c_str());
    }

    RCLCPP_INFO(logger_, "Floors configuration loaded. Available floors: %zu", floor_configs_.size());
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Error loading config: %s", e.what());
    return false;
  }
}

bool ConfigLoader::getFloorConfig(const std::string& floor_number, FloorConfig& config) const
{
  auto it = floor_configs_.find(floor_number);
  if (it != floor_configs_.end()) {
    config = it->second;
    return true;
  }
  return false;
}

int ConfigLoader::getFloorIndex(const std::string& floor_number) const
{
  auto it = floor_configs_.find(floor_number);
  if (it != floor_configs_.end()) {
    return it->second.floor_index;
  }
  return -1;
}

double ConfigLoader::getFloorHeight(int floor_index) const
{
  auto it = floor_heights_.find(floor_index);
  if (it != floor_heights_.end()) {
    return it->second;
  }
  return 0.0;
}

std::vector<std::string> ConfigLoader::getAvailableFloors() const
{
  std::vector<std::string> floors;
  for (const auto& [floor_num, config] : floor_configs_) {
    floors.push_back(floor_num);
  }
  return floors;
}

bool ConfigLoader::loadPatrolWaypoints(const std::string& floor_number, const std::string& patrol_config_dir)
{
  // Construct patrol yaml path: patrol_config_dir/patrol_L*.yaml
  std::string patrol_file = patrol_config_dir + "/patrol_" + floor_number + ".yaml";

  RCLCPP_INFO(logger_, "Loading patrol waypoints from: %s", patrol_file.c_str());

  try {
    YAML::Node config = YAML::LoadFile(patrol_file);

    if (!config["patrol"]) {
      RCLCPP_ERROR(logger_, "No 'patrol' section found in %s", patrol_file.c_str());
      return false;
    }

    auto patrol = config["patrol"];

    // Get floor height
    double floor_height = 0.0;
    if (patrol["height"]) {
      floor_height = patrol["height"].as<double>();
    }

    // Get thresholds (optional, for future use)
    double position_tolerance = 0.3;
    double orientation_tolerance = 0.2;
    double wait_time = 5.0;

    if (config["thresholds"]) {
      auto thresholds = config["thresholds"];
      if (thresholds["distance"]) position_tolerance = thresholds["distance"].as<double>();
      else if (thresholds["position"]) position_tolerance = thresholds["position"].as<double>();
      if (thresholds["orientation"]) orientation_tolerance = thresholds["orientation"].as<double>();
      if (thresholds["wait_time"]) wait_time = thresholds["wait_time"].as<double>();
    }

    // Load waypoints
    auto it = floor_configs_.find(floor_number);
    if (it == floor_configs_.end()) {
      RCLCPP_ERROR(logger_, "Floor %s not found in floor_configs", floor_number.c_str());
      return false;
    }

    // Clear existing patrol waypoints
    it->second.patrol_waypoints.clear();

    if (patrol["waypoints"]) {
      int wp_index = 0;
      for (const auto& wp_node : patrol["waypoints"]) {
        Waypoint wp;
        wp.name = "waypoint_" + std::to_string(wp_index++);

        // Position
        wp.pose.position.x = wp_node["x"].as<double>();
        wp.pose.position.y = wp_node["y"].as<double>();
        wp.pose.position.z = floor_height;  // Use floor height from patrol config

        // Orientation from yaw
        double yaw = wp_node["yaw"].as<double>();
        wp.pose.orientation = quaternionFromYaw(yaw);

        it->second.patrol_waypoints.push_back(wp);
      }

      // Store thresholds in FloorConfig
      it->second.position_tolerance = position_tolerance;
      it->second.orientation_tolerance = orientation_tolerance;

      RCLCPP_INFO(logger_, "Loaded %zu patrol waypoints for floor %s (height: %.2f, position_tol: %.2f)",
                  it->second.patrol_waypoints.size(), floor_number.c_str(), floor_height, position_tolerance);
      return true;
    }

    return false;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Error loading patrol waypoints: %s", e.what());
    return false;
  }
}

bool ConfigLoader::getInitialPoseFromPatrol(const std::string& floor_number,
                                             const std::string& pose_type,
                                             const std::string& patrol_config_dir,
                                             geometry_msgs::msg::Pose& pose)
{
  std::string patrol_file = patrol_config_dir + "/patrol_" + floor_number + ".yaml";

  RCLCPP_INFO(logger_, "Getting initial_pose.%s from %s", pose_type.c_str(), patrol_file.c_str());

  try {
    YAML::Node config = YAML::LoadFile(patrol_file);

    if (!config["patrol"] || !config["patrol"]["initial_pose"]) {
      RCLCPP_ERROR(logger_, "No patrol.initial_pose found in %s", patrol_file.c_str());
      return false;
    }

    auto initial_poses = config["patrol"]["initial_pose"];

    // Check if pose_type exists
    if (!initial_poses[pose_type]) {
      RCLCPP_ERROR(logger_, "Pose type '%s' not found in patrol.initial_pose", pose_type.c_str());
      return false;
    }

    auto pose_node = initial_poses[pose_type];

    // Get floor height
    double floor_height = 0.0;
    if (config["patrol"]["height"]) {
      floor_height = config["patrol"]["height"].as<double>();
    }

    // Parse pose
    pose.position.x = pose_node["x"].as<double>();
    pose.position.y = pose_node["y"].as<double>();
    pose.position.z = floor_height;

    double yaw = pose_node["yaw"].as<double>();
    pose.orientation = quaternionFromYaw(yaw);

    RCLCPP_INFO(logger_, "Loaded initial_pose.%s: (%.2f, %.2f, %.2f, yaw=%.2f)",
                pose_type.c_str(), pose.position.x, pose.position.y, pose.position.z, yaw);
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Error loading initial pose: %s", e.what());
    return false;
  }
}

std::vector<Waypoint> ConfigLoader::sortWaypointsByDistance(
  const std::vector<Waypoint>& waypoints,
  const geometry_msgs::msg::Pose& reference_pose)
{
  if (waypoints.empty()) {
    return waypoints;
  }

  // Create a copy with distance information
  std::vector<std::pair<double, Waypoint>> waypoints_with_distance;

  for (const auto& wp : waypoints) {
    double distance = calculateDistance(reference_pose, wp.pose);
    waypoints_with_distance.push_back({distance, wp});
  }

  // Sort by distance
  std::sort(waypoints_with_distance.begin(), waypoints_with_distance.end(),
    [](const auto& a, const auto& b) { return a.first < b.first; });

  // Extract sorted waypoints
  std::vector<Waypoint> sorted_waypoints;
  for (const auto& [dist, wp] : waypoints_with_distance) {
    sorted_waypoints.push_back(wp);
  }

  return sorted_waypoints;
}

}  // namespace sparo_navigation_core
