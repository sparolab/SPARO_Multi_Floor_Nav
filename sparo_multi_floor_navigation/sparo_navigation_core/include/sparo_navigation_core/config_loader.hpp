#ifndef SPARO_NAVIGATION_CORE__CONFIG_LOADER_HPP_
#define SPARO_NAVIGATION_CORE__CONFIG_LOADER_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace sparo_navigation_core
{

struct Waypoint
{
  std::string name;
  geometry_msgs::msg::Pose pose;
};

struct StairsConfig
{
  geometry_msgs::msg::Pose front;        // 계단 앞 위치
  geometry_msgs::msg::Pose mid_landing;  // 중간층 도착 위치
  bool has_stairs = false;               // 계단 정보 존재 여부
};

struct FloorConfig
{
  std::string floor_number;
  int floor_index;
  double height;        // Floor height from floors.yaml
  double floor_z;       // Same as height (for backward compatibility)
  std::string map_file;
  geometry_msgs::msg::Pose initial_pose;  // Deprecated - use patrol YAML
  geometry_msgs::msg::Pose elevator_front;  // Deprecated - use patrol YAML
  geometry_msgs::msg::Pose elevator_inside;  // Deprecated - use patrol YAML
  geometry_msgs::msg::Pose elevator_exit;  // Deprecated - use patrol YAML
  StairsConfig stairs;  // Deprecated - use stairs.yaml
  std::vector<Waypoint> patrol_waypoints;  // Loaded from patrol_L*.yaml
  std::vector<Waypoint> stair_descent_waypoints;  // Loaded from patrol_L*.yaml

  // Patrol thresholds from patrol_L*.yaml
  double position_tolerance = 0.3;
  double orientation_tolerance = 0.2;
};

class ConfigLoader
{
public:
  ConfigLoader();

  bool loadConfig(const std::string& config_file_path);
  bool getFloorConfig(const std::string& floor_number, FloorConfig& config) const;
  int getFloorIndex(const std::string& floor_number) const;
  double getFloorHeight(int floor_index) const;
  std::vector<std::string> getAvailableFloors() const;

  // Load patrol waypoints from separate patrol_L*.yaml file
  bool loadPatrolWaypoints(const std::string& floor_number, const std::string& patrol_config_dir);

  // Get initial pose from patrol YAML file
  bool getInitialPoseFromPatrol(const std::string& floor_number,
                                 const std::string& pose_type,
                                 const std::string& patrol_config_dir,
                                 geometry_msgs::msg::Pose& pose);

  // Sort waypoints by distance from a given pose (closest first)
  static std::vector<Waypoint> sortWaypointsByDistance(
    const std::vector<Waypoint>& waypoints,
    const geometry_msgs::msg::Pose& reference_pose);

private:
  std::map<std::string, FloorConfig> floor_configs_;
  std::map<int, double> floor_heights_;
  rclcpp::Logger logger_;
  std::string patrol_config_dir_;
};

// Global shared config loader
extern std::shared_ptr<ConfigLoader> g_config_loader;

}  // namespace sparo_navigation_core

#endif  // SPARO_NAVIGATION_CORE__CONFIG_LOADER_HPP_
