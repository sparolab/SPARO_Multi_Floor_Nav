#include "sparo_navigation_core/elevator/get_cabin_goal.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>

namespace sparo_navigation_core {

GetCabinGoal::GetCabinGoal(const std::string& name,
                           const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetCabinGoal::tick()
{
  std::string yaml_path;
  if (!getInput("elevator_yaml", yaml_path)) {
    throw BT::RuntimeError("GetCabinGoal: missing required input [elevator_yaml]");
  }

  int floor_idx = 0;
  // floor_idx는 현재 계산에는 사용하지 않지만, 포트만 읽어서 보관 가능
  getInput("floor_idx", floor_idx);

  auto goal = makeCabinGoal(yaml_path, floor_idx);
  setOutput("cabin_goal", goal);

  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::PoseStamped
GetCabinGoal::makeCabinGoal(const std::string& yaml_path, int floor_idx)
{
  YAML::Node root = YAML::LoadFile(yaml_path);

  auto zones = root["zones"];
  auto cabin_min = zones["cabin_min"];
  auto cabin_max = zones["cabin_max"];

  if (!cabin_min || !cabin_max || cabin_min.size() < 2 || cabin_max.size() < 2) {
    throw BT::RuntimeError("GetCabinGoal: invalid zones.cabin_min/max in yaml: " + yaml_path);
  }

  double min_x = cabin_min[0].as<double>();
  double min_y = cabin_min[1].as<double>();
  double max_x = cabin_max[0].as<double>();
  double max_y = cabin_max[1].as<double>();

  geometry_msgs::msg::PoseStamped goal;

  goal.header.frame_id = "map";
  goal.header.stamp = rclcpp::Clock().now();

  goal.pose.position.x = 0.5 * (min_x + max_x);
  goal.pose.position.y = 0.5 * (min_y + max_y);
  goal.pose.position.z = 0.0;

  // Get yaw from entrances[floor_idx]
  double yaw = 0.0;
  auto elevator = root["elevator"];
  if (elevator && elevator["entrances"] && floor_idx >= 0) {
    auto entrances = elevator["entrances"];
    if (static_cast<size_t>(floor_idx) < entrances.size()) {
      auto entrance = entrances[floor_idx];
      if (entrance["yaw"]) {
        double entrance_yaw = entrance["yaw"].as<double>();
        yaw = entrance_yaw + M_PI;  // Reverse to face door
        while (yaw > M_PI) yaw -= 2.0 * M_PI;
        while (yaw < -M_PI) yaw += 2.0 * M_PI;
      }
    }
  }

  // Set orientation from yaw
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.orientation.w = std::cos(yaw / 2.0);

  return goal;
}

}  
