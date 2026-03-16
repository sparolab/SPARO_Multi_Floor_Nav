#pragma once
#include <string>
#include <yaml-cpp/yaml.h>

namespace sparo_navigation_core {

struct ElevatorZonesConfig
{
  double zone_min_x{0.0}, zone_min_y{0.0};
  double zone_max_x{0.0}, zone_max_y{0.0};

  double cabin_min_x{0.0}, cabin_min_y{0.0};
  double cabin_max_x{0.0}, cabin_max_y{0.0};

  double front_offset{0.4};
  double rear_offset{-0.4};
  double entrance_yaw{0.0};  // Entrance direction (robot should face opposite inside cabin)
};

inline bool load_zones_config(const std::string& yaml_path, ElevatorZonesConfig& out)
{
  try
  {
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root["zones"]) {
      return false;
    }
    auto zones = root["zones"];

    auto zmin = zones["zone_min"];
    auto zmax = zones["zone_max"];
    out.zone_min_x = zmin[0].as<double>();
    out.zone_min_y = zmin[1].as<double>();
    out.zone_max_x = zmax[0].as<double>();
    out.zone_max_y = zmax[1].as<double>();

    auto cmin = zones["cabin_min"];
    auto cmax = zones["cabin_max"];
    out.cabin_min_x = cmin[0].as<double>();
    out.cabin_min_y = cmin[1].as<double>();
    out.cabin_max_x = cmax[0].as<double>();
    out.cabin_max_y = cmax[1].as<double>();

    if (root["robot"])
    {
      auto robot = root["robot"];
      if (robot["front_offset"]) {
        out.front_offset = robot["front_offset"].as<double>();
      }
      if (robot["rear_offset"]) {
        out.rear_offset = robot["rear_offset"].as<double>();
      }
    }

    // Get entrance yaw from entrances array
    if (root["elevator"] && root["elevator"]["entrances"]) {
      auto entrances = root["elevator"]["entrances"];
      if (entrances.IsSequence() && entrances.size() > 0) {
        // Use first entrance yaw as reference
        if (entrances[0]["yaw"]) {
          out.entrance_yaw = entrances[0]["yaw"].as<double>();
        }
      }
    }

    return true;
  }
  catch (const std::exception& e)
  {
    (void)e;
    return false;
  }
}

} // namespace sparo_navigation_core
