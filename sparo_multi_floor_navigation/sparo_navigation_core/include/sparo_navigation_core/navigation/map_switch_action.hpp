#ifndef MULTI_FLOOR_NAV2_BT__MAP_SWITCH_ACTION_HPP_
#define MULTI_FLOOR_NAV2_BT__MAP_SWITCH_ACTION_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sparo_navigation_core/config_loader.hpp"

namespace sparo_navigation_core
{

/**
 * @brief BT Action Node to switch map to target floor
 */
class SwitchMap : public BT::StatefulActionNode
{
public:
  SwitchMap(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Target floor number")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  void clearWaypointMarkers();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_load_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_markers_pub_;
  std::shared_future<std::shared_ptr<nav2_msgs::srv::LoadMap::Response>> future_result_;
  bool service_called_;
};

}  // namespace sparo_navigation_core

#endif  // MULTI_FLOOR_NAV2_BT__MAP_SWITCH_ACTION_HPP_
