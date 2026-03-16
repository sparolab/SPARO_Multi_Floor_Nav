#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace sparo_navigation_core
{

/**
 * CheckElevatorAccessible - Costmap 기반 엘리베이터 진입 가능 여부 확인
 *
 * 엘리베이터 내부 좌표의 costmap cost를 확인하여:
 * - cost가 낮으면 SUCCESS (진입 가능)
 * - cost가 높으면 (장애물) FAILURE (계단으로)
 */
class CheckElevatorAccessible : public BT::ConditionNode
{
public:
  CheckElevatorAccessible(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor", "Floor number"),
      BT::InputPort<double>("timeout", 10.0, "Unused (kept for compatibility)")
    };
  }

  BT::NodeStatus tick() override;

  void initialize(rclcpp::Node::SharedPtr node);

private:
  int8_t getCostAt(double x, double y);
  void publishStatusMarker(const std::string& text, bool accessible, double x, double y);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Costmap data
  std::vector<int8_t> costmap_data_;
  double costmap_resolution_{0.05};
  double costmap_origin_x_{0.0};
  double costmap_origin_y_{0.0};
  uint32_t costmap_width_{0};
  uint32_t costmap_height_{0};
  bool costmap_received_{false};
};

}  // namespace sparo_navigation_core
