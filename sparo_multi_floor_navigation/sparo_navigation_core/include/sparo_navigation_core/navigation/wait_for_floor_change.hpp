#ifndef SPARO_NAVIGATION_CORE__WAIT_FOR_FLOOR_CHANGE_HPP_
#define SPARO_NAVIGATION_CORE__WAIT_FOR_FLOOR_CHANGE_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sparo_navigation_core
{

/**
 * @brief Wait until robot reaches target floor (by checking TF z position)
 */
class WaitForFloorChange : public BT::StatefulActionNode
{
public:
  WaitForFloorChange(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("target_floor", "Target floor index (0, 1, 2)"),
      BT::InputPort<std::string>("map_frame", "map", "Map frame"),
      BT::InputPort<std::string>("base_frame", "base_link", "Base frame"),
      BT::InputPort<double>("floor0_z", 0.0, "Floor 0 height"),
      BT::InputPort<double>("floor1_z", 5.0, "Floor 1 height"),
      BT::InputPort<double>("floor2_z", 10.0, "Floor 2 height"),
      BT::InputPort<double>("timeout", 180.0, "Timeout in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Time start_time_;
};

}  // namespace sparo_navigation_core

#endif  // SPARO_NAVIGATION_CORE__WAIT_FOR_FLOOR_CHANGE_HPP_
