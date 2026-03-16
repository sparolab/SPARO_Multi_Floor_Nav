#include "sparo_navigation_core/stairs/compute_next_floor.hpp"

namespace sparo_navigation_core
{

BT::NodeStatus ComputeNextFloor::tick()
{
  int current = 0;
  std::string direction;

  if (!getInput<int>("current_floor", current) ||
      !getInput<std::string>("direction", direction))
  {
    RCLCPP_ERROR(node_->get_logger(), "[ComputeNextFloor] missing input");
    return BT::NodeStatus::FAILURE;
  }

  if (direction == "up")
  {
    setOutput<int>("next_floor", current + 1);
    return BT::NodeStatus::SUCCESS;
  }
  else if (direction == "down")
  {
    setOutput<int>("next_floor", current - 1);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(node_->get_logger(),
               "[ComputeNextFloor] invalid direction: %s", direction.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace sparo_navigation_core
