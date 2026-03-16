#include "sparo_navigation_core/elevator/wait_floor_command.hpp"

namespace sparo_navigation_core {

using std_msgs::msg::Int32;

WaitFloorCommand::WaitFloorCommand(const std::string& name,
                                   const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  static int instance_count = 0;
  node_ = std::make_shared<rclcpp::Node>("wait_floor_command_" + std::to_string(instance_count++));
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
}

void WaitFloorCommand::ensureSubscription()
{
  if (sub_) {
    return;
  }

  // topic_name 포트에서 토픽 이름을 읽어옴
  if (!getInput("topic_name", topic_name_) || topic_name_.empty()) {
    topic_name_ = "/elevator/floor_request";  // 기본값 (추측입니다)
  }

  sub_ = node_->create_subscription<Int32>(
    topic_name_, rclcpp::QoS(10),
    [this](const Int32::SharedPtr msg)
    {
      floor_.store(msg->data, std::memory_order_relaxed);
      got_msg_.store(true, std::memory_order_relaxed);
      RCLCPP_INFO(node_->get_logger(),
        "[WaitFloorCommand] got floor request: %d", msg->data);
    });
}

BT::NodeStatus WaitFloorCommand::onStart()
{
  got_msg_.store(false, std::memory_order_relaxed);
  ensureSubscription();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitFloorCommand::onRunning()
{
  // 콜백 처리를 위해 spin_some
  exec_->spin_some();

  if (!got_msg_.load(std::memory_order_relaxed)) {
    return BT::NodeStatus::RUNNING;
  }

  int floor = floor_.load(std::memory_order_relaxed);
  setOutput("target_floor", floor);

  RCLCPP_INFO(node_->get_logger(),
    "[WaitFloorCommand] set target_floor = %d", floor);

  return BT::NodeStatus::SUCCESS;
}

void WaitFloorCommand::onHalted()
{
  // 필요한 경우 상태 초기화
  got_msg_.store(false, std::memory_order_relaxed);
}

} // namespace sparo_navigation_core
