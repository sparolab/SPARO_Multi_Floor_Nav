#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

// Include all elevator nodes
#include "sparo_navigation_core/elevator/select_nearest_elevator.hpp"
#include "sparo_navigation_core/elevator/call_elevator_to_floor.hpp"
#include "sparo_navigation_core/elevator/wait_cabin_at_target.hpp"
#include "sparo_navigation_core/elevator/wait_door_open.hpp"
#include "sparo_navigation_core/elevator/set_elevator_door.hpp"
#include "sparo_navigation_core/elevator/wait_robot_near_elevator.hpp"
#include "sparo_navigation_core/elevator/wait_robot_inside_cabin.hpp"
#include "sparo_navigation_core/elevator/wait_robot_outside_elevator.hpp"
#include "sparo_navigation_core/elevator/get_cabin_goal.hpp"
#include "sparo_navigation_core/elevator/check_elevator_accessible.hpp"
#include "sparo_navigation_core/stairs/nav2_navigate_to_pose.hpp"
#include "sparo_navigation_core/stairs/get_current_floor.hpp"
#include "sparo_navigation_core/utility/utility_nodes.hpp"
#include "sparo_navigation_core/navigation/map_switch_action.hpp"
#include "sparo_navigation_core/navigation/set_initial_pose_action.hpp"
#include "sparo_navigation_core/config_loader.hpp"

using namespace std::chrono_literals;
using namespace sparo_navigation_core;

class sparoElevatorBT : public rclcpp::Node
{
public:
  sparoElevatorBT()
  : Node("sparo_elevator_bt"), target_floor_(-1), tree_running_(false), current_floor_index_(0)
  {
    this->declare_parameter("config_file", "");
    this->declare_parameter("patrol_config_dir", "");
  }

  void init()
  {
    std::string config_file = this->get_parameter("config_file").as_string();
    std::string patrol_config_dir = this->get_parameter("patrol_config_dir").as_string();

    if (config_file.empty()) {
      config_file = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/config/navigation/floors.yaml";
    }
    if (patrol_config_dir.empty()) {
      patrol_config_dir = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/config/navigation";
    }

    RCLCPP_INFO(this->get_logger(), "=== sparo Elevator BT ===");
    RCLCPP_INFO(this->get_logger(), "Config: %s", config_file.c_str());

    // Initialize config loader
    g_config_loader = std::make_shared<ConfigLoader>();
    if (!g_config_loader->loadConfig(config_file)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load config!");
      return;
    }

    // Subscribe to floor request
    floor_request_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/elevator/floor_request", 10,
      std::bind(&sparoElevatorBT::floorRequestCallback, this, std::placeholders::_1));

    // Subscribe to current floor
    current_floor_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/current_floor", 10,
      std::bind(&sparoElevatorBT::currentFloorCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for floor requests on /elevator/floor_request");
  }

  void currentFloorCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string floor_name = msg->data;
    if (floor_name == "L1") current_floor_index_ = 0;
    else if (floor_name == "L2") current_floor_index_ = 1;
    else if (floor_name == "L3") current_floor_index_ = 2;
    else {
      RCLCPP_WARN(this->get_logger(), "Unknown floor name: %s", floor_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Current floor updated: %s (index=%d)", floor_name.c_str(), current_floor_index_);
  }

  void floorRequestCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (tree_running_) {
      RCLCPP_WARN(this->get_logger(), "BT already running, ignoring request");
      return;
    }

    target_floor_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received floor request: %d", target_floor_);

    // Execute elevator BT
    executeElevatorBT();
  }

  void executeElevatorBT()
  {
    tree_running_ = true;

    // Create BT factory
    BT::BehaviorTreeFactory factory;
    registerNodes(factory);

    // Load elevator BT from sparo subtrees
    std::string xml_path = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/behavior_trees/subtrees/elevator_only.xml";

    try {
      // Load and specify ElevatorSequence as the tree to execute
      auto tree = factory.createTreeFromFile(xml_path);

      // Set floor variables in blackboard
      tree.rootBlackboard()->set("current_floor", current_floor_index_);
      tree.rootBlackboard()->set("target_floor", target_floor_);
      RCLCPP_INFO(this->get_logger(), "Loaded ElevatorSequence, current_floor=%d, target_floor=%d",
                  current_floor_index_, target_floor_);

      // Tick tree until completion (without spinning - parent node handles spinning)
      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      int tick_count = 0;
      while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();
        std::this_thread::sleep_for(100ms);
        tick_count++;

        if (tick_count % 50 == 0) {
          RCLCPP_INFO(this->get_logger(), "Elevator BT running... (%.1fs)", tick_count * 0.1);
        }
      }

      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Elevator BT succeeded - publishing completion");
        std::string elevator_ns = tree.rootBlackboard()->get<std::string>("elevator_ns");
        auto complete_pub = this->create_publisher<std_msgs::msg::String>("/floor_change_complete", 10);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std_msgs::msg::String msg;
        msg.data = elevator_ns;  // "/lift1" or "/lift2"
        complete_pub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published completion with elevator: %s", elevator_ns.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Elevator BT failed - fallback to stairs");
        auto stairs_pub = this->create_publisher<std_msgs::msg::Int32>("/stairs/floor_request", 10);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std_msgs::msg::Int32 stairs_msg;
        stairs_msg.data = target_floor_;
        stairs_pub->publish(stairs_msg);
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "BT error: %s", e.what());
    }

    tree_running_ = false;
  }

  void switchMapAndSetPose()
  {
    // Determine floor name from target_floor index
    std::string floor_name;
    if (target_floor_ == 0) floor_name = "L1";
    else if (target_floor_ == 1) floor_name = "L2";
    else if (target_floor_ == 2) floor_name = "L3";
    else {
      RCLCPP_ERROR(this->get_logger(), "Invalid target_floor: %d", target_floor_);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Switching to %s map and setting initial pose...", floor_name.c_str());

    // TODO: Call SwitchMap and SetInitialPose
    // For now, just log
    RCLCPP_INFO(this->get_logger(), "Map switch and pose set complete for %s", floor_name.c_str());
  }

  void publishStairsFallback()
  {
    auto stairs_pub = this->create_publisher<std_msgs::msg::Int32>("/stairs/floor_request", 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std_msgs::msg::Int32 msg;
    msg.data = target_floor_;
    stairs_pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published fallback to /stairs/floor_request (floor=%d)", target_floor_);
  }

private:
  void registerNodes(BT::BehaviorTreeFactory& factory)
  {
    auto node_ptr = shared_from_this();

    factory.registerNodeType<GetCurrentFloor>("GetCurrentFloor");
    factory.registerNodeType<SelectNearestElevator>("SelectNearestElevator");
    factory.registerNodeType<CallElevatorToFloor>("CallElevatorToFloor");
    factory.registerNodeType<WaitCabinAtTarget>("WaitCabinAtTarget");
    factory.registerNodeType<WaitDoorOpen>("WaitDoorOpen");
    factory.registerNodeType<SetElevatorDoor>("SetElevatorDoor");
    factory.registerNodeType<WaitRobotNearElevator>("WaitRobotNearElevator");
    factory.registerNodeType<WaitRobotInsideCabin>("WaitRobotInsideCabin");
    factory.registerNodeType<WaitRobotOutsideElevator>("WaitRobotOutsideElevator");
    factory.registerNodeType<GetCabinGoal>("GetCabinGoal");
    factory.registerNodeType<Nav2NavigateToPose>("Nav2NavigateToPose");

    factory.registerBuilder<CheckElevatorAccessible>("CheckElevatorAccessible",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<CheckElevatorAccessible>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerBuilder<LogMessage>("LogMessage",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<LogMessage>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerBuilder<SwitchMap>("SwitchMap",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<SwitchMap>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerBuilder<SetInitialPoseAction>("SetInitialPose",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<SetInitialPoseAction>(name, config);
        node->initialize(node_ptr);
        return node;
      });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr floor_request_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_floor_sub_;
  int target_floor_;
  int current_floor_index_;
  bool tree_running_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sparoElevatorBT>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
