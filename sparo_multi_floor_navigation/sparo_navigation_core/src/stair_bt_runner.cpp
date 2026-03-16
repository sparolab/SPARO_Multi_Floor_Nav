#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

// Include all stair nodes
#include "sparo_navigation_core/stairs/align_with_stairs.hpp"
#include "sparo_navigation_core/stairs/check_floor_different.hpp"
#include "sparo_navigation_core/stairs/climb_flight_rl.hpp"
#include "sparo_navigation_core/stairs/compute_next_floor.hpp"
#include "sparo_navigation_core/stairs/get_current_floor.hpp"
#include "sparo_navigation_core/stairs/select_stair_entrance.hpp"
#include "sparo_navigation_core/stairs/turn_relative.hpp"
#include "sparo_navigation_core/stairs/move_forward_distance.hpp"
#include "sparo_navigation_core/stairs/nav2_navigate_to_pose.hpp"
#include "sparo_navigation_core/utility/utility_nodes.hpp"
#include "sparo_navigation_core/navigation/map_switch_action.hpp"
#include "sparo_navigation_core/navigation/set_initial_pose_action.hpp"
#include "sparo_navigation_core/config_loader.hpp"

using namespace std::chrono_literals;
using namespace sparo_navigation_core;

class sparoStairBT : public rclcpp::Node
{
public:
  sparoStairBT()
  : Node("sparo_stair_bt"), target_floor_(-1), tree_running_(false), current_floor_index_(0)
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

    RCLCPP_INFO(this->get_logger(), "=== sparo Stairs BT ===");
    RCLCPP_INFO(this->get_logger(), "Config: %s", config_file.c_str());

    // Initialize config loader
    g_config_loader = std::make_shared<ConfigLoader>();
    if (!g_config_loader->loadConfig(config_file)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load config!");
      return;
    }

    // Subscribe to floor request
    floor_request_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/stairs/floor_request", 10,
      std::bind(&sparoStairBT::floorRequestCallback, this, std::placeholders::_1));

    // Subscribe to current floor
    current_floor_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/current_floor", 10,
      std::bind(&sparoStairBT::currentFloorCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for floor requests on /stairs/floor_request");
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

    // Execute stairs BT
    executeStairBT();
  }

  void executeStairBT()
  {
    tree_running_ = true;

    // Create BT factory
    BT::BehaviorTreeFactory factory;
    registerNodes(factory);

    // Load stairs BT
    std::string xml_path = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/behavior_trees/subtrees/stairs_only.xml";

    try {
      auto tree = factory.createTreeFromFile(xml_path);

      // Set floor variables
      tree.rootBlackboard()->set("current_floor", current_floor_index_);
      tree.rootBlackboard()->set("target_floor", target_floor_);
      RCLCPP_INFO(this->get_logger(), "Loaded StairsSubtree, current_floor=%d, target_floor=%d",
                  current_floor_index_, target_floor_);

      // Tick tree
      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      int tick_count = 0;
      while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();
        std::this_thread::sleep_for(100ms);
        tick_count++;

        if (tick_count % 50 == 0) {
          RCLCPP_INFO(this->get_logger(), "Stairs BT running... (%.1fs)", tick_count * 0.1);
        }
      }

      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Stairs BT succeeded");
        auto complete_pub = this->create_publisher<std_msgs::msg::String>("/floor_change_complete", 10);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std_msgs::msg::String msg;
        msg.data = "stair";
        complete_pub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published /floor_change_complete");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Stairs BT failed");
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "BT error: %s", e.what());
    }

    tree_running_ = false;
  }

private:
  void registerNodes(BT::BehaviorTreeFactory& factory)
  {
    auto node_ptr = shared_from_this();

    factory.registerNodeType<AlignWithStairs>("AlignWithStairs");
    factory.registerNodeType<CheckFloorDifferent>("CheckFloorDifferent");
    factory.registerNodeType<ClimbFlightRL>("ClimbFlightRL");
    factory.registerNodeType<ComputeNextFloor>("ComputeNextFloor");
    factory.registerNodeType<GetCurrentFloor>("GetCurrentFloor");
    factory.registerNodeType<SelectStairEntrance>("SelectStairEntrance");
    factory.registerNodeType<TurnRelative>("TurnRelative");
    factory.registerNodeType<MoveForwardDistance>("MoveForwardDistance");
    factory.registerNodeType<Nav2NavigateToPose>("Nav2NavigateToPose");

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
  auto node = std::make_shared<sparoStairBT>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
