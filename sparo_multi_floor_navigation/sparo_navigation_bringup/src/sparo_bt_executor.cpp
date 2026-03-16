#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "sparo_navigation_core/config_loader.hpp"

// Include all BT node headers
#include "sparo_navigation_core/navigation/patrol_waypoints.hpp"
#include "sparo_navigation_core/navigation/set_initial_pose_action.hpp"
#include "sparo_navigation_core/navigation/map_switch_action.hpp"
#include "sparo_navigation_core/navigation/get_patrol_pose_action.hpp"
#include "sparo_navigation_core/navigation/navigate_to_intruder_action.hpp"
#include "sparo_navigation_core/navigation/publish_floor_request.hpp"
#include "sparo_navigation_core/navigation/wait_for_floor_change.hpp"
#include "sparo_navigation_core/navigation/wait_for_topic_message.hpp"
#include "sparo_navigation_core/navigation/navigation_actions.hpp"
#include "sparo_navigation_core/utility/utility_nodes.hpp"
#include "sparo_navigation_core/elevator/check_elevator_accessible.hpp"
#include "sparo_navigation_core/stairs/nav2_navigate_to_pose.hpp"

// Elevator nodes
#include "sparo_navigation_core/elevator/call_elevator_to_floor.hpp"
#include "sparo_navigation_core/elevator/set_elevator_door.hpp"
#include "sparo_navigation_core/elevator/wait_door_open.hpp"
#include "sparo_navigation_core/elevator/wait_cabin_at_target.hpp"
#include "sparo_navigation_core/elevator/wait_robot_near_elevator.hpp"
#include "sparo_navigation_core/elevator/wait_robot_inside_cabin.hpp"
#include "sparo_navigation_core/elevator/wait_robot_outside_elevator.hpp"
#include "sparo_navigation_core/elevator/select_nearest_elevator.hpp"
#include "sparo_navigation_core/elevator/get_cabin_goal.hpp"
#include "sparo_navigation_core/elevator/ok_node.hpp"

// Stairs nodes
#include "sparo_navigation_core/stairs/align_with_stairs.hpp"
#include "sparo_navigation_core/stairs/check_floor_different.hpp"
#include "sparo_navigation_core/stairs/climb_flight_rl.hpp"
#include "sparo_navigation_core/stairs/compute_next_floor.hpp"
#include "sparo_navigation_core/stairs/get_current_floor.hpp"
#include "sparo_navigation_core/stairs/select_stair_entrance.hpp"
#include "sparo_navigation_core/stairs/turn_relative.hpp"
#include "sparo_navigation_core/stairs/move_forward_distance.hpp"
#include "sparo_navigation_core/door/door_plan_controller.hpp"

using namespace std::chrono_literals;
using namespace sparo_navigation_core;

class sparoBTExecutor : public rclcpp::Node
{
public:
  sparoBTExecutor()
  : Node("sparo_bt_executor"), tree_complete_(false)
  {
    // Declare parameters (use_sim_time is auto-declared by ROS2)
    this->declare_parameter("bt_xml", "");
    this->declare_parameter("config_file", "");
    this->declare_parameter("patrol_config_dir", "");
  }

  void init()
  {
    // Get parameters
    std::string bt_xml_path = this->get_parameter("bt_xml").as_string();
    std::string config_file = this->get_parameter("config_file").as_string();
    std::string patrol_config_dir = this->get_parameter("patrol_config_dir").as_string();

    // Use defaults if not provided
    std::string pkg_share = ament_index_cpp::get_package_share_directory("sparo_navigation_bringup");
    if (bt_xml_path.empty()) {
      bt_xml_path = pkg_share + "/behavior_trees/main_scenario.xml";
    }
    if (config_file.empty()) {
      config_file = pkg_share + "/config/navigation/floors.yaml";
    }
    if (patrol_config_dir.empty()) {
      patrol_config_dir = pkg_share + "/config/navigation";
    }

    RCLCPP_INFO(this->get_logger(), "=== sparo BT Executor ===");
    RCLCPP_INFO(this->get_logger(), "BT XML: %s", bt_xml_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Config: %s", config_file.c_str());
    RCLCPP_INFO(this->get_logger(), "Patrol Config Dir: %s", patrol_config_dir.c_str());

    // Initialize config loader
    g_config_loader = std::make_shared<ConfigLoader>();
    if (!g_config_loader->loadConfig(config_file)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load config file!");
      return;
    }

    // Create BT factory
    BT::BehaviorTreeFactory factory;

    // Register all BT nodes
    registerNodes(factory);

    // Load the behavior tree
    try {
      tree_ = factory.createTreeFromFile(bt_xml_path);
      RCLCPP_INFO(this->get_logger(), "Behavior tree loaded successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load BT: %s", e.what());
      return;
    }

    // Create timer to tick the tree
    timer_ = this->create_wall_timer(100ms, std::bind(&sparoBTExecutor::tickTree, this));

    RCLCPP_INFO(this->get_logger(), "BT Executor initialized, starting tree execution");
  }

private:
  void registerNodes(BT::BehaviorTreeFactory& factory)
  {
    auto node_ptr = shared_from_this();

    // Navigation nodes
    factory.registerBuilder<PatrolWaypointsAction>("PatrolWaypoints",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<PatrolWaypointsAction>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerBuilder<SetInitialPoseAction>("SetInitialPose",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<SetInitialPoseAction>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerBuilder<SwitchMap>("SwitchMap",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<SwitchMap>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerBuilder<GetPatrolPoseAction>("GetPatrolPose",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<GetPatrolPoseAction>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerBuilder<NavigateToIntruderAction>("NavigateToIntruder",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<NavigateToIntruderAction>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerBuilder<PublishFloorRequest>("PublishFloorRequest",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<PublishFloorRequest>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerNodeType<WaitForFloorChange>("WaitForFloorChange");
    factory.registerNodeType<WaitForTopicMessage>("WaitForTopicMessage");

    factory.registerBuilder<CheckElevatorAccessible>("CheckElevatorAccessible",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<CheckElevatorAccessible>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    // Utility nodes
    factory.registerBuilder<LogMessage>("LogMessage",
      [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<LogMessage>(name, config);
        node->initialize(node_ptr);
        return node;
      });

    factory.registerNodeType<Nav2NavigateToPose>("Nav2NavigateToPose");

    // Elevator nodes
    factory.registerNodeType<CallElevatorToFloor>("CallElevatorToFloor");
    factory.registerNodeType<SetElevatorDoor>("SetElevatorDoor");
    factory.registerNodeType<WaitDoorOpen>("WaitDoorOpen");
    factory.registerNodeType<WaitCabinAtTarget>("WaitCabinAtTarget");
    factory.registerNodeType<WaitRobotNearElevator>("WaitRobotNearElevator");
    factory.registerNodeType<WaitRobotInsideCabin>("WaitRobotInsideCabin");
    factory.registerNodeType<WaitRobotOutsideElevator>("WaitRobotOutsideElevator");
    factory.registerNodeType<SelectNearestElevator>("SelectNearestElevator");
    factory.registerNodeType<GetCabinGoal>("GetCabinGoal");
    factory.registerNodeType<Ok>("Ok");

    // Stairs nodes
    factory.registerNodeType<AlignWithStairs>("AlignWithStairs");
    factory.registerNodeType<CheckFloorDifferent>("CheckFloorDifferent");
    factory.registerNodeType<ClimbFlightRL>("ClimbFlightRL");
    factory.registerNodeType<ComputeNextFloor>("ComputeNextFloor");
    factory.registerNodeType<GetCurrentFloor>("GetCurrentFloor");
    factory.registerNodeType<SelectStairEntrance>("SelectStairEntrance");
    factory.registerNodeType<TurnRelative>("TurnRelative");
    factory.registerNodeType<MoveForwardDistance>("MoveForwardDistance");
    
    // Door node
    factory.registerNodeType<DoorPlanController>("DoorPlanController");

    // Built-in BT nodes

    RCLCPP_INFO(this->get_logger(), "All BT nodes registered");
  }

  void tickTree()
  {
    if (tree_complete_) {
      return;
    }

    BT::NodeStatus status = tree_.tickRoot();

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "=== Behavior Tree SUCCEEDED ===");
      tree_complete_ = true;
      timer_->cancel();
    }
    else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "=== Behavior Tree FAILED ===");
      tree_complete_ = true;
      timer_->cancel();
    }
  }

  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool tree_complete_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sparoBTExecutor>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
