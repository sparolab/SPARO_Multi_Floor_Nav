#include "behaviortree_cpp_v3/bt_factory.h"

// Navigation (SPARO)
#include "sparo_navigation_core/navigation/patrol_waypoints.hpp"
#include "sparo_navigation_core/navigation/navigation_actions.hpp"
#include "sparo_navigation_core/navigation/map_switch_action.hpp"
#include "sparo_navigation_core/navigation/set_initial_pose_action.hpp"

// Elevator (SPARO + RCI)
#include "sparo_navigation_core/elevator/call_elevator_to_floor.hpp"
#include "sparo_navigation_core/elevator/set_elevator_door.hpp"
#include "sparo_navigation_core/elevator/wait_door_open.hpp"
#include "sparo_navigation_core/elevator/wait_cabin_at_target.hpp"
#include "sparo_navigation_core/elevator/wait_robot_near_elevator.hpp"
#include "sparo_navigation_core/elevator/wait_robot_inside_cabin.hpp"
#include "sparo_navigation_core/elevator/wait_robot_outside_elevator.hpp"
#include "sparo_navigation_core/stairs/get_current_floor.hpp"
#include "sparo_navigation_core/elevator/select_nearest_elevator.hpp"
#include "sparo_navigation_core/elevator/get_cabin_goal.hpp"
#include "sparo_navigation_core/elevator/wait_floor_command.hpp"
#include "sparo_navigation_core/stairs/nav2_navigate_to_pose.hpp"
#include "sparo_navigation_core/elevator/ok_node.hpp"
#include "sparo_navigation_core/elevator/check_elevator_accessible.hpp"

// Stairs (RCI)
#include "sparo_navigation_core/stairs/align_with_stairs.hpp"
#include "sparo_navigation_core/stairs/check_floor_different.hpp"
#include "sparo_navigation_core/stairs/climb_flight_rl.hpp"
#include "sparo_navigation_core/stairs/compute_next_floor.hpp"
#include "sparo_navigation_core/stairs/get_current_floor.hpp"
#include "sparo_navigation_core/stairs/get_target_floor_from_topic.hpp"
#include "sparo_navigation_core/stairs/move_forward_distance.hpp"
#include "sparo_navigation_core/stairs/select_stair_entrance.hpp"
#include "sparo_navigation_core/stairs/turn_relative.hpp"

// Door (RCI)
#include "sparo_navigation_core/door/door_plan_controller.hpp"

// Utility (SPARO)
#include "sparo_navigation_core/utility/utility_nodes.hpp"

#include "pluginlib/class_list_macros.hpp"

BT_REGISTER_NODES(factory)
{
  using namespace sparo_navigation_core;

  // Navigation nodes (SPARO)
  factory.registerNodeType<PatrolWaypointsAction>("PatrolWaypoints");
  factory.registerNodeType<NavigateToElevatorFront>("NavigateToElevatorFront");
  factory.registerNodeType<EnterElevator>("EnterElevator");
  factory.registerNodeType<ExitElevator>("ExitElevator");
  factory.registerNodeType<TurnInElevator>("TurnInElevator");
  factory.registerNodeType<NavigateToStairs>("NavigateToStairs");
  factory.registerNodeType<SwitchMap>("SwitchMap");
  factory.registerNodeType<SetInitialPoseAction>("SetInitialPose");

  // Elevator nodes (RCI - explicit namespace)
  factory.registerNodeType<sparo_navigation_core::CallElevatorToFloor>("CallElevatorToFloor");
  factory.registerNodeType<sparo_navigation_core::SetElevatorDoor>("SetElevatorDoor");
  factory.registerNodeType<sparo_navigation_core::WaitDoorOpen>("WaitDoorOpen");
  factory.registerNodeType<sparo_navigation_core::WaitCabinAtTarget>("WaitCabinAtTarget");
  factory.registerNodeType<sparo_navigation_core::WaitRobotNearElevator>("WaitRobotNearElevator");
  factory.registerNodeType<sparo_navigation_core::WaitRobotInsideCabin>("WaitRobotInsideCabin");
  factory.registerNodeType<sparo_navigation_core::WaitRobotOutsideElevator>("WaitRobotOutsideElevator");
  factory.registerNodeType<sparo_navigation_core::GetCurrentFloor>("GetCurrentFloor");
  factory.registerNodeType<sparo_navigation_core::SelectNearestElevator>("SelectNearestElevator");
  factory.registerNodeType<sparo_navigation_core::GetCabinGoal>("GetCabinGoal");
  factory.registerNodeType<sparo_navigation_core::WaitFloorCommand>("WaitFloorCommand");
  factory.registerNodeType<sparo_navigation_core::Nav2NavigateToPose>("Nav2NavigateToPose");
  factory.registerNodeType<sparo_navigation_core::Ok>("Ok");
  
  // Elevator nodes (SPARO)
  factory.registerNodeType<CheckElevatorAccessible>("CheckElevatorAccessible");

  // Stairs nodes (RCI - explicit namespace)
  factory.registerNodeType<sparo_navigation_core::AlignWithStairs>("AlignWithStairs");
  factory.registerNodeType<sparo_navigation_core::CheckFloorDifferent>("CheckFloorDifferent");
  factory.registerNodeType<sparo_navigation_core::ClimbFlightRL>("ClimbFlightRL");
  factory.registerNodeType<sparo_navigation_core::ComputeNextFloor>("ComputeNextFloor");
  factory.registerNodeType<sparo_navigation_core::GetTargetFloorFromTopic>("GetTargetFloorFromTopic");
  factory.registerNodeType<sparo_navigation_core::MoveForwardDistance>("MoveForwardDistance");
  factory.registerNodeType<sparo_navigation_core::SelectStairEntrance>("SelectStairEntrance");
  factory.registerNodeType<sparo_navigation_core::TurnRelative>("TurnRelative");

  // Door nodes (RCI - no namespace)
  factory.registerNodeType<DoorPlanController>("DoorPlanController");

  // Utility nodes (SPARO)
  factory.registerNodeType<LogMessage>("LogMessage");
  factory.registerNodeType<IsSystemInitialized>("IsSystemInitialized");
}
