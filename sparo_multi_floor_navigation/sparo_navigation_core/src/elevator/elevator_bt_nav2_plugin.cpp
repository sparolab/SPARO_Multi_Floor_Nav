#include <behaviortree_cpp_v3/bt_factory.h>
#include "sparo_navigation_core/elevator/ok_node.hpp"
#include "sparo_navigation_core/elevator/call_elevator_to_floor.hpp"
#include "sparo_navigation_core/elevator/set_elevator_door.hpp"
#include "sparo_navigation_core/elevator/wait_door_open.hpp"
#include "sparo_navigation_core/elevator/wait_cabin_at_target.hpp"
#include "sparo_navigation_core/elevator/wait_robot_near_elevator.hpp"
#include "sparo_navigation_core/elevator/wait_robot_inside_cabin.hpp"
#include "sparo_navigation_core/elevator/wait_robot_outside_elevator.hpp"
#include "sparo_navigation_core/elevator/get_current_floor.hpp"
#include "sparo_navigation_core/elevator/select_nearest_elevator.hpp"
#include "sparo_navigation_core/elevator/get_cabin_goal.hpp"
#include "sparo_navigation_core/elevator/wait_floor_command.hpp"

using namespace sparo_navigation_core;

BT_REGISTER_NODES(factory)
{
  // XML에서 쓰는 태그 이름이랑 **완전히 동일**해야 함
  factory.registerNodeType<Ok>("Ok");

  factory.registerNodeType<CallElevatorToFloor>("CallElevatorToFloor");
  factory.registerNodeType<SetElevatorDoor>("SetElevatorDoor");
  factory.registerNodeType<WaitDoorOpen>("WaitDoorOpen");
  factory.registerNodeType<WaitCabinAtTarget>("WaitCabinAtTarget");

  factory.registerNodeType<WaitRobotNearElevator>("WaitRobotNearElevator");
  factory.registerNodeType<WaitRobotInsideCabin>("WaitRobotInsideCabin");
  factory.registerNodeType<WaitRobotOutsideElevator>("WaitRobotOutsideElevator");

  factory.registerNodeType<GetCurrentFloor>("GetCurrentFloor");
  factory.registerNodeType<SelectNearestElevator>("SelectNearestElevator");
  factory.registerNodeType<GetCabinGoal>("GetCabinGoal");
  factory.registerNodeType<WaitFloorCommand>("WaitFloorCommand");

}
