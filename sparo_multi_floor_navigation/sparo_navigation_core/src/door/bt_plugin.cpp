#include <behaviortree_cpp_v3/bt_factory.h>
#include "sparo_navigation_core/door/door_plan_controller.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<DoorPlanController>("DoorPlanController"); 
}
