cmake_minimum_required(VERSION 3.8)
project(sparo_navigation_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  behaviortree_cpp_v3
  nav2_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  lifecycle_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  nav2_util
  pluginlib
)

# Build sparo BT Nav2 plugin library
add_library(sparo_bt_nav2_plugin SHARED
  # Navigation (SPARO)
  src/navigation/patrol_waypoints.cpp
  src/navigation/navigation_actions.cpp
  src/navigation/map_switch_action.cpp
  src/navigation/set_initial_pose_action.cpp
  src/navigation/get_patrol_pose_action.cpp
  src/navigation/navigate_to_intruder_action.cpp
  src/navigation/publish_floor_request.cpp
  src/navigation/wait_for_floor_change.cpp
  src/navigation/wait_for_topic_message.cpp

  # Elevator (SPARO + RCI)
  src/elevator/call_elevator_to_floor.cpp
  src/elevator/set_elevator_door.cpp
  src/elevator/wait_door_open.cpp
  src/elevator/wait_cabin_at_target.cpp
  src/elevator/wait_robot_near_elevator.cpp
  src/elevator/wait_robot_inside_cabin.cpp
  src/elevator/wait_robot_outside_elevator.cpp
  src/elevator/elevator_client.cpp
  src/elevator/select_nearest_elevator.cpp
  src/elevator/get_cabin_goal.cpp
  src/elevator/wait_floor_command.cpp
  src/elevator/ok_node.cpp
  src/elevator/check_elevator_accessible.cpp

  # Stairs (RCI)
  src/stairs/align_with_stairs.cpp
  src/stairs/check_floor_different.cpp
  src/stairs/climb_flight_rl.cpp
  src/stairs/compute_next_floor.cpp
  src/stairs/get_current_floor.cpp
  src/stairs/get_target_floor_from_topic.cpp
  src/stairs/move_forward_distance.cpp
  src/stairs/select_stair_entrance.cpp
  src/stairs/turn_relative.cpp
  src/stairs/nav2_navigate_to_pose.cpp

  # Door (RCI)
  src/door/door_plan_controller.cpp

  # Utility (SPARO)
  src/utility/utility_nodes.cpp

  # Core
  src/config_loader.cpp

  # Plugin registration
  src/sparo_bt_nav2_plugin.cpp
)

target_include_directories(sparo_bt_nav2_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sparo_bt_nav2_plugin
  ${dependencies}
)

target_link_libraries(sparo_bt_nav2_plugin
  yaml-cpp
)

# Build elevator BT runner
add_executable(sparo_elevator_bt_runner src/elevator_bt_runner.cpp)
ament_target_dependencies(sparo_elevator_bt_runner ${dependencies})
target_link_libraries(sparo_elevator_bt_runner sparo_bt_nav2_plugin yaml-cpp)

# Build stair BT runner
add_executable(sparo_stair_bt_runner src/stair_bt_runner.cpp)
ament_target_dependencies(sparo_stair_bt_runner ${dependencies})
target_link_libraries(sparo_stair_bt_runner sparo_bt_nav2_plugin yaml-cpp)

# Build door BT runner
add_executable(sparo_door_bt_runner src/door_bt_runner.cpp)
# Build intruder BT runner
add_executable(sparo_intruder_runner src/intruder_runner.cpp)
ament_target_dependencies(sparo_intruder_runner ${dependencies})
target_link_libraries(sparo_intruder_runner sparo_bt_nav2_plugin yaml-cpp)
ament_target_dependencies(sparo_door_bt_runner ${dependencies})
target_link_libraries(sparo_door_bt_runner sparo_bt_nav2_plugin yaml-cpp)



# Export Nav2 BT plugin
pluginlib_export_plugin_description_file(nav2_behavior_tree sparo_bt_plugins.xml)

# Install
install(TARGETS sparo_bt_nav2_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS sparo_elevator_bt_runner sparo_stair_bt_runner sparo_door_bt_runner sparo_intruder_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)


install(FILES sparo_bt_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/pc2_to_scan.py
  scripts/initial_pose_setter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(sparo_bt_nav2_plugin)
ament_export_dependencies(${dependencies})

ament_package()
