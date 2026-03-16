cmake_minimum_required(VERSION 3.8)
project(sparo_navigation_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sparo_navigation_core REQUIRED)

include_directories(
  ${sparo_navigation_core_INCLUDE_DIRS}
)

# Build BT executor
add_executable(sparo_bt_executor src/sparo_bt_executor.cpp)

target_include_directories(sparo_bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${sparo_navigation_core_INCLUDE_DIRS}
)

ament_target_dependencies(sparo_bt_executor
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  ament_index_cpp
  tf2_msgs
  sparo_navigation_core
)

target_link_libraries(sparo_bt_executor
  ${sparo_navigation_core_LIBRARIES}
)

# Install executable
install(TARGETS sparo_bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  behavior_trees
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
