#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#include "sparo_navigation_core/door/door_plan_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sparo_door_bt_runner");

  RCLCPP_INFO(node->get_logger(), "=== sparo Door BT Runner ===");

  std::string bt_xml = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/behavior_trees/subtrees/door_test_L1.xml";

  RCLCPP_INFO(node->get_logger(), "BT XML: %s", bt_xml.c_str());

  BT::BehaviorTreeFactory factory;

  // Register door nodes
  factory.registerNodeType<DoorPlanController>("DoorPlanController");

  RCLCPP_INFO(node->get_logger(), "Creating BT from XML...");
  auto tree = factory.createTreeFromFile(bt_xml);

  RCLCPP_INFO(node->get_logger(), "Door BT loaded successfully");

  // Publisher to change door YAML based on floor
  auto yaml_pub = node->create_publisher<std_msgs::msg::String>("/door_controller/config_yaml", 10);

  // Subscribe to odom to detect current floor
  int current_floor = 0;
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [&current_floor, &yaml_pub, &node](const nav_msgs::msg::Odometry::SharedPtr msg) {
      double z = msg->pose.pose.position.z;

      int new_floor = 0;
      if (z < 2.5) {
        new_floor = 0;  // L1
      } else if (z < 7.5) {
        new_floor = 1;  // L2
      } else {
        new_floor = 2;  // L3
      }

      if (new_floor != current_floor) {
        current_floor = new_floor;

        std::string yaml_path;
        if (current_floor == 0) {
          yaml_path = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/config/door/L1.yaml";
        } else if (current_floor == 1) {
          yaml_path = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/config/door/L2.yaml";
        } else {
          yaml_path = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/config/door/L3.yaml";
        }

        RCLCPP_INFO(node->get_logger(), "[DoorBT] Floor changed to %d, loading %s", current_floor, yaml_path.c_str());

        std_msgs::msg::String msg_yaml;
        msg_yaml.data = yaml_path;
        yaml_pub->publish(msg_yaml);
      }
    });

  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  rclcpp::Rate rate(1.0);
  int tick_count = 0;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);

    status = tree.tickRoot();
    tick_count++;

    if (tick_count % 5 == 0) {
      RCLCPP_INFO(node->get_logger(), "Door BT running... (%d ticks, floor=%d)", tick_count, current_floor);
    }

    rate.sleep();
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Door BT completed successfully");
  } else {
    RCLCPP_WARN(node->get_logger(), "Door BT failed");
  }

  rclcpp::shutdown();
  return 0;
}
