#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ament_index_cpp/get_package_prefix.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sparo_bt_runner");

  node->declare_parameter<std::string>("bt_xml_file", "");
  node->declare_parameter<std::string>("sparo_bt_plugin", "libsparo_bt_nav2_plugin.so");
  node->declare_parameter<std::string>("sparo_pkg", "sparo_navigation_core");

  const std::string xml_file = node->get_parameter("bt_xml_file").as_string();
  if (xml_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'bt_xml_file' is empty.");
    return 1;
  }

  BT::BehaviorTreeFactory factory;

  const std::string sparo_pkg = node->get_parameter("sparo_pkg").as_string();
  const auto sparo_prefix = ament_index_cpp::get_package_prefix(sparo_pkg);

  const std::string so_name = node->get_parameter("sparo_bt_plugin").as_string();
  const std::string sparo_so = sparo_prefix + "/lib/" + so_name;

  RCLCPP_INFO(node->get_logger(), "Loading BT plugin library: %s", sparo_so.c_str());

  try {
    factory.registerFromPlugin(sparo_so);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load plugin library: %s", e.what());
    return 1;
  }

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_file);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create BT from XML: %s", e.what());
    return 1;
  }

  rclcpp::Rate rate(10.0);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "BT finished with status: %s", BT::toStr(status, true).c_str());
  rclcpp::shutdown();
  return 0;
}
