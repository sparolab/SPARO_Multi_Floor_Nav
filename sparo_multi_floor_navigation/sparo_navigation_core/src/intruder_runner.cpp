#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sparo_navigation_core/config_loader.hpp"

class IntruderResponseNode : public rclcpp::Node
{
public:
  IntruderResponseNode() : Node("sparo_intruder_response_node")
  {
    RCLCPP_INFO(get_logger(), "=== Intruder Response Node ===");

    // Subscribers
    intruder_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/intruder_alert", 10,
      std::bind(&IntruderResponseNode::intruderCallback, this, std::placeholders::_1));

    floor_complete_sub_ = create_subscription<std_msgs::msg::String>(
      "/floor_change_complete", 10,
      std::bind(&IntruderResponseNode::floorCompleteCallback, this, std::placeholders::_1));

    // Publishers
    floor_req_pub_ = create_publisher<std_msgs::msg::Int32>("/elevator/floor_request", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/intruder_marker", 10);
    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    // Action client
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    // Map service client
    map_client_ = create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

    RCLCPP_INFO(get_logger(), "Intruder Response Node initialized");
  }

private:
  void intruderCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Only process if not already handling an intruder
    if (has_intruder_) {
      RCLCPP_DEBUG(get_logger(), "[INTRUDER] Already processing an intruder, ignoring new alert");
      return;
    }

    RCLCPP_WARN(get_logger(), "[INTRUDER] Alert! Floor=%s, pos=(%.2f, %.2f)",
      msg->header.frame_id.c_str(), msg->pose.position.x, msg->pose.position.y);

    intruder_alert_ = *msg;
    has_intruder_ = true;

    // Publish marker
    // publishIntruderMarker(*msg);

    // Parse floor and publish request
    std::string floor_name = msg->header.frame_id;
    int target_floor = -1;
    if (floor_name == "L1") target_floor = 0;
    else if (floor_name == "L2") target_floor = 1;
    else if (floor_name == "L3") target_floor = 2;

    if (target_floor >= 0) {
      std_msgs::msg::Int32 req;
      req.data = target_floor;
      floor_req_pub_->publish(req);
      RCLCPP_INFO(get_logger(), "[INTRUDER] Published floor request: %d", target_floor);
    }
  }

  void floorCompleteCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!has_intruder_) return;

    bool elevator_used = (msg->data.find("lift") != std::string::npos);
    std::string transport_info = msg->data;
    RCLCPP_INFO(get_logger(), "[INTRUDER] Floor change complete! transport=%s, elevator=%d", transport_info.c_str(), elevator_used);

    std::string floor_name = intruder_alert_.header.frame_id;

    // 1. Map switch
    switchMap(floor_name);

    // 2. Set initial pose
    setInitialPose(floor_name, elevator_used);

    // 3. Navigate to intruder location
    navigateToIntruder(intruder_alert_);

    has_intruder_ = false;
  }

  void switchMap(const std::string& floor)
  {
    std::string map_file = "/home/test_ws/install/sparo_navigation_bringup/share/sparo_navigation_bringup/maps/hotel_" + floor + ".yaml";

    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = map_file;

    RCLCPP_INFO(get_logger(), "[INTRUDER] Switching map to %s", map_file.c_str());

    if (!map_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(get_logger(), "[INTRUDER] Map service not available");
      return;
    }

    auto result = map_client_->async_send_request(request);
    // Don't wait for response, continue
  }

  void setInitialPose(const std::string& floor, bool elevator_used)
  {
    RCLCPP_WARN(get_logger(), "[INTRUDER DEBUG] ===== setInitialPose called =====");
    RCLCPP_WARN(get_logger(), "[INTRUDER DEBUG] Input: floor=%s, elevator_used=%d", floor.c_str(), elevator_used);

    // Determine pose_type based on elevator_used (like SetInitialPoseAction's auto logic)
    std::string pose_type = elevator_used ? "elevator_lift1_inside" : "stair";
    RCLCPP_WARN(get_logger(), "[INTRUDER DEBUG] Selected pose_type: %s", pose_type.c_str());

    if (!sparo_navigation_core::g_config_loader) {
      RCLCPP_ERROR(get_logger(), "[INTRUDER] Config loader not initialized!");
      return;
    }
    RCLCPP_INFO(get_logger(), "[INTRUDER DEBUG] Config loader OK");

    // Get initial pose from patrol YAML
    geometry_msgs::msg::Pose initial_pose;
    std::string patrol_config_dir = "/home/test_ws/src/sparo_multi_floor_navigation/sparo_navigation_bringup/config/navigation";

    RCLCPP_WARN(get_logger(), "[INTRUDER DEBUG] Loading from: %s/patrol_%s.yaml", patrol_config_dir.c_str(), floor.c_str());

    if (!sparo_navigation_core::g_config_loader->getInitialPoseFromPatrol(floor, pose_type, patrol_config_dir, initial_pose)) {
      RCLCPP_ERROR(get_logger(), "[INTRUDER] Failed to get initial_pose.%s for floor %s",
                   pose_type.c_str(), floor.c_str());
      return;
    }

    RCLCPP_WARN(get_logger(), "[INTRUDER DEBUG] Loaded pose: x=%.3f, y=%.3f, z=%.3f",
                initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
    RCLCPP_WARN(get_logger(), "[INTRUDER DEBUG] Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                initial_pose.orientation.x, initial_pose.orientation.y,
                initial_pose.orientation.z, initial_pose.orientation.w);

    // Wait for map to be fully loaded (especially after SwitchMap)
    RCLCPP_INFO(get_logger(), "[INTRUDER] Waiting 2s for map to load...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Publish initial pose multiple times for AMCL convergence
    for (int i = 0; i < 5; ++i) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.frame_id = "map";
      pose_msg.header.stamp = now();
      pose_msg.pose.pose = initial_pose;

      // Set covariance
      pose_msg.pose.covariance[0] = 0.25;   // x
      pose_msg.pose.covariance[7] = 0.25;   // y
      pose_msg.pose.covariance[35] = 0.07;  // yaw

      initial_pose_pub_->publish(pose_msg);

      RCLCPP_WARN(get_logger(),
                  "[INTRUDER] Published initial_pose.%s for floor %s (attempt %d/5): pos(%.3f, %.3f) orient(%.3f, %.3f, %.3f, %.3f)",
                  pose_type.c_str(), floor.c_str(), i + 1,
                  initial_pose.position.x, initial_pose.position.y,
                  initial_pose.orientation.x, initial_pose.orientation.y,
                  initial_pose.orientation.z, initial_pose.orientation.w);

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_WARN(get_logger(), "[INTRUDER DEBUG] ===== setInitialPose complete =====");
  }

  void navigateToIntruder(const geometry_msgs::msg::PoseStamped& alert)
  {
    RCLCPP_INFO(get_logger(), "[INTRUDER] Navigating to (%.2f, %.2f)",
      alert.pose.position.x, alert.pose.position.y);

    // Create Nav2 goal
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.pose = alert.pose;

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(get_logger(), "[INTRUDER] Nav2 action server not available");
      return;
    }

    nav_client_->async_send_goal(goal_msg);
    RCLCPP_INFO(get_logger(), "[INTRUDER] Navigation goal sent");
  }

  void publishIntruderMarker(const geometry_msgs::msg::PoseStamped& alert)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "intruder";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.pose.position.x = alert.pose.position.x;
    marker.pose.position.y = alert.pose.position.y + 2.0;
    marker.pose.position.z = 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.text = "INTRUDER!";
    marker.lifetime = rclcpp::Duration::from_seconds(10.0);
    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr intruder_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr floor_complete_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr floor_req_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;

  geometry_msgs::msg::PoseStamped intruder_alert_;
  bool has_intruder_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IntruderResponseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
