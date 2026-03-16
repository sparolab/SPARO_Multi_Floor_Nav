#!/usr/bin/env python3
"""
Planner Node - Dynamic multi-floor navigation orchestrator.
Receives text commands and orchestrates floor changes, map switching, and navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from std_srvs.srv import Empty
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import yaml
import math
import os
import time
import threading
from enum import Enum, auto
from ament_index_python.packages import get_package_share_directory


# ── Floor aliases ──
FLOOR_ALIASES = {
    '1층': 'L1', '1F': 'L1', 'L1': 'L1', '일층': 'L1',
    '2층': 'L2', '2F': 'L2', 'L2': 'L2', '이층': 'L2',
    '3층': 'L3', '3F': 'L3', 'L3': 'L3', '삼층': 'L3',
}

# ── Location aliases ──
LOCATION_ALIASES = {
    '복도': 'corridor', '엘리베이터': 'elevator', '엘베': 'elevator',
    '계단': 'stair', '로비': 'lobby', '입구': 'entrance',
    'corridor': 'corridor', 'elevator': 'elevator', 'stair': 'stair',
    'lobby': 'lobby', 'entrance': 'entrance', 'default': 'default',
}

# ── Action keywords ──
PATROL_KEYWORDS = ['순찰', 'patrol', '돌아', '순찰해', '순회']
GOTO_KEYWORDS = ['가', '가줘', '이동', 'go', 'goto', 'navigate', '가자']


class PlannerState(Enum):
    IDLE = auto()
    NAVIGATING_TO_ELEVATOR = auto()
    NAVIGATING_TO_STAIRS = auto()
    WAITING_FLOOR_CHANGE = auto()
    SWITCHING_MAP = auto()
    SETTING_INITIAL_POSE = auto()
    NAVIGATING_TO_TARGET = auto()
    PATROLLING = auto()


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        # Parameters
        self.declare_parameter('config_dir', '')
        self.declare_parameter('maps_dir', '')
        self.declare_parameter('elevator_timeout', 180.0)
        self.declare_parameter('nav_timeout', 120.0)
        self.declare_parameter('initial_floor', 'L1')

        # State
        self.state = PlannerState.IDLE
        self.current_odom_z = 0.0
        self.floor_change_info = None
        self.state_lock = threading.Lock()

        # Config
        self.floors = {}
        self.patrol_configs = {}
        self.maps_dir = ''

        # Load configs
        self._load_configs()

        # Publishers
        self.elevator_request_pub = self.create_publisher(Int32, '/elevator/floor_request', 10)
        self.stairs_request_pub = self.create_publisher(Int32, '/stairs/floor_request', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.status_pub = self.create_publisher(String, '/planner/status', 10)
        self.current_floor_pub = self.create_publisher(String, '/current_floor', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/planner/command', self._command_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.floor_complete_sub = self.create_subscription(
            String, '/floor_change_complete', self._floor_change_callback, 10)

        # Service clients
        self.load_map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.clear_costmaps_client = self.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global_costmaps_client = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')

        # Action client
        self.nav_action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        # TF for distance-based goal checking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('=== Planner Node Initialized ===')
        self._publish_status('IDLE', 'Planner ready, waiting for commands')

        # Publish initial floor
        initial_floor = self.get_parameter('initial_floor').get_parameter_value().string_value
        self._publish_current_floor(initial_floor)

    # ──────────────────────────────────────────────
    #  Config loading
    # ──────────────────────────────────────────────

    def _load_configs(self):
        pkg_share = get_package_share_directory('sparo_navigation_bringup')
        config_dir = self.get_parameter('config_dir').get_parameter_value().string_value
        if not config_dir:
            config_dir = os.path.join(pkg_share, 'config', 'navigation')

        maps_dir = self.get_parameter('maps_dir').get_parameter_value().string_value
        if not maps_dir:
            self.maps_dir = os.path.join(pkg_share, 'maps')
        else:
            self.maps_dir = maps_dir

        # Load floors.yaml
        with open(os.path.join(config_dir, 'floors.yaml'), 'r') as f:
            floors_data = yaml.safe_load(f)

        for name, data in floors_data['floors'].items():
            self.floors[name] = {
                'floor_index': data['floor_index'],
                'height': data['height'],
                'map_file': data['map_file'],
            }

        # Load patrol_L*.yaml
        for floor_name in self.floors:
            patrol_file = os.path.join(config_dir, f'patrol_{floor_name}.yaml')
            if not os.path.exists(patrol_file):
                continue

            with open(patrol_file, 'r') as f:
                patrol_data = yaml.safe_load(f)

            patrol = patrol_data['patrol']

            initial_poses = {}
            for key, val in patrol.get('initial_pose', {}).items():
                initial_poses[key] = (val['x'], val['y'], val['yaw'])

            waypoints = []
            for i, wp in enumerate(patrol.get('waypoints', [])):
                waypoints.append({
                    'name': wp.get('name', f'waypoint_{i}'),
                    'x': wp['x'], 'y': wp['y'], 'yaw': wp['yaw'],
                })

            self.patrol_configs[floor_name] = {
                'initial_poses': initial_poses,
                'waypoints': waypoints,
            }

        total_wp = sum(len(c['waypoints']) for c in self.patrol_configs.values())
        self.get_logger().info(f'Loaded {len(self.floors)} floors, {total_wp} waypoints')

    # ──────────────────────────────────────────────
    #  Callbacks
    # ──────────────────────────────────────────────

    def _command_callback(self, msg):
        with self.state_lock:
            if self.state != PlannerState.IDLE:
                self.get_logger().warn(f'Busy ({self.state.name}), ignoring: {msg.data}')
                self._publish_status('BUSY', f'Ignored: {msg.data}')
                return

        cmd = self._parse_command(msg.data)
        if not cmd:
            self._publish_status('ERROR', f'Cannot parse: {msg.data}')
            return

        # Execute in separate thread so spin() keeps processing callbacks
        thread = threading.Thread(target=self._execute_command, args=(cmd,), daemon=True)
        thread.start()

    def _odom_callback(self, msg):
        self.current_odom_z = msg.pose.pose.position.z

    def _floor_change_callback(self, msg):
        self.floor_change_info = msg.data
        self.get_logger().info(f'Floor change complete: {msg.data}')

    # ──────────────────────────────────────────────
    #  Command parsing
    # ──────────────────────────────────────────────

    def _parse_command(self, text):
        text = text.strip()
        if not text:
            return None

        action_type = 'goto'
        target_floor = None
        target_location = None

        # Detect action
        for kw in PATROL_KEYWORDS:
            if kw in text:
                action_type = 'patrol'
                break

        # Extract floor
        for alias, floor_name in FLOOR_ALIASES.items():
            if alias in text or alias.lower() in text.lower():
                target_floor = floor_name
                break

        # Extract location
        for alias, loc_name in LOCATION_ALIASES.items():
            if alias in text or alias.lower() in text.lower():
                target_location = loc_name
                break

        # Defaults
        if not target_floor:
            target_floor = self._detect_current_floor()

        if not target_location and action_type == 'goto':
            target_location = 'default'

        self.get_logger().info(
            f'Parsed: floor={target_floor}, location={target_location}, '
            f'action={action_type}, raw="{text}"')

        return {
            'target_floor': target_floor,
            'target_location': target_location,
            'action_type': action_type,
            'raw_text': text,
        }

    # ──────────────────────────────────────────────
    #  Main execution
    # ──────────────────────────────────────────────

    def _execute_command(self, cmd):
        try:
            self._publish_status('EXECUTING', f'Processing: {cmd["raw_text"]}')

            current_floor = self._detect_current_floor()
            target_floor = cmd['target_floor']

            # Floor change if needed
            if target_floor != current_floor:
                if not self._do_floor_change(current_floor, target_floor):
                    self._handle_error('Floor change failed')
                    return

            # Navigate or patrol
            if cmd['action_type'] == 'patrol':
                self._set_state(PlannerState.PATROLLING)
                self._execute_patrol(target_floor)
            else:
                self._set_state(PlannerState.NAVIGATING_TO_TARGET)
                self._navigate_to_location(target_floor, cmd['target_location'])

            self._set_state(PlannerState.IDLE)
            self._publish_status('COMPLETE', f'Done: {cmd["raw_text"]}')

        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')
            self._handle_error(str(e))

    def _do_floor_change(self, current_floor, target_floor):
        elevator_timeout = self.get_parameter('elevator_timeout').get_parameter_value().double_value
        target_index = self.floors[target_floor]['floor_index']

        # Publish current floor before starting elevator BT
        self._publish_current_floor(current_floor)

        # 1. Navigate to elevator entrance on current floor
        self._set_state(PlannerState.NAVIGATING_TO_ELEVATOR)
        elevator_pose = self._get_pose(current_floor, 'elevator')
        if elevator_pose:
            self._publish_status('NAVIGATING', f'Going to elevator on {current_floor}')
            if not self._navigate_to_pose(*elevator_pose):
                self.get_logger().warn('Failed to reach elevator, trying stairs directly')
                return self._do_stairs_fallback(current_floor, target_floor, target_index)

        # 2. Request elevator
        self._set_state(PlannerState.WAITING_FLOOR_CHANGE)
        self.floor_change_info = None
        self._publish_status('FLOOR_CHANGE', f'Requesting elevator to {target_floor}')

        msg = Int32()
        msg.data = target_index
        self.elevator_request_pub.publish(msg)

        # 3. Wait for completion
        if self._wait_for_floor_change(elevator_timeout):
            return self._finish_floor_change(target_floor)

        # 4. Elevator timeout -> stairs fallback
        self.get_logger().warn('Elevator timeout, falling back to stairs')
        return self._do_stairs_fallback(current_floor, target_floor, target_index)

    def _do_stairs_fallback(self, current_floor, target_floor, target_index):
        elevator_timeout = self.get_parameter('elevator_timeout').get_parameter_value().double_value

        self._set_state(PlannerState.NAVIGATING_TO_STAIRS)
        stair_pose = self._get_pose(current_floor, 'stair')
        if stair_pose:
            self._publish_status('NAVIGATING', f'Going to stairs on {current_floor}')
            if not self._navigate_to_pose(*stair_pose):
                self.get_logger().error('Failed to reach stairs')
                return False

        self._set_state(PlannerState.WAITING_FLOOR_CHANGE)
        self.floor_change_info = None
        self._publish_status('FLOOR_CHANGE', f'Requesting stairs to {target_floor}')

        msg = Int32()
        msg.data = target_index
        self.stairs_request_pub.publish(msg)

        if self._wait_for_floor_change(elevator_timeout * 1.5):
            return self._finish_floor_change(target_floor)

        return False

    def _finish_floor_change(self, target_floor):
        # Switch map
        self._set_state(PlannerState.SWITCHING_MAP)
        map_path = os.path.join(self.maps_dir, self.floors[target_floor]['map_file'])
        if not self._switch_map(map_path):
            return False

        # Publish current floor after map switch
        self._publish_current_floor(target_floor)

        # Clear costmaps so old floor data doesn't linger
        self._clear_costmaps()

        # Set initial pose
        self._set_state(PlannerState.SETTING_INITIAL_POSE)
        pose_type = self._pose_type_from_floor_change()
        self._set_initial_pose(target_floor, pose_type)

        # Wait for costmap to fully rebuild with new map
        self.get_logger().info('Waiting for costmap rebuild...')
        time.sleep(3.0)

        return True

    # ──────────────────────────────────────────────
    #  Floor detection
    # ──────────────────────────────────────────────

    def _detect_current_floor(self):
        z = self.current_odom_z
        if z < 2.5:
            return 'L1'
        elif z < 7.5:
            return 'L2'
        else:
            return 'L3'

    # ──────────────────────────────────────────────
    #  Wait for floor change (polling with time.sleep)
    # ──────────────────────────────────────────────

    def _wait_for_floor_change(self, timeout):
        start = time.time()
        while rclpy.ok():
            if self.floor_change_info is not None:
                return True
            elapsed = time.time() - start
            if elapsed > timeout:
                return False
            time.sleep(0.1)
        return False

    # ──────────────────────────────────────────────
    #  Map switching
    # ──────────────────────────────────────────────

    def _wait_for_future(self, future, timeout=30.0):
        """Poll a future until done (thread-safe, no spin conflict)."""
        start = time.time()
        while rclpy.ok():
            if future.done():
                return True
            if time.time() - start > timeout:
                return False
            time.sleep(0.1)
        return False

    def _switch_map(self, map_path):
        self._publish_status('MAP_SWITCH', f'Loading {os.path.basename(map_path)}')

        if not self.load_map_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('LoadMap service not available')
            return False

        request = LoadMap.Request()
        request.map_url = map_path

        future = self.load_map_client.call_async(request)
        if not self._wait_for_future(future, timeout=30.0):
            self.get_logger().error('Map load service call timed out')
            return False

        if future.result() is not None:
            result = future.result()
            if result.result == LoadMap.Response.RESULT_SUCCESS:
                self.get_logger().info(f'Map switched: {map_path}')
                return True
            else:
                self.get_logger().error(f'Map load failed: result={result.result}')
                return False

        self.get_logger().error('Map load service call failed')
        return False

    def _clear_costmaps(self):
        """Clear both local and global costmaps after map switch."""
        self.get_logger().info('Clearing costmaps...')
        for client, name in [
            (self.clear_costmaps_client, 'local'),
            (self.clear_global_costmaps_client, 'global'),
        ]:
            if client.wait_for_service(timeout_sec=5.0):
                future = client.call_async(Empty.Request())
                self._wait_for_future(future, timeout=5.0)
                self.get_logger().info(f'Cleared {name} costmap')
            else:
                self.get_logger().warn(f'{name} costmap clear service not available')

    # ──────────────────────────────────────────────
    #  Initial pose
    # ──────────────────────────────────────────────

    def _set_initial_pose(self, floor, pose_type):
        pose_data = self._get_pose(floor, pose_type)
        if not pose_data:
            self.get_logger().warn(f'Pose "{pose_type}" not found for {floor}, using default')
            pose_data = self._get_pose(floor, 'default')
        if not pose_data:
            self.get_logger().error(f'No initial pose for {floor}')
            return

        x, y, yaw = pose_data

        # Wait for map to load
        time.sleep(2.0)

        # Publish 5 times for AMCL convergence
        for i in range(5):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.07
            self.initial_pose_pub.publish(msg)
            time.sleep(0.5)

        self.get_logger().info(f'Initial pose set: {floor}/{pose_type} ({x:.2f}, {y:.2f}, {yaw:.2f})')

    def _pose_type_from_floor_change(self):
        info = self.floor_change_info
        if info == '/lift1':
            return 'elevator_lift1_inside'
        elif info == '/lift2':
            return 'elevator_lift2_inside'
        elif info == 'stair':
            return 'stair'
        return 'default'

    # ──────────────────────────────────────────────
    #  Navigation
    # ──────────────────────────────────────────────

    def _navigate_to_pose(self, x, y, yaw, success_radius=0.7):
        """Navigate to pose with distance-based arrival detection."""
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Nav goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f}), success_radius={success_radius}')

        send_future = self.nav_action_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, timeout=10.0):
            self.get_logger().error('Send goal timed out')
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav goal rejected')
            return False

        # Poll distance while waiting for result
        nav_timeout = self.get_parameter('nav_timeout').get_parameter_value().double_value
        start_time = time.time()

        while rclpy.ok():
            # Check timeout
            if time.time() - start_time > nav_timeout:
                self.get_logger().error('Navigation timed out')
                goal_handle.cancel_goal_async()
                return False

            # Check distance to goal
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y

                dist = math.hypot(x - robot_x, y - robot_y)

                if dist < success_radius:
                    self.get_logger().info(f'Goal reached by distance: {dist:.3f}m < {success_radius}m')
                    goal_handle.cancel_goal_async()
                    return True

            except TransformException as ex:
                pass  # TF not ready yet, continue

            # Check if action completed
            result_future = goal_handle.get_result_async()
            if self._wait_for_future(result_future, timeout=0.1):
                result = result_future.result()
                if result is not None:
                    status = result.status
                    if status == 4:  # SUCCEEDED
                        self.get_logger().info('Navigation succeeded')
                        return True
                    else:
                        self.get_logger().warn(f'Navigation ended: status={status}')
                        return False

            time.sleep(0.1)  # Small sleep between checks

        return False

    def _navigate_to_location(self, floor, location):
        config = self.patrol_configs.get(floor)
        if not config:
            self.get_logger().error(f'No config for {floor}')
            return

        # Try initial_poses first
        if location and location in config['initial_poses']:
            x, y, yaw = config['initial_poses'][location]
            self._publish_status('NAVIGATING', f'Going to {location} on {floor}')
            self._navigate_to_pose(x, y, yaw)
            return

        # Try waypoint by name
        if location:
            for wp in config['waypoints']:
                if wp['name'] == location:
                    self._publish_status('NAVIGATING', f'Going to {wp["name"]} on {floor}')
                    self._navigate_to_pose(wp['x'], wp['y'], wp['yaw'])
                    return

        # Fallback to default
        if 'default' in config['initial_poses']:
            x, y, yaw = config['initial_poses']['default']
            self.get_logger().warn(f'Location "{location}" not found, using default')
            self._navigate_to_pose(x, y, yaw)
        elif config['waypoints']:
            wp = config['waypoints'][0]
            self.get_logger().warn(f'Location "{location}" not found, using first waypoint')
            self._navigate_to_pose(wp['x'], wp['y'], wp['yaw'])

    def _execute_patrol(self, floor):
        config = self.patrol_configs.get(floor)
        if not config or not config['waypoints']:
            self.get_logger().warn(f'No waypoints for {floor}')
            return

        total = len(config['waypoints'])
        self.get_logger().info(f'Patrol {floor}: {total} waypoints')

        for i, wp in enumerate(config['waypoints']):
            self._publish_status('PATROLLING', f'{floor} {i+1}/{total}: {wp["name"]}')
            success = self._navigate_to_pose(wp['x'], wp['y'], wp['yaw'])
            if not success:
                self.get_logger().warn(f'Failed waypoint {wp["name"]}, skipping')
                continue
            self.get_logger().info(f'Reached {wp["name"]}')

        self._publish_status('PATROL_COMPLETE', f'Patrol {floor} done')

    # ──────────────────────────────────────────────
    #  Utilities
    # ──────────────────────────────────────────────

    def _get_pose(self, floor, pose_type):
        config = self.patrol_configs.get(floor)
        if config and pose_type in config['initial_poses']:
            return config['initial_poses'][pose_type]
        return None

    def _set_state(self, new_state):
        with self.state_lock:
            old = self.state
            self.state = new_state
            self.get_logger().info(f'State: {old.name} -> {new_state.name}')

    def _publish_status(self, status, detail=''):
        msg = String()
        msg.data = f'[{status}] {detail}'
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: [{status}] {detail}')

    def _publish_current_floor(self, floor_name):
        """Publish current floor for elevator/stair BT nodes."""
        msg = String()
        msg.data = floor_name
        self.current_floor_pub.publish(msg)
        self.get_logger().info(f'Published current floor: {floor_name}')

    def _handle_error(self, error_msg):
        self.get_logger().error(f'ERROR: {error_msg}')
        self._publish_status('ERROR', error_msg)
        self._set_state(PlannerState.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
