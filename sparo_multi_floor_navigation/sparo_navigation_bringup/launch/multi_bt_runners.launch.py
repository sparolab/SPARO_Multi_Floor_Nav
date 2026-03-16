#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('sparo_navigation_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')

    # TF nodes
    odom_to_tf_node = Node(
        package='sparo_navigation_core',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    pc2_to_scan_node = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        name='velodyne_laserscan',
        remappings=[
            ('velodyne_points', '/velodyne_points'),
            ('scan', '/scan'),
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_id': 'base_scan'},
            {'range_min': 0.3},
            {'range_max': 30.0},
        ],
        output='screen'
    )

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # Nav2 params file
    nav2_params_file = os.path.join(bringup_dir, 'config', 'navigation', 'nav2_params.yaml')

    # Start with L1 map
    initial_map = os.path.join(bringup_dir, 'maps', 'hotel_L1.yaml')

    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'map': initial_map,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    # RViz
    # rviz_config_file = os.path.join(bringup_dir, 'config', 'visualization', 'sparo_nav2.rviz')
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Main BT Executor (patrol only)
    bt_executor_node = Node(
        package='sparo_navigation_bringup',
        executable='sparo_bt_executor',
        name='sparo_bt_executor',
        output='screen',
        parameters=[{
            'bt_xml': os.path.join(bringup_dir, 'behavior_trees', 'patrol_all_floors.xml'),
            'config_file': os.path.join(bringup_dir, 'config', 'navigation', 'floors.yaml'),
            'patrol_config_dir': os.path.join(bringup_dir, 'config', 'navigation'),
            'use_sim_time': use_sim_time,
        }]
    )

    delayed_bt_executor = TimerAction(
        period=3.0,
        actions=[bt_executor_node]
    )

    # Elevator BT runner (listens to /elevator/floor_request) - uses RCI elevator_bt
    elevator_bt_node = Node(
        package='sparo_navigation_core',
        executable='sparo_elevator_bt_runner',
        name='sparo_elevator_bt_runner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': os.path.join(bringup_dir, 'config', 'navigation', 'floors.yaml'),
            'patrol_config_dir': os.path.join(bringup_dir, 'config', 'navigation'),
        }]
    )

    # Stairs BT runner (listens to /stairs/floor_request)
    stairs_bt_node = Node(
        package='sparo_navigation_core',
        executable='sparo_stair_bt_runner',
        name='sparo_stair_bt_runner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': os.path.join(bringup_dir, 'config', 'navigation', 'floors.yaml'),
            'patrol_config_dir': os.path.join(bringup_dir, 'config', 'navigation'),
        }]
    )
    # Door BT runner (monitors /plan and opens doors automatically)
    door_bt_node = Node(
        package='sparo_navigation_core',
        executable='sparo_door_bt_runner',
        name='sparo_door_bt_runner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Intruder response node
    intruder_node = Node(
        package='sparo_navigation_core',
        executable='sparo_intruder_runner',
        name='sparo_intruder_runner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    return LaunchDescription([
        declare_use_sim_time_cmd,
        odom_to_tf_node,
        pc2_to_scan_node,
        base_footprint_tf,
        nav2_bringup,
        elevator_bt_node,  # Elevator BT runner
        stairs_bt_node,    # Stairs BT runner
        door_bt_node,      # Door BT runner
        intruder_node,     # Intruder response node
        rviz_node,
        delayed_bt_executor,
    ])
