#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('sparo_navigation_bringup')

    # Floor selection prompt
    print('\n=============================')
    print('  Floor Selection (Current)')
    print('=============================')
    print('  1) L1 (1층)')
    print('  2) L2 (2층)')
    print('  3) L3 (3층)')
    print('=============================')
    choice = input('  Select current floor [1/2/3]: ').strip()

    floor_map = {'1': 'L1', '2': 'L2', '3': 'L3'}
    floor = floor_map.get(choice, 'L1')
    if choice not in floor_map:
        print(f'  Invalid input "{choice}", defaulting to L1')

    print(f'\n  Current floor: {floor}')
    print('=============================\n')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')

    # Publish /current_floor once at startup
    publish_current_floor = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--times', '5',
            '/current_floor', 'std_msgs/msg/String',
            f'{{data: "{floor}"}}'
        ],
        output='screen'
    )

    # Stairs BT runner only (listens to /stairs/floor_request)
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

    return LaunchDescription([
        declare_use_sim_time_cmd,
        publish_current_floor,
        stairs_bt_node,
    ])
