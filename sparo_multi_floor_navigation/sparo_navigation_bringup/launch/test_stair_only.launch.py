#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        stairs_bt_node,  # Only stair BT runner (no Nav2, no RViz)
    ])
