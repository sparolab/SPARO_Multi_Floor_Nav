#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('sparo_navigation_bringup')

    # Floor selection prompt
    print('\n=============================')
    print('  Floor Selection')
    print('=============================')
    print('  1) L1 (1층)')
    print('  2) L2 (2층)')
    print('  3) L3 (3층)')
    print('=============================')
    choice = input('  Select floor [1/2/3]: ').strip()

    floor_map = {'1': 'L1', '2': 'L2', '3': 'L3'}
    floor = floor_map.get(choice, 'L1')
    if choice not in floor_map:
        print(f'  Invalid input "{choice}", defaulting to L1')

    # Per-floor config: gazebo_z, initial pose (x, y, yaw)
    floor_config = {
        'L1': {'z': 1.0,  'x': 20.051, 'y': -35.023, 'yaw': -0.092},
        'L2': {'z': 6.0,  'x': 20.051, 'y': -35.023, 'yaw': -0.092},
        'L3': {'z': 11.0, 'x': 20.051, 'y': -35.023, 'yaw': -0.092},
    }
    cfg = floor_config[floor]
    gazebo_z = cfg['z']
    init_x = cfg['x']
    init_y = cfg['y']
    init_yaw = cfg['yaw']

    print(f'\n  Floor: {floor}')
    print(f'  Initial pose: x={init_x:.3f}, y={init_y:.3f}, yaw={init_yaw:.3f}')
    print(f'  Gazebo z:     {gazebo_z}')
    print(f'  Map: hotel_{floor}.yaml')
    print('=============================\n')

    # Map file
    map_file = os.path.join(bringup_dir, 'maps', f'hotel_{floor}.yaml')

    # Nav2 params
    nav2_params_file = os.path.join(
        bringup_dir, 'config', 'navigation', 'nav2_params.yaml')

    # TF nodes
    odom_to_tf_node = Node(
        package='sparo_navigation_core',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    pc2_to_scan_node = Node(
        package='sparo_navigation_core',
        executable='pc2_to_scan.py',
        name='pc2_to_scan',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

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
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    # RViz
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
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Teleport robot in Gazebo to selected floor
    # Gazebo spawn position is fixed (x=20.0, y=-35.0), only z changes per floor
    spawn_x = 20.0
    spawn_y = -35.0
    teleport_robot = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/set_entity_state', 'gazebo_msgs/srv/SetEntityState',
            '{state: {name: "robot_model", pose: {position: '
            f'{{x: {spawn_x}, y: {spawn_y}, z: {gazebo_z}}}, '
            'orientation: {w: 1.0}}}}'
        ],
        output='screen'
    )

    # Initial pose setter
    initial_pose_setter = Node(
        package='sparo_navigation_core',
        executable='initial_pose_setter.py',
        name='initial_pose_setter',
        parameters=[{
            'use_sim_time': True,
            'floor': floor,
            'x': init_x,
            'y': init_y,
            'yaw': init_yaw,
        }],
        output='screen'
    )

    return LaunchDescription([
        teleport_robot,
        odom_to_tf_node,
        pc2_to_scan_node,
        base_footprint_tf,
        nav2_bringup,
        rviz_node,
        initial_pose_setter,
    ])
