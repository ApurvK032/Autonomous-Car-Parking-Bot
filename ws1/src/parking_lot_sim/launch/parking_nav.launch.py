#!/usr/bin/env python3
"""
Launch file for autonomous parking with Nav2 navigation
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('parking_lot_sim')
    
    # Paths
    world_file = os.path.join(pkg_dir, '..', '..', '..', '..',
                               'src', 'parking_lot_sim', 'worlds', 'parking_lot.world')
    nav2_params = os.path.join(pkg_dir, '..', '..', '..', '..',
                                'src', 'parking_lot_sim', 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg_dir, '..', '..', '..', '..',
                             'src', 'parking_lot_sim', 'maps', 'parking_lot_nav_map.yaml')
    urdf_file = os.path.join(pkg_dir, '..', '..', '..', '..',
                              'src', 'parking_lot_sim', 'urdf', 'parking_bot.urdf')
    bt_xml_path = os.path.join(pkg_dir, '..', '..', '..', '..',
                               'src', 'parking_lot_sim', 'config', 'navigate_simple.xml')
    
    return LaunchDescription([
        # 1. Launch Gazebo server with ROS plugins
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ]),
            launch_arguments={'world': world_file, 'verbose': 'true'}.items()
        ),
        # 1b. Launch Gazebo client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
            ])
        ),

        # 1c. Static transform map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # 2. Spawn parking_bot entity in Gazebo
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_parking_bot',
                    output='screen',
                    arguments=[
                        '-entity', 'parking_bot',
                        '-file', urdf_file,
                        '-x', '-15',
                        '-y', '-15',
                        '-z', '0.1'
                    ]
                )
            ]
        ),
        
        # 3. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}]
        ),
        
        # 4. Lifecycle manager for map server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server']}]
        ),
        
        # 5. Nav2 Stack (delayed start) - direct node launches with our params
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params]
                ),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params]
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[
                        nav2_params,
                        {
                            'default_bt_xml_filename': bt_xml_path,
                            'default_nav_to_pose_bt_xml': bt_xml_path,
                            'default_nav_through_poses_bt_xml': bt_xml_path,
                            'navigate_to_pose_bt_xml': bt_xml_path,
                            'navigate_through_poses_bt_xml': bt_xml_path
                        }
                    ]
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params]
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': [
                            'controller_server',
                            'planner_server',
                            'bt_navigator',
                            'behavior_server',
                            'smoother_server',
                            'waypoint_follower'
                        ]
                    }]
                ),
                Node(
                    package='nav2_smoother',
                    executable='smoother_server',
                    name='smoother_server',
                    output='screen',
                    parameters=[nav2_params]
                ),
                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    parameters=[nav2_params]
                ),
            ]
        ),
        
        # 6. Smart Detector (delayed start)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='parking_lot_sim',
                    executable='smart_detector',
                    name='smart_detector',
                    output='screen'
                )
            ]
        ),
        
        # 7. Goal Publisher (delayed start)
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='parking_lot_sim',
                    executable='goal_publisher',
                    name='goal_publisher',
                    output='screen'
                )
            ]
        ),
    ])