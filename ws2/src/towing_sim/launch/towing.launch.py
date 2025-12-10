#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('towing_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'simple_parking.world')
    bot_urdf = os.path.join(pkg_share, 'urdf', 'towing_bot.urdf')
    car_urdf = os.path.join(pkg_share, 'urdf', 'simple_car.urdf')
    controller_config = os.path.join(pkg_share, 'config', 'lift_controller.yaml')
    
    # Read URDFs
    with open(bot_urdf, 'r') as f:
        bot_desc = f.read()
    
    with open(car_urdf, 'r') as f:
        car_desc = f.read()
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )
    
    # Load controller YAML content
    with open(controller_config, 'r') as f:
        controller_yaml = yaml.safe_load(f)
    
    # Robot State Publisher (for bot) with controller config
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': bot_desc},
            controller_yaml
        ]
    )
    
    # Spawn Bot at origin
    spawn_bot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'towing_bot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.125'
        ],
        output='screen'
    )
    
    # Spawn Car outside rectangle
    spawn_car = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_car',
            '-file', car_urdf,
            '-x', '5',
            '-y', '5',
            '-z', '0.35'
        ],
        output='screen'
    )
    
    # Delay car spawn slightly
    spawn_car_delayed = TimerAction(
        period=2.0,
        actions=[spawn_car]
    )
    
    # Spawn controllers after bot is loaded - using individual timers
    spawn_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )]
    )
    
    spawn_lift_controller = TimerAction(
        period=7.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['lift_position_controller'],
            output='screen'
        )]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_bot,
        spawn_car_delayed,
        spawn_joint_state_broadcaster,
        spawn_lift_controller,
    ])