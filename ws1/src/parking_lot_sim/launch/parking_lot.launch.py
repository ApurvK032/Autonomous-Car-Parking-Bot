import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'parking_lot_sim'
    
    # Get package share directory
    pkg_share = FindPackageShare(package_name).find(package_name)
    
    # Path to world file
    world_file = os.path.join(pkg_share, 'worlds', 'parking_lot.world')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'parking_bot.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    return LaunchDescription([
        # Launch Gazebo with world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # Robot State Publisher - publishes URDF to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': True}
            ]
        ),
        
        # Spawn entity with delay to ensure Gazebo is ready
        TimerAction(
            period=5.0,  # Wait 5 seconds for Gazebo to start
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_parking_bot',
                    output='screen',
                    arguments=[
                        '-entity', 'parking_bot',
                        '-topic', 'robot_description',
                        '-x', '-15',
                        '-y', '-15',
                        '-z', '0.5',
                        '-R', '0',
                        '-P', '0',
                        '-Y', '0'
                    ]
                )
            ]
        )
    ])
