from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_share = os.path.dirname(os.path.realpath(__file__))
    
    return LaunchDescription([
        # Start Gazebo with parking world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r',
                 os.path.join(pkg_share, '../../parking_world/worlds/parking_flat.sdf'),
                 '-v', '4'],
            output='screen'
        ),
        
        # Use ros_gz_image bridge instead of parameter_bridge
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/lot_cam/image'],
            remappings=[
                ('/lot_cam/image', '/lot_cam/image_raw'),
            ],
            output='screen'
        ),
        
        # Static TF: map -> lot_cam
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '25', '0', '0', '0', 'map', 'lot_cam'],
            output='screen'
        ),
        
        # Static TF: lot_cam -> lot_cam_optical (pitch -90°, yaw -90°)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'lot_cam', 'lot_cam_optical'],
            output='screen'
        ),
    ])