from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_share = os.path.dirname(os.path.realpath(__file__))
    urdf_path = os.path.join(pkg_share, '../../parking_robot_description/urdf/robot_base.urdf.xacro')
    
    return LaunchDescription([
        # Spawn robot in Gazebo (at origin, facing +X)
        ExecuteProcess(
            cmd=['gz', 'service', 'call', '/world/parking_lot/create',
                 'gz.msgs.EntityFactory',
                 '{sdf_filename: "' + urdf_path + '", name: "parking_bot", pose: {position: {x: 0, y: 0, z: 0.2}}}'],
            output='screen'
        ),
        
        # Bridge camera image
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/bot/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            ],
            remappings=[
                ('/bot/camera/image', '/bot/camera/image_raw'),
            ],
            output='screen'
        ),
        
        # Bridge lidar scan
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/bot/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ],
            output='screen'
        ),
        
        # Static TF: map -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),
    ])