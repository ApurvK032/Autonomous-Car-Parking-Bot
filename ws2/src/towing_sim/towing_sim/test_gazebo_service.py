#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetJointProperties, GetJointProperties
from std_msgs.msg import Float64
import numpy as np
import math

class GapApproach(Node):
    def __init__(self):
        super().__init__('gap_approach')
        
        self.car_position_sub = self.create_subscription(
            Pose2D,
            '/car_position',
            self.car_position_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/bot/scan',
            self.lidar_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.lift_pub = self.create_publisher(Float64, '/lift_joint_position', 10)
        
        self.car_detected = False
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_angle = 0.0
        self.bot_x = 0.0
        self.bot_y = 0.0
        self.bot_yaw = 0.0
        
        self.state = "IDLE"
        
        self.lidar_ranges = []
        
        self.lift_height = 0.0
        self.car_lifted = False
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🤖 Gap Approach Controller started!')
        
    def car_position_callback(self, msg):
        """Receive car position from overhead camera"""
        self.car_detected = True
        self.car_x = msg.x
        self.car_y = msg.y
        self.car_angle = msg.theta
        
        if self.state == "IDLE":
            self.state = "NAVIGATE_TO_CAR"
            self.get_logger().info(f'🚗 Car center at ({self.car_x:.2f}, {self.car_y:.2f})')
            self.get_logger().info('➡️  State: NAVIGATE_TO_CAR')
    
    def lidar_callback(self, msg):
        """Process lidar scan data"""
        self.lidar_ranges = msg.ranges

        front_ranges = list(msg.ranges[330:]) + list(msg.ranges[:30])
        valid_front = [r for r in front_ranges if r > 0.1]
        if valid_front:
            min_front = min(valid_front)
            self.get_logger().info(
                f'📡 Lidar front min: {min_front:.2f}m, samples: {len(front_ranges)}',
                throttle_duration_sec=1.0
            )
        
    def odom_callback(self, msg: Odometry):
        """Update bot position and yaw from odometry."""
        self.bot_x = msg.pose.pose.position.x
        self.bot_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.bot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def lift_car(self, target_height):
        """Lift the car by extending the lift joint"""
        msg = Float64()
        msg.data = target_height
        self.lift_pub.publish(msg)
        
        self.lift_height = target_height
        self.get_logger().info(f'🔧 Lift command published: {target_height:.2f}m')
        return True
    
    def control_loop(self):
        """Main control loop"""
        cmd = Twist()
        
        if self.state == "IDLE":
            pass
            
        elif self.state == "NAVIGATE_TO_CAR":
            dx = self.car_x - self.bot_x
            dy = self.car_y - self.bot_y
            distance = math.sqrt(dx**2 + dy**2)
            
            self.get_logger().info(
                f'\n'
                f'═══════════════════════════════════════════════════════\n'
                f'🎯 DESTINATION: x={self.car_x:.4f}, y={self.car_y:.4f}\n'
                f'🤖 BOT POSITION: x={self.bot_x:.4f}, y={self.bot_y:.4f}\n'
                f'📏 SEPARATION:   dx={dx:.4f}, dy={dy:.4f}, distance={distance:.4f}\n'
                f'📊 THRESHOLDS:   X threshold=0.08, Y threshold=0.08, Success=0.10\n'
                f'═══════════════════════════════════════════════════════',
                throttle_duration_sec=0.3
            )
            
            front_ranges = list(self.lidar_ranges[345:]) + list(self.lidar_ranges[:15])
            min_front = min([r for r in front_ranges if 0.1 < r < 10.0], default=10.0)
            
            if abs(dx) > 0.08:
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                
                if distance > 2.0:
                    speed = 2.0
                    status = "FAST"
                else:
                    speed = 0.5
                    status = "SLOW"
                
                if min_front < 0.4:
                    cmd.linear.x = 0.0
                    self.get_logger().warn(f'🛑 X BLOCKED: obstacle at {min_front:.2f}m')
                else:
                    cmd.linear.x = speed if dx > 0 else -speed
                    self.get_logger().info(
                        f'→ X PHASE [{status}]: moving at {speed:.1f}m/s, dx={dx:.4f}m remaining, dist={distance:.4f}m'
                    )
            
            elif abs(dy) > 0.08:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
                if distance > 2.0:
                    speed = 2.0
                    status = "FAST"
                else:
                    speed = 0.5
                    status = "SLOW"
                
                left_ranges = list(self.lidar_ranges[75:105])
                right_ranges = list(self.lidar_ranges[255:285])
                min_left = min([r for r in left_ranges if 0.1 < r < 10.0], default=10.0)
                min_right = min([r for r in right_ranges if 0.1 < r < 10.0], default=10.0)
                
                moving_left = dy > 0
                obstacle_side = min_left if moving_left else min_right
                
                if obstacle_side < 0.4:
                    cmd.linear.y = 0.0
                    side_name = "left" if moving_left else "right"
                    self.get_logger().warn(f'🛑 Y BLOCKED: {side_name} obstacle at {obstacle_side:.2f}m')
                else:
                    cmd.linear.y = speed if moving_left else -speed
                    self.get_logger().info(
                        f'↔ Y PHASE [{status}]: moving at {speed:.1f}m/s, dy={dy:.4f}m remaining, dist={distance:.4f}m'
                    )
            
            elif distance < 0.10:
                self.get_logger().info(
                    f'\n'
                    f'🎉🎉🎉 SUCCESS! 🎉🎉🎉\n'
                    f'Bot reached exact center!\n'
                    f'Final position: bot=({self.bot_x:.4f}, {self.bot_y:.4f}), target=({self.car_x:.4f}, {self.car_y:.4f})\n'
                    f'Final error: dx={dx:.4f}, dy={dy:.4f}, distance={distance:.4f}\n'
                )
                
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                
                if not hasattr(self, 'towing_start_x'):
                    self.towing_start_x = self.bot_x
                    self.towing_start_y = self.bot_y
                
                self.state = "ADVANCE_TO_TOWING"
                self.get_logger().info('➡️  State: ADVANCE_TO_TOWING')
            
            else:
                self.get_logger().error(
                    f'\n'
                    f'❌ STUCK STATE DETECTED ❌\n'
                    f'Bot: ({self.bot_x:.4f}, {self.bot_y:.4f})\n'
                    f'Target: ({self.car_x:.4f}, {self.car_y:.4f})\n'
                    f'Separation: dx={abs(dx):.4f} (threshold 0.08), dy={abs(dy):.4f} (threshold 0.08), dist={distance:.4f} (threshold 0.10)\n'
                    f'Both dx and dy are < 0.08, but distance {distance:.4f} > 0.10\n'
                    f'This should not happen - possible oscillation or measurement issue\n'
                )
                
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
        
        elif self.state == "ADVANCE_TO_TOWING":
            TOWING_DISTANCE = 0.75

            dx = self.bot_x - getattr(self, 'towing_start_x', self.bot_x)
            dy = self.bot_y - getattr(self, 'towing_start_y', self.bot_y)
            distance_traveled = math.sqrt(dx**2 + dy**2)

            front_ranges = list(self.lidar_ranges[345:]) + list(self.lidar_ranges[:15])
            min_front = min([r for r in front_ranges if 0.1 < r < 10.0], default=10.0)

            if distance_traveled < TOWING_DISTANCE:
                if min_front < 0.3:
                    cmd.linear.x = 0.05
                    self.get_logger().warn(
                        f'⚠️  Obstacle at {min_front:.2f}m while advancing - crawling',
                        throttle_duration_sec=0.3
                    )
                elif min_front < 0.6:
                    cmd.linear.x = 0.1
                else:
                    cmd.linear.x = 0.2

                cmd.linear.y = 0.0
                cmd.angular.z = 0.0

                self.get_logger().info(
                    f'➡️  Advancing: {distance_traveled:.2f}m / {TOWING_DISTANCE}m',
                    throttle_duration_sec=0.3
                )
            else:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                self.state = "LIFTING"
                self.get_logger().info('🎯 Towing position reached! Starting lift...')
                self.get_logger().info('➡️  State: LIFTING')
        
        elif self.state == "LIFTING":
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            
            TARGET_LIFT_HEIGHT = 0.35
            
            if not hasattr(self, 'lift_started'):
                self.get_logger().info('⬆️  Starting lift...')
                self.lift_start_time = self.get_clock().now()
                self.lift_car(TARGET_LIFT_HEIGHT)
                self.lift_started = True
            
            elapsed = (self.get_clock().now() - self.lift_start_time).nanoseconds / 1e9
            
            if elapsed < 3.0:
                self.lift_car(TARGET_LIFT_HEIGHT)
                self.get_logger().info(
                    f'⬆️  Lifting... {elapsed:.1f}s / 3.0s',
                    throttle_duration_sec=0.5
                )
            else:
                self.get_logger().info('✅ Lift extended! Car should be lifted now.')
                self.car_lifted = True
                self.state = "READY_TO_TOW"
        
        elif self.state == "READY_TO_TOW":
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            
            self.get_logger().info(
                '🎉 LIFT RAISED!\n'
                '   The lift arm should be pushing the car up.\n'
                '   If car is lifted, you can control bot with teleop.\n'
                '   Test with: ros2 run teleop_twist_keyboard teleop_twist_keyboard',
                throttle_duration_sec=3.0
            )
        
        elif self.state == "DONE":
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GapApproach()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()