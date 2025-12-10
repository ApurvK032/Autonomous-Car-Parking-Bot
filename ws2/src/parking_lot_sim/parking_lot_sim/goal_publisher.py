#!/usr/bin/env python3
"""
Goal Publisher Node - Converts car detections to navigation goals
Subscribes to detection results and publishes Nav2 goals
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json
import math

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        self.detection_sub = self.create_subscription(
            String,
            '/car_detections',
            self.detection_callback,
            10
        )
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('🎯 Goal Publisher started')
        self.get_logger().info('   Waiting for car detections...')
        
    def detection_callback(self, msg):
        """Process detection results and publish goal"""
        try:
            self.get_logger().info(f'📥 Raw detection data: {msg.data}')
            
            detections = json.loads(msg.data)
            
            misparked = [car for car in detections if car['status'] != 'CORRECT']
            
            if not misparked:
                self.get_logger().info('✅ All cars parked correctly!')
                return
            
            target = misparked[0]
            
            raw_pos = target['position']

            car_x = -raw_pos[0]
            car_y = raw_pos[1]

            self.get_logger().info(f"🧭 Raw detection pos: {raw_pos} -> transformed: ({car_x:.3f}, {car_y:.3f})")
            angle_rad = math.radians(target['orientation_deg'])
            
            self.get_logger().info(f'🚗 Target: {target["color"]} car at ({car_x:.1f}, {car_y:.1f})')
            self.get_logger().info(f'   Status: {target["status"]}, Angle: {math.degrees(angle_rad):.1f}°')
            
            goal = self.calculate_goal_pose(car_x, car_y, angle_rad)
            self.get_logger().info(f'   Goal (world): ({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f})')
            
            if not self.nav_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error('❌ Nav2 action server not available')
                return
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal
            
            self.get_logger().info(f'📨 Sending navigation goal to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')
            self.nav_client.send_goal_async(goal_msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error processing detection: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def calculate_goal_pose(self, car_x, car_y, car_angle):
        """Calculate goal pose 2m behind the car"""
        approach_distance = 3.5
        goal_x = car_x - approach_distance * math.cos(car_angle)
        goal_y = car_y - approach_distance * math.sin(car_angle)
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        
        goal.pose.orientation.z = math.sin(car_angle / 2.0)
        goal.pose.orientation.w = math.cos(car_angle / 2.0)
        
        return goal

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()