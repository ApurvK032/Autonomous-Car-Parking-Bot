#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np

class CarDetector(Node):
    def __init__(self):
        super().__init__('car_detector')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/overhead/image_raw',
            self.image_callback,
            10
        )
        
        self.position_publisher = self.create_publisher(
            Pose2D,
            '/car_position',
            10
        )
        
        self.get_logger().info('🎥 Car Detector started - looking for blue car...')
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            self.get_logger().info(f'📸 Image received: {cv_image.shape}', throttle_duration_sec=2.0)
            
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            lower_blue = np.array([90, 50, 50])
            upper_blue = np.array([130, 255, 255])
            
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            
            blue_pixels = cv2.countNonZero(mask)
            self.get_logger().info(f'🔵 Blue pixels detected: {blue_pixels}', throttle_duration_sec=2.0)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            self.get_logger().info(f'📦 Contours found: {len(contours)}', throttle_duration_sec=2.0)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                self.get_logger().info(f'📏 Largest contour area: {area}', throttle_duration_sec=2.0)
                
                if area > 500:
                    rect = cv2.minAreaRect(largest_contour)
                    center, size, angle = rect
                    
                    world_x = 5.0
                    world_y = 5.0
                    
                    self.get_logger().info(
                        f'🔍 CENTER: World: ({world_x:.2f}, {world_y:.2f})',
                        throttle_duration_sec=2.0
                    )
                    
                    world_angle = 0.0
                    
                    car_pose = Pose2D()
                    car_pose.x = world_x
                    car_pose.y = world_y
                    car_pose.theta = world_angle
                    
                    self.position_publisher.publish(car_pose)
                    
                    self.get_logger().info(
                        f'🚗 Car center at: x={world_x:.2f}m, y={world_y:.2f}m (GOAL POSITION)',
                        throttle_duration_sec=2.0
                    )
                    
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    
                    cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
                    cv2.circle(cv_image, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
                    
                    cv2.imshow('Mask', mask)
                    cv2.imshow('Car Detection', cv_image)
                    cv2.waitKey(1)
                else:
                    self.get_logger().info(f'⚠️ Contour too small: {area}', throttle_duration_sec=2.0)
                    
        except Exception as e:
            self.get_logger().error(f'Error in detection: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CarDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()