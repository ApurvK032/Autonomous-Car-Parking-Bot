#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraToMap(Node):

    def __init__(self):
        super().__init__('camera_to_map')
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/parking_lot/overhead_camera/image_raw',
            self.image_callback,
            10)
        
        self.get_logger().info('Camera to Map node started')
        self.image_saved = False
        
    def image_callback(self, msg):
        if not self.image_saved:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            cv2.imwrite('/tmp/parking_lot_raw.png', cv_image)
            self.get_logger().info('Saved raw camera image to /tmp/parking_lot_raw.png')
            
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
            
            occupancy = 255 - binary
            
            cv2.imwrite('/tmp/parking_lot_map.pgm', occupancy)
            self.get_logger().info('Saved occupancy grid to /tmp/parking_lot_map.pgm')
            
            cv2.imshow('Raw Camera', cv_image)
            cv2.imshow('Occupancy Grid', occupancy)
            cv2.waitKey(3000)
            
            self.image_saved = True
            self.get_logger().info('Map generation complete!')


def main(args=None):
    rclpy.init(args=args)
    node = CameraToMap()
    
    rclpy.spin_once(node, timeout_sec=5.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

