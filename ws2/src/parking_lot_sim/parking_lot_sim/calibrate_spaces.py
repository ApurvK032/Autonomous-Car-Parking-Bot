#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SpaceCalibrator(Node):
    def __init__(self):
        super().__init__('space_calibrator')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/parking_lot/overhead_camera/image_raw',
            self.image_callback,
            10)
        self.latest_image = None
        
    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"Clicked at: x={x}, y={y}")

def main():
    rclpy.init()
    calibrator = SpaceCalibrator()
    
    cv2.namedWindow('Calibration')
    cv2.setMouseCallback('Calibration', calibrator.mouse_callback)
    
    rate = calibrator.create_rate(10)
    
    print("Click on parking space corners to get coordinates")
    print("Press 'q' to quit")
    
    while rclpy.ok():
        rclpy.spin_once(calibrator, timeout_sec=0.1)
        
        if calibrator.latest_image is not None:
            cv2.imshow('Calibration', calibrator.latest_image)
        
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
    
    cv2.destroyAllWindows()
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
