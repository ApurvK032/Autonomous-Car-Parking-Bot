#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml
import subprocess
import sys
import json


class SmartDetector(Node):

    def __init__(self):
        super().__init__('smart_detector')
        
        self.get_logger().info('Generating fresh config from world file...')
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_script = os.path.join(script_dir, 'world_to_config.py')
            
            result = subprocess.run(
                [sys.executable, config_script],
                capture_output=True,
                text=True,
                check=True
            )
            
            self.get_logger().info('Config generation successful!')
            if result.stdout:
                self.get_logger().info(result.stdout.strip())
                
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Config generation failed: {e.stderr}')
            self.get_logger().warn('Proceeding with existing config file...')
        except FileNotFoundError:
            self.get_logger().warn('world_to_config.py not found, using existing config')
        
        self.bridge = CvBridge()
        
        config_file = self._find_config_file()
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            self.parking_spaces = {}
            for space_id, space_info in config.get('spaces', {}).items():
                self.parking_spaces[int(space_id)] = {
                    'center': space_info['center'],
                    'dimensions': space_info['dimensions'],
                    'corners': space_info['corners'],
                    'orientation': space_info.get('orientation', 0.0),
                    'occupied': False
                }
            
            camera_config = config.get('camera', {})
            self.pixels_per_meter = camera_config.get('pixels_per_meter', 15.24)
            resolution = camera_config.get('resolution', [640, 480])
            self.image_center_x = resolution[0] // 2
            self.image_center_y = resolution[1] // 2
            
            self.expected_cars = config.get('cars', {})
            
            self.get_logger().info(f'Loaded config from {config_file}')
        else:
            self.get_logger().warn(f'Config file not found at {config_file}, using defaults')
            self.parking_spaces = {
                1: {"center": [-12.0, 8.0], "occupied": False, "orientation": 0.0, "corners": []},
                2: {"center": [0.0, 8.0], "occupied": False, "orientation": 0.0, "corners": []},
                3: {"center": [12.0, 8.0], "occupied": False, "orientation": 0.0, "corners": []},
                4: {"center": [-12.0, -8.0], "occupied": False, "orientation": 0.0, "corners": []},
                5: {"center": [0.0, -8.0], "occupied": False, "orientation": 0.0, "corners": []},
                6: {"center": [12.0, -8.0], "occupied": False, "orientation": 0.0, "corners": []},
            }
            self.pixels_per_meter = 15.24
            self.image_center_x = 320
            self.image_center_y = 240
            self.expected_cars = {}
        
        self.TOLERANCE = 2.0
        
        self.color_ranges = {
            'red': [
                {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])},
                {'lower': np.array([170, 100, 100]), 'upper': np.array([180, 255, 255])}
            ],
            'blue': [
                {'lower': np.array([100, 80, 80]), 'upper': np.array([130, 255, 255])}
            ],
            'yellow': [
                {'lower': np.array([20, 100, 100]), 'upper': np.array([35, 255, 255])}
            ],
        }
        
        self.subscription = self.create_subscription(
            Image,
            '/parking_lot/overhead_camera/image_raw',
            self.image_callback,
            10)
        
        self.detection_publisher = self.create_publisher(
            String,
            '/car_detections',
            10)
        
        self.last_process_time = 0.0
        self.detection_counter = 0
        
        self.get_logger().info('Smart Detector node initialized')
        self.get_logger().info(f'Parking spaces configured: {len(self.parking_spaces)} spaces')
        self.get_logger().info(f'Tolerance: {self.TOLERANCE}m')
        self.get_logger().info(f'Pixels per meter: {self.pixels_per_meter:.2f}')
        self.get_logger().info(f'Expected cars: {len(self.expected_cars)}')
    
    def _find_config_file(self):
        """Find config file in various possible locations"""
        possible_paths = [
            '/root/ros2_ws/src/parking_lot_sim/config/parking_config.yaml',
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'parking_config.yaml'),
            os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'src', 'parking_lot_sim', 'config', 'parking_config.yaml'),
        ]
        for path in possible_paths:
            if os.path.exists(path):
                return path
        return possible_paths[0]  # Return first path as default

    def pixel_to_world(self, pixel_x, pixel_y):
        """Convert pixel coordinates to world coordinates"""
        world_x = (pixel_x - self.image_center_x) / self.pixels_per_meter
        world_y = -(pixel_y - self.image_center_y) / self.pixels_per_meter
        return world_x, world_y

    def world_to_pixel(self, world_x, world_y):
        """Convert world coordinates to pixel coordinates"""
        pixel_x = int(world_x * self.pixels_per_meter + self.image_center_x)
        pixel_y = int(-world_y * self.pixels_per_meter + self.image_center_y)
        return pixel_x, pixel_y

    def detect_cars(self, cv_image):
        """Detect cars in image and return with orientation"""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        cars = []
        
        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for range_def in ranges:
                mask |= cv2.inRange(hsv, range_def['lower'], range_def['upper'])
            
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 500:
                    continue
                
                M = cv2.moments(contour)
                if M['m00'] == 0:
                    continue
                
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                rect = cv2.minAreaRect(contour)
                angle = rect[2]
                
                if rect[1][0] < rect[1][1]:
                    angle = angle + 90
                
                if angle < 0:
                    angle = angle + 180
                elif angle >= 180:
                    angle = angle - 180
                
                angle_rad = (np.deg2rad(angle) - np.pi/2) % (2 * np.pi)
                
                world_x, world_y = self.pixel_to_world(cx, cy)
                
                cars.append({
                    'color': color_name,
                    'position': [world_x, world_y],
                    'pixel_pos': [cx, cy],
                    'area': area,
                    'orientation': angle_rad,
                    'rect': rect
                })
        
        return cars

    def check_car_in_space(self, car_position, car_orientation):
        """Check if car is inside a parking space using point-in-polygon algorithm"""
        ANGLE_TOLERANCE = 0.2
        
        def point_in_polygon(point, polygon):
            """Check if point is inside polygon using ray casting algorithm"""
            x, y = point
            n = len(polygon)
            inside = False
            
            p1x, p1y = polygon[0]
            for i in range(1, n + 1):
                p2x, p2y = polygon[i % n]
                if y > min(p1y, p2y):
                    if y <= max(p1y, p2y):
                        if x <= max(p1x, p2x):
                            if p1y != p2y:
                                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
                p1x, p1y = p2x, p2y
            
            return inside
        
        for space_id, space_info in self.parking_spaces.items():
            corners = space_info.get('corners', [])
            if not corners:
                continue
            
            if point_in_polygon(car_position, corners):
                space_orientation = space_info.get('orientation', 0.0)
                
                angle_diff = abs(car_orientation - space_orientation)
                
                if angle_diff > np.pi:
                    angle_diff = 2 * np.pi - angle_diff
                
                if angle_diff < ANGLE_TOLERANCE:
                    return {
                        'status': 'CORRECT',
                        'space_id': space_id,
                        'angle_diff': angle_diff
                    }
                else:
                    return {
                        'status': 'MISPARKED_ANGLE',
                        'space_id': space_id,
                        'angle_diff': angle_diff
                    }
        
        return {
            'status': 'MISPARKED',
            'space_id': None,
            'angle_diff': None
        }

    def check_parking_status(self, car_pos, car_orientation):
        """Check if car is properly parked in a space using corner-based detection"""
        result = self.check_car_in_space(car_pos, car_orientation)
        
        distance = None
        if result['space_id']:
            space_info = self.parking_spaces[result['space_id']]
            space_center = space_info['center']
            distance = np.sqrt(
                (car_pos[0] - space_center[0])**2 + 
                (car_pos[1] - space_center[1])**2
            )
        else:
            min_distance = float('inf')
            for space_id, space_info in self.parking_spaces.items():
                space_center = space_info['center']
                dist = np.sqrt(
                    (car_pos[0] - space_center[0])**2 + 
                    (car_pos[1] - space_center[1])**2
                )
                if dist < min_distance:
                    min_distance = dist
            distance = min_distance
        
        result['distance'] = distance
        return result

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec - self.last_process_time < 0.5:
                return
            self.last_process_time = now_sec
            self.detection_counter += 1
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            detected_cars = self.detect_cars(cv_image)
            
            vis_image = cv_image.copy()
            
            for space_id in self.parking_spaces:
                self.parking_spaces[space_id]['occupied'] = False
            
            car_reports = []
            for i, car in enumerate(detected_cars):
                status = self.check_parking_status(car['position'], car['orientation'])
                
                if status['space_id']:
                    self.parking_spaces[status['space_id']]['occupied'] = True
                
                car_report = {
                    'id': i + 1,
                    'color': car['color'],
                    'position': car['position'],
                    'orientation_deg': np.rad2deg(car['orientation']),
                    'status': status['status'],
                    'space_id': status['space_id'],
                    'distance_from_center': status['distance'],
                    'angle_difference_deg': np.rad2deg(status.get('angle_diff', 0)) if status.get('angle_diff') is not None else None
                }
                car_reports.append(car_report)
                
                if status['status'] == 'MISPARKED':
                    self.get_logger().warn(
                        f"MISPARKED (wrong location): {car['color']} car at ({car['position'][0]:.1f}, {car['position'][1]:.1f})"
                    )
                elif status['status'] == 'MISPARKED_ANGLE':
                    self.get_logger().warn(
                        f"MISPARKED (wrong angle): {car['color']} car in space {status['space_id']}, "
                        f"angle off by {np.rad2deg(status['angle_diff']):.1f} degrees"
                    )
                else:
                    self.get_logger().info(
                        f"Correctly parked: {car['color']} car in space {status['space_id']}"
                    )
            
            for space_id, space_info in self.parking_spaces.items():
                px, py = self.world_to_pixel(space_info['center'][0], space_info['center'][1])
                color = (0, 255, 0) if not space_info['occupied'] else (0, 0, 255)
                cv2.circle(vis_image, (px, py), 10, color, -1)
                cv2.putText(vis_image, f"S{space_id}", (px-15, py-20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                status_text = "FREE" if not space_info['occupied'] else "OCCUPIED"
                cv2.putText(vis_image, status_text, (px-25, py+25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            for i, car in enumerate(detected_cars):
                px, py = car['pixel_pos']
                
                if 'rect' in car:
                    box = cv2.boxPoints(car['rect'])
                    box = np.int0(box)
                    cv2.drawContours(vis_image, [box], 0, (0, 255, 255), 2)
                
                cv2.circle(vis_image, (px, py), 8, (255, 0, 255), -1)
                
                car_report = car_reports[i] if i < len(car_reports) else None
                
                if car_report:
                    if car_report['status'] == 'CORRECT':
                        label_color = (0, 255, 0)
                        status_emoji = "✓"
                    elif car_report['status'] == 'MISPARKED_ANGLE':
                        label_color = (0, 165, 255)
                        status_emoji = "∠"
                    else:
                        label_color = (0, 0, 255)
                        status_emoji = "✗"
                    
                    label_lines = [
                        f"{car['color'].upper()} #{i+1}",
                        f"Pos: ({car_report['position'][0]:.1f}, {car_report['position'][1]:.1f})",
                        f"{status_emoji} {car_report['status']}",
                    ]
                    
                    if car_report['space_id']:
                        label_lines.append(f"Space {car_report['space_id']}")
                    
                    label_y_start = py - 60
                    for j, line in enumerate(label_lines):
                        y_pos = label_y_start + (j * 20)
                        cv2.putText(vis_image, line, (px + 20, y_pos), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_color, 2)
            
            title = "Parking Lot Detection Analysis"
            cv2.putText(vis_image, title, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            
            correct_count = sum(1 for r in car_reports if r['status'] == 'CORRECT')
            misparked_count = len(car_reports) - correct_count
            
            stats = f"Cars: {len(car_reports)} | Correct: {correct_count} | Misparked: {misparked_count}"
            cv2.putText(vis_image, stats, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            maps_dir = '/root/ros2_ws/src/parking_lot_sim/maps'
            os.makedirs(maps_dir, exist_ok=True)
            
            if self.detection_counter % 5 == 0:
                cv2.imwrite(f'{maps_dir}/detection_result.png', vis_image)
                cv2.imwrite('/tmp/parking_visualization.png', vis_image)
                self.get_logger().info(f'Saved detection visualization to {maps_dir}/detection_result.png')
            
            if self.expected_cars:
                expected_count = len(self.expected_cars)
                detected_count = len(detected_cars)
                if detected_count != expected_count:
                    self.get_logger().warn(
                        f"Car count mismatch: Expected {expected_count} cars from config, "
                        f"detected {detected_count} cars"
                    )
            
            if detected_cars:
                self.get_logger().info(f"Detected {len(detected_cars)} cars")
            
            detection_msg = String()
            detection_msg.data = json.dumps(car_reports)
            self.detection_publisher.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = SmartDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
