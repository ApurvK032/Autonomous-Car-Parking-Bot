#!/usr/bin/env python3
"""
Parse parking_lot.world XML file and generate parking_config.yaml
Extracts parking spaces, cars, and camera information from the world file.
"""

import xml.etree.ElementTree as ET
import yaml
import numpy as np
import os
import re


def parse_pose(pose_text):
    """Parse pose string 'X Y Z Roll Pitch Yaw' to list of floats"""
    try:
        values = pose_text.strip().split()
        if len(values) < 3:
            raise ValueError(f"Pose must have at least 3 values, got {len(values)}")
        return [float(v) for v in values]
    except ValueError as e:
        raise ValueError(f"Failed to parse pose '{pose_text}': {e}")


def parse_size(size_text):
    """Parse size string 'X Y Z' to list of floats"""
    try:
        values = size_text.strip().split()
        if len(values) < 3:
            raise ValueError(f"Size must have at least 3 values, got {len(values)}")
        return [float(v) for v in values]
    except ValueError as e:
        raise ValueError(f"Failed to parse size '{size_text}': {e}")


def parse_color(ambient_text):
    """Parse ambient color 'R G B A' and return color name"""
    try:
        values = ambient_text.strip().split()
        if len(values) < 3:
            return "unknown"
        r, g, b = float(values[0]), float(values[1]), float(values[2])
        
        # Color detection logic
        if r > 0.8 and g > 0.8:
            return "yellow"
        elif r > 0.7:
            return "red"
        elif b > 0.7:
            return "blue"
        else:
            return "unknown"
    except (ValueError, IndexError):
        return "unknown"


def extract_parking_spaces(root):
    """Extract parking spaces from world XML"""
    spaces = {}
    space_pattern = re.compile(r'space_(\d+)_line_(front|back|left|right)')
    
    space_lines = {}
    
    for model in root.findall('.//model'):
        name = model.get('name', '')
        match = space_pattern.match(name)
        if match:
            space_num = int(match.group(1))
            line_type = match.group(2)
            
            if space_num not in space_lines:
                space_lines[space_num] = {}
            
            pose_elem = model.find('.//pose')
            if pose_elem is None:
                raise ValueError(f"Space {space_num} line {line_type} missing <pose> element")
            
            pose = parse_pose(pose_elem.text)
            x, y = pose[0], pose[1]
            
            size_elem = model.find('.//size')
            if size_elem is None:
                raise ValueError(f"Space {space_num} line {line_type} missing <size> element")
            
            size = parse_size(size_elem.text)
            
            space_lines[space_num][line_type] = {
                'position': [x, y],
                'size': size
            }
    
    for space_num in sorted(space_lines.keys()):
        lines = space_lines[space_num]
        
        required_lines = ['front', 'back', 'left', 'right']
        missing = [line for line in required_lines if line not in lines]
        if missing:
            raise ValueError(f"Space {space_num} missing lines: {missing}. Found: {list(lines.keys())}")
        
        front = lines['front']
        back = lines['back']
        left = lines['left']
        right = lines['right']
        
        front_y = front['position'][1]
        back_y = back['position'][1]
        left_x = left['position'][0]
        right_x = right['position'][0]
        
        center_x = (front_y + back_y) / 2.0
        center_y = (left_x + right_x) / 2.0
        
        width = abs(front_y - back_y)
        height = abs(right_x - left_x)
        
        corners = [
            [front_y, left_x],
            [front_y, right_x],
            [back_y, right_x],
            [back_y, left_x]
        ]
        
        spaces[space_num] = {
            'center': [float(center_x), float(center_y)],
            'dimensions': [float(width), float(height)],
            'corners': [[float(c[0]), float(c[1])] for c in corners],
            'orientation': 0.0
        }
    
    return spaces


def extract_cars(root):
    """Extract cars from world XML"""
    cars = {}
    
    for model in root.findall('.//model'):
        name = model.get('name', '')
        
        if 'sedan' not in name.lower() and 'suv' not in name.lower():
            continue
        
        pose_elem = model.find('.//pose')
        if pose_elem is None:
            raise ValueError(f"Car '{name}' missing <pose> element")
        
        pose = parse_pose(pose_elem.text)
        x, y, z = pose[1], pose[0], pose[2]
        yaw = pose[5] if len(pose) > 5 else 0.0
        
        size_elem = model.find('.//size')
        if size_elem is None:
            raise ValueError(f"Car '{name}' missing <size> element")
        
        size = parse_size(size_elem.text)
        
        ambient_elem = model.find('.//ambient')
        color = "unknown"
        if ambient_elem is not None:
            color = parse_color(ambient_elem.text)
        
        space_id = None
        
        cars[name] = {
            'position': [float(x), float(y), float(z)],
            'orientation': float(yaw),
            'dimensions': [float(s) for s in size],
            'color': color,
            'space_id': space_id
        }
    
    return cars


def extract_camera(root):
    """Extract camera information from world XML"""
    camera_model = root.find(".//model[@name='overhead_camera']")
    if camera_model is None:
        raise ValueError("Camera model 'overhead_camera' not found")
    
    pose_elem = camera_model.find('.//pose')
    if pose_elem is None:
        raise ValueError("Camera missing <pose> element")
    
    pose = parse_pose(pose_elem.text)
    position = [pose[0], pose[1], pose[2]]
    
    fov_elem = camera_model.find('.//horizontal_fov')
    if fov_elem is None:
        raise ValueError("Camera missing <horizontal_fov> element")
    
    fov = float(fov_elem.text)
    
    width_elem = camera_model.find('.//width')
    height_elem = camera_model.find('.//height')
    if width_elem is None or height_elem is None:
        raise ValueError("Camera missing <width> or <height> element")
    
    width = int(width_elem.text)
    height = int(height_elem.text)
    
    camera_height = float(position[2])
    fov_float = float(fov)
    view_width_meters = 2.0 * camera_height * np.tan(fov_float / 2.0)
    pixels_per_meter = float(width) / float(view_width_meters)
    
    return {
        'position': [float(p) for p in position],
        'fov': float(fov),
        'resolution': [int(width), int(height)],
        'pixels_per_meter': pixels_per_meter
    }


def assign_cars_to_spaces(cars, spaces):
    """Assign space_id to cars based on their position"""
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
    
    for car_name, car_info in cars.items():
        car_pos = [car_info['position'][0], car_info['position'][1]]
        
        # Check each space
        for space_id, space_info in spaces.items():
            corners = space_info['corners']
            if point_in_polygon(car_pos, corners):
                car_info['space_id'] = space_id
                break


def parse_world_to_config(world_file, output_file):
    """Main function to parse world file and generate config"""
    try:
        tree = ET.parse(world_file)
        root = tree.getroot()
    except ET.ParseError as e:
        raise ValueError(f"Failed to parse XML file: {e}")
    
    spaces = extract_parking_spaces(root)
    cars = extract_cars(root)
    camera = extract_camera(root)
    
    assign_cars_to_spaces(cars, spaces)
    
    config = {
        'camera': camera,
        'spaces': spaces,
        'cars': cars
    }
    
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print(f"Successfully generated config file: {output_file}")
    print(f"  - Found {len(spaces)} parking spaces")
    print(f"  - Found {len(cars)} cars")
    print(f"  - Camera: {camera['resolution'][0]}x{camera['resolution'][1]}, FOV={camera['fov']:.3f} rad")
    print(f"  - Pixels per meter: {camera['pixels_per_meter']:.2f}")


def main():
    """Main entry point"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    possible_world_paths = [
        os.path.join(script_dir, '..', '..', 'worlds', 'parking_lot.world'),
        os.path.join(script_dir, '..', 'worlds', 'parking_lot.world'),
        '/root/ros2_ws/src/parking_lot_sim/worlds/parking_lot.world',
    ]
    
    world_file = None
    for path in possible_world_paths:
        abs_path = os.path.abspath(path)
        if os.path.exists(abs_path):
            world_file = abs_path
            break
    
    if world_file is None:
        world_file = os.path.abspath(possible_world_paths[0])
    
    if world_file:
        world_dir = os.path.dirname(world_file)
        config_dir = os.path.join(os.path.dirname(world_dir), 'config')
        output_file = os.path.join(config_dir, 'parking_config.yaml')
    else:
        output_file = os.path.join(script_dir, '..', 'config', 'parking_config.yaml')
        output_file = os.path.abspath(output_file)
    
    parse_world_to_config(world_file, output_file)


if __name__ == '__main__':
    main()
