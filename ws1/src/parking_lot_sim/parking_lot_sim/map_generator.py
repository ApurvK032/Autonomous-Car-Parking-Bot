#!/usr/bin/env python3
"""
Generate static occupancy grid map for Nav2 from parking lot dimensions
"""
import numpy as np
from PIL import Image, ImageDraw
import yaml
import os
import math

def generate_map():
    """Create 40m x 40m parking lot map"""
    
    MAP_SIZE_M = 40.0
    RESOLUTION = 0.05
    MAP_SIZE_PIX = int(MAP_SIZE_M / RESOLUTION)
    
    map_array = np.ones((MAP_SIZE_PIX, MAP_SIZE_PIX), dtype=np.uint8) * 254
    
    img = Image.fromarray(map_array)
    draw = ImageDraw.Draw(img)
    
    def world_to_pixel(x, y):
        """Convert world coordinates to pixel coordinates"""
        px = int((x + MAP_SIZE_M/2) / RESOLUTION)
        py = int((MAP_SIZE_M/2 - y) / RESOLUTION)
        return (px, py)
    
    spaces = [
        (-12, 8), (0, 8), (12, 8),
        (-12, -8), (0, -8), (12, -8)
    ]
    print(f"Parking space centers (world): {spaces}")
    
    SPACE_WIDTH = 6.0
    SPACE_HEIGHT = 5.0
    
    for x, y in spaces:
        x1, y1 = world_to_pixel(x - SPACE_WIDTH/2, y - SPACE_HEIGHT/2)
        x2, y2 = world_to_pixel(x + SPACE_WIDTH/2, y + SPACE_HEIGHT/2)
        x_min, x_max = sorted([x1, x2])
        y_min, y_max = sorted([y1, y2])
        draw.rectangle([x_min, y_min, x_max, y_max], outline=200, width=2)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'config', 'parking_config.yaml')
    if os.path.exists(config_path):
        print(f"Reading parking_config.yaml from: {config_path}")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f) or {}
        print(f"Loaded parking_config.yaml contents:\n{config}\n")
        cars = config.get('cars', {})

        CAR_LENGTH = 4.0
        CAR_WIDTH = 2.0
        half_len = CAR_LENGTH / 2.0
        half_wid = CAR_WIDTH / 2.0
        local_corners = [
            (half_len, half_wid),
            (half_len, -half_wid),
            (-half_len, -half_wid),
            (-half_len, half_wid),
        ]

        for car_name, car in cars.items():
            pos = car.get('position', [0, 0])
            car_x = pos[1]
            car_y = pos[0]

            if 'orientation_deg' in car:
                angle_rad = math.radians(car['orientation_deg'])
            elif 'orientation' in car:
                angle_rad = float(car['orientation'])
            else:
                angle_rad = 0.0

            print(f"Car '{car_name}': pos=({car_x}, {car_y}), angle_rad={angle_rad} ({math.degrees(angle_rad):.1f} deg)")

            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            world_corners = []
            for lx, ly in local_corners:
                wx = car_x + (lx * cos_a - ly * sin_a)
                wy = car_y + (lx * sin_a + ly * cos_a)
                world_corners.append((wx, wy))

            pixel_corners = [world_to_pixel(wx, wy) for wx, wy in world_corners]
            print(f"   pixel corners: {pixel_corners}")
            draw.polygon(pixel_corners, fill=0)
    
    WALL_THICKNESS_PIX = int(0.5 / RESOLUTION)
    
    draw.rectangle([0, 0, MAP_SIZE_PIX, WALL_THICKNESS_PIX], fill=0)
    draw.rectangle([0, MAP_SIZE_PIX-WALL_THICKNESS_PIX, MAP_SIZE_PIX, MAP_SIZE_PIX], fill=0)
    draw.rectangle([0, 0, WALL_THICKNESS_PIX, MAP_SIZE_PIX], fill=0)
    draw.rectangle([MAP_SIZE_PIX-WALL_THICKNESS_PIX, 0, MAP_SIZE_PIX, MAP_SIZE_PIX], fill=0)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_dir = os.path.join(script_dir, '..', 'maps')
    fallback_dir = os.path.join(os.path.expanduser('~'), '.parking_lot_sim_maps')
    
    os.makedirs(map_dir, exist_ok=True)
    
    map_path = os.path.join(map_dir, 'parking_lot_nav_map.pgm')
    yaml_path = os.path.join(map_dir, 'parking_lot_nav_map.yaml')
    map_metadata = {
        'image': 'parking_lot_nav_map.pgm',
        'resolution': RESOLUTION,
        'origin': [-MAP_SIZE_M/2, -MAP_SIZE_M/2, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    try:
        img.save(map_path)
        with open(yaml_path, 'w') as f:
            yaml.dump(map_metadata, f, default_flow_style=False)
        print(f"✅ Map saved to: {map_path}")
        print(f"✅ YAML saved to: {yaml_path}")
    except PermissionError:
        print(f"⚠️ Permission denied writing to {map_dir}, using fallback {fallback_dir}")
        os.makedirs(fallback_dir, exist_ok=True)
        map_path = os.path.join(fallback_dir, 'parking_lot_nav_map.pgm')
        yaml_path = os.path.join(fallback_dir, 'parking_lot_nav_map.yaml')
        img.save(map_path)
    with open(yaml_path, 'w') as f:
        yaml.dump(map_metadata, f, default_flow_style=False)
        print(f"✅ Map saved to: {map_path}")
    print(f"✅ YAML saved to: {yaml_path}")
    
    print(f"   Size: {MAP_SIZE_PIX}x{MAP_SIZE_PIX} pixels ({MAP_SIZE_M}m x {MAP_SIZE_M}m)")
    print(f"\n📁 Absolute paths:")
    print(f"   Map: {os.path.abspath(map_path)}")
    print(f"   YAML: {os.path.abspath(yaml_path)}")

if __name__ == '__main__':
    generate_map()
    print("\n🎉 Map generation complete!")