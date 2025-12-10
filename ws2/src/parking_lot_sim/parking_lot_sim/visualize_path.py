#!/usr/bin/env python3
"""
Visualize a simple planned path on the static map.
- Loads parking_lot_nav_map.pgm and parking_config.yaml
- Draws parked cars as obstacles
- Marks start (home), misparked car, goal, and a straight-line path
"""

import os
import math
import yaml
from PIL import Image, ImageDraw


def load_map_and_metadata():
    """Load map image and YAML metadata."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(script_dir, '..', 'maps')
    map_yaml = os.path.join(maps_dir, 'parking_lot_nav_map.yaml')

    if not os.path.exists(map_yaml):
        raise FileNotFoundError(f"Map metadata not found: {map_yaml}")

    with open(map_yaml, 'r') as f:
        meta = yaml.safe_load(f) or {}

    map_image_path = os.path.join(maps_dir, meta['image'])
    if not os.path.exists(map_image_path):
        raise FileNotFoundError(f"Map image not found: {map_image_path}")

    img = Image.open(map_image_path).convert('RGB')
    return img, meta


def world_to_pixel(x, y, meta, img_height):
    """
    Convert world coordinates to pixel coordinates using map metadata.
    origin is bottom-left in ROS map; image y=0 is top, so flip y.
    """
    res = meta['resolution']
    origin_x, origin_y = meta['origin'][0], meta['origin'][1]
    top_y = origin_y + img_height * res
    px = int((x - origin_x) / res)
    py = int((top_y - y) / res)
    return px, py


def draw_car_obstacles(draw, meta, cars):
    """Draw parked cars as filled black polygons."""
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

    height = draw.im.size[1]
    for name, car in cars.items():
        pos = car.get('position', [0, 0])
        car_x = pos[1]
        car_y = pos[0]

        if 'orientation_deg' in car:
            angle_rad = math.radians(car['orientation_deg'])
        elif 'orientation' in car:
            angle_rad = float(car['orientation'])
        else:
            angle_rad = 0.0

        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        world_corners = []
        for lx, ly in local_corners:
            wx = car_x + (lx * cos_a - ly * sin_a)
            wy = car_y + (lx * sin_a + ly * cos_a)
            world_corners.append((wx, wy))

        pixel_corners = [
            world_to_pixel(wx, wy, meta, height) for wx, wy in world_corners
        ]
        draw.polygon(pixel_corners, fill=(0, 0, 0))


def choose_misparked_car(cars):
    """Pick a misparked car: first with 'misparked' in name, else first car."""
    for name, data in cars.items():
        if 'misparked' in name.lower():
            return name, data
    # Fallback to first car
    for name, data in cars.items():
        return name, data
    return None, None


def compute_goal(car_data, approach_distance=3.5):
    """Compute goal 3.5m behind the car."""
    pos = car_data.get('position', [0, 0])
    car_x = pos[1]
    car_y = pos[0]

    if 'orientation_deg' in car_data:
        car_angle = math.radians(car_data['orientation_deg'])
    elif 'orientation' in car_data:
        car_angle = float(car_data['orientation'])
    else:
        car_angle = 0.0

    goal_x = car_x - approach_distance * math.cos(car_angle)
    goal_y = car_y - approach_distance * math.sin(car_angle)
    return (car_x, car_y), (goal_x, goal_y), car_angle


def visualize():
    img, meta = load_map_and_metadata()
    draw = ImageDraw.Draw(img)
    height = img.size[1]

    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'config', 'parking_config.yaml')
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"parking_config.yaml not found: {config_path}")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f) or {}
    cars = config.get('cars', {})

    draw_car_obstacles(draw, meta, cars)

    car_name, car_data = choose_misparked_car(cars)
    if not car_data:
        raise RuntimeError("No car found in parking_config.yaml to visualize")

    car_pos, goal_pos, _ = compute_goal(car_data, approach_distance=3.5)

    start_world = (-15.0, -15.0)

    start_px = world_to_pixel(start_world[0], start_world[1], meta, height)
    car_px = world_to_pixel(car_pos[0], car_pos[1], meta, height)
    goal_px = world_to_pixel(goal_pos[0], goal_pos[1], meta, height)

    def draw_circle(center, radius, color):
        x, y = center
        draw.ellipse([x - radius, y - radius, x + radius, y + radius], fill=color, outline=color)

    draw_circle(start_px, 6, (0, 255, 0))
    draw_circle(car_px, 6, (255, 0, 0))
    draw_circle(goal_px, 6, (0, 0, 255))

    draw.line([start_px, goal_px], fill=(0, 0, 255), width=2)

    maps_dir = os.path.join(script_dir, '..', 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    out_path = os.path.join(maps_dir, 'path_visualization.png')
    img.save(out_path)
    print(f"✅ Path visualization saved to: {out_path}")


if __name__ == '__main__':
    visualize()
