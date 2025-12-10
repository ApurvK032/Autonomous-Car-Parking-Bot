# Autonomous Car Parking Bot

ROS 2 Humble implementation of autonomous parking management system for indoor facilities.

## Overview
This project implements a two-workspace system for autonomous parking management:
- **Workspace 1**: Overhead camera detection with Nav2 navigation (40m×40m parking lot)
- **Workspace 2**: Towing robot with lifting mechanism (20m×20m environment)

## Features
- OpenCV-based vehicle detection with coordinate transformation
- Nav2 navigation with obstacle avoidance
- Manhattan-style precise approach navigation
- ros2_control lift mechanism
- Trajectory planning framework (Li et al. 2022)

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11
- Python 3.10+
- OpenCV
- Nav2 navigation stack

## Installation & Usage

### Workspace 1: Detection & Navigation
```bash
cd ws1
colcon build
source install/setup.bash
ros2 launch parking_lot_sim parking_lot.launch.py
```

### Workspace 2: Towing Robot
```bash
cd ws2
colcon build
source install/setup.bash
ros2 launch towing_sim towing.launch.py
```

## Project Structure
```
├── ws1/                    # Workspace 1
│   └── src/
│       ├── parking_lot_sim/      # Main simulation environment
│       ├── smart_detector/       # OpenCV-based detection
│       ├── goal_publisher/       # Navigation goal management
│       └── map_generator/        # Static map generation
└── ws2/                    # Workspace 2
    └── src/
        ├── towing_sim/           # Towing robot simulation
        ├── towing_bot/           # Robot URDF and control
        └── trajectory_planner/   # Path planning (Phase 5)
```

## Technical Highlights
- **Detection**: HSV color segmentation with coordinate transformation handling
- **Navigation**: Nav2 stack with DWB controller for mecanum drive
- **Manipulation**: ros2_control position controller for lift mechanism
- **Trajectory Planning**: Optimization-based corridor construction approach

## Authors
- **Apurv Kushwaha** - kushw009@umn.edu
- **Rutav Narkhede** - narkh003@umn.edu

## Course
CSCI 5551 - Introduction to Robotics and Intelligent Systems  
University of Minnesota - Fall 2024

## License
MIT License
