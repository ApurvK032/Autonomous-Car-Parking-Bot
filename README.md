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

# Option 1: Launch with Nav2 navigation (recommended)
ros2 launch parking_lot_sim parking_nav.launch.py

# Option 2: Launch parking lot environment only (detection without navigation)
ros2 launch parking_lot_sim parking_lot.launch.py
```

### Workspace 2: Towing Robot
```bash
cd ws2
colcon build
source install/setup.bash

# Launch towing robot simulation
ros2 launch towing_sim towing.launch.py
```

## Project Structure
```
├── ws1/                    # Workspace 1
│   └── src/
│       └── parking_lot_sim/      
│           ├── launch/
│           │   ├── parking_lot.launch.py      # Environment only
│           │   └── parking_nav.launch.py      # With Nav2
│           ├── smart_detector/                # OpenCV detection
│           ├── goal_publisher/                # Navigation goals
│           └── map_generator/                 # Static maps
└── ws2/                    # Workspace 2
    └── src/
        ├── parking_lot_sim/      # Detection (copied from ws1)
        └── towing_sim/
            ├── launch/
            │   └── towing.launch.py           # Towing robot
            ├── towing_bot/                    # Robot URDF
            └── trajectory_planner/            # Path planning
```

## Technical Highlights
- **Detection**: HSV color segmentation with coordinate transformation handling (90° pitch compensation)
- **Navigation**: Nav2 stack with DWB controller for mecanum drive
- **Manipulation**: ros2_control position controller for lift mechanism
- **Trajectory Planning**: Optimization-based corridor construction approach

## System Phases
1. **Phase 1**: Overhead camera detection with parking status classification ✅
2. **Phase 2**: Nav2 navigation to approach misparked vehicles ✅
3. **Phase 3**: Manhattan navigation for precise positioning ✅
4. **Phase 4**: Lift mechanism with ros2_control ✅
5. **Phase 5**: Trajectory planning (implemented, awaiting robust towing mechanism) ⚠️

## Key Achievements
- Sub-degree detection accuracy (within 0.1°)
- Successful coordinate transformation (X-Y swap, Y-negation)
- Nav2 integration with static maps and obstacle avoidance
- Friction-based attachment (μ = 10¹²) for simulation
- Complete transform tree management (map→odom→base_link)

## Known Limitations
- Friction-based towing only works in static scenarios
- Color-based detection (red, blue, yellow cars)
- Static environment assumption
- Simulation-only (requires mechanical attachment for hardware)

## Authors
- **Apurv Kushwaha** - kushw009@umn.edu
- **Rutav Narkhede** - narkh003@umn.edu

## Course
CSCI 5551 - Introduction to Robotics and Intelligent Systems  
University of Minnesota - Fall 2024

## Citation
If you use this work, please cite:
```
A. Kushwaha and R. Narkhede, "Autonomous Parking Management System," 
CSCI 5551 Final Project, University of Minnesota, 2024.
```

## License
MIT License
