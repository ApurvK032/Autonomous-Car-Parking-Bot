# Autonomous Car-Lifting & Parking Robot

A ROS 2-based autonomous system for intelligent car parking and precise manipulation using motion planning, sensor fusion, and real-time path optimization in simulated and physical environments.

## 🎯 Project Overview

This project implements an autonomous robotic system capable of:
- **Autonomous Navigation**: Real-time path planning and obstacle avoidance
- **Car Detection & Localization**: Computer vision-based parking space identification
- **Precise Manipulation**: 6-DOF arm control for vehicle lifting and positioning
- **Motion Planning**: Collision-free trajectory generation and execution
- **Simulation & Validation**: Gazebo-based testing and performance benchmarking

## 📊 Key Results

| Metric | Value | Notes |
|--------|-------|-------|
| Path Planning Success Rate | XX% | A* with dynamic obstacle avoidance |
| Average Planning Time | XXms | Real-time performance |
| Path Efficiency | XX% | vs. theoretical optimal |
| Positional Accuracy | ±X cm | End-effector precision |
| Manipulation Speed | X moves/min | Lifting and parking cycles |

## 🛠️ Technical Stack

- **Framework**: ROS 2 (Humble/Iron)
- **Simulation**: Gazebo (version X)
- **Motion Planning**: MoveIt 2, A* pathfinding
- **Programming**: Python, C++
- **Sensors**: LiDAR, RGB-D Camera, Odometry
- **Hardware Support**: (Add your target platform)

## 📁 Project Structure

```
car-parking-bot/
├── README.md
├── docs/
│   ├── ARCHITECTURE.md          # System design & component overview
│   ├── SETUP.md                 # Installation & environment setup
│   ├── USAGE.md                 # Running the system
│   └── RESULTS.md               # Experimental results & analysis
├── src/
│   ├── navigation/
│   │   ├── path_planner.py      # A* and motion planning
│   │   ├── obstacle_detector.py # Real-time obstacle detection
│   │   └── costmap_manager.py   # Dynamic costmap updates
│   ├── perception/
│   │   ├── car_detector.py      # Vehicle detection & tracking
│   │   ├── parking_space_finder.py  # Parking space localization
│   │   └── sensor_fusion.py     # Multi-sensor integration
│   ├── manipulation/
│   │   ├── arm_controller.py    # 6-DOF arm control
│   │   ├── grasp_planner.py     # Grasp planning & execution
│   │   └── trajectory_generator.py
│   └── launch/
│       ├── simulation.launch.py
│       ├── hardware.launch.py
│       └── full_system.launch.py
├── config/
│   ├── planner_params.yaml
│   ├── robot_config.yaml
│   └── sensor_calib.yaml
├── gazebo_models/
│   ├── parking_environment/
│   ├── robot_model/
│   └── car_model/
├── tests/
│   ├── test_path_planning.py
│   ├── test_perception.py
│   └── test_arm_control.py
├── results/
│   ├── benchmark_results.csv
│   ├── trajectories.txt
│   └── evaluation_plots/
├── requirements.txt
└── setup.py
```

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04 / 24.04
- ROS 2 (Humble or Iron)
- Python 3.10+
- Gazebo 11/12

### Installation

```bash
# Clone repository
git clone https://github.com/ApurvK032/car-parking-bot.git
cd car-parking-bot

# Install dependencies
pip install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build
source install/setup.bash
```

### Running Simulation

```bash
# Launch full simulation with visualization
ros2 launch car_parking_bot simulation.launch.py

# In another terminal, start planning and control
ros2 run car_parking_bot path_planner_node
ros2 run car_parking_bot manipulation_controller_node
```

### Running on Hardware

```bash
ros2 launch car_parking_bot hardware.launch.py
```

## 🔍 Core Features

### 1. Motion Planning
- **Algorithm**: A* with Euclidean heuristic and dynamic obstacle handling
- **Replanning**: Real-time replanning when obstacles detected
- **Performance**: ~XXms average computation time

### 2. Perception Pipeline
- **Car Detection**: YOLO v8 / Faster R-CNN on RGB imagery
- **Parking Space Localization**: Multi-sensor fusion (LiDAR + Camera)
- **Accuracy**: ±X cm localization error

### 3. Arm Control
- **Inverse Kinematics**: Closed-form / numerical solver via MoveIt 2
- **Trajectory Execution**: Smooth spline interpolation
- **Safety**: Collision checking throughout motion

### 4. Sensor Fusion
- Combines LiDAR point clouds, camera images, and odometry
- Extended Kalman Filter for state estimation
- Robust to sensor noise and dropouts

## 📈 Performance Benchmarks

### Path Planning (100 random scenarios)
- Median planning time: XXms
- 95th percentile time: XXms
- Success rate: XX%

### Manipulation Tasks (50 cycles)
- Average cycle time: XX seconds
- Success rate: XX%
- Positioning accuracy: ±X.X cm

### Simulation vs. Reality Transfer
- Model accuracy: XX%
- Transfer learning performance: XX%

## 📚 Documentation

- **[Architecture](docs/ARCHITECTURE.md)** - System design, component interactions, and design decisions
- **[Setup Guide](docs/SETUP.md)** - Detailed installation and environment configuration
- **[Usage Guide](docs/USAGE.md)** - Running experiments and interpreting results
- **[Results Analysis](docs/RESULTS.md)** - Experimental validation and performance analysis

## 🧪 Testing

```bash
# Run all tests
colcon test

# Run specific test suite
colcon test --packages-select car_parking_bot --ctest-args -R test_path_planning
```

## 🔧 Configuration

Key parameters in `config/`:
- **Path Planner**: Grid resolution, inflation radius, replanning frequency
- **Perception**: Detection thresholds, fusion weights
- **Manipulation**: Joint limits, speed profiles, collision margins

See individual config files for detailed parameter descriptions.

## 🎓 Learning Resources

- ROS 2 Motion Planning: [MoveIt 2 Docs](https://moveit.picknik.ai/)
- Path Planning Algorithms: [Optimal Motion Planning](https://arxiv.org/abs/1105.1186)
- Gazebo Simulation: [Gazebo Tutorials](https://gazebosim.org/docs)

## 📊 Future Improvements

- [ ] Real-world hardware deployment
- [ ] Deep reinforcement learning for policy refinement
- [ ] Multi-robot coordination for parallel parking
- [ ] Integration with fleet management systems
- [ ] Advanced sensor fusion with IMU

## 📝 Results & Publications

Experimental results and performance metrics are documented in:
- `results/benchmark_results.csv` - Quantitative metrics
- `results/evaluation_plots/` - Visualization comparisons
- See [Results Analysis](docs/RESULTS.md) for detailed interpretation

## 🤝 Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Commit changes with clear messages
4. Submit a pull request with description

## 📞 Contact

**Apurv Kushwaha**
- Email: kushw022@umn.edu
- LinkedIn: [linkedin.com/in/kushwahaapurv](https://linkedin.com/in/kushwahaapurv)
- Portfolio: [ApurvK032.github.io](https://ApurvK032.github.io)

## 📄 License

This project is licensed under the MIT License - see LICENSE file for details.

## 🙏 Acknowledgments

- University of Minnesota Robotics Lab
- ROS 2 Community
- MoveIt 2 Contributors
- Gazebo Simulation Community

---

**Last Updated**: November 2025  
**Status**: In Development
