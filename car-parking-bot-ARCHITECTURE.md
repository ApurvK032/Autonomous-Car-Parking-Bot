# System Architecture - Autonomous Car-Lifting & Parking Robot

## High-Level System Design

```
SENSOR INPUTS
├── LiDAR (360° 2D/3D point clouds)
├── RGB-D Camera (depth + color images)
├── IMU (acceleration, angular velocity)
└── Wheel Odometry (motion estimates)
        ↓
┌───────────────────────────────────┐
│  SENSOR FUSION & STATE EST.       │
│  (Extended Kalman Filter)         │
└───────────────────────────────────┘
        ↓
    ┌───────────────────────────────────┬────────────────────────────┐
    ↓                                   ↓                            ↓
┌──────────────────┐      ┌──────────────────────┐     ┌──────────────┐
│   PERCEPTION     │      │   NAVIGATION        │     │  MANIPULATION│
│   PIPELINE       │      │   & PLANNING        │     │  & CONTROL   │
├──────────────────┤      ├──────────────────────┤     ├──────────────┤
│ • Car Detection  │      │ • Global Path Plan   │     │ • IK Solver  │
│ • Parking Find   │      │ • Local Costmap      │     │ • Trajectory │
│ • Localization   │      │ • A* Pathfinding     │     │ • Execution  │
│ • Tracking       │      │ • Obstacle Avoid     │     │ • Collision  │
└──────────────────┘      └──────────────────────┘     └──────────────┘
    ↓                           ↓                          ↓
    └───────────────────────────────────────────────────────┘
                ↓
        ┌──────────────────┐
        │  ROS 2 Coord     │
        │  & Orchestration │
        └──────────────────┘
                ↓
        HARDWARE CONTROL
        ├── Drive Motors
        ├── Arm Motors
        └── End-Effector
```

## Component Details

### 1. Sensor Fusion Module

**Purpose**: Combine multiple sensor streams for robust state estimation

**Components**:
- **LiDAR Processing**: Convert raw point clouds to occupancy grids
- **RGB-D Processing**: Extract depth maps and semantic information
- **IMU Integration**: Fuse orientation and acceleration data
- **Odometry**: Integrate wheel encoder feedback
- **EKF Filter**: Estimate (x, y, θ, vx, vy, ωz)

**Inputs**: 
- LiDAR: 10 Hz, 360° FOV
- RGB-D: 30 Hz, 640×480
- IMU: 100 Hz
- Odometry: 50 Hz

**Outputs**:
- Fused pose estimate (x, y, θ, uncertainty)
- Velocity estimate (vx, vy, ωz)
- Merged occupancy grid for planning

**Performance Target**: 
- Localization accuracy: ±5cm, ±5°
- Latency: <50ms

---

### 2. Perception Pipeline

**Purpose**: Detect vehicles and identify parking locations

#### 2.1 Car Detection
```
RGB Image (640×480, 30Hz)
    ↓
Preprocessing (resize, normalize)
    ↓
YOLO v8 / Faster R-CNN Inference
    ↓
Bounding Boxes + Confidence Scores
    ↓
Post-processing (NMS, filtering)
    ↓
Vehicle Detections (x, y, w, h, confidence)
```

**Model**: YOLO v8 Medium (640×640, ~45fps)
**Target Accuracy**: mAP >0.85

#### 2.2 Parking Space Localization
```
LiDAR Point Cloud + Depth Map
    ↓
Empty Space Detection (low occupancy regions)
    ↓
Geometry Analysis (size, accessibility)
    ↓
Candidate Generation (potential parking spots)
    ↓
Scoring & Ranking (feasibility metric)
    ↓
Best Parking Location (x, y, θ_target)
```

**Algorithm**: 
- Voxel-based occupancy analysis
- Geometric feasibility checking
- Accessibility scoring

**Target Accuracy**: ±10cm center point, ±5° angle

#### 2.3 Vehicle Tracking
```
Detections(t) + Detections(t-1)
    ↓
Hungarian Algorithm for Association
    ↓
Kalman Filter State Update (per vehicle)
    ↓
Tracked Vehicle States
```

**Output**: Vehicle ID, position, velocity over time

---

### 3. Navigation & Path Planning Module

**Purpose**: Generate collision-free paths from current to target location

#### 3.1 Global Path Planning
```
Start Pose (x_s, y_s, θ_s)
Target Pose (x_g, y_g, θ_g)
Static Map
    ↓
A* Pathfinding with Euclidean Heuristic
    ↓
Global Path Waypoints
    ↓
Path Smoothing (Bezier curves)
    ↓
Smooth Reference Trajectory
```

**Algorithm**: A* on grid with 8-connectivity
**Grid Resolution**: 0.05m (5cm)
**Inflation Radius**: 0.3m (robot safety margin)
**Target**: <100ms planning time

#### 3.2 Local Path Planning & Obstacle Avoidance
```
Current Pose + Velocity
Local Sensor Data (LiDAR)
Global Path Waypoints
    ↓
Dynamic Window Approach (DWA) / TEB Planner
    ↓
Candidate Velocity Commands
    ↓
Cost Evaluation:
  - Distance to goal
  - Collision distance
  - Heading angle
  - Velocity smoothness
    ↓
Optimal Velocity Command (v, ω)
```

**Planner**: TEB (Timed Elastic Band) with ROS 2
**Update Rate**: 10 Hz
**Replanning**: Triggered on new obstacles or path deviation

#### 3.3 Costmap Management
```
Static Map Initialization
    ↓
Layer 1: Static Obstacles
Layer 2: Inflation (safety margins)
Layer 3: LiDAR Occupancy (dynamic)
Layer 4: RGB-D Depth (dynamic)
    ↓
Combined Costmap
    ↓
Decay Unknown Regions (aging)
    ↓
Updated Costmap @ 5 Hz
```

---

### 4. Motion Planning & Arm Control Module

**Purpose**: Execute precise 6-DOF manipulation tasks

#### 4.1 Inverse Kinematics
```
Target End-Effector Pose (x, y, z, roll, pitch, yaw)
    ↓
MoveIt 2 IK Solver
├─ Analytical Solver (if available)
└─ Numerical Solver (KDL, IKFast)
    ↓
Joint Angles (θ1, θ2, θ3, θ4, θ5, θ6)
    ↓
Feasibility Check:
  - Joint limits: θ_min ≤ θ_i ≤ θ_max
  - Singularity avoidance
  - Self-collision check
    ↓
Valid Joint Configuration
```

**Target**: <50ms IK computation, >95% success rate

#### 4.2 Trajectory Planning
```
Start Configuration (θ_start)
Goal Configuration (θ_goal)
    ↓
Trajectory Generator:
  - Time-optimal trajectory
  - Smooth spline interpolation
  - Jerk limitation
    ↓
Waypoint Trajectory (θ(t), θ̇(t), θ̈(t))
    ↓
Collision Checking (continuous):
  - Self-collisions
  - Environment collisions
  - Swept volume analysis
    ↓
Collision-Free Trajectory
    ↓
Execution via Joint Controllers
```

**Interpolation**: Cubic splines (C2 continuous)
**Collision Check**: FCL (Flexible Collision Library)
**Update Rate**: 100 Hz (trajectory execution)

#### 4.3 Control Loop
```
Desired Joint Angle (θ_d)
Actual Joint Angle (θ_a) [from encoders]
    ↓
PID Controller:
  error = θ_d - θ_a
  u = K_p * error + K_i * ∫error + K_d * dError/dt
    ↓
Motor Command (voltage/current)
    ↓
Motor Driver → Actuator
```

**Control Rate**: 100 Hz per joint
**Tracking Accuracy**: ±1° per joint

---

### 5. ROS 2 Orchestration Layer

**Node Architecture**:

```
┌─────────────────────────────────────────────────┐
│              ROS 2 Master (Parameter Server)    │
└─────────────────────────────────────────────────┘

Sensor Nodes:
├── /lidar_driver (publishes: /scan, /cloud)
├── /rgbd_camera (publishes: /color, /depth)
├── /imu_driver (publishes: /imu/data)
└── /odometry (publishes: /odom)

Processing Nodes:
├── /sensor_fusion (subscribes: all sensors → publishes: /tf, /state)
├── /perception (subscribes: /color, /depth → publishes: /car_detections, /parking_spots)
├── /path_planner (subscribes: /tf, /map → publishes: /global_path)
├── /local_planner (subscribes: /tf, /scan, /global_path → publishes: /cmd_vel)
└── /manipulation_controller (subscribes: /tf → publishes: /arm_commands)

Visualization:
└── /rviz_visualizer (subscribes: all topics for display)

Hardware Interface:
├── /drive_motor_controller (subscribes: /cmd_vel → sends motor commands)
└── /arm_motor_controller (subscribes: /arm_commands → sends arm commands)
```

**Communication Pattern**: Publisher-Subscriber with TF (Transform) tree

**Key Topics**:
- `/tf`: Transform tree (pose relationships)
- `/scan`: LiDAR point clouds
- `/color`, `/depth`: RGB-D streams
- `/cmd_vel`: Navigation velocity commands
- `/arm_commands`: Arm joint commands
- `/state`: Fused state estimate

---

## Data Flow Diagram

```
PERCEPTION LOOP (30 Hz):
RGB Image → [Car Detector] → Vehicle Bboxes
                    ↓
            Parking Space Finder
                    ↓
            (x_park, y_park, θ_park)

NAVIGATION LOOP (10 Hz):
LiDAR → Occupancy Grid
  ↓
Sensor Fusion → (x, y, θ) estimate
  ↓
Global Planner → Path Waypoints
  ↓
Local Planner → (v, ω) command
  ↓
Drive Controller → Motor PWM

MANIPULATION LOOP (100 Hz):
Task Goal (e.g., lift car)
  ↓
IK Solver → θ_goal
  ↓
Trajectory Planner → θ(t)
  ↓
Joint Controller → Motor Command
  ↓
Arm Execution
```

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| ROS 2 Framework | Industry standard, modular, mature ecosystem |
| A* Pathfinding | Fast, optimal guaranteed, well-understood |
| TEB Planner | Real-time obstacle avoidance, smooth paths |
| MoveIt 2 | Proven motion planning, active community support |
| EKF Fusion | Linear approximation sufficient, low computational cost |
| Gazebo Simulation | Open-source, accurate physics, hardware-in-loop ready |

---

## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Path Planning Time | <100 ms | 95th percentile |
| Localization Error | ±5 cm, ±5° | In Gazebo simulation |
| Perception Accuracy | >85 mAP | Vehicle detection |
| Parking Spot Accuracy | ±10 cm center | Localization |
| Arm Positioning | ±2 cm end-effector | Final placement |
| Overall Cycle Time | <30 seconds | Full parking maneuver |
| Success Rate | >95% | Across 100 scenarios |

---

## Scalability & Extensibility

- **Multi-Robot**: Namespace separation for fleet coordination
- **Hardware Variants**: Abstracted hardware interface layer
- **Algorithm Swapping**: Plugin architecture for planning algorithms
- **Sensor Flexibility**: Generic sensor abstraction layer

---

**Last Updated**: November 2025  
**Version**: 1.0
