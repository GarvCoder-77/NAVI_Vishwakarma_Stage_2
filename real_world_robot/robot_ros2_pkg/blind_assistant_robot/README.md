# Blind Assistant Robot — Indoor Autonomous Navigation & Semantic Escort

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?logo=ros)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic%20(gz--sim8)-FF6F00?logo=gazebo)](https://gazebosim.org/)
[![Tests](https://img.shields.io/badge/Tests-7%2F7%20Passed-brightgreen)](test/run_indoor_tests.sh)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

An autonomous assistive mobile robot system engineered to guide visually impaired individuals through unfamiliar indoor environments. Built with ROS 2, Gazebo Harmonic, SLAM Toolbox, OpenCV, and dual-sensor depth perception.

---

## 🌟 Key Capabilities

1. **Zero Prior Knowledge (True Blank Slate)**:
   - Starts at $(x=-2.0, y=-2.0)$ with zero prior map data.
   - Autonomously discovers walls, rooms, doors, and functional landmarks using frontier exploration.
2. **Dual-Sensor Depth Perception**:
   - **Intel RealSense D435i-style RGB-D Camera** (`640x480` @ 15 Hz) + **YDLidar X4 360° Planar LiDAR**.
   - Central 50% ROI depth extraction with median filtering and LiDAR ray barrier cross-check.
3. **Cognitive Path Planning & Motion Control**:
   - 8-connected global A* path planner with Bresenham line-of-sight string-pulling smoothing.
   - Regulated Pure Pursuit feedback controller with adaptive lookahead ($0.25\text{m}$ to $0.48\text{m}$) and acceleration slew limiting.
4. **Doorway Corridor Funneling**:
   - When traversing narrow partition thresholds ($1.2\text{m}$ width), dynamically contracts the front LiDAR collision cone from $\pm 30^\circ$ to $\pm 14^\circ$ and lowers the stop threshold from $0.22\text{m}$ to $0.16\text{m}$, eliminating door jamb oscillation.
5. **Dynamic Human Obstacle Yielding & Detour**:
   - Detects moving pedestrians using **only onboard sensors** (camera bounding boxes and depth/LiDAR ranges).
   - Pauses and announces an audible warning when a pedestrian enters the travel corridor ($<1.25\text{m}$).
   - If blocked for $>4.0\text{s}$, computes an autonomous A* detour around the obstacle.
6. **Return-to-User & Natural Language Escort**:
   - Automatically navigates back to the starting position when mapping is complete.
   - User commands destinations via terminal CLI (e.g. `"take me to the restroom"`, `"desk"`, `"cabinet"`).
   - Robot computes safe line-of-sight standoff points and escorts the user to the target.
7. **Zero-Web Native Desktop UI**:
   - Native OpenCV SLAM Map Visualizer ($700 \times 700$ 2D map + $280 \times 700$ HUD Telemetry Sidebar).
   - Native Camera Vision Window ($640 \times 480$ RGB stream with bounding boxes and 3D map coordinates).
   - Interactive Terminal CLI (`room_cli`) with natural language intent normalization.

---

## 📁 Repository Directory Structure

```text
blind_assistant_robot/
├── CMakeLists.txt / setup.py     # Python package build configuration
├── setup.cfg                      # Ament python package configuration
├── package.xml                    # ROS 2 dependencies and metadata
├── .gitignore                     # Git ignore rules for build/cache artifacts
├── README.md                      # Complete project documentation
│
├── blind_assistant_robot/         # Core Python ROS 2 Nodes
│   ├── __init__.py
│   ├── autonomous_explorer.py     # Frontier exploration, A*, Pure Pursuit & Doorway Funneling
│   ├── semantic_vision_node.py    # HSV segmentation, RealSense depth ROI & 3D projection
│   ├── indoor_traffic_simulator.py# Dynamic moving pedestrian controller (6-waypoint circuit)
│   ├── slam_map_visualizer.py     # Native OpenCV 700x700 SLAM map & 280x700 HUD sidebar
│   ├── room_cli.py                # Interactive terminal CLI with NLP synonym parser
│   ├── voice_guidance.py          # Assistive audio TTS announcements on /voice_speech
│   ├── arduino_serial_bridge.py   # Serial bridge for physical Arduino motor/encoder control
│   └── ydlidar_x4_driver.py       # Serial driver for physical YDLidar X4 sensor
│
├── launch/                        # ROS 2 Launch Files
│   ├── autonomous_explore.launch.py # MASTER LAUNCH: Gazebo Harmonic, SLAM, Vision & Explorer
│   ├── rsp.launch.py              # Robot State Publisher & TF tree broadcaster
│   ├── sim_robot.launch.py        # Spawns robot into Gazebo
│   ├── navigation.launch.py       # Nav2 navigation stack bringup
│   ├── mapping.launch.py          # SLAM Toolbox mapping bringup
│   ├── complete_sim.launch.py     # Complete multi-node simulation bringup
│   └── real_hardware.launch.py    # Physical robot hardware bringup (Arduino + LiDAR)
│
├── urdf/                          # Robot Mechanical & Sensor Description
│   └── blind_robot.urdf.xacro     # Differential drive chassis, RealSense RGB-D & LiDAR
│
├── worlds/                        # Gazebo Simulation Worlds
│   └── indoor_room.sdf            # 10m x 10m partitioned room, 7 landmarks & moving human
│
├── config/                        # Configuration & Parameter Files
│   ├── exploration.rviz           # RViz2 visualization layout
│   ├── slam_toolbox_params.yaml   # SLAM Toolbox synchronous mapping parameters
│   ├── nav2_params.yaml           # Nav2 controller & costmap parameters
│   └── ydlidar_x4_params.yaml     # Physical YDLidar serial port & baud rate settings
│
├── meshes/                        # 3D CAD Meshes (SolidWorks STL)
│   ├── chassis.stl                # Robot main chassis
│   ├── wheel.stl                  # Drive wheels
│   ├── caster.stl                 # Rear omni-caster housing & wheel
│   └── ydlidar_x4.stl             # LiDAR sensor housing
│
├── firmware/                      # Microcontroller Firmware
│   └── arduino_uno_motor_encoder/
│       └── arduino_uno_motor_encoder.ino # Arduino Uno PWM & encoder count firmware
│
├── test/                          # Automated Verification Suite
│   ├── run_indoor_tests.sh        # One-click test runner
│   └── test_indoor_simulation.py  # 7 automated pytest unit & integration tests
│
├── scripts/                       # Utility Scripts
│   └── generate_cad_meshes.py     # Procedural CAD mesh generator fallback
│
└── resource/                      # ROS 2 Resource Index
    └── blind_assistant_robot
```

---

## 🛠️ Prerequisites & Installation

### Requirements
- **Ubuntu**: 22.04 LTS (Jammy) or 24.04 LTS (Noble)
- **ROS 2**: Humble Hawksbill, Iron Irwini, or Jazzy Jalisco
- **Gazebo**: Gazebo Sim 8 (Harmonic)
- **Python**: 3.10+ with `numpy`, `opencv-python`, `pytest`

### Step 1: Create a Colcon Workspace & Clone
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/blind_assistant_robot.git
```

### Step 2: Install ROS 2 Dependencies
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 3: Build the Package
```bash
colcon build --packages-select blind_assistant_robot
source install/setup.bash
```

---

## 🚀 Running the System

### 1. Launch the Complete Autonomous Indoor Simulation
```bash
ros2 launch blind_assistant_robot autonomous_explore.launch.py
```
*What starts automatically:*
- **Gazebo Harmonic**: $10\text{m} \times 10\text{m}$ room with furniture, partition doorway, and dynamic pedestrian.
- **SLAM Toolbox**: Real-time synchronous 2D occupancy mapping on `/map`.
- **Vision Pipeline**: RealSense depth extraction and HSV contour classification.
- **Cognitive Explorer**: Autonomous frontier exploration and path planning.
- **Native OpenCV SLAM Map Pop-Up**: $700 \times 700$ 2D map with pins and $280 \times 700$ telemetry sidebar.
- **Native OpenCV Camera Pop-Up**: $640 \times 480$ live RGB stream with 3D landmark coordinates.
- **RViz2**: 3D spatial visualization.

### 2. Launch the Interactive Voice/Terminal Navigator
In a second terminal window:
```bash
ros2 run blind_assistant_robot room_cli
```
*Example Commands:*
- `take me to the restroom`
- `guide me to the desk`
- `go to the chair`
- `navigate to cabinet`
- `pillar`
- `take me to exit`
- `return to entrance`

---

## 🧪 Automated Testing

Run the full automated verification suite:
```bash
bash src/blind_assistant_robot/test/run_indoor_tests.sh
```

All 7 test suites pass 100%:
1. `test_indoor_world_sdf_and_physics` (400 Hz ODE physics, step size, VelocityControl plugin)
2. `test_indoor_traffic_simulator_logic` (6-waypoint circuit, proximity yielding)
3. `test_semantic_vision_landmark_classes` (7 landmark classes, non-overlapping HSV, RealSense depth ROI)
4. `test_autonomous_explorer_yielding` (Blank-slate start, human yield reflex, WAITING_FOR_USER state)
5. `test_room_cli_intent_parser` (NLP synonym normalization)
6. `test_launch_and_rviz_config` (Launch entity configuration)
7. `test_slam_map_visualizer_rendering` (World-to-pixel transforms & HUD rendering)

---

## 📡 Core ROS 2 Topics

| Topic Name | Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Differential drive velocity commands to robot |
| `/scan` | `sensor_msgs/msg/LaserScan` | 360° Planar LiDAR range scan from YDLidar X4 |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry from DiffDrive plugin |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | $640 \times 480$ RGB color video feed |
| `/camera/depth/image_raw`| `sensor_msgs/msg/Image` | $640 \times 480$ 32-bit float depth map in meters |
| `/map` | `nav_msgs/msg/OccupancyGrid`| 2D SLAM metric occupancy grid ($0.05\text{m/cell}$) |
| `/planned_path` | `nav_msgs/msg/Path` | Active global A* trajectory waypoints |
| `/robot_trajectory` | `nav_msgs/msg/Path` | Historical breadcrumb trajectory trail |
| `/room_landmarks` | `std_msgs/msg/String` | Latched JSON dictionary of confirmed room landmarks |
| `/voice_speech` | `std_msgs/msg/String` | Spoken natural language assistive audio |
| `/guide_to` | `std_msgs/msg/String` | Target destination command from user |

---

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
