# Repository File & Folder Manifest — Blind Assistant Robot

This document provides an exhaustive, labeled directory index of all files and folders in this repository, explaining the role, inputs, outputs, and dependencies of each component.

---

## 🗂️ Complete Directory Map

| Directory / File | Label / Category | Role & Technical Description |
| :--- | :--- | :--- |
| **`package.xml`** | `Manifest / Dependencies` | Official ROS 2 package manifest declaring build and runtime dependencies (`rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `slam_toolbox`, `cv_bridge`). |
| **`setup.py`** | `Build Configuration` | Python setuptools build script registering package data (URDF, worlds, meshes, launch, config) and defining CLI console script entry points. |
| **`setup.cfg`** | `Build Configuration` | Configures script installation directory (`lib/blind_assistant_robot`). |
| **`README.md`** | `Documentation` | Primary GitHub documentation with badges, system overview, architecture diagrams, prerequisites, build instructions, and topic dictionary. |
| **`LICENSE`** | `Legal` | Open-source Apache License 2.0. |
| **`.gitignore`** | `Git Configuration` | Excludes build artifacts (`build/`, `install/`, `log/`), Python bytecode (`__pycache__/`, `*.pyc`), and test caches (`.pytest_cache/`). |
| **`FILE_MANIFEST.md`** | `Directory Index` | This file: complete labeled inventory of every file and folder in the repository. |

---

## 🧠 Core ROS 2 Python Nodes (`blind_assistant_robot/`)

Directory path: `blind_assistant_robot/`

| File | Label | Role & Key Features |
| :--- | :--- | :--- |
| **`__init__.py`** | `Python Module Marker` | Marks the directory as a Python package. |
| **`autonomous_explorer.py`** | `Cognitive Brain & Motion` | **The central autonomous planning engine**: <br>• OccupancyGrid frontier extraction & clustering.<br>• 8-connected global A* path planner with Bresenham line-of-sight string-pulling.<br>• Regulated Pure Pursuit feedback controller with adaptive lookahead ($0.25\text{m}$–$0.48\text{m}$) and acceleration slew limits.<br>• **Doorway Corridor Funneling**: contracts front collision cone from $\pm 30^\circ$ to $\pm 14^\circ$ and lowers stop threshold to $0.16\text{m}$ inside doorway corridor $[-0.85, 0.85] \times [-2.50, -0.60]$ to eliminate door jamb oscillation.<br>• Dynamic moving pedestrian yielding (<1.25m) and 4.0-second detour replanning.<br>• Return-to-user at $(x_{start}, y_{start})$ and `WAITING_FOR_USER` escort state machine. |
| **`semantic_vision_node.py`** | `Perception & Depth Fusion` | **Real-time computer vision and 3D spatial labeling**: <br>• OpenCV HSV segmentation for 7 distinct non-overlapping landmark classes + pedestrian red jacket.<br>• Central 50% ROI depth extraction from RealSense `/camera/depth/image_raw`.<br>• Planar LiDAR ray cross-check within $\pm 0.08\text{ rad}$ bearing window to reject reflections.<br>• TF2 3D spatial projection into global `map` frame.<br>• Multi-frame EMA coordinate smoothing, 3-frame confirmation threshold, and latched `/room_landmarks` JSON broadcasting.<br>• Safe line-of-sight standoff vector calculation ($\vec{v} = \mathbf{p}_{rob} - \mathbf{p}_{landmark}$).<br>• Native $640 \times 480$ OpenCV live camera pop-up window. |
| **`indoor_traffic_simulator.py`** | `Simulation Dynamics` | **Dynamic pedestrian simulation agent**: <br>• Controls the `indoor_human` model via Gazebo's `VelocityControl` plugin on `/indoor_human/cmd_vel`.<br>• Paces an authentic 6-waypoint circuit crossing between Room 1, the doorway corridor, and Room 2.<br>• Emulates human hesitation with random waypoint pauses ($1.5\text{s}$ to $3.5\text{s}$) and yields when robot is within $0.55\text{m}$. |
| **`slam_map_visualizer.py`** | `Desktop Visualizer` | **Native zero-web desktop map interface**: <br>• Left Panel ($700 \times 700$): real-time 2D SLAM occupancy grid with minimalist numbered pin badges ($[1], [2], \dots$), heading vector pointer $[R]$, human indicator $[P]$, historical breadcrumb trail, and active planned trajectory.<br>• Right Panel ($280 \times 700$): high-contrast HUD Telemetry Sidebar with live velocity gauges, navigation state, pedestrian distance card, and discovered landmark cards. Runs at $12\text{ Hz}$ in OpenCV. |
| **`room_cli.py`** | `User Interface / CLI` | **Natural language terminal navigator**: <br>• Regex-based synonym parser mapping phrases (e.g. *"take me to the washroom"* $\rightarrow$ `restroom`, *"show me the table"* $\rightarrow$ `desk`, *"exit"*, *"start"*).<br>• Dispatches commands to `/guide_to` and transcribes live robot speech from `/voice_speech`. |
| **`voice_guidance.py`** | `Audio Feedback` | Subscribes to `/voice_speech` and outputs spoken notifications to system audio via `spd-say` or `espeak-ng`. |
| **`arduino_serial_bridge.py`** | `Hardware Interface` | Bridges ROS 2 `/cmd_vel` velocity commands and wheel encoders to an Arduino Uno over serial (`/dev/ttyUSB0` or `/dev/ttyACM0`). |
| **`ydlidar_x4_driver.py`** | `Hardware Driver` | Standalone Python serial driver for the physical YDLidar X4 sensor, publishing raw packets to `/scan`. |

---

## 🚀 ROS 2 Launch Files (`launch/`)

Directory path: `launch/`

| File | Label | Role & Execution |
| :--- | :--- | :--- |
| **`autonomous_explore.launch.py`** | `Master Launch` | **The primary launch file for full indoor simulation**: <br>1. Launches Gazebo Harmonic with `indoor_room.sdf`.<br>2. Publishes URDF robot model and TF tree via `rsp.launch.py`.<br>3. Spawns `blind_assistant_robot` at $(x=-2.0, y=-2.0)$.<br>4. Bridges Gazebo topics to ROS 2 via `ros_gz_bridge`.<br>5. Launches SLAM Toolbox with asynchronous warmup.<br>6. Starts `autonomous_explorer`, `semantic_vision_node`, `indoor_traffic_simulator`, `slam_map_visualizer`, `voice_guidance`, and RViz2. |
| **`rsp.launch.py`** | `Robot State Publisher` | Converts `blind_robot.urdf.xacro` into XML and runs `robot_state_publisher` to broadcast the kinematics transform tree. |
| **`sim_robot.launch.py`** | `Simulation Bringup` | Spawns the robot model into an existing Gazebo instance. |
| **`navigation.launch.py`** | `Nav2 Bringup` | Launches the Nav2 navigation stack configured with `nav2_params.yaml`. |
| **`mapping.launch.py`** | `SLAM Bringup` | Standalone launch for SLAM Toolbox synchronous mapping. |
| **`complete_sim.launch.py`** | `Full Stack Bringup` | Combined launch bringing up simulation, mapping, and manual teleoperation. |
| **`real_hardware.launch.py`** | `Physical Bringup` | Master launch for physical robot testing: starts LiDAR driver, Arduino bridge, robot state publisher, and camera. |

---

## 🤖 Robot Mechanical & Sensor Description (`urdf/`)

Directory path: `urdf/`

| File | Label | Role & Technical Parameters |
| :--- | :--- | :--- |
| **`blind_robot.urdf.xacro`** | `Robot Model` | Complete kinematic, dynamic, and sensor description: <br>• Chassis: $2.8\text{ kg}$, $0.26\text{m} \times 0.19\text{m} \times 0.076\text{m}$, inscribed radius $0.22\text{m}$.<br>• Drive Wheels: $r_w = 0.033\text{m}$, track $L = 0.176\text{m}$, rubber friction $\mu = 1.2$.<br>• Rear Caster: $r = 0.0175\text{m}$, frictionless $\mu = 0.001$.<br>• Front Skid: $r = 0.0135\text{m}$, frictionless $\mu = 0.001$.<br>• **Intel RealSense D435i RGB-D Camera**: $640 \times 480$, $1.15\text{ rad}$ FOV, $0.10\text{m}$–$10.0\text{m}$ clip range.<br>• **YDLidar X4**: 360 rays @ $10\text{ Hz}$, $0.12\text{m}$–$10.0\text{m}$ range.<br>• ODE DiffDrive plugin with $40\text{ Nm}$ torque limit. |

---

## 🌐 Gazebo Simulation World (`worlds/`)

Directory path: `worlds/`

| File | Label | Role & World Specifications |
| :--- | :--- | :--- |
| **`indoor_room.sdf`** | `Gazebo World` | **The indoor evaluation testbed**: <br>• $400\text{ Hz}$ ODE physics engine ($\Delta t = 0.0025\text{s}$, real-time factor $1.0$).<br>• $10\text{m} \times 10\text{m}$ footprint with $8\text{m} \times 8\text{m}$ interior walls.<br>• $5.0\text{m}$ interior partition wall with a $1.20\text{m}$ doorway opening.<br>• Elevated doorway sign header at $x=0.0, y=-0.90, z=0.95\text{m}$.<br>• 7 semantic landmarks: Desk (Brown), Chair (Yellow), Exit Sign (Green), Doorway Sign (Cyan), Cabinet (Blue), Pillar (Purple), Restroom Sign (Magenta).<br>• Dynamic humanoid pedestrian model (`indoor_human`) with `VelocityControl` plugin. |

---

## ⚙️ Parameters & Visualizer Layouts (`config/`)

Directory path: `config/`

| File | Label | Purpose |
| :--- | :--- | :--- |
| **`exploration.rviz`** | `RViz Configuration` | Pre-configured RViz2 layout showing 3D robot model, TF transform tree, SLAM occupancy grid `/map`, LiDAR `/scan`, depth point cloud `/camera/points`, planned path, breadcrumb trail, and 3D floating landmark markers. |
| **`slam_toolbox_params.yaml`**| `SLAM Parameters` | Synchronous online SLAM parameters for loop closure, scan matching thresholds, search windows, and map resolution ($0.05\text{m/cell}$). |
| **`nav2_params.yaml`** | `Nav2 Configuration` | Controller server, planner server, recovery behaviors, costmap inflation radius ($0.22\text{m}$), and velocity limits. |
| **`ydlidar_x4_params.yaml`** | `LiDAR Parameters` | Serial device configuration (`/dev/ttyUSB0`, baud rate $128000$, inverted scan flags) for the physical YDLidar X4. |

---

## 📐 3D CAD Meshes (`meshes/`)

Directory path: `meshes/`

| File | Label | Description |
| :--- | :--- | :--- |
| **`chassis.stl`** | `CAD Mesh` | High-fidelity SolidWorks STL mesh of the main chassis assembly, side panels, vents, and electronics slab. |
| **`wheel.stl`** | `CAD Mesh` | SolidWorks STL mesh of the differential drive wheel and rubber tire. |
| **`caster.stl`** | `CAD Mesh` | SolidWorks STL mesh of the rear omni-directional caster wheel and swivel bracket. |
| **`ydlidar_x4.stl`** | `CAD Mesh` | SolidWorks STL mesh of the rotating LiDAR turret and mount. |

---

## 🔌 Microcontroller Firmware (`firmware/`)

Directory path: `firmware/`

| File | Label | Description |
| :--- | :--- | :--- |
| **`arduino_uno_motor_encoder.ino`** | `Arduino Firmware` | C++ firmware for Arduino Uno: reads quadrature optical encoder interrupts on pins 2 and 3, receives motor PWM velocity commands over serial, and reports wheel tick counts back to ROS 2. |

---

## 🧪 Automated Verification Suite (`test/`)

Directory path: `test/`

| File | Label | Description |
| :--- | :--- | :--- |
| **`run_indoor_tests.sh`** | `Test Runner` | Bash script that automatically builds the package with colcon and executes pytest with full verbosity. |
| **`test_indoor_simulation.py`** | `Pytest Test Suite` | 7 automated unit & integration test cases verifying ODE physics rates, moving human patrol circuit, non-overlapping HSV landmark bands, frontier yielding reflex, NLP synonym parsing, launch configuration, and SLAM visualizer rendering. Passes 100% (7/7). |

---

## 📚 Detailed Documentation (`docs/`)

Directory path: `docs/`

| File | Label | Description |
| :--- | :--- | :--- |
| **`ENGINEERING_DOSSIER.md`** | `Master Technical Dossier` | Complete, exhaustive engineering documentation covering mathematical derivations (A* heuristics, pure pursuit curvature, ROI depth sampling, standoff vector geometry, doorway corridor funneling), full ROS 2 topic schemas, and a ready-to-paste Gemini system prompt. |
