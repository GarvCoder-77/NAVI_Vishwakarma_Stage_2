# Indoor ROS 2 Simulation Stack — Blind Assistant Robot

This directory contains the complete **Gazebo Harmonic** indoor room simulation, autonomous frontier exploration engine, dynamic pedestrian traffic simulator, and automated test suite for the **Blind Assistant Robot**.

---

## 📂 Directory Layout

```text
ros_simulation/
└── src/
    └── blind_assistant_robot/
        ├── worlds/
        │   └── indoor_room.sdf             # 2-room indoor environment with furniture & lighting
        ├── launch/
        │   ├── autonomous_explore.launch.py # Master simulation launch file
        │   ├── rsp.launch.py               # Robot state publisher & TF broadcaster
        │   └── navigation.launch.py        # Nav2 navigation stack bringup
        ├── urdf/
        │   └── blind_robot.urdf.xacro      # Kinematics, sensors, and Gazebo plugins
        ├── blind_assistant_robot/          # ROS 2 Python nodes
        │   ├── autonomous_explorer.py      # Frontier extraction, A* planning, Pure Pursuit
        │   ├── indoor_traffic_simulator.py # Dynamic pedestrian pacing across doorway corridor
        │   ├── semantic_vision_node.py     # 3D spatial landmark recognition
        │   ├── slam_map_visualizer.py      # Native OpenCV live SLAM map pop-up & HUD sidebar
        │   ├── room_cli.py                 # Natural language terminal intent commander
        │   └── voice_guidance.py           # Text-to-speech audio notification engine
        ├── config/
        │   ├── slam_toolbox_params.yaml    # SLAM Toolbox online async mapping configuration
        │   └── exploration.rviz            # RViz 3D visualization layout
        ├── test/
        │   ├── run_indoor_tests.sh         # Master automated verification script
        │   ├── test_indoor_simulation.py   # Simulation ODE physics & explorer test suite
        │   └── test_webcam_lidar_fusion.py # Mathematical fusion & ray-casting unit tests
        ├── package.xml
        └── setup.py
```

---

## 🛠️ Build Instructions

From the root of `ros_simulation`:

```bash
# Source ROS 2 (Jazzy, Humble, or Rolling)
source /opt/ros/jazzy/setup.bash   # Or /opt/ros/humble/setup.bash

# Build the simulation package
colcon build --packages-select blind_assistant_robot

# Source workspace install overlay
source install/setup.bash
```

---

## 🚀 Running the Indoor Simulation

Launch the complete autonomous simulation with a single command:

```bash
ros2 launch blind_assistant_robot autonomous_explore.launch.py
```

### What Happens Automatically:
1. **Gazebo Harmonic** spins up with the `indoor_room.sdf` environment.
2. The robot spawns at $(x=-2.0, y=-2.0)$.
3. **SLAM Toolbox** initializes and begins mapping from 360° LiDAR and odometry.
4. `indoor_traffic_simulator` commands the simulated human pedestrian to pace a 6-waypoint patrol circuit crossing the doorway.
5. `autonomous_explorer` detects frontiers and drives through the doorway between Room 1 and Room 2.
6. The native **OpenCV Live SLAM Map Window** renders the real-time occupancy grid, historical breadcrumb trail, and landmark pins.
7. Upon reaching completion, the robot returns to $(x_{start}, y_{start})$ and enters `WAITING_FOR_USER` mode.

---

## 🧪 Automated Verification Suite

Run all unit and integration tests with 100% automated coverage:

```bash
bash src/blind_assistant_robot/test/run_indoor_tests.sh
```

### Verified Test Cases (14/14 Passing):
- `test_indoor_world_sdf_and_physics`: ODE solver step size, real-time update rate, humanoid velocity control plugin.
- `test_indoor_traffic_simulator_logic`: Humanoid 6-waypoint pacing circuit, hesitation timers, proximity yielding.
- `test_semantic_vision_landmark_classes_and_standoffs`: Non-overlapping HSV hue bands, standoff distance offsets.
- `test_autonomous_explorer_yielding_and_states`: Frontier detection, collision avoidance, return-to-user state transitions.
- `test_room_cli_intent_parser`: Natural language synonym matching (restroom, desk, table, door).
- `test_slam_map_visualizer_rendering`: OpenCV coordinate transformation and HUD rendering.
- `test_exploration_completion_criteria`: Fixed time cap, frontier depletion, area growth saturation.
- `test_webcam_lidar_fusion`: Bearing angle math, 1D density clustering wall rejection, TF2 3D spatial projection.
