# ROS 2 Gazebo Simulation & Verification Guide

This guide describes how to build, launch, and evaluate the full indoor room exploration simulation environment for the NAVI robot.

---

## 🛠️ Build Workspace

From the `ros_simulation` directory on Ubuntu 24.04 (ROS 2 Jazzy or Iron):

```bash
cd ros_simulation
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Launching the Simulation

### 1. Complete Autonomous Exploration Demo
Launch Gazebo Harmonic, robot model, SLAM Toolbox, autonomous frontier exploration node, and RViz:

```bash
ros2 launch blind_assistant_robot indoor_simulation.launch.py
```

### 2. Launch with Dynamic Pedestrian Traffic
Spawn dynamic obstacles crossing the room corridors to test real-time obstacle avoidance and yielding:

```bash
ros2 launch blind_assistant_robot indoor_simulation.launch.py enable_pedestrians:=true
```

### 3. Native OpenCV HUD Visualizer (Low-Compute Alternative to RViz)
For resource-constrained bench test rigs or headless environments:

```bash
ros2 run blind_assistant_robot map_visualizer_node
```

---

## 🧪 Automated Unit & Integration Test Suite

The simulation includes 14 automated unit and integration tests covering transform integrity, sensor streams, frontier exploration logic, and speech alerts.

To run all automated verification tests:

```bash
pytest ros_simulation/src/blind_assistant_robot/test/
```

### Test Case Coverage
1. `test_urdf_syntax`: Validates XML/Xacro tree and joint limits.
2. `test_laser_scan_rate`: Confirms `/scan` publishes at $\ge 10\text{ Hz}$.
3. `test_odom_twist`: Verifies linear and angular velocity integration.
4. `test_tf_tree_continuity`: Checks `odom -> base_footprint -> laser_frame` transform continuity.
5. `test_costmap_inflation`: Validates obstacle padding in Nav2 local costmap.
6. `test_frontier_detection`: Tests cluster discovery from occupancy grid cells.
7. `test_voice_speech_alerts`: Verifies audio feedback string generation for navigation cues.
