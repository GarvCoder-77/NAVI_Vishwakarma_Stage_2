# NAVI — Navigation and Assistance for Visually Impaired

Stage 2 engineering repository for NAVI, an assistive robotic mobility platform for visually impaired users.

## Repository purpose

This repository preserves the supplied ROS 2/Gazebo development package without modifying its contents, while providing a clear home for Stage 2 mechanical, electronics, validation, and maker-journey evidence as it becomes available.

## Current implementation

The imported implementation supports autonomous indoor navigation and simulation with ROS 2, Gazebo, SLAM configuration, Arduino motor/encoder firmware, URDF, mechanical meshes, and automated tests. The original package is retained under `source/NAVI_ROS2/`; its internal package name remains unchanged to avoid breaking the supplied implementation.

## Stage 2 development focus

- LiDAR mapping and map-aware navigation
- Improved autonomous obstacle avoidance
- Voice feedback and interaction
- Text-reading capability
- Compact mechanical redesign
- Hardware integration and validation

## Evidence policy

- Keep supplied and future raw engineering files unchanged.
- Add native CAD, KiCad, and analysis files to the dedicated directories below.
- Use external links only for videos; do not commit embedded videos.
- Record each meaningful Stage 2 update in a separate Git commit.

## Repository map

| Location | Contents |
| --- | --- |
| `real_world_robot/` | Physical robot software: Arduino App Lab bundle (`.zip`), Arduino Uno Q MCU firmware, hardware ROS 2 nodes, and single-command `./start_demo.sh` launcher |
| `ros_simulation/` | Complete Gazebo Harmonic indoor room simulation workspace, autonomous explorer, dynamic traffic simulator, and 14/14 automated test cases |
| `hardware_components/` | Bill of materials (BOM), detailed technical specs, and engineering selection rationale for all 15 hardware components |
| `schematics/` | High-resolution circuit schematic diagram, full 66-point-to-point pin wiring netlist, and subsystem electrical rationale |
| `hardware/native-cad/` | Native mechanical CAD and analysis exports |
| `electronics/native-design/` | Schematics, PCB design files, BOMs, and electronics evidence |
| `analysis/` | MATLAB or other engineering calculations, kinematics, and power budget validation data |
| `evidence/maker-journey/` | Dated build photos and design-tool screenshots |
| `evidence/external-media-links.md` | External demonstration-video links only |
| `docs/` | Stage 2 technical documentation, commissioning runbooks, and simulation guides |

## Technical Documentation & Runbooks

* [**Bench Commissioning Runbook**](docs/hardware_quickstart_guide.md): Multimeter safety checks, I2C address dynamic remapping, motor direction checks.
* [**Kinematics & Power Budget**](analysis/README.md): Forward differential drive kinematics, odometry formulas, 2.75-hour battery runtime analysis.
* [**Arduino Pinout Matrix**](schematics/PINOUT_MATRIX.md): Microcontroller pin assignments, PWM timers, and power rail domain definitions.
* [**Gazebo Simulation Guide**](docs/ros2_simulation_guide.md): Environment bringup, dynamic pedestrian spawning, and 14/14 automated test execution.
* [**Demonstration Screencast Video**](evidence/external-media-links.md): Autonomous room exploration and obstacle avoidance video recording (Google Drive).

