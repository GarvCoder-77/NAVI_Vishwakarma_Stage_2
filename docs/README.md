# Stage 2 Documentation — NAVI Assistive Robotics Platform

Welcome to the Stage 2 technical documentation for **NAVI (Navigation and Assistance for Visually Impaired)**.

## Documentation Index

| Document | Description | Focus Area |
| :--- | :--- | :--- |
| [`NAVI_Vishwakarma_Stage_2_Presentation.pdf`](./NAVI_Vishwakarma_Stage_2_Presentation.pdf) | Official Stage 2 Engineering Presentation Deck | Architecture, System Overview, CAD, & Testing |
| [`hardware_quickstart_guide.md`](./hardware_quickstart_guide.md) | Bench Commissioning & Bringup Runbook | Multimeter checks, I2C address scanning, Motor polarity |
| [`ros2_simulation_guide.md`](./ros2_simulation_guide.md) | Gazebo Harmonic Simulation & Automated Testing | Room exploration, SLAM Toolbox, unit/integration suites |
| [`../hardware_components/README.md`](../hardware_components/README.md) | Component Specifications & Selection Rationale | Microcontroller, sensors, drivers, battery calculations |
| [`../schematics/README.md`](../schematics/README.md) | 66-Point Point-to-Point Circuit Netlist & Diagrams | Power bus topology, interrupt allocation, level isolation |
| [`../schematics/PINOUT_MATRIX.md`](../schematics/PINOUT_MATRIX.md) | Pinout Cheat Sheet & Electrical Matrix | MCU pin multiplexing, PWM timers, logic domains |
| [`../evidence/external-media-links.md`](../evidence/external-media-links.md) | Demonstration Screencast Video Link (Google Drive) | Full 2-room autonomous exploration and dynamic yielding |

## Stage 2 Engineering Objectives

1. **Autonomous Room Exploration & Mapping**:
   * Continuous 2D LiDAR ray-casting SLAM with online loop closure.
   * Autonomous frontier exploration with triple-criterion termination (time ceiling, frontier clearance, map saturation).
2. **Dual-Brain Architecture**:
   * Qualcomm Linux MPU for high-level SLAM, point-cloud fusion, and ROS 2 navigation.
   * STM32 MCU for real-time 100 Hz PID motor control, hardware encoder interrupts, and 500 ms safety watchdog.
3. **Multi-Modal Haptic & Audio Feedback**:
   * Dual-channel ERM vibration motors for intuitive direction and urgency alerting.
   * Bluetooth headset voice synthesis for landmark announcements and user commands.
4. **Bench-Validated Safety**:
   * Hardware kill-switch on primary 12V rail.
   * Optocoupler isolation on BTS7960 MOSFET bridges to eliminate motor inductive spikes on logic rails.
