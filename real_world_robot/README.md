# Real-World Robot Stack — Blind Assistant Robot

This directory contains the complete software, firmware, and deployment automation for the physical **Blind Assistant Robot** running on the **Arduino UNO Q** dual-brain platform and the Host Laptop over wireless ROS 2 (`ROS_DOMAIN_ID=42`).

---

## 📂 Directory Layout

```text
real_world_robot/
├── arduino_app_lab/
│   ├── blind_assistant_robot/       # Full Arduino App Lab project directory
│   │   ├── app.yaml                 # App manifest & metadata
│   │   ├── README.md                # In-app guide
│   │   ├── sketch/                  # STM32 MCU real-time firmware
│   │   │   ├── sketch.ino           # Motor PWM, encoder ISRs, MPU-6050 IMU, watchdog
│   │   │   └── sketch.yaml          # Uno Q MCU target profile
│   │   └── python/                  # Qualcomm Linux MPU autostart script
│   │       ├── main.py              # Boots ROS 2 base stack & Bluetooth audio
│   │       └── requirements.txt     # pip dependencies (pyserial, opencv-python-headless)
│   └── blind_assistant_robot_app.zip # Ready-to-import turnkey App Lab package
├── firmware/
│   └── arduino_uno_motor_encoder/
│       └── arduino_uno_motor_encoder.ino # Standalone Arduino IDE sketch
├── robot_ros2_pkg/
│   └── blind_assistant_robot/       # Complete ROS 2 package for host & base nodes
│       ├── launch/
│       │   ├── physical_robot_base.launch.py # Runs on Uno Q (LiDAR, webcam, bridge, TF)
│       │   └── competition_demo.launch.py   # Runs on Laptop (SLAM, fusion, explorer, HUD)
│       ├── blind_assistant_robot/   # Core Python ROS 2 nodes
│       ├── config/                  # SLAM Toolbox params & RViz layout
│       └── urdf/                    # Robot kinematic description
├── start_demo.sh                    # Turnkey single-command execution script
└── README.md                        # This documentation
```

---

## ⚡ Hardware Pinout Reference (Arduino UNO Q MCU)

| Component | Function | Uno Q MCU Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Left Motor ENA** | PWM Speed Control | **D5** | Timer PWM (0–255) |
| **Left Motor IN1 / IN2** | Direction Control | **D7 / D8** | Logic HIGH/LOW |
| **Right Motor ENB** | PWM Speed Control | **D6** | Timer PWM (0–255) |
| **Right Motor IN3 / IN4** | Direction Control | **D9 / D10** | Logic HIGH/LOW |
| **Left Encoder Phase A** | Hardware Interrupt | **D2** | External Interrupt 0 (`RISING`) |
| **Right Encoder Phase A** | Hardware Interrupt | **D3** | External Interrupt 1 (`RISING`) |
| **MPU-6050 IMU SDA** | I2C Data | **A4 (SDA)** | Gyro yaw stabilization |
| **MPU-6050 IMU SCL** | I2C Clock | **A5 (SCL)** | 400 kHz Fast I2C |
| **Power / GND** | 5V Supply & Common GND | **5V / GND** | Shared common ground |

---

## 🚀 Quick Start Guide

### Step 1: Deploy on Arduino UNO Q (Robot Base)
1. Open **Arduino App Lab** on your computer.
2. Click **Import App** (or **Open Existing App**) and select:
   `real_world_robot/arduino_app_lab/blind_assistant_robot_app.zip`
3. Connect your **Arduino UNO Q** board via USB-C or Wi-Fi Network Mode.
4. Click **Deploy / Run**:
   - Flashes `sketch.ino` to the STM32 MCU.
   - Starts `python/main.py` on the Linux MPU core, streaming `/scan`, `/odom`, and `/camera/image_raw` on `ROS_DOMAIN_ID=42`.
   - Click **Create Release** to enable automatic boot on battery power.

### Step 2: Run Competition Demo on Laptop
In a terminal on your laptop, simply run:
```bash
./start_demo.sh
```
*(Or specify a custom exploration time limit in seconds: `./start_demo.sh 90`)*.

---

## 🧠 Sensor Data Fusion & Autonomy Pipeline

1. **Monocular Webcam + 2D LiDAR Ray-Casting**:
   - Calculates optical bearing angle $\theta_{bearing} = -((u - W/2)/(W/2)) \cdot (HFOV/2)$.
   - Gathers LiDAR beams inside visual cone and applies **1D Density Clustering ($0.40\text{m}$ gap rejection)** to isolate the foreground object and discard the background room wall.
   - Projects coordinates into the global SLAM `map` frame via TF2.

2. **Triple-Criterion Completion Guard**:
   - **Criterion 1 (Time Limit)**: Auto-completes when elapsed time reaches `max_exploration_time` (default: 120s).
   - **Criterion 2 (Frontier Depletion)**: Completes when all reachable room frontiers are scanned (`len(frontiers) == 0`).
   - **Criterion 3 (Area Saturation)**: Completes when no new floor space is uncovered for 20 seconds.

3. **Return-to-User & Voice Escort**:
   - Upon completion, the robot navigates back to $(x_{start}, y_{start})$, parks at the user's feet, and announces:
     > *"Exploration complete. Returning to user. I am ready. Where would you like to go?"*
   - Speak your destination into the Bluetooth headset (e.g. *"Take me to the washroom"*).
   - Robot calculates the safe line-of-sight standoff ($0.65\text{m}$ clearance) and autonomously escorts the user to the target.
