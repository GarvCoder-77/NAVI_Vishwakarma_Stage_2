# Blind Assistant Robot — Arduino App Lab Project

This directory contains the ready-to-deploy **Arduino App Lab** project for the **Arduino UNO Q** dual-brain board.

## Structure

```text
blind_assistant_robot/
├── app.yaml              # App Manifest & metadata
├── README.md             # This guide
├── sketch/
│   ├── sketch.ino        # Dual-motor PWM H-Bridge + quadrature encoder interrupts + I2C IMU (STM32 MCU)
│   └── sketch.yaml       # Board target configuration
└── python/
    ├── main.py           # Linux MPU bringup script (launches base ROS 2 hardware bridge & publishers)
    └── requirements.txt  # Python dependencies (pyserial, opencv-python-headless)
```

## How to Deploy in Arduino App Lab

1. Launch **Arduino App Lab** on your computer.
2. Select **Open Existing App** and browse to this folder:
   `/home/jatin/.gemini/antigravity/scratch/blind_assistant_robot/arduino_app_lab/blind_assistant_robot`
3. Connect your **Arduino UNO Q** board via USB-C or Wi-Fi Network Mode.
4. Click **Run / Deploy**:
   - Arduino App Lab automatically compiles and flashes `sketch/sketch.ino` to the STM32 MCU.
   - It starts `python/main.py` on the Linux MPU core.
5. In App Lab, click **Create Release** to enable **Autostart on Boot**. When powered on battery, the robot will start streaming sensor data and listening for commands automatically.
