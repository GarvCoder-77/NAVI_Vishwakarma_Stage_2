#!/usr/bin/env python3
"""
Arduino App Lab Entry Point for Arduino UNO Q (Linux MPU).
Automatically launched by Arduino App Lab when the App is deployed or booted.
Brings up:
1. ROS 2 Environment configuration (ROS_DOMAIN_ID=42)
2. Robot Base Hardware Stack (URDF, Arduino serial bridge, YDLidar X4, Monocular USB Webcam)
3. Bluetooth Headset Audio pipeline (/voice_speech text-to-speech)
"""

import os
import sys
import time
import subprocess
import signal
import threading


# Set ROS 2 Domain ID for network isolation & multi-machine discovery
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'


def run_process(cmd, name):
    print(f"[{name}] Starting: {cmd}")
    try:
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in iter(p.stdout.readline, ''):
            if line:
                print(f"[{name}] {line.strip()}")
        p.stdout.close()
        p.wait()
    except Exception as e:
        print(f"[{name}] Process error: {e}")


def launch_bluetooth_audio_pipeline(ros_setup):
    """Brings up voice guidance subscriber to speak into the Bluetooth Headset."""
    print("[Audio] Initializing Bluetooth Headset Voice Audio pipeline on /voice_speech...")
    cmd = f"bash -c 'source {ros_setup} && export ROS_DOMAIN_ID=42 && ros2 run blind_assistant_robot voice_guidance'"
    threading.Thread(target=run_process, args=(cmd, "BT_AUDIO"), daemon=True).start()


def main():
    print("=======================================================")
    print("   BLIND ASSISTANT ROBOT — ARDUINO APP LAB RUNNER     ")
    print("   Hardware: Arduino UNO Q (Linux MPU + STM32 MCU)    ")
    print("   Sensors: USB Monocular Webcam + YDLidar X4 (2D)     ")
    print("   Perception: Webcam + 2D LiDAR Ray-Casting Fusion   ")
    print("   Domain ID: 42 | Network: Wi-Fi Discovery Active     ")
    print("=======================================================")

    arduino_port = os.environ.get('ARDUINO_PORT', '/dev/ttyACM0')
    lidar_port = os.environ.get('LIDAR_PORT', '/dev/ttyUSB0')
    video_device = os.environ.get('VIDEO_DEVICE', '0')

    # Source ROS 2 setup script
    ros_setup = None
    for candidate in [
        "/opt/ros/jazzy/setup.bash",
        "/opt/ros/humble/setup.bash",
        "/opt/ros/iron/setup.bash",
        "/opt/ros/rolling/setup.bash",
    ]:
        if os.path.exists(candidate):
            ros_setup = candidate
            break

    if not ros_setup:
        print("[App Lab] Error: No ROS 2 installation found in /opt/ros. Ensure ROS 2 is installed on Uno Q MPU.")
        sys.exit(1)

    print(f"[App Lab] Sourcing ROS 2 environment: {ros_setup}")

    # 1. Launch Bluetooth Headset Audio pipeline
    launch_bluetooth_audio_pipeline(ros_setup)

    # 2. Launch Robot Base Hardware Stack (URDF, Arduino serial bridge, YDLidar X4, Monocular USB Webcam)
    launch_cmd = (
        f"bash -c 'source {ros_setup} && "
        f"export ROS_DOMAIN_ID=42 && "
        f"ros2 launch blind_assistant_robot physical_robot_base.launch.py "
        f"arduino_port:={arduino_port} lidar_port:={lidar_port} camera_device:={video_device}'"
    )

    t = threading.Thread(target=run_process, args=(launch_cmd, "ROBOT_BASE"), daemon=True)
    t.start()

    print("[App Lab] Robot base stack launched successfully.")
    print("[App Lab] Streaming /camera/image_raw, /scan, /odom on ROS_DOMAIN_ID=42.")
    print("[App Lab] Ready for Wi-Fi ROS 2 pairing with Host Laptop.")

    # Main keep-alive loop
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[App Lab] Shutting down...")
        sys.exit(0)


if __name__ == '__main__':
    main()

