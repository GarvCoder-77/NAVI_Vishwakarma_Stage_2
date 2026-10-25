#!/usr/bin/env python3
"""
Arduino Serial Bridge Node for Differential Drive Blind Assistant Robot.
- Subscribes to /cmd_vel and converts twist commands into motor PWM for Arduino.
- Reads wheel encoder feedback from Arduino over USB Serial.
- Computes dead-reckoning odometry and publishes /odom and TF (odom -> base_footprint).
"""

import math
import sys
import time

try:
    import serial
except ImportError:
    serial = None

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros


def quaternion_from_euler(ai, aj, ak):
    ai, aj, ak = ai / 2.0, aj / 2.0, ak / 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = Quaternion()
    q.x = cj * sc - sj * cs
    q.y = cj * ss + sj * cc
    q.z = cj * cs - sj * sc
    q.w = cj * cc + sj * ss
    return q


class ArduinoSerialBridge(Node):
    def __init__(self):
        super().__init__('arduino_serial_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.033)        # 33 mm radius (66 mm wheel)
        self.declare_parameter('wheel_separation', 0.170)    # 170 mm track width
        self.declare_parameter('ticks_per_rev', 360)        # Encoder ticks per complete wheel revolution
        self.declare_parameter('max_speed', 0.50)           # Max speed in m/s (safe walking speed)
        self.declare_parameter('publish_tf', True)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        self.serial_conn = None
        self.connect_serial()

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = self.get_clock().now()

        # ROS publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Serial read timer (50 Hz)
        self.timer = self.create_timer(0.02, self.read_serial_callback)
        self.get_logger().info(f'Arduino Serial Bridge initialized on {self.port} at {self.baudrate} baud.')

    def connect_serial(self):
        if serial is None:
            self.get_logger().error('pyserial is not installed! Cannot open serial port.')
            return

        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.05)
            time.sleep(1.5)  # Allow Arduino reset to complete
            self.get_logger().info(f'Successfully connected to Arduino on {self.port}')
        except Exception as e:
            self.get_logger().warning(f'Could not open serial port {self.port}: {e}. Retrying periodically...')
            self.serial_conn = None

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z

        # Differential drive inverse kinematics
        # v_left = vx - (wz * L / 2)
        # v_right = vx + (wz * L / 2)
        v_left = vx - (wz * self.wheel_separation / 2.0)
        v_right = vx + (wz * self.wheel_separation / 2.0)

        # Scale velocity to PWM [-255, 255]
        pwm_left = int((v_left / self.max_speed) * 255.0)
        pwm_right = int((v_right / self.max_speed) * 255.0)

        pwm_left = max(-255, min(255, pwm_left))
        pwm_right = max(-255, min(255, pwm_right))

        cmd_str = f'm {pwm_left} {pwm_right}\n'

        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(cmd_str.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f'Failed to send command to Arduino: {e}')

    def read_serial_callback(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        try:
            while self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('e '):
                    parts = line.split()
                    if len(parts) >= 3:
                        left_ticks = int(parts[1])
                        right_ticks = int(parts[2])
                        yaw_rate = float(parts[3]) / 1000.0 if len(parts) >= 4 else None
                        self.update_odometry(left_ticks, right_ticks, yaw_rate)
        except Exception as e:
            self.get_logger().warning(f'Error reading serial data: {e}')

    def update_odometry(self, left_ticks, right_ticks, yaw_rate=None):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_time = current_time
            return

        delta_l = left_ticks - self.last_left_ticks
        delta_r = right_ticks - self.last_right_ticks

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time

        # Distance traveled by each wheel
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = delta_l * meters_per_tick
        d_right = delta_r * meters_per_tick

        d_center = (d_left + d_right) / 2.0
        d_theta_wheel = (d_right - d_left) / self.wheel_separation

        # Complementary heading fusion if IMU gyro yaw rate is available
        if yaw_rate is not None and dt > 0:
            d_theta_imu = yaw_rate * dt
            # 90% IMU gyro (low slip drift), 10% wheel kinematics (bias compensation)
            d_theta = 0.90 * d_theta_imu + 0.10 * d_theta_wheel
            vth = yaw_rate
            
            # Publish Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.angular_velocity.z = yaw_rate
            imu_msg.angular_velocity_covariance[8] = 0.01
            self.imu_pub.publish(imu_msg)
        else:
            d_theta = d_theta_wheel
            vth = d_theta / dt if dt > 0 else 0.0

        # Update pose integration
        vx = d_center / dt if dt > 0 else 0.0
        if yaw_rate is None:
            vth = d_theta / dt if dt > 0 else 0.0

        if d_theta != 0.0:
            self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
            self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
            self.theta += d_theta
        else:
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Orientation quaternion
        q = quaternion_from_euler(0.0, 0.0, self.theta)

        # 1. Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        # 2. Broadcast Transform (odom -> base_footprint)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.write(b'm 0 0\n')
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
