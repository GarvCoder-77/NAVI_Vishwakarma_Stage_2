#!/usr/bin/env python3
"""
Lightweight native ROS 2 Driver for YDLidar X4.
- Connects to YDLidar X4 via USB serial at 128000 baud.
- Sends start-scan command (0xA5 0x60).
- Parses 0xAA 0x55 sample packets.
- Publishes standard sensor_msgs/msg/LaserScan on /scan.
"""

import math
import struct
import sys
import time

try:
    import serial
except ImportError:
    serial = None

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class YDLidarX4Driver(Node):
    def __init__(self):
        super().__init__('ydlidar_x4_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 128000)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 10.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.serial_conn = None
        self.scan_data = {}  # angle_deg -> distance_m

        self.connect_and_start()
        self.timer = self.create_timer(0.005, self.read_packet_callback) # Read frequently
        self.pub_timer = self.create_timer(0.14, self.publish_scan)      # Publish scan at ~7 Hz

    def connect_and_start(self):
        if serial is None:
            self.get_logger().error('pyserial is not installed!')
            return

        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.5)
            time.sleep(0.5)
            # Stop scan first
            self.serial_conn.write(b'\xA5\x65')
            time.sleep(0.1)
            self.serial_conn.flushInput()
            # Start scan command
            self.serial_conn.write(b'\xA5\x60')
            time.sleep(0.1)
            # Read response header (7 bytes)
            header = self.serial_conn.read(7)
            if header and len(header) == 7 and header[0:2] == b'\xA5\x5A':
                self.get_logger().info(f'YDLidar X4 connected and scanning on {self.port}!')
            else:
                self.get_logger().warning(f'YDLidar connected to {self.port}, streaming...')
        except Exception as e:
            self.get_logger().warning(f'Could not connect to YDLidar on {self.port}: {e}. Retrying...')

    def read_packet_callback(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        try:
            while self.serial_conn.in_waiting > 10:
                b1 = self.serial_conn.read(1)
                if b1 != b'\xAA':
                    continue
                b2 = self.serial_conn.read(1)
                if b2 != b'\x55':
                    continue

                # Header matched (0xAA 0x55)
                # Next: CT (1 byte), LSN (1 byte), FSA (2 bytes), LSA (2 bytes), CS (2 bytes)
                info = self.serial_conn.read(8)
                if len(info) < 8:
                    continue

                ct, lsn, fsa, lsa, cs = struct.unpack('<BBHHH', info)
                if lsn == 0:
                    continue

                # Read sample points (2 * LSN bytes)
                raw_samples = self.serial_conn.read(lsn * 2)
                if len(raw_samples) < lsn * 2:
                    continue

                fsa_deg = (fsa >> 1) / 64.0
                lsa_deg = (lsa >> 1) / 64.0

                if lsa_deg < fsa_deg:
                    diff_deg = (lsa_deg + 360.0) - fsa_deg
                else:
                    diff_deg = lsa_deg - fsa_deg

                step = diff_deg / (lsn - 1) if lsn > 1 else 0

                for i in range(lsn):
                    sample_dist = struct.unpack_from('<H', raw_samples, i * 2)[0]
                    dist_m = (sample_dist / 4.0) / 1000.0  # mm to meters

                    cur_angle = (fsa_deg + i * step) % 360.0
                    angle_idx = int(round(cur_angle)) % 360

                    if self.range_min <= dist_m <= self.range_max:
                        self.scan_data[angle_idx] = dist_m
                    else:
                        self.scan_data[angle_idx] = float('inf')

        except Exception as e:
            self.get_logger().debug(f'Read error: {e}')

    def publish_scan(self):
        if not self.scan_data:
            return

        now = self.get_clock().now()
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.frame_id

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / 360.0
        scan.time_increment = (1.0 / 7.0) / 360.0
        scan.scan_time = 1.0 / 7.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Map 360 degree array from [-pi, pi]
        # In ROS, angle 0 is front (index 180), -pi is rear (index 0), +pi is rear (index 359)
        ranges = []
        for i in range(360):
            # i = 0 corresponds to -180 deg (rear) -> 180 in circular deg
            deg = (i + 180) % 360
            ranges.append(self.scan_data.get(deg, float('inf')))

        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = YDLidarX4Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.write(b'\xA5\x65')  # Stop scanning
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
