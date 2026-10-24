#!/usr/bin/env python3
"""
Voice Guidance Node for Blind Assistant Robot.
Provides non-spamming, intelligent auditory notifications:
- Announces system status
- Only warns about obstacles directly ahead when moving forward
- 6-second cooldown to prevent repetitive audio loops
"""

import math
import subprocess
import threading
import time
import queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class VoiceGuidanceNode(Node):
    def __init__(self):
        super().__init__('voice_guidance')

        self.declare_parameter('min_obstacle_distance', 0.38)
        self.declare_parameter('enable_speech', True)

        self.min_dist = self.get_parameter('min_obstacle_distance').get_parameter_value().double_value
        self.enable_speech = self.get_parameter('enable_speech').get_parameter_value().bool_value

        self.last_speech_time = 0.0
        self.speech_cooldown = 7.0  # Cooldown between proximity warnings to prevent repetitive chatter
        self.current_linear_v = 0.0

        # Dedicated sequential FIFO speech worker
        self.speech_queue = queue.Queue()
        self.speech_worker = threading.Thread(target=self._speech_worker_loop, daemon=True)
        self.speech_worker.start()

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/exploration_status', self.status_callback, 10)
        self.speech_sub = self.create_subscription(
            String, '/voice_speech', self.voice_speech_callback, 10)

        self.get_logger().info('Voice Guidance Node running.')
        self.speak('Blind Assistant Robot is online. Autonomous exploration active.')

    def _speech_worker_loop(self):
        while True:
            text = self.speech_queue.get()
            try:
                subprocess.run(['spd-say', '-r', '-10', text],
                               check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                try:
                    subprocess.run(['espeak-ng', '-s', '140', text],
                                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except Exception:
                    pass
            self.speech_queue.task_done()

    def voice_speech_callback(self, msg: String):
        if msg.data:
            self.speak(msg.data)

    def status_callback(self, msg: String):
        self.get_logger().info(f'Status event: {msg.data}')
        if 'COMPLETED' in msg.data:
            self.speak('Exploration complete. All areas have been mapped. Final map is published and saved.')
        elif 'DOORWAY' in msg.data:
            self.speak('Doorway detected. Entering adjoining room.')
        elif 'REPLAN' in msg.data:
            self.speak('Path blocked in updated map. Replanning detour.')

    def speak(self, text: str):
        if not self.enable_speech:
            self.get_logger().info(f'[Muted Speech]: {text}')
            return

        self.get_logger().info(f'[Voice Announcement]: "{text}"')
        self.speech_queue.put(text)

    def cmd_vel_callback(self, msg: Twist):
        self.current_linear_v = msg.linear.x

    def scan_callback(self, msg: LaserScan):
        now = time.time()
        if now - self.last_speech_time < self.speech_cooldown:
            return

        # Only warn if the robot is actually driving forward toward an obstacle
        if self.current_linear_v <= 0.05:
            return

        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        # Check ONLY the front cone (-25 deg to +25 deg)
        min_front_dist = float('inf')
        for idx in range(n):
            r = ranges[idx]
            if not (msg.range_min < r < msg.range_max):
                continue
            ang = msg.angle_min + idx * msg.angle_increment
            ang = math.atan2(math.sin(ang), math.cos(ang))

            if abs(ang) < math.radians(25):
                if r < min_front_dist:
                    min_front_dist = r

        if min_front_dist < self.min_dist:
            self.last_speech_time = now
            dist_cm = int(min_front_dist * 100)
            self.speak(f'Obstacle ahead at {dist_cm} centimeters.')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.speak('Assistant robot stopping.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
