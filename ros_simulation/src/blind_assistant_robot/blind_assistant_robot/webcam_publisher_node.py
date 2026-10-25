#!/usr/bin/env python3
"""
Lightweight UVC Webcam Publisher Node for Blind Assistant Robot.
Publishes /camera/image_raw and /camera/camera_info using OpenCV and cv_bridge.
Does not require external ros2 usb_cam or v4l2_camera packages.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class WebcamPublisherNode(Node):
    def __init__(self):
        super().__init__('webcam_publisher_node')

        # Parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('framerate', 30.0)
        self.declare_parameter('frame_id', 'camera_link')

        device_param = self.get_parameter('video_device').value
        # Allow either integer index (0) or device string ("/dev/video0")
        try:
            self.video_device = int(device_param)
        except (ValueError, TypeError):
            self.video_device = device_param

        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        self.framerate = max(1.0, float(self.get_parameter('framerate').value))
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()
        self.cap = None

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.open_camera()

        timer_period = 1.0 / self.framerate
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        self.get_logger().info(
            f'Webcam Publisher started on device {self.video_device} ({self.width}x{self.height} @ {self.framerate} FPS)'
        )

    def open_camera(self):
        try:
            if isinstance(self.video_device, int):
                self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
            else:
                self.cap = cv2.VideoCapture(self.video_device)

            if not self.cap.isOpened():
                # Fallback to standard backend if V4L2 backend failed
                self.cap = cv2.VideoCapture(self.video_device)

            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.framerate)
                self.get_logger().info(f'Camera device {self.video_device} opened successfully.')
            else:
                self.get_logger().warn(f'Could not open video device {self.video_device}. Will retry.')
        except Exception as e:
            self.get_logger().error(f'Exception opening camera: {e}')

    def capture_and_publish(self):
        if self.cap is None or not self.cap.isOpened():
            self.open_camera()
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Empty frame captured from camera, attempting reconnect...')
            self.cap.release()
            self.cap = None
            return

        now = self.get_clock().now().to_msg()

        # Convert OpenCV BGR image to ROS Image message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)

            # CameraInfo message
            info_msg = CameraInfo()
            info_msg.header = img_msg.header
            info_msg.width = frame.shape[1]
            info_msg.height = frame.shape[0]
            # Standard focal length approx fx = fy = width, cx = width/2, cy = height/2
            fx = float(info_msg.width)
            fy = float(info_msg.width)
            cx = float(info_msg.width) / 2.0
            cy = float(info_msg.height) / 2.0
            info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_pub.publish(info_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing camera image: {e}')

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
