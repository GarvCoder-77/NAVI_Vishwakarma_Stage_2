#!/usr/bin/env python3
"""
Monocular Webcam + 2D LiDAR Spatial Fusion Node for Blind Assistant Robot.

Bridges 2D computer vision with 2D planar LiDAR without requiring an RGB-D depth camera:
1. Extracts landmark candidates via dual cues:
   - OpenCV ArUco Marker detection (cv2.aruco DICT_4X4_50).
   - Non-overlapping HSV color contour classification.
2. Derives precise horizontal bearing angles from visual bounding boxes:
   theta_bearing = -((u - W/2) / (W/2)) * (HFOV / 2)
3. Intersects visual bearing with planar 2D LiDAR beams, clustering foreground returns
   to reject background room walls.
4. Projects relative polar detections into global 3D 'map' frame using TF2.
5. Performs multi-frame EMA smoothing, confirms landmarks (N >= 3), and broadcasts:
   - /room_landmarks (latched JSON catalog)
   - /semantic_landmarks (3D visualization markers)
   - /voice_speech (audio announcements)
"""

import os
import json
import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


# ArUco Marker Dictionary Mappings (ArUco ID -> Landmark Class)
LANDMARK_ARUCO_MAP = {
    0: {'name': 'Desk', 'voice_name': 'Desk', 'standoff_offset': 0.70, 'color_rgb': (0.7, 0.4, 0.15)},
    1: {'name': 'Chair', 'voice_name': 'Chair', 'standoff_offset': 0.65, 'color_rgb': (1.0, 0.85, 0.0)},
    2: {'name': 'Doorway', 'voice_name': 'Doorway', 'standoff_offset': 0.65, 'color_rgb': (0.0, 0.85, 1.0)},
    3: {'name': 'Restroom Sign', 'voice_name': 'Restroom Sign', 'standoff_offset': 0.70, 'color_rgb': (1.0, 0.1, 0.9)},
    4: {'name': 'Exit Sign', 'voice_name': 'Exit Sign', 'standoff_offset': 0.70, 'color_rgb': (0.0, 1.0, 0.2)},
    5: {'name': 'Cabinet', 'voice_name': 'Storage Cabinet', 'standoff_offset': 0.75, 'color_rgb': (0.2, 0.3, 1.0)},
    6: {'name': 'Pillar', 'voice_name': 'Pillar Column', 'standoff_offset': 0.70, 'color_rgb': (0.6, 0.2, 0.9)},
}

# Non-Overlapping HSV Color Signatures (OpenCV: H 0-179, S 0-255, V 0-255)
LANDMARK_HSV_CLASSES = [
    {
        'name': 'Exit Sign',
        'voice_name': 'Exit Sign',
        'h_low': 50, 'h_high': 75,
        's_low': 140, 's_high': 255,
        'v_low': 130, 'v_high': 255,
        'min_area': 140,
        'color_rgb': (0.0, 1.0, 0.2),
        'standoff_offset': 0.70,
    },
    {
        'name': 'Doorway',
        'voice_name': 'Doorway to Room 2',
        'h_low': 88, 'h_high': 104,
        's_low': 140, 's_high': 255,
        'v_low': 140, 'v_high': 255,
        'min_area': 160,
        'color_rgb': (0.0, 0.85, 1.0),
        'standoff_offset': 0.65,
    },
    {
        'name': 'Restroom Sign',
        'voice_name': 'Restroom Sign',
        'h_low': 152, 'h_high': 170,
        's_low': 140, 's_high': 255,
        'v_low': 130, 'v_high': 255,
        'min_area': 140,
        'color_rgb': (1.0, 0.1, 0.9),
        'standoff_offset': 0.70,
    },
    {
        'name': 'Cabinet',
        'voice_name': 'Storage Cabinet',
        'h_low': 110, 'h_high': 124,
        's_low': 140, 's_high': 255,
        'v_low': 120, 'v_high': 255,
        'min_area': 250,
        'color_rgb': (0.2, 0.3, 1.0),
        'standoff_offset': 0.75,
    },
    {
        'name': 'Desk',
        'voice_name': 'Desk',
        'h_low': 7, 'h_high': 16,
        's_low': 140, 's_high': 255,
        'v_low': 50, 'v_high': 190,
        'min_area': 280,
        'color_rgb': (0.7, 0.4, 0.15),
        'standoff_offset': 0.70,
    },
    {
        'name': 'Chair',
        'voice_name': 'Chair',
        'h_low': 22, 'h_high': 36,
        's_low': 150, 's_high': 255,
        'v_low': 150, 'v_high': 255,
        'min_area': 200,
        'color_rgb': (1.0, 0.85, 0.0),
        'standoff_offset': 0.65,
    },
    {
        'name': 'Pillar',
        'voice_name': 'Pillar',
        'h_low': 126, 'h_high': 140,
        's_low': 140, 's_high': 255,
        'v_low': 100, 'v_high': 255,
        'min_area': 250,
        'color_rgb': (0.6, 0.2, 0.9),
        'standoff_offset': 0.70,
    },
]


def calculate_bearing(u: float, img_width: int = 640, hfov: float = 1.15) -> float:
    """Computes horizontal bearing angle in radians relative to camera forward (+X).

    Left of center is positive (+angle), Right of center is negative (-angle).
    """
    norm_x = (u - (img_width / 2.0)) / (img_width / 2.0)
    return float(-norm_x * (hfov / 2.0))


def extract_roi_depth_distance(depth_img: np.ndarray, x: int, y: int, bw: int, bh: int) -> float | None:
    """
    Extracts median metric depth distance from central 50% ROI of bounding box.
    Supports both 32-bit float meters ('32FC1') and 16-bit uint millimeters ('16UC1' like YDLIDAR HP60C).
    """
    if depth_img is None:
        return None
    dh, dw = depth_img.shape[:2]
    rx0 = max(0, int(x + bw * 0.25))
    rx1 = min(dw, int(x + bw * 0.75))
    ry0 = max(0, int(y + bh * 0.25))
    ry1 = min(dh, int(y + bh * 0.75))
    if rx1 <= rx0 or ry1 <= ry0:
        return None

    roi = depth_img[ry0:ry1, rx0:rx1]
    if roi.dtype == np.uint16:
        # Convert millimeters to meters for 16-bit depth sensors
        roi = roi.astype(np.float32) / 1000.0

    valid = roi[np.isfinite(roi) & (roi > 0.15) & (roi < 9.0)]
    if len(valid) == 0:
        return None
    return float(np.median(valid))


def extract_lidar_distance_at_bearing(
    ranges: list,
    angle_min: float,
    angle_increment: float,
    bearing_rad: float,
    window_rad: float = 0.08,
    range_min: float = 0.12,
    range_max: float = 10.0,
) -> float | None:
    """Intersects visual bearing with 2D LiDAR scan and filters foreground clusters.

    Uses a 1D density cluster to isolate the nearest target obstacle from background walls.
    """
    n_rays = len(ranges)
    if n_rays == 0:
        return None

    angles = np.array([angle_min + i * angle_increment for i in range(n_rays)])
    # Normalize angle differences to [-pi, pi]
    diffs = np.abs(np.arctan2(np.sin(angles - bearing_rad), np.cos(angles - bearing_rad)))

    in_window = np.where(diffs <= window_rad)[0]
    if len(in_window) == 0:
        return None

    valid = []
    for idx in in_window:
        r = ranges[idx]
        if range_min <= r <= range_max and not math.isnan(r) and not math.isinf(r):
            valid.append(r)

    if not valid:
        return None

    # Sort valid ranges
    sorted_ranges = sorted(valid)

    # Foreground clustering: find the closest dense cluster (gap threshold: 0.40m)
    clusters = [[sorted_ranges[0]]]
    for r in sorted_ranges[1:]:
        if r - clusters[-1][-1] < 0.40:
            clusters[-1].append(r)
        else:
            clusters.append([r])

    # Select the nearest cluster (the foreground landmark)
    nearest_cluster = clusters[0]
    return float(np.median(nearest_cluster))


def project_detection_to_map(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    measured_dist: float,
    bearing: float,
    cam_offset_x: float = 0.062,
    cam_offset_y: float = 0.0,
) -> tuple[float, float]:
    """Projects relative polar detection into global 2D map coordinates via TF2 math."""
    rel_x = cam_offset_x + measured_dist * math.cos(bearing)
    rel_y = cam_offset_y + measured_dist * math.sin(bearing)

    map_x = robot_x + rel_x * math.cos(robot_yaw) - rel_y * math.sin(robot_yaw)
    map_y = robot_y + rel_x * math.sin(robot_yaw) + rel_y * math.cos(robot_yaw)
    return float(map_x), float(map_y)


def calculate_standoff_pose(landmark: dict, robot_pose: tuple[float, float]) -> tuple[float, float]:
    """Derives a safe standoff pose facing the landmark on open floor along the line of sight."""
    gx, gy = landmark['x'], landmark['y']
    rx, ry = robot_pose
    offset = landmark.get('standoff_offset', 0.70)

    vx = rx - gx
    vy = ry - gy
    v_dist = math.hypot(vx, vy)

    if v_dist > 0.35:
        eff_offset = min(offset, v_dist * 0.70)
        standoff_x = gx + (vx / v_dist) * eff_offset
        standoff_y = gy + (vy / v_dist) * eff_offset
        return float(standoff_x), float(standoff_y)

    # Fallback offset
    return float(gx - offset), float(gy)


class WebcamLidarFusionNode(Node):
    """ROS 2 Node fusing monocular webcam RGB video with 2D planar LiDAR for competition."""

    def __init__(self):
        super().__init__('webcam_lidar_fusion_node')

        self.declare_parameter('camera_fov_h', 1.15)
        self.declare_parameter('confirm_threshold', 3)
        self.declare_parameter('show_window', True)
        self.declare_parameter('enable_aruco', True)

        self.camera_fov_h = self.get_parameter('camera_fov_h').get_parameter_value().double_value
        self.confirm_threshold = self.get_parameter('confirm_threshold').get_parameter_value().integer_value
        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        self.enable_aruco = self.get_parameter('enable_aruco').get_parameter_value().bool_value

        self.gui_available = bool(os.environ.get('DISPLAY')) and self.show_window
        if self.gui_available:
            try:
                cv2.namedWindow('Robot Vision — Webcam + LiDAR Fusion', cv2.WINDOW_AUTOSIZE)
                self.get_logger().info('Native OpenCV Fusion Window initialized.')
            except Exception as e:
                self.get_logger().warning(f'Could not initialize OpenCV window: {e}')
                self.gui_available = False

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State caches
        self.latest_scan = None
        self.latest_depth_img = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_pose = False
        self.landmarks = {}
        self.last_catalog_pub = 0.0

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 5)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 5)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publishers
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.annotated_image_pub = self.create_publisher(Image, '/camera/detections_image', 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_landmarks', 10)
        self.catalog_pub = self.create_publisher(String, '/room_landmarks', latched_qos)
        self.voice_speech_pub = self.create_publisher(String, '/voice_speech', 10)

        # Timer for periodic RViz marker broadcast (2 Hz)
        self.marker_timer = self.create_timer(0.5, self.publish_markers)

        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info('=== Webcam + 2D LiDAR Spatial Fusion Online ===')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            pass

    def update_robot_pose_from_tf(self) -> bool:
        """Fetches latest map -> base_footprint transform."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny, cosy)
            self.has_pose = True
            return True
        except Exception:
            return False

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        has_map = self.update_robot_pose_from_tf()
        if not has_map or self.latest_scan is None:
            return

        h, w, _ = cv_img.shape
        annotated_img = cv_img.copy()

        # Overlay HUD
        cv2.rectangle(annotated_img, (0, 0), (w, 32), (30, 30, 30), -1)
        confirmed_count = sum(1 for lm in self.landmarks.values() if lm.get('confirmed', False))
        depth_mode = "HP60C 3D Depth Active" if self.latest_depth_img is not None else "2D LiDAR Raycast Active"
        hud_text = f"Hardware Perception | {depth_mode} | Confirmed: {confirmed_count} Landmarks"
        cv2.putText(annotated_img, hud_text, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 200), 2)

        # CUE 1: ArUco Tag Identification (Robust under varying competition lighting)
        if self.enable_aruco:
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None:
                for i, tag_id in enumerate(ids.flatten()):
                    if tag_id in LANDMARK_ARUCO_MAP:
                        lm_info = LANDMARK_ARUCO_MAP[tag_id]
                        pts = corners[i][0]
                        cx = float(np.mean(pts[:, 0]))
                        bx = int(min(pts[:, 0]))
                        by = int(min(pts[:, 1]))
                        bw = int(max(pts[:, 0]) - bx)
                        bh = int(max(pts[:, 1]) - by)

                        bearing = calculate_bearing(cx, img_width=w, hfov=self.camera_fov_h)
                        # Priority 1: Native HP60C RGB-D Depth ROI
                        depth_dist = extract_roi_depth_distance(self.latest_depth_img, bx, by, bw, bh)
                        if depth_dist is not None:
                            dist = depth_dist
                            sensor_src = "Depth"
                        else:
                            # Priority 2: 2D LiDAR Ray-Casting Fallback
                            dist = extract_lidar_distance_at_bearing(
                                self.latest_scan.ranges,
                                self.latest_scan.angle_min,
                                self.latest_scan.angle_increment,
                                bearing,
                            )
                            sensor_src = "LiDAR"

                        if dist is not None and 0.20 <= dist <= 6.5:
                            map_x, map_y = project_detection_to_map(
                                self.robot_x, self.robot_y, self.robot_yaw, dist, bearing)
                            self.register_landmark(lm_info['name'], lm_info, map_x, map_y)
                            cv2.polylines(annotated_img, [pts.astype(np.int32)], True, (0, 255, 0), 2)
                            cv2.putText(annotated_img, f"{lm_info['name']} [{dist:.2f}m ({sensor_src})]",
                                        (int(pts[0][0]), max(int(pts[0][1]) - 10, 45)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # CUE 2: HSV Color Segmentation
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        for lm_class in LANDMARK_HSV_CLASSES:
            lower = np.array([lm_class['h_low'], lm_class['s_low'], lm_class['v_low']], dtype=np.uint8)
            upper = np.array([lm_class['h_high'], lm_class['s_high'], lm_class['v_high']], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            kernel = np.ones((4, 4), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) < lm_class['min_area']:
                    continue
                bx, by, bw, bh = cv2.boundingRect(cnt)
                cx = bx + bw / 2.0
                bearing = calculate_bearing(cx, img_width=w, hfov=self.camera_fov_h)

                # Priority 1: Native HP60C RGB-D Depth ROI
                depth_dist = extract_roi_depth_distance(self.latest_depth_img, bx, by, bw, bh)
                if depth_dist is not None:
                    dist = depth_dist
                    sensor_src = "Depth"
                else:
                    # Priority 2: 2D LiDAR Ray-Casting Fallback
                    dist = extract_lidar_distance_at_bearing(
                        self.latest_scan.ranges,
                        self.latest_scan.angle_min,
                        self.latest_scan.angle_increment,
                        bearing,
                    )
                    sensor_src = "LiDAR"

                if dist is None or dist < 0.20 or dist > 6.5:
                    continue

                map_x, map_y = project_detection_to_map(
                    self.robot_x, self.robot_y, self.robot_yaw, dist, bearing)
                self.register_landmark(lm_class['name'], lm_class, map_x, map_y)

                bgr_color = (
                    int(lm_class['color_rgb'][2] * 255),
                    int(lm_class['color_rgb'][1] * 255),
                    int(lm_class['color_rgb'][0] * 255)
                )
                cv2.rectangle(annotated_img, (bx, by), (bx + bw, by + bh), bgr_color, 2)
                cv2.putText(annotated_img, f"{lm_class['name']} [{dist:.2f}m ({sensor_src})]",
                            (bx, max(by - 8, 45)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, bgr_color, 2)

        if self.gui_available:
            try:
                cv2.imshow('Robot Vision — Webcam + LiDAR Fusion', annotated_img)
                cv2.waitKey(1)
            except Exception:
                pass

        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            out_msg.header = msg.header
            self.annotated_image_pub.publish(out_msg)
        except Exception:
            pass

    def register_landmark(self, name: str, lm_info: dict, map_x: float, map_y: float):
        """Spatial clustering, filtering, and catalog management for detected objects."""
        if name in self.landmarks:
            lm = self.landmarks[name]
            d = math.hypot(map_x - lm['x'], map_y - lm['y'])
            if lm['confirmed'] and d > 2.0:
                return  # Outlier rejection

            alpha = 0.15 if lm['confirmed'] else 0.25
            lm['x'] = (1.0 - alpha) * lm['x'] + alpha * map_x
            lm['y'] = (1.0 - alpha) * lm['y'] + alpha * map_y
            lm['count'] += 1
            lm['last_seen'] = time.time()
            lm['standoff_x'], lm['standoff_y'] = calculate_standoff_pose(
                lm, (self.robot_x, self.robot_y))

            if lm['count'] >= self.confirm_threshold and not lm['confirmed']:
                lm['confirmed'] = True
                self.on_landmark_confirmed(lm)
            elif time.time() - self.last_catalog_pub > 3.0:
                self.publish_catalog()
        else:
            self.landmarks[name] = {
                'id': name,
                'class': name,
                'voice_name': lm_info.get('voice_name', name),
                'x': map_x,
                'y': map_y,
                'count': 1,
                'confirmed': False,
                'color_rgb': lm_info.get('color_rgb', (1.0, 1.0, 1.0)),
                'standoff_offset': lm_info.get('standoff_offset', 0.70),
                'standoff_x': map_x,
                'standoff_y': map_y,
                'first_seen': time.time(),
                'last_seen': time.time(),
            }
            self.landmarks[name]['standoff_x'], self.landmarks[name]['standoff_y'] = calculate_standoff_pose(
                self.landmarks[name], (self.robot_x, self.robot_y))

    def on_landmark_confirmed(self, lm: dict):
        v_name = lm['voice_name']
        gx, gy = lm['x'], lm['y']
        self.get_logger().info(f'🌟 CONFIRMED LANDMARK: {v_name} at ({gx:.2f}, {gy:.2f})!')

        voice_msg = String()
        voice_msg.data = f"Identified {v_name} at coordinates {gx:.1f}, {gy:.1f}."
        self.voice_speech_pub.publish(voice_msg)
        self.publish_catalog()

    def publish_catalog(self):
        self.last_catalog_pub = time.time()
        catalog_dict = {}
        for idx, (name, lm) in enumerate(self.landmarks.items()):
            if lm['confirmed']:
                catalog_dict[str(idx + 1)] = {
                    'name': lm['class'],
                    'voice_name': lm['voice_name'],
                    'x': lm['x'],
                    'y': lm['y'],
                    'standoff_x': lm['standoff_x'],
                    'standoff_y': lm['standoff_y'],
                }
        if catalog_dict:
            msg = String()
            msg.data = json.dumps(catalog_dict)
            self.catalog_pub.publish(msg)

    def publish_markers(self):
        if not self.landmarks:
            return
        m_array = MarkerArray()
        for idx, (name, lm) in enumerate(self.landmarks.items()):
            if not lm['confirmed']:
                continue
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'landmarks'
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = lm['x']
            marker.pose.position.y = lm['y']
            marker.pose.position.z = 0.4
            marker.scale.x = 0.35
            marker.scale.y = 0.35
            marker.scale.z = 0.8
            marker.color.r = float(lm['color_rgb'][0])
            marker.color.g = float(lm['color_rgb'][1])
            marker.color.b = float(lm['color_rgb'][2])
            marker.color.a = 0.85
            m_array.markers.append(marker)
        self.marker_pub.publish(m_array)


def main(args=None):
    rclpy.init(args=args)
    node = WebcamLidarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
