#!/usr/bin/env python3
"""
Real-Time SLAM Map Desktop Visualizer with Minimalist Map Pins & Interactive HUD Sidebar.

Features:
1. High-Performance Dual-Panel Desktop Window:
   - Left Panel (700x700): Crisp 2D occupancy grid SLAM map with trajectory trail, planned path,
     robot heading, and minimalist numbered pin badges ([1], [2], [3]...). Zero floating text clutter.
   - Right Panel (280x700): Dedicated Telemetry & Discovered Landmarks HUD Sidebar with high-contrast
     cards, live distance from robot, dynamic pedestrian status, and navigation states.
2. Minimalist Map Pins:
   - Each discovered landmark is rendered as a clean, colored concentric pin badge with its catalog number.
   - Dynamic human rendered as an orange pin [P] with proximity indicator.
   - Robot rendered as a cyan circular badge [R] with directional heading vector.
3. Zero Web Bloat:
   - 100% native desktop OpenCV window running at 12 Hz.
"""

import os
import math
import time
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


# Distinct color palette for landmarks (BGR format for OpenCV)
LANDMARK_COLORS = {
    'desk': (30, 110, 210),         # Warm orange/brown
    'chair': (0, 140, 255),         # Amber
    'doorway': (255, 220, 0),       # Cyan / Light blue
    'cabinet': (220, 70, 30),       # Deep blue
    'pillar': (80, 200, 60),        # Emerald
    'restroom sign': (220, 30, 200),# Magenta
    'exit sign': (50, 230, 50),     # Bright green
}


class SlamMapVisualizer(Node):
    def __init__(self):
        super().__init__('slam_map_visualizer')

        self.declare_parameter('window_size', 700)
        self.declare_parameter('show_window', True)
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value

        self.map_w = self.window_size
        self.map_h = self.window_size
        self.sidebar_w = 280
        self.total_w = self.map_w + self.sidebar_w
        self.total_h = self.map_h

        self.gui_available = bool(os.environ.get('DISPLAY')) and self.show_window
        if self.gui_available:
            try:
                cv2.namedWindow('Robot SLAM Map - Live Occupancy Grid', cv2.WINDOW_AUTOSIZE)
                self.get_logger().info('Native SLAM Map Pop-Up Window initialized with HUD Sidebar.')
            except Exception as e:
                self.get_logger().warning(f'Could not initialize OpenCV map window: {e}')
                self.gui_available = False

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State cache
        self.latest_map = None
        self.latest_path = []
        self.latest_trajectory = []
        self.human_pose = None
        self.last_human_seen_time = 0.0
        self.landmarks = {}
        self.status_text = "INITIALIZING SLAM"
        self.robot_pose = None  # (x, y, yaw)
        self.current_cmd_v = 0.0
        self.current_cmd_w = 0.0

        # QoS for latched topics
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, latched_qos)
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)
        self.traj_sub = self.create_subscription(
            Path, '/robot_trajectory', self.trajectory_callback, 10)
        # Sensory-only dynamic pedestrian detection (NO ground-truth cheat)
        self.human_sub = self.create_subscription(
            PoseStamped, '/detected_human', self.human_callback, 10)
        self.landmarks_sub = self.create_subscription(
            String, '/room_landmarks', self.landmarks_callback, latched_qos)
        self.status_sub = self.create_subscription(
            String, '/exploration_status', self.status_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publisher for optional image topic
        self.map_image_pub = self.create_publisher(Image, '/map_visualizer/image', 5)

        # Render loop at 12 Hz
        self.render_timer = self.create_timer(0.08, self.render_loop)
        self.get_logger().info('=== SLAM Map Desktop Visualizer (Minimalist UI) Active ===')

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def path_callback(self, msg: Path):
        self.latest_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def trajectory_callback(self, msg: Path):
        self.latest_trajectory = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def human_callback(self, msg: PoseStamped):
        self.human_pose = (msg.pose.position.x, msg.pose.position.y)
        self.last_human_seen_time = time.time()

    def landmarks_callback(self, msg: String):
        try:
            self.landmarks = json.loads(msg.data)
        except Exception:
            pass

    def status_callback(self, msg: String):
        self.status_text = msg.data

    def cmd_callback(self, msg: Twist):
        self.current_cmd_v = msg.linear.x
        self.current_cmd_w = msg.angular.z

    def update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            self.robot_pose = (rx, ry, yaw)
        except Exception:
            pass

    def world_to_pixel(self, wx: float, wy: float, info) -> tuple:
        """Transforms world coordinate (wx, wy) into canvas pixel (px, py) on left map panel."""
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        mw = info.width
        mh = info.height

        # Grid cell
        gx = (wx - ox) / res
        gy = (wy - oy) / res

        # Map grid to left map panel with padding
        scale_x = (self.map_w - 40) / max(1, mw)
        scale_y = (self.map_h - 90) / max(1, mh)
        scale = min(scale_x, scale_y)

        # Center within left map panel
        disp_w = int(mw * scale)
        disp_h = int(mh * scale)
        offset_x = int((self.map_w - disp_w) / 2)
        offset_y = int(45 + ((self.map_h - 75) - disp_h) / 2)

        px = int(offset_x + gx * scale)
        # Flip Y so North (+Y) is upward
        py = int(offset_y + (mh - gy) * scale)
        return (px, py)

    def render_loop(self):
        self.update_robot_pose()
        canvas = np.zeros((self.total_h, self.total_w, 3), dtype=np.uint8)

        # Base background: dark graphite
        canvas[:] = (20, 22, 28)

        if self.latest_map is None:
            # Display waiting banner on map area
            cv2.putText(canvas, "Awaiting SLAM Map from SLAM Toolbox...", (40, self.map_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
            cv2.putText(canvas, "Aligning TF frames: map -> odom -> base_footprint", (40, (self.map_h // 2) + 36),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1)
            self.render_sidebar(canvas, None)
            self.display_and_publish(canvas)
            return

        grid = self.latest_map
        info = grid.info
        w = info.width
        h = info.height
        raw_data = np.array(grid.data, dtype=np.int8).reshape((h, w))

        # 1. Base map coloring
        map_img = np.zeros((h, w, 3), dtype=np.uint8)
        # Unknown: dark slate (38, 36, 32)
        map_img[raw_data == -1] = [38, 36, 32]
        # Free space: crisp off-white (245, 245, 248)
        map_img[raw_data == 0] = [245, 245, 248]
        # Obstacles / walls: dark charcoal (25, 25, 25)
        map_img[raw_data >= 50] = [25, 25, 25]
        # Mid-cost inflation: subtle silver-gray
        mid_mask = (raw_data > 0) & (raw_data < 50)
        map_img[mid_mask] = [210, 212, 218]

        # Flip vertically so +Y is up
        map_img = cv2.flip(map_img, 0)

        # Scale map to left panel
        scale_x = (self.map_w - 40) / max(1, w)
        scale_y = (self.map_h - 90) / max(1, h)
        scale = min(scale_x, scale_y)
        disp_w = int(w * scale)
        disp_h = int(h * scale)

        resized_map = cv2.resize(map_img, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)

        offset_x = int((self.map_w - disp_w) / 2)
        offset_y = int(45 + ((self.map_h - 75) - disp_h) / 2)

        # Paste map onto left canvas panel
        canvas[offset_y:offset_y + disp_h, offset_x:offset_x + disp_w] = resized_map

        # Draw map boundary line
        cv2.rectangle(canvas, (offset_x - 1, offset_y - 1), (offset_x + disp_w + 1, offset_y + disp_h + 1), (60, 65, 75), 1)

        # 2. Draw Exploration Trajectory Trail
        if len(self.latest_trajectory) > 1:
            pts = [self.world_to_pixel(x, y, info) for x, y in self.latest_trajectory]
            for i in range(len(pts) - 1):
                cv2.line(canvas, pts[i], pts[i + 1], (180, 230, 50), 2, cv2.LINE_AA)

        # 3. Draw Planned A* Global Path
        if len(self.latest_path) > 1:
            path_pts = [self.world_to_pixel(x, y, info) for x, y in self.latest_path]
            for i in range(len(path_pts) - 1):
                cv2.line(canvas, path_pts[i], path_pts[i + 1], (0, 210, 255), 2, cv2.LINE_AA)
            for pt in path_pts[::3]:
                cv2.circle(canvas, pt, 2, (0, 180, 255), -1)

        # 4. Draw Minimalist Numbered Landmark Pins on Map
        sorted_landmarks = sorted(
            self.landmarks.items(),
            key=lambda x: int(x[0]) if x[0].isdigit() else 99
        )

        for pin_num, (k, lm) in enumerate(sorted_landmarks, start=1):
            lx = lm.get('x', 0.0)
            ly = lm.get('y', 0.0)
            name = lm.get('name', lm.get('voice_name', k))
            color_key = name.lower()
            color = LANDMARK_COLORS.get(color_key, (0, 200, 255))

            lpx, lpy = self.world_to_pixel(lx, ly, info)

            # Standoff target dot & connection line
            if 'standoff_x' in lm and 'standoff_y' in lm:
                spx, spy = self.world_to_pixel(lm['standoff_x'], lm['standoff_y'], info)
                cv2.circle(canvas, (spx, spy), 3, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.line(canvas, (lpx, lpy), (spx, spy), (100, 120, 140), 1, cv2.LINE_AA)

            # Outer ring
            cv2.circle(canvas, (lpx, lpy), 12, color, 2, cv2.LINE_AA)
            # Filled center badge
            cv2.circle(canvas, (lpx, lpy), 9, (25, 28, 35), -1, cv2.LINE_AA)
            cv2.circle(canvas, (lpx, lpy), 9, color, 1, cv2.LINE_AA)

            # Number badge inside pin
            num_str = str(k) if str(k).isdigit() else str(pin_num)
            text_size = cv2.getTextSize(num_str, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)[0]
            tx = lpx - text_size[0] // 2
            ty = lpy + text_size[1] // 2
            cv2.putText(canvas, num_str, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

        # 5. Draw Dynamic Moving Human Pin [P] (Sensory line-of-sight only: within last 2.0s)
        human_in_sight = (self.human_pose is not None) and (time.time() - self.last_human_seen_time < 2.0)
        if human_in_sight:
            hx, hy = self.human_pose
            hpx, hpy = self.world_to_pixel(hx, hy, info)
            # Pulsing orange indicator
            cv2.circle(canvas, (hpx, hpy), 13, (0, 140, 255), 2, cv2.LINE_AA)
            cv2.circle(canvas, (hpx, hpy), 9, (0, 100, 255), -1, cv2.LINE_AA)
            cv2.circle(canvas, (hpx, hpy), 9, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, "P", (hpx - 4, hpy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

        # 6. Draw Robot Current Pose & Heading Arrow [R]
        if self.robot_pose is not None:
            rx, ry, ryaw = self.robot_pose
            rpx, rpy = self.world_to_pixel(rx, ry, info)

            # Heading arrow endpoint (North is -py in pixel coords)
            arrow_len = 18
            ax = int(rpx + arrow_len * math.cos(ryaw))
            ay = int(rpy - arrow_len * math.sin(ryaw))

            # Robot body circle with [R]
            cv2.circle(canvas, (rpx, rpy), 12, (255, 140, 0), -1, cv2.LINE_AA)
            cv2.circle(canvas, (rpx, rpy), 12, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.arrowedLine(canvas, (rpx, rpy), (ax, ay), (0, 255, 255), 2, tipLength=0.35)
            cv2.putText(canvas, "R", (rpx - 4, rpy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

        # 7. Render Top HUD on Left Map Panel
        cv2.rectangle(canvas, (0, 0), (self.map_w, 40), (16, 18, 22), -1)
        title_text = "BLIND ASSISTANT ROBOT - SLAM MAP"
        cv2.putText(canvas, title_text, (15, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 220), 2, cv2.LINE_AA)

        # 8. Render Right HUD Sidebar
        self.render_sidebar(canvas, sorted_landmarks)

        self.display_and_publish(canvas)

    def render_sidebar(self, canvas: np.ndarray, sorted_landmarks):
        """Draws a clean, high-contrast telemetry and landmark catalog HUD in the right panel."""
        sx = self.map_w
        # Vertical divider line
        cv2.line(canvas, (sx, 0), (sx, self.total_h), (50, 55, 65), 1)

        # Background fill
        cv2.rectangle(canvas, (sx + 1, 0), (self.total_w, self.total_h), (22, 25, 32), -1)

        # Panel Title
        cv2.putText(canvas, "MISSION TELEMETRY", (sx + 16, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 240, 220), 2, cv2.LINE_AA)
        cv2.line(canvas, (sx + 16, 34), (self.total_w - 16, 34), (45, 50, 60), 1)

        y = 54
        # Section 1: Robot Status
        if self.robot_pose is not None:
            rx, ry, ryaw = self.robot_pose
            cv2.putText(canvas, f"Pose: ({rx:.2f}, {ry:.2f})m", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 225, 230), 1, cv2.LINE_AA)
            y += 20
            yaw_deg = math.degrees(ryaw)
            cv2.putText(canvas, f"Heading: {yaw_deg:.1f} deg", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 185, 195), 1, cv2.LINE_AA)
            y += 20
            cv2.putText(canvas, f"Speed: {self.current_cmd_v:.2f} m/s", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 185, 195), 1, cv2.LINE_AA)
            y += 20

        # State badge
        state_str = self.status_text[:24]
        cv2.rectangle(canvas, (sx + 16, y - 2), (self.total_w - 16, y + 20), (32, 36, 46), -1)
        cv2.rectangle(canvas, (sx + 16, y - 2), (self.total_w - 16, y + 20), (0, 200, 220), 1)
        cv2.putText(canvas, state_str, (sx + 24, y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 230, 255), 1, cv2.LINE_AA)
        y += 36

        # Section 2: Dynamic Obstacle (Human) Monitor
        cv2.line(canvas, (sx + 16, y), (self.total_w - 16, y), (45, 50, 60), 1)
        y += 18
        cv2.putText(canvas, "MOVING HUMAN STATUS", (sx + 16, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 165, 255), 2, cv2.LINE_AA)
        y += 20

        human_in_sight = (self.human_pose is not None) and (time.time() - self.last_human_seen_time < 2.0)
        if human_in_sight and self.robot_pose is not None:
            hx, hy = self.human_pose
            rx, ry, ryaw = self.robot_pose
            dist_h = math.hypot(hx - rx, hy - ry)
            bearing_h = math.degrees(math.atan2(hy - ry, hx - rx) - ryaw)
            bearing_h = (bearing_h + 180) % 360 - 180

            cv2.putText(canvas, "Status:   [ IN SIGHT ]", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 220, 255), 1, cv2.LINE_AA)
            y += 18
            cv2.putText(canvas, f"Distance: {dist_h:.2f} m", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 225, 230), 1, cv2.LINE_AA)
            y += 18
            cv2.putText(canvas, f"Bearing:  {bearing_h:+.1f} deg", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 185, 195), 1, cv2.LINE_AA)
            y += 20

            # Yielding condition
            is_yielding = (dist_h < 1.25 and abs(bearing_h) < 45.0)
            tag_text = "[ YIELDING ]" if is_yielding else "[ CLEAR ]"
            tag_color = (0, 140, 255) if is_yielding else (0, 220, 100)
            cv2.putText(canvas, f"Corridor: {tag_text}", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, tag_color, 1, cv2.LINE_AA)
            y += 26
        else:
            cv2.putText(canvas, "Status:   [ OUT OF SIGHT ]", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (130, 140, 150), 1, cv2.LINE_AA)
            y += 18
            cv2.putText(canvas, "Corridor: [ CLEAR ]", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 220, 100), 1, cv2.LINE_AA)
            y += 26

        # Section 3: Discovered Landmarks Catalog
        cv2.line(canvas, (sx + 16, y), (self.total_w - 16, y), (45, 50, 60), 1)
        y += 18
        discovered_count = len(sorted_landmarks) if sorted_landmarks else 0
        cv2.putText(canvas, f"LANDMARKS ({discovered_count} Found)", (sx + 16, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (50, 220, 120), 2, cv2.LINE_AA)
        y += 22

        if sorted_landmarks:
            rx = self.robot_pose[0] if self.robot_pose is not None else 0.0
            ry = self.robot_pose[1] if self.robot_pose is not None else 0.0

            for pin_num, (k, lm) in enumerate(sorted_landmarks, start=1):
                if y > self.total_h - 75:
                    break
                name = lm.get('name', lm.get('voice_name', k))
                short_name = name.replace(" Sign", "")
                lx = lm.get('x', 0.0)
                ly = lm.get('y', 0.0)
                dist_lm = math.hypot(lx - rx, ly - ry)
                color_key = name.lower()
                color = LANDMARK_COLORS.get(color_key, (0, 200, 255))

                # Number badge pin
                badge_str = str(k) if str(k).isdigit() else str(pin_num)
                cv2.circle(canvas, (sx + 24, y), 8, color, -1, cv2.LINE_AA)
                cv2.putText(canvas, badge_str, (sx + 21, y + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1, cv2.LINE_AA)

                # Name
                cv2.putText(canvas, short_name, (sx + 38, y + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (230, 235, 240), 1, cv2.LINE_AA)

                # Standoff / Distance subtext
                sub_str = f"({lx:.1f}, {ly:.1f}) | {dist_lm:.1f}m away"
                cv2.putText(canvas, sub_str, (sx + 38, y + 17),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.33, (140, 150, 165), 1, cv2.LINE_AA)
                y += 32
        else:
            cv2.putText(canvas, "Scanning room...", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 165, 175), 1, cv2.LINE_AA)
            y += 18
            cv2.putText(canvas, "No landmarks cataloged yet", (sx + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 125, 135), 1, cv2.LINE_AA)
            y += 30

        # Section 4: Bottom Legend
        cv2.line(canvas, (sx + 16, self.total_h - 52), (self.total_w - 16, self.total_h - 52), (45, 50, 60), 1)
        legend_line1 = "[R] Robot   [P] Person   [1..N] Pins"
        legend_line2 = "[--] Trajectory   [--] Planned Path"
        cv2.putText(canvas, legend_line1, (sx + 16, self.total_h - 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (150, 155, 165), 1, cv2.LINE_AA)
        cv2.putText(canvas, legend_line2, (sx + 16, self.total_h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (150, 155, 165), 1, cv2.LINE_AA)

    def display_and_publish(self, canvas: np.ndarray):
        # Native OpenCV window pop-up
        if self.gui_available:
            try:
                cv2.imshow('Robot SLAM Map - Live Occupancy Grid', canvas)
                cv2.waitKey(1)
            except Exception:
                pass

        # Publish image message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'map'
            self.map_image_pub.publish(img_msg)
        except Exception:
            pass

    def destroy_node(self):
        if getattr(self, 'gui_available', False):
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SlamMapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
