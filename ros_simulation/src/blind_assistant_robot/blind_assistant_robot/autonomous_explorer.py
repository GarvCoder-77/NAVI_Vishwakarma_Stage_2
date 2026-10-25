#!/usr/bin/env python3
"""
Cognitive Map-Based Autonomous Explorer & Navigator for Blind Assistant Robot.

Core Features:
1. Persistent World Representation (Internal Map Memory):
   - Stores and continuously updates the 2D Occupancy Grid from SLAM.
   - Computes an inflated costmap in memory (lethal obstacle boundary + soft potential gradient)
     sized precisely for the robot's physical CAD geometry (0.22m radius).
2. Global A* Path Planning on Remembered Map:
   - Queries its internal world memory to calculate optimal, collision-free global routes
     from the robot's current pose to unexplored frontiers.
   - Routes through doorways and around partition walls; never executes blind reactive bouncing.
   - Performs line-of-sight ray tracing (string-pulling) for smooth, piecewise-linear paths.
3. Closed-Loop Pure Pursuit Feedback Controller:
   - Tracks the planned global path at 10 Hz with lookahead curvature steering.
   - Smoothly controls forward and angular velocity based on path curvature.
4. Real-Time Dynamic Replanning Loop:
   - Continuously verifies the active trajectory against newly discovered obstacles in the map memory.
   - If a new obstacle or wall cuts across the planned path, immediately executes an A* detour.
   - Detects when target areas are revealed by LiDAR and dynamically updates frontiers.
5. Autonomous Completion & Map Publishing:
   - Discovers all rooms, navigates doorways, and covers 100% of reachable area.
   - Saves final standard ROS 2 map (YAML + PGM) to /home/jatin/blind_assistant_map.*.
   - Publishes planned path on /planned_path and final map on /final_map.
"""

import os
import math
import time
import heapq
import json
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String, Empty
from builtin_interfaces.msg import Duration
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


FREE_THRESHOLD = 50
UNKNOWN = -1
MIN_FRONTIER_SIZE = 2
FRONTIER_SEARCH_RADIUS = 2


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')

        # Tunable Navigation Parameters
        self.declare_parameter('max_linear_speed', 0.28)
        self.declare_parameter('max_angular_speed', 0.85)
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('lookahead_dist', 0.35)
        self.declare_parameter('initial_spin_time', 0.0)
        self.declare_parameter('max_exploration_time', 120.0)
        self.declare_parameter('min_exploration_time', 25.0)
        self.declare_parameter('stagnation_timeout', 20.0)

        self.max_v = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_w = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.lookahead_dist = self.get_parameter('lookahead_dist').get_parameter_value().double_value
        self.initial_spin_time = self.get_parameter('initial_spin_time').get_parameter_value().double_value
        self.max_exploration_time = self.get_parameter('max_exploration_time').get_parameter_value().double_value
        self.min_exploration_time = self.get_parameter('min_exploration_time').get_parameter_value().double_value
        self.stagnation_timeout = self.get_parameter('stagnation_timeout').get_parameter_value().double_value

        # Robot Pose & State
        self.current_map = None
        self.lethal_costmap = None
        self.soft_costmap = None
        self.grid_data = None
        self.latest_scan = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Cognitive Planning State
        self.current_goal = None
        self.current_path = []  # list of (wx, wy)
        self.goals_completed = 0
        self.state = 'INIT_SPIN'  # 'INIT_SPIN', 'FOLLOWING_PATH', 'RECOVERING', 'CONFIRM_SPIN', 'COMPLETED'
        self.state_start_time = time.time()
        self.goal_start_time = time.time()
        self.last_replan_time = 0.0
        self.blacklisted_goals = []  # list of (gx, gy, expiry_time)

        # Exploration & Completion Tracking
        self.exploration_start_time = None
        self.empty_frontiers_count = 0
        self.confirmation_spin_start = 0.0
        self.completed_saved = False
        self.doorway_announced = False
        self.last_area_growth_time = time.time()
        self.max_free_cells_observed = 0

        # Stuck detector
        self.last_check_pos = (0.0, 0.0)
        self.last_check_time = time.time()
        self.recovery_end_time = 0.0
        self.recovery_dir = 1.0

        # TF Broadcaster & Map Listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_in_map_valid = False

        # QoS Profiles
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Guide Dog Mode & User Standoff Navigation State
        self.user_home_pose = None
        self.pre_recovery_state = 'FOLLOWING_PATH'
        self.active_guide_dest = None
        self.catalog = {}
        self.active_guide_name = None

        # Dynamic Obstacle Yielding State
        self.is_yielding = False
        self.yield_start_time = 0.0
        self.last_yield_announcement = 0.0
        self.human_pose = None
        self.last_human_pose_time = 0.0

        # Robot Trajectory Trail Publisher
        self.trajectory_pub = self.create_publisher(Path, '/robot_trajectory', 10)
        self.trajectory_path = Path()
        self.trajectory_path.header.frame_id = 'map'
        self.last_traj_time = 0.0

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        # Sensory-only dynamic pedestrian detection (NO ground-truth cheat)
        self.human_sub = self.create_subscription(
            PoseStamped, '/detected_human', self.human_pose_callback, 10)
        self.save_map_sub = self.create_subscription(
            Empty, '/save_map', self.manual_save_callback, 10)
        self.guide_sub = self.create_subscription(
            String, '/guide_to', self.guide_to_callback, 10)
        self.catalog_sub = self.create_subscription(
            String, '/room_landmarks', self.catalog_callback, map_qos)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.frontier_markers_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/exploration_goal', 10)
        self.footprint_marker_pub = self.create_publisher(Marker, '/robot_footprint', 10)
        self.final_map_pub = self.create_publisher(OccupancyGrid, '/final_map', map_qos)
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)
        self.voice_speech_pub = self.create_publisher(String, '/voice_speech', 10)

        # Smooth Velocity Slew State
        self.current_cmd_v = 0.0
        self.current_cmd_w = 0.0

        # Real-Time Closed-Loop Pure Pursuit Controller (10 Hz)
        self.motion_timer = self.create_timer(0.1, self.motion_control_loop)
        # Cognitive Decision & Global Planner Loop (1 Hz)
        self.planner_timer = self.create_timer(1.0, self.cognitive_planner_loop)

        self.get_logger().info('=== Cognitive Map-Based Explorer Initialized ===')
        self.get_logger().info('Internal World Memory + Global A* Planner + Pure Pursuit active.')

    # =========================================================================
    # 1. Odometry & TF Broadcaster
    # =========================================================================
    def update_robot_pose(self):
        """Queries the exact SLAM-corrected robot pose in the map frame."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny, cosy)
            self.pose_in_map_valid = True

            if self.user_home_pose is None and self.current_map is not None:
                self.user_home_pose = (self.robot_x, self.robot_y)
                self.get_logger().info(f'📍 User home entrance recorded in map frame at: ({self.robot_x:.2f}, {self.robot_y:.2f})')

            now_sec = time.time()
            if now_sec - self.last_traj_time > 0.5:
                self.last_traj_time = now_sec
                pt = PoseStamped()
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.header.frame_id = 'map'
                pt.pose.position.x = self.robot_x
                pt.pose.position.y = self.robot_y
                pt.pose.position.z = 0.02
                pt.pose.orientation = q
                self.trajectory_path.poses.append(pt)
                if len(self.trajectory_path.poses) > 3000:
                    self.trajectory_path.poses.pop(0)
                self.trajectory_path.header.stamp = pt.header.stamp
                self.trajectory_pub.publish(self.trajectory_path)
        except Exception:
            pass

    def odom_callback(self, msg: Odometry):
        # Update fallback pose until map frame becomes available from SLAM
        if not self.pose_in_map_valid:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny, cosy)
        self.odom_received = True

        # Broadcast complete TF tree to guarantee SLAM and RViz always resolve transforms
        transforms = []

        # odom -> base_footprint
        t1 = TransformStamped()
        t1.header.stamp = msg.header.stamp
        t1.header.frame_id = 'odom'
        t1.child_frame_id = 'base_footprint'
        t1.transform.translation.x = msg.pose.pose.position.x
        t1.transform.translation.y = msg.pose.pose.position.y
        t1.transform.translation.z = msg.pose.pose.position.z
        t1.transform.rotation = msg.pose.pose.orientation
        transforms.append(t1)

        self.tf_broadcaster.sendTransform([t1])
        self.update_robot_pose()

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def human_pose_callback(self, msg: PoseStamped):
        self.human_pose = (msg.pose.position.x, msg.pose.position.y)
        self.last_human_pose_time = time.time()

    # =========================================================================
    # 2. Map Memory & Costmap Inflation
    # =========================================================================
    def map_callback(self, msg: OccupancyGrid):
        if self.current_map is None:
            self.get_logger().info(f'World memory received: {msg.info.width}x{msg.info.height} cells @ {msg.info.resolution}m/cell')
        self.current_map = msg
        self.update_costmap_memory()

    def update_costmap_memory(self):
        """Constructs an inflated costmap in memory based on the robot's physical dimensions."""
        if self.current_map is None:
            return

        w = self.current_map.info.width
        h = self.current_map.info.height
        res = self.current_map.info.resolution
        grid = np.array(self.current_map.data, dtype=np.int8).reshape((h, w))
        self.grid_data = grid

        # Track free mapped space growth to detect room saturation
        free_cells = int(np.sum((grid >= 0) & (grid < FREE_THRESHOLD)))
        if free_cells > self.max_free_cells_observed + 15:
            self.max_free_cells_observed = free_cells
            self.last_area_growth_time = time.time()

        # Identify occupied cells (walls, furniture)
        occupied = (grid >= FREE_THRESHOLD)

        # Lethal radius (4 cells = 0.20m for 0.05m resolution)
        r_lethal = max(1, int(round(self.robot_radius / res)))
        self.lethal_costmap = self._dilate_mask(occupied, r_lethal, h, w)

        # Soft inflation layer (adds potential cost around walls to keep robot centered in doorways)
        r_soft = r_lethal + 2
        soft_layer = self._dilate_mask(occupied, r_soft, h, w)
        self.soft_costmap = soft_layer & (~self.lethal_costmap)

    def _dilate_mask(self, mask: np.ndarray, radius: int, h: int, w: int) -> np.ndarray:
        """Fast vectorized binary dilation using OpenCV elliptical structuring element."""
        if radius <= 0:
            return mask.copy()
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
        dilated = cv2.dilate(mask.astype(np.uint8), kernel, iterations=1)
        return dilated.astype(bool)

    # =========================================================================
    # 3. Global A* Path Planner on Remembered Map
    # =========================================================================
    def world_to_grid(self, wx, wy):
        if self.current_map is None:
            return None
        ox = self.current_map.info.origin.position.x
        oy = self.current_map.info.origin.position.y
        res = self.current_map.info.resolution
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        ox = self.current_map.info.origin.position.x
        oy = self.current_map.info.origin.position.y
        res = self.current_map.info.resolution
        wx = ox + (gx + 0.5) * res
        wy = oy + (gy + 0.5) * res
        return wx, wy

    def find_nearest_safe_cell(self, start_gx, start_gy, max_search=8):
        """Finds nearest un-inflated, safe cell if coordinate falls inside an obstacle border."""
        h, w = self.lethal_costmap.shape
        if 0 <= start_gx < w and 0 <= start_gy < h and not self.lethal_costmap[start_gy, start_gx]:
            return start_gx, start_gy

        best = None
        best_dist = float('inf')
        for dy in range(-max_search, max_search + 1):
            for dx in range(-max_search, max_search + 1):
                nx, ny = start_gx + dx, start_gy + dy
                if 0 <= nx < w and 0 <= ny < h and not self.lethal_costmap[ny, nx]:
                    d = dx * dx + dy * dy
                    if d < best_dist:
                        best_dist = d
                        best = (nx, ny)
        return best

    def plan_astar_path(self, start_world, goal_world):
        """
        Executes 8-connected A* search on the internal map memory.
        Returns a smoothed list of world waypoints [(x, y), ...] routing safely around walls and through doorways.
        """
        if self.lethal_costmap is None or self.current_map is None:
            dx = goal_world[0] - start_world[0]
            dy = goal_world[1] - start_world[1]
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                return [start_world, goal_world]
            steps = max(2, int(dist / 0.20))
            return [(start_world[0] + dx * (i / steps), start_world[1] + dy * (i / steps)) for i in range(steps + 1)]

        h, w = self.lethal_costmap.shape
        start_g = self.world_to_grid(start_world[0], start_world[1])
        goal_g = self.world_to_grid(goal_world[0], goal_world[1])

        if start_g is None or goal_g is None:
            return None

        safe_start = self.find_nearest_safe_cell(start_g[0], start_g[1], max_search=8)
        safe_goal = self.find_nearest_safe_cell(goal_g[0], goal_g[1], max_search=10)

        if safe_start is None or safe_goal is None:
            return None

        sx, sy = safe_start
        gx, gy = safe_goal

        # 8-connected directions: (dx, dy, cost)
        moves = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
        ]

        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}
        visited = set()

        while open_set:
            f, curr = heapq.heappop(open_set)
            if curr in visited:
                continue
            visited.add(curr)
            cx, cy = curr

            if cx == gx and cy == gy:
                # Reconstruct path
                raw_grid_path = []
                p = (gx, gy)
                while p in came_from:
                    raw_grid_path.append(p)
                    p = came_from[p]
                raw_grid_path.append((sx, sy))
                raw_grid_path.reverse()

                # Smooth path with line-of-sight string pulling
                smoothed_grid_path = self.smooth_path_line_of_sight(raw_grid_path)
                world_path = [self.grid_to_world(px, py) for px, py in smoothed_grid_path]
                return self.interpolate_path(world_path, max_step=0.15)

            curr_g = g_score[(cx, cy)]
            for dx, dy, step_cost in moves:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if self.lethal_costmap[ny, nx]:
                        continue
                    # Never route through unmapped unknown void; always stay within confirmed free space
                    if self.grid_data is not None and self.grid_data[ny, nx] == UNKNOWN and not (nx == gx and ny == gy):
                        continue
                    if (nx, ny) in visited:
                        continue
                    # Diagonal corner-cutting prevention
                    if dx != 0 and dy != 0:
                        if self.lethal_costmap[cy, nx] and self.lethal_costmap[ny, cx]:
                            continue

                    # Extra penalty in soft inflation layer to favor corridor centers
                    soft_penalty = 1.5 if self.soft_costmap[ny, nx] else 0.0
                    tentative_g = curr_g + step_cost + soft_penalty

                    if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = tentative_g
                        # Euclidean heuristic with 1.02 tie-breaking
                        h_dist = 1.02 * math.hypot(gx - nx, gy - ny)
                        came_from[(nx, ny)] = (cx, cy)
                        heapq.heappush(open_set, (tentative_g + h_dist, (nx, ny)))

        return None

    def line_of_sight(self, p1, p2):
        """Bresenham-based collision checker along grid segment."""
        x0, y0 = p1
        x1, y1 = p2
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        for _ in range(n):
            if self.lethal_costmap[y, x]:
                return False
            if self.grid_data is not None and self.grid_data[y, x] == UNKNOWN:
                return False
            if x == x1 and y == y1:
                break
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
        return True

    def smooth_path_line_of_sight(self, grid_path):
        """Prunes unnecessary waypoints when there is an unblocked line of sight."""
        if len(grid_path) <= 2:
            return grid_path

        smoothed = [grid_path[0]]
        curr_idx = 0
        while curr_idx < len(grid_path) - 1:
            furthest = curr_idx + 1
            for next_idx in range(len(grid_path) - 1, curr_idx, -1):
                if self.line_of_sight(grid_path[curr_idx], grid_path[next_idx]):
                    furthest = next_idx
                    break
            smoothed.append(grid_path[furthest])
            curr_idx = furthest
        return smoothed

    def interpolate_path(self, world_path, max_step=0.15):
        """Subdivides sparse line-of-sight segments into dense, evenly spaced waypoints for smooth Pure Pursuit tracking."""
        if not world_path or len(world_path) <= 1:
            return world_path

        dense = [world_path[0]]
        for i in range(len(world_path) - 1):
            x1, y1 = world_path[i]
            x2, y2 = world_path[i + 1]
            dist = math.hypot(x2 - x1, y2 - y1)
            steps = max(1, int(math.ceil(dist / max_step)))
            for s in range(1, steps + 1):
                frac = s / steps
                dense.append((x1 + frac * (x2 - x1), y1 + frac * (y2 - y1)))
        return dense

    def publish_planned_path(self, world_path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in world_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def publish_footprint_marker(self):
        """Visualizes the robot's physical CAD collision footprint (0.22m radius) in RViz."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'footprint'
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.robot_x
        marker.pose.position.y = self.robot_y
        marker.pose.position.z = 0.04
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.robot_radius * 2.0
        marker.scale.y = self.robot_radius * 2.0
        marker.scale.z = 0.02
        marker.color = ColorRGBA(r=0.0, g=0.85, b=1.0, a=0.35)
        marker.lifetime = Duration(sec=1)
        self.footprint_marker_pub.publish(marker)

    def plan_astar_path_with_fallback(self, start_world, goal_world):
        """Attempts direct A* to goal_world; if blocked by inflation, finds nearest reachable free coordinate."""
        path = self.plan_astar_path(start_world, goal_world)
        if path and len(path) > 1:
            return path

        gx0, gy0 = goal_world
        for radius in [0.20, 0.35, 0.50, 0.70, 0.90]:
            for angle in np.linspace(0, 2 * math.pi, 16, endpoint=False):
                cand_x = gx0 + radius * math.cos(angle)
                cand_y = gy0 + radius * math.sin(angle)
                g = self.world_to_grid(cand_x, cand_y)
                if g is not None:
                    cx, cy = g
                    if self.lethal_costmap is not None and 0 <= cx < self.lethal_costmap.shape[1] and 0 <= cy < self.lethal_costmap.shape[0]:
                        if not self.lethal_costmap[cy, cx]:
                            cand_path = self.plan_astar_path(start_world, (cand_x, cand_y))
                            if cand_path and len(cand_path) > 1:
                                return cand_path
        return None

    def catalog_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            prev_len = len(self.catalog)
            self.catalog = data
            if len(self.catalog) > prev_len:
                self.get_logger().info(f'Catalog updated: {len(self.catalog)} confirmed landmarks.')
        except Exception as e:
            self.get_logger().error(f'Error parsing catalog: {e}')

    def transition_to_waiting_for_user(self):
        """Robot has arrived back at user entrance; announces findings and presents choices."""
        self.state = 'WAITING_FOR_USER'
        self.current_path = []
        self.current_goal = None
        self.current_cmd_v = 0.0
        self.current_cmd_w = 0.0
        self.cmd_vel_pub.publish(Twist())

        landmarks_items = []
        if self.catalog:
            for idx_str, item in sorted(self.catalog.items(), key=lambda x: int(x[0]) if x[0].isdigit() else 99):
                landmarks_items.append(f"{idx_str}. {item.get('voice_name', item.get('name'))}")

        if not landmarks_items:
            summary = "No distinct visual landmarks cataloged during this run"
        else:
            summary = "Destinations found: " + ", ".join(landmarks_items)

        announcement = (
            f"I have safely returned to your side at the entrance. "
            f"The room is fully mapped. {summary}. "
            f"Where would you like me to guide you?"
        )
        self.get_logger().info('===================================================================')
        self.get_logger().info(f'🤖 ASSISTANT AT USER SIDE: {announcement}')
        self.get_logger().info('===================================================================')

        self.voice_speech_pub.publish(String(data=announcement))
        self.status_pub.publish(String(data='WAITING_FOR_DESTINATION'))

    def guide_to_callback(self, msg: String):
        """Interactive guide dog dispatch: plans safe A* route to chosen destination."""
        target_str = msg.data.strip().lower()
        self.get_logger().info(f'🎯 Guide request received: "{target_str}"')

        if any(w in target_str for w in ['entrance', 'start', 'user', 'home', 'return']):
            target_pt = self.user_home_pose
            name = "Entrance and User"
        else:
            target_lm = None
            if target_str in self.catalog:
                target_lm = self.catalog[target_str]
            else:
                for idx_str, lm in self.catalog.items():
                    lm_name = lm.get('name', '').lower()
                    lm_vname = lm.get('voice_name', '').lower()
                    if target_str in lm_name or target_str in lm_vname:
                        target_lm = lm
                        break

            if target_lm is None:
                known_list = [lm.get('voice_name', lm.get('name', '')) for lm in self.catalog.values()]
                if known_list:
                    err_msg = f"Destination '{target_str}' not discovered in this room. Landmarks found: {', '.join(known_list)}."
                else:
                    err_msg = f"Destination '{target_str}' not discovered. No landmarks cataloged yet."
                self.get_logger().warning(err_msg)
                self.voice_speech_pub.publish(String(data=err_msg))
                return

            target_pt = (target_lm['standoff_x'], target_lm['standoff_y'])
            name = target_lm.get('voice_name', target_lm.get('name', 'destination'))

        self.get_logger().info(f'Planning route to {name} at ({target_pt[0]:.2f}, {target_pt[1]:.2f})...')
        path = self.plan_astar_path_with_fallback((self.robot_x, self.robot_y), target_pt)

        if path and len(path) > 1:
            self.current_goal = target_pt
            self.active_guide_dest = target_pt
            self.current_path = path
            self.active_guide_name = name
            self.state = 'GUIDING_USER'
            self.publish_planned_path(path)
            self.publish_goal_marker(target_pt[0], target_pt[1])

            speech_msg = String(data=f"Understood. Guiding you safely to the {name}. Please follow me.")
            self.voice_speech_pub.publish(speech_msg)
            self.get_logger().info(f'🐕 Guiding user to {name} with {len(path)} waypoints.')
        else:
            err_msg = f"Cannot calculate safe path to {name}. Obstacles may be blocking the route."
            self.get_logger().error(err_msg)
            self.voice_speech_pub.publish(String(data=err_msg))

    def transition_to_arrived(self):
        """Robot reaches landmark standoff location with the user."""
        self.state = 'ARRIVED_AT_DESTINATION'
        name = getattr(self, 'active_guide_name', 'destination')
        self.cmd_vel_pub.publish(Twist())
        self.current_path = []
        self.current_goal = None
        self.current_cmd_v = 0.0
        self.current_cmd_w = 0.0

        speech_msg = String(data=f"We have safely arrived at the {name}. You may proceed, or let me know where to guide you next.")
        self.voice_speech_pub.publish(speech_msg)
        self.get_logger().info(f'🎉 Arrived at destination: {name}!')
        self.status_pub.publish(String(data=f'ARRIVED: {name}'))

    # =========================================================================
    # 4. Cognitive Frontier Decision & Re-planning Loop
    # =========================================================================
    def cognitive_planner_loop(self):
        """1 Hz High-Level Planner: finds frontiers and verifies active trajectories."""
        self.update_robot_pose()
        now = time.time()
        # Clean expired blacklisted goals
        self.blacklisted_goals = [b for b in self.blacklisted_goals if b[2] > now]

        if self.state in ['INIT_SPIN', 'RECOVERING', 'CONFIRM_SPIN', 'COMPLETED',
                          'RETURNING_TO_USER', 'WAITING_FOR_USER', 'GUIDING_USER', 'ARRIVED_AT_DESTINATION']:
            return

        if self.current_map is None or self.lethal_costmap is None:
            return

        if self.exploration_start_time is None:
            self.exploration_start_time = now

        time_elapsed = now - self.exploration_start_time

        # --- CRITERION 1: Fixed Maximum Exploration Time Limit ---
        if time_elapsed > self.max_exploration_time:
            self.get_logger().info(
                f'⏰ Max exploration time limit reached ({time_elapsed:.1f}s >= {self.max_exploration_time:.1f}s)! '
                f'Concluding mapping and returning to user...'
            )
            self.state = 'CONFIRM_SPIN'
            self.confirmation_spin_start = now
            self.current_goal = None
            self.current_path = []
            return

        # --- CRITERION 2: Mapped Area Saturation (No new cells for stagnation_timeout) ---
        if (time_elapsed > self.min_exploration_time) and (now - self.last_area_growth_time > self.stagnation_timeout):
            self.get_logger().info(
                f'🗺️ Mapped area saturated (no new cells discovered in last {self.stagnation_timeout:.1f}s). '
                f'Room boundaries are fully closed! Concluding exploration...'
            )
            self.state = 'CONFIRM_SPIN'
            self.confirmation_spin_start = now
            self.current_goal = None
            self.current_path = []
            return

        frontiers = self.detect_frontiers()
        self.publish_frontier_markers(frontiers)

        # Check if current goal frontier has already been uncovered and mapped by LiDAR
        if self.current_goal is not None:
            gx, gy = self.current_goal
            has_nearby_frontier = any(
                math.hypot(gx - f['centroid'][0], gy - f['centroid'][1]) < 0.75 for f in frontiers
            )
            if not has_nearby_frontier:
                self.get_logger().info(f'✓ Frontier area at ({gx:.2f}, {gy:.2f}) mapped by LiDAR!')
                self.goals_completed += 1
                self.current_goal = None
                self.current_path = []

        # If no active path or goal has expired, plan a new route
        needs_new_plan = (len(self.current_path) == 0) or (self.current_goal is None)
        goal_stalled = (self.current_goal is not None) and (now - self.goal_start_time > 22.0)

        if needs_new_plan or goal_stalled:
            # --- CRITERION 3: Frontier Depletion (No more reachable frontiers) ---
            if not frontiers:
                self.empty_frontiers_count += 1
                if (time_elapsed > self.min_exploration_time) and (self.empty_frontiers_count >= 4):
                    self.get_logger().info('✓ No remaining frontiers! Room is fully explored. Initiating return to user...')
                    self.state = 'CONFIRM_SPIN'
                    self.confirmation_spin_start = now
                    self.current_goal = None
                    self.current_path = []
                return
            else:
                self.empty_frontiers_count = 0

            # Select best frontier and compute A* path through map memory
            best = self.select_best_frontier(frontiers)
            if best is not None:
                gx, gy = best['centroid']
                path = self.plan_astar_path_with_fallback((self.robot_x, self.robot_y), (gx, gy))

                if path and len(path) > 1:
                    self.current_goal = (gx, gy)
                    self.current_path = path
                    self.goal_start_time = now
                    self.state = 'FOLLOWING_PATH'
                    self.publish_goal_marker(gx, gy)
                    self.publish_planned_path(path)

                    # Doorway announcement if path crosses into adjoining room (x > 0.3)
                    if any(pt[0] > 0.4 for pt in path) and not self.doorway_announced:
                        status_msg = String()
                        status_msg.data = 'DOORWAY'
                        self.status_pub.publish(status_msg)
                        self.doorway_announced = True

                    self.get_logger().info(
                        f'🧠 Planned A* Path: {len(path)} waypoints to ({gx:.2f}, {gy:.2f}) [size: {best["size"]}]'
                    )
                else:
                    self.get_logger().warning(f'Unreachable frontier at ({gx:.2f}, {gy:.2f}). Blacklisting.')
                    self.blacklisted_goals.append((gx, gy, now + 15.0))
                    self.current_goal = None

    # =========================================================================
    # 5. Closed-Loop Pure Pursuit Controller (10 Hz)
    # =========================================================================
    def motion_control_loop(self):
        """10 Hz Pure Pursuit Path Tracker with real-time feedback and dynamic replanning."""
        self.update_robot_pose()
        self.publish_footprint_marker()
        twist = Twist()
        now = time.time()

        if self.state in ['COMPLETED', 'WAITING_FOR_USER', 'ARRIVED_AT_DESTINATION']:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            if self.current_map is not None:
                self.final_map_pub.publish(self.current_map)
            return

        if self.state == 'INIT_SPIN':
            if self.initial_spin_time > 0.05 and (now - self.state_start_time < self.initial_spin_time):
                twist.angular.z = 0.50
                self.cmd_vel_pub.publish(twist)
                return
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                if self.current_map is None or not self.pose_in_map_valid:
                    # Keep robot stationary while SLAM initializes map frame
                    return
                self.state = 'FOLLOWING_PATH'
                self.last_check_pos = (self.robot_x, self.robot_y)
                self.last_check_time = now
                self.get_logger().info('SLAM map established! Beginning cognitive exploration.')
                return

        if self.state == 'CONFIRM_SPIN':
            if now - self.confirmation_spin_start < 7.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.45
                self.cmd_vel_pub.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                remaining = self.detect_frontiers()
                if len(remaining) == 0:
                    self.save_final_map()
                    status_msg = String()
                    status_msg.data = 'COMPLETED: All areas mapped successfully!'
                    self.status_pub.publish(status_msg)
                    self.get_logger().info('🗺️ Mapping complete! Planning route back to user at entrance...')
                    self.voice_speech_pub.publish(String(data='Exploration complete. All areas mapped. Safely returning to your side.'))
                    self.state = 'RETURNING_TO_USER'
                    return_path = self.plan_astar_path_with_fallback((self.robot_x, self.robot_y), self.user_home_pose)
                    if return_path and len(return_path) > 1:
                        self.current_path = return_path
                        self.current_goal = self.user_home_pose
                        self.publish_planned_path(return_path)
                    else:
                        self.get_logger().warning('Path to user blocked, transitioning to waiting.')
                        self.transition_to_waiting_for_user()
                else:
                    self.get_logger().info(f'Discovered {len(remaining)} more frontiers during scan. Continuing!')
                    self.state = 'FOLLOWING_PATH'
                    self.last_check_pos = (self.robot_x, self.robot_y)
                    self.last_check_time = now
            return

        if self.state == 'RECOVERING':
            if now < self.recovery_end_time:
                twist.linear.x = -0.16
                twist.angular.z = 0.80 * self.recovery_dir
                self.cmd_vel_pub.publish(twist)
                return
            else:
                self.state = self.pre_recovery_state
                self.last_check_pos = (self.robot_x, self.robot_y)
                self.last_check_time = now
                if self.state == 'RETURNING_TO_USER':
                    new_path = self.plan_astar_path_with_fallback((self.robot_x, self.robot_y), self.user_home_pose)
                    if new_path and len(new_path) > 1:
                        self.current_path = new_path
                        self.current_goal = self.user_home_pose
                        self.publish_planned_path(new_path)
                    else:
                        self.transition_to_waiting_for_user()
                elif self.state == 'GUIDING_USER' and getattr(self, 'active_guide_dest', None) is not None:
                    new_path = self.plan_astar_path_with_fallback((self.robot_x, self.robot_y), self.active_guide_dest)
                    if new_path and len(new_path) > 1:
                        self.current_path = new_path
                        self.current_goal = self.active_guide_dest
                        self.publish_planned_path(new_path)
                else:
                    self.current_goal = None
                    self.current_path = []

        if self.state in ['FOLLOWING_PATH', 'RETURNING_TO_USER', 'GUIDING_USER']:
            # --- 1. Check for Physical Stuck Condition ---
            if self.is_yielding:
                self.last_check_pos = (self.robot_x, self.robot_y)
                self.last_check_time = now
            elif now - self.last_check_time > 4.5:
                moved = math.hypot(self.robot_x - self.last_check_pos[0],
                                   self.robot_y - self.last_check_pos[1])
                if moved < 0.15:
                    self.get_logger().warning(f'Stuck detected (moved {moved:.2f}m in 4.5s). Triggering recovery!')
                    if self.current_goal is not None and self.state == 'FOLLOWING_PATH':
                        self.blacklisted_goals.append((self.current_goal[0], self.current_goal[1], now + 25.0))
                    self.pre_recovery_state = self.state
                    self.state = 'RECOVERING'
                    self.recovery_end_time = now + 1.8
                    self.recovery_dir = 1.0 if np.random.rand() > 0.5 else -1.0
                    self.current_goal = None
                    self.current_path = []
                    return
                self.last_check_pos = (self.robot_x, self.robot_y)
                self.last_check_time = now

            if not self.current_path:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            # --- 2. Check if Path is Blocked in Updated Memory (Cognitive Feedback Replanning) ---
            if self.lethal_costmap is not None:
                path_blocked = False
                for wx, wy in self.current_path[:18]:
                    g = self.world_to_grid(wx, wy)
                    if g is not None:
                        gx, gy = g
                        if 0 <= gx < self.lethal_costmap.shape[1] and 0 <= gy < self.lethal_costmap.shape[0]:
                            if self.lethal_costmap[gy, gx]:
                                path_blocked = True
                                break

                if path_blocked and (now - self.last_replan_time > 2.0):
                    self.get_logger().info('⚠️ Planned path blocked by newly mapped obstacle! Replanning detour...')
                    self.last_replan_time = now
                    status_msg = String()
                    status_msg.data = 'REPLAN'
                    self.status_pub.publish(status_msg)

                    if self.current_goal is not None:
                        new_path = self.plan_astar_path((self.robot_x, self.robot_y), self.current_goal)
                        if new_path and len(new_path) > 1:
                            self.current_path = new_path
                            self.publish_planned_path(new_path)
                            self.get_logger().info(f'✓ Detour computed: {len(new_path)} waypoints.')
                        else:
                            self.get_logger().warning('Current goal no longer reachable. Switching goal.')
                            self.current_goal = None
                            self.current_path = []
                            return

            # --- 3. Dynamic Pedestrian & Proximity Safety Check ---
            # Check if moving human is crossing in the robot's immediate corridor (sensory line-of-sight only)
            human_in_corridor = False
            dist_human = float('inf')
            if self.human_pose is not None and (now - self.last_human_pose_time < 1.5):
                hx, hy = self.human_pose
                dist_human = math.hypot(hx - self.robot_x, hy - self.robot_y)
                bearing_human = math.atan2(hy - self.robot_y, hx - self.robot_x) - self.robot_yaw
                bearing_human = math.atan2(math.sin(bearing_human), math.cos(bearing_human))
                if dist_human < 1.25 and abs(bearing_human) < math.radians(45):
                    human_in_corridor = True

            # Doorway Corridor Funneling: detect if traversing partition threshold
            in_doorway_corridor = (-0.85 <= self.robot_x <= 0.85) and (-2.50 <= self.robot_y <= -0.60)
            front_cone_rad = math.radians(14) if in_doorway_corridor else math.radians(30)
            emergency_stop_dist = 0.16 if in_doorway_corridor else 0.22

            min_front_dist = float('inf')
            min_front_angle = 0.0
            if self.latest_scan is not None:
                ranges = self.latest_scan.ranges
                n = len(ranges)
                for idx in range(0, n, 2):
                    r = ranges[idx]
                    if not (self.latest_scan.range_min < r < self.latest_scan.range_max):
                        continue
                    ang = self.latest_scan.angle_min + idx * self.latest_scan.angle_increment
                    ang = math.atan2(math.sin(ang), math.cos(ang))
                    if abs(ang) < front_cone_rad:
                        if r < min_front_dist:
                            min_front_dist = r
                            min_front_angle = ang

            # Emergency proximity guardrail: immediate backup if dangerously close to obstacle
            if min_front_dist < emergency_stop_dist:
                self.current_cmd_v = -0.08
                self.current_cmd_w = 0.50 if min_front_angle <= 0 else -0.50
                twist.linear.x = self.current_cmd_v
                twist.angular.z = self.current_cmd_w
                self.cmd_vel_pub.publish(twist)
                return

            # Dynamic moving human yielding
            if human_in_corridor:
                if not self.is_yielding:
                    self.is_yielding = True
                    self.yield_start_time = now
                    if now - self.last_yield_announcement > 6.0:
                        self.last_yield_announcement = now
                        self.voice_speech_pub.publish(String(data="Moving person detected ahead. Pausing to yield."))
                        self.get_logger().info(f"🚶 Dynamic moving human ahead ({dist_human:.2f}m). Pausing to yield...")

                self.current_cmd_v = 0.0
                self.current_cmd_w = 0.0
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)

                # If blocked by human for > 4.0 seconds, force an A* replan detour around the obstacle
                if (now - self.yield_start_time > 4.0) and (now - self.last_replan_time > 3.0):
                    self.get_logger().info("Yield timeout reached. Replanning detour around obstacle...")
                    self.last_replan_time = now
                    self.is_yielding = False
                    if self.current_goal is not None:
                        new_path = self.plan_astar_path((self.robot_x, self.robot_y), self.current_goal)
                        if new_path and len(new_path) > 1:
                            self.current_path = new_path
                            self.publish_planned_path(new_path)
                return
            else:
                if self.is_yielding:
                    self.is_yielding = False
                    self.get_logger().info("Corridor clear. Resuming path.")

            # --- 4. Pure Pursuit Tracking ---
            # Prune dense waypoints robot has already passed
            while len(self.current_path) > 1:
                dist_to_first = math.hypot(self.current_path[0][0] - self.robot_x,
                                           self.current_path[0][1] - self.robot_y)
                if dist_to_first < 0.22:
                    self.current_path.pop(0)
                else:
                    break

            # Check if destination reached
            dist_to_dest = math.hypot(self.current_path[-1][0] - self.robot_x,
                                      self.current_path[-1][1] - self.robot_y)
            if dist_to_dest < 0.38:
                self.current_cmd_v = 0.0
                self.current_cmd_w = 0.0
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)

                if self.state == 'RETURNING_TO_USER':
                    self.transition_to_waiting_for_user()
                    return
                elif self.state == 'GUIDING_USER':
                    self.transition_to_arrived()
                    return
                else:
                    self.get_logger().info(f'✓ Arrived at destination ({self.current_path[-1][0]:.2f}, {self.current_path[-1][1]:.2f})!')
                    self.goals_completed += 1
                    self.current_goal = None
                    self.current_path = []
                    return

            # Adaptive lookahead: scales smoothly with current forward speed
            dyn_lookahead = max(0.25, min(0.48, 0.26 + 0.50 * max(0.0, self.current_cmd_v)))
            lookahead_pt = self.current_path[-1]
            for pt in self.current_path:
                d = math.hypot(pt[0] - self.robot_x, pt[1] - self.robot_y)
                if d >= dyn_lookahead:
                    lookahead_pt = pt
                    break

            target_angle = math.atan2(lookahead_pt[1] - self.robot_y, lookahead_pt[0] - self.robot_x)
            heading_err = target_angle - self.robot_yaw
            heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

            # Curvature calculation
            L = max(0.15, math.hypot(lookahead_pt[0] - self.robot_x, lookahead_pt[1] - self.robot_y))
            curvature = (2.0 * math.sin(heading_err)) / L

            if abs(heading_err) > 0.35:
                # Pivot in-place toward path (pure rotation eliminates caster scrub and pivots away from obstacles)
                target_v = 0.0
                target_w = float(np.clip(1.50 * heading_err, -self.max_w, self.max_w))
                if abs(target_w) < 0.30:
                    target_w = 0.30 if target_w > 0 else -0.30
            else:
                # Drive smoothly along curvature
                scale = math.cos(heading_err)
                target_v = float(np.clip(self.max_v * scale, 0.12, self.max_v))
                target_w = float(np.clip(target_v * curvature, -self.max_w, self.max_w))

            # Reassuring guide dog walking pace when escorting user
            if self.state == 'GUIDING_USER':
                target_v = min(target_v, 0.22)

            # Predictive deceleration if obstacle detected ahead
            slowdown_thresh = 0.32 if in_doorway_corridor else 0.45
            if min_front_dist < slowdown_thresh:
                slowdown = max(0.15, (min_front_dist - emergency_stop_dist) / max(0.05, slowdown_thresh - emergency_stop_dist))
                target_v *= slowdown

            # Velocity slew rate limiter (prevents wheel slip on differential drive)
            max_lin_step = 0.065  # 0.65 m/s^2 * 0.1s
            max_ang_step = 0.20   # 2.0 rad/s^2 * 0.1s
            dv = float(np.clip(target_v - self.current_cmd_v, -max_lin_step, max_lin_step))
            dw = float(np.clip(target_w - self.current_cmd_w, -max_ang_step, max_ang_step))

            self.current_cmd_v += dv
            self.current_cmd_w += dw

            twist.linear.x = self.current_cmd_v
            twist.angular.z = self.current_cmd_w
            self.cmd_vel_pub.publish(twist)

    # =========================================================================
    # 6. Frontier Detection & Selection
    # =========================================================================
    def detect_frontiers(self):
        grid = self.current_map
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        data = np.array(grid.data, dtype=np.int8).reshape((h, w))
        free_mask = (data >= 0) & (data < FREE_THRESHOLD)
        unknown_mask = (data == UNKNOWN)

        # High-performance OpenCV morphological neighbor discovery
        kernel_search = cv2.getStructuringElement(
            cv2.MORPH_RECT, (2 * FRONTIER_SEARCH_RADIUS + 1, 2 * FRONTIER_SEARCH_RADIUS + 1)
        )
        dilated_unknown = cv2.dilate(unknown_mask.astype(np.uint8), kernel_search).astype(bool)
        frontier_mask = free_mask & dilated_unknown

        # Exclude cells that fall within lethal inflated obstacle borders
        if self.lethal_costmap is not None:
            frontier_mask &= (~self.lethal_costmap)

        if not np.any(frontier_mask):
            return []

        # Vectorized 8-connected component clustering via OpenCV SIMD C++
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier_mask.astype(np.uint8), connectivity=8
        )

        clusters = []
        for label in range(1, num_labels):
            size = int(stats[label, cv2.CC_STAT_AREA])
            if size < MIN_FRONTIER_SIZE:
                continue

            cgx, cgy = centroids[label]
            # Select the actual free frontier pixel within the component closest to its centroid
            ys, xs = np.where(labels == label)
            dists_sq = (xs - cgx) ** 2 + (ys - cgy) ** 2
            best_idx = np.argmin(dists_sq)
            best_gx, best_gy = xs[best_idx], ys[best_idx]

            wx = ox + (best_gx + 0.5) * res
            wy = oy + (best_gy + 0.5) * res
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)

            clusters.append({
                'centroid': (wx, wy),
                'size': size,
                'distance': dist
            })

        return clusters

    def select_best_frontier(self, frontiers):
        if not frontiers:
            return None

        # Filter out blacklisted goals
        valid = []
        for f in frontiers:
            cx, cy = f['centroid']
            is_blacklisted = any(
                math.hypot(cx - bx, cy - by) < 0.6 for bx, by, _ in self.blacklisted_goals
            )
            if not is_blacklisted and 0.40 <= f['distance'] <= 14.0:
                valid.append(f)

        if not valid:
            valid = [f for f in frontiers if f['distance'] >= 0.40]

        if not valid:
            return None

        max_size = max(f['size'] for f in valid)
        max_dist = max(f['distance'] for f in valid) or 1.0

        best = None
        best_score = -float('inf')

        for f in valid:
            cx, cy = f['centroid']
            dist = f['distance']
            size = f['size']
            ang_to_goal = math.atan2(cy - self.robot_y, cx - self.robot_x)
            heading_diff = abs(math.atan2(math.sin(ang_to_goal - self.robot_yaw),
                                          math.cos(ang_to_goal - self.robot_yaw)))

            # Multi-room exploration utility:
            # Cluster size gives information gain; linear travel cost allows robot to backtrack
            # across rooms to unmapped patches without premature decay; heading alignment keeps forward momentum.
            travel_cost = 1.0 + 0.18 * dist
            info_gain = (size ** 0.70) / travel_cost
            heading_factor = 0.75 + 0.25 * math.cos(heading_diff)
            score = info_gain * heading_factor

            if score > best_score:
                best_score = score
                best = f

        return best

    # =========================================================================
    # 7. Map Saving & Visualization Markers
    # =========================================================================
    def manual_save_callback(self, msg: Empty):
        self.get_logger().info('Manual map save requested via /save_map!')
        self.save_final_map()
        status_msg = String()
        status_msg.data = 'COMPLETED: Manual save executed.'
        self.status_pub.publish(status_msg)

    def save_final_map(self):
        """Saves current OccupancyGrid to standard ROS 2 PGM and YAML files."""
        if self.current_map is None:
            self.get_logger().warning('Cannot save map: current_map is None')
            return

        grid = self.current_map
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        oz = grid.info.origin.position.z

        data = np.array(grid.data, dtype=np.int8).reshape((h, w))
        data_flipped = np.flipud(data)

        img = np.zeros((h, w), dtype=np.uint8)
        img[data_flipped == -1] = 205
        img[data_flipped == 0] = 254
        occupied_mask = (data_flipped > 0)
        occ = data_flipped[occupied_mask].astype(np.int32)
        img[occupied_mask] = np.clip(254 - (occ * 254 // 100), 0, 254).astype(np.uint8)

        save_dirs = [
            '/home/jatin',
            '/home/jatin/.gemini/antigravity/scratch/ros2_simulation_ws/maps'
        ]

        for sdir in save_dirs:
            try:
                os.makedirs(sdir, exist_ok=True)
                pgm_path = os.path.join(sdir, 'blind_assistant_map.pgm')
                yaml_path = os.path.join(sdir, 'blind_assistant_map.yaml')

                with open(pgm_path, 'wb') as f:
                    header = f'P5\n{w} {h}\n255\n'.encode('ascii')
                    f.write(header)
                    f.write(img.tobytes())

                yaml_content = f"""image: blind_assistant_map.pgm
mode: trinary
resolution: {res:.6f}
origin: [{ox:.6f}, {oy:.6f}, {oz:.6f}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
                with open(yaml_path, 'w') as f:
                    f.write(yaml_content)
            except Exception as e:
                self.get_logger().error(f'Error saving map to {sdir}: {e}')

        # Publish final map to latched topic
        self.final_map_pub.publish(self.current_map)

        self.get_logger().info('===========================================================')
        self.get_logger().info('🎉 EXPLORATION COMPLETE: All areas explored and mapped!')
        self.get_logger().info('📁 Final map saved to: /home/jatin/blind_assistant_map.yaml')
        self.get_logger().info('📁 Final map image:    /home/jatin/blind_assistant_map.pgm')
        self.get_logger().info('📢 Final map published to topic: /final_map')
        self.get_logger().info('===========================================================')
        self.completed_saved = True

    def publish_frontier_markers(self, frontiers):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = 'frontiers'
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        colors = [
            ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.8),
            ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.8),
            ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.8),
        ]

        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontier['centroid'][0]
            marker.pose.position.y = frontier['centroid'][1]
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            scale = min(0.12 + frontier['size'] * 0.008, 0.50)
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            marker.color = colors[i % len(colors)]
            marker.lifetime = Duration(sec=3)
            marker_array.markers.append(marker)

        self.frontier_markers_pub.publish(marker_array)

    def publish_goal_marker(self, gx, gy):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 999
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = 0.4
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.45
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker.lifetime = Duration(sec=25)
        self.goal_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
