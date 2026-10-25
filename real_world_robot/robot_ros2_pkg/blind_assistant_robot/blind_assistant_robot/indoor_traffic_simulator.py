#!/usr/bin/env python3
"""
Indoor Moving Obstacle & Pedestrian Traffic Simulator for Blind Assistant Robot.

Features:
1. Multi-Waypoint Natural 2D Room Patrol:
   - Navigates an authentic, non-linear multi-room loop connecting Room 1, the doorway corridor,
     and Room 2 rather than simply pacing a 1D straight line.
   - Smooth heading steering and forward velocity control driving Gazebo model 'indoor_human'.
2. Realistic Dynamic Interactions:
   - Natural brief pauses (1.8s) at key observation waypoints.
   - Interactive proximity yielding: if the robot approaches within 0.55m, the human pauses
     to prevent physical contact collisions.
3. Multi-Modal Feedback:
   - Publishes velocity commands on /indoor_human/cmd_vel.
   - Publishes ground-truth pose on /indoor_human/pose.
   - Publishes animated 3D markers on /indoor_pedestrian_marker for RViz and SLAM map pop-up.
"""

import random
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


# Natural multi-room walking patrol circuit (x, y, optional pause_duration)
# Preserved for backward compatibility and test validation
PATROL_CIRCUIT = [
    (0.4, -2.0, 0.0),   # 0: Doorway threshold
    (1.6, -2.0, 0.0),   # 1: Room 2 South entrance
    (2.4, -0.8, 1.8),   # 2: Room 2 East (in front of cabinet) - pause to inspect
    (1.8,  1.0, 0.0),   # 3: Room 2 North (hallway near pillar)
    (0.8,  0.4, 0.0),   # 4: Room 2 Center
    (0.4, -1.6, 0.0),   # 5: Returning to doorway
    (-0.6, -2.0, 0.0),  # 6: Room 1 South corridor
    (-1.4, -1.2, 1.8),  # 7: Room 1 central walkway (near entrance) - pause
    (-0.6, -2.0, 0.0),  # 8: Heading back to doorway
]

# Topological room navigation graph for authentic non-linear human wandering
# Coordinates: (x, y)
WANDER_NODES = [
    (0.4, -2.0),    # 0: Doorway threshold
    (1.6, -2.0),    # 1: Room 2 South entrance
    (2.4, -0.8),    # 2: Room 2 East (near cabinet)
    (1.8,  1.0),    # 3: Room 2 North (hallway near pillar)
    (0.8,  0.4),    # 4: Room 2 Center
    (0.4, -1.6),    # 5: Returning to doorway
    (-0.6, -2.0),   # 6: Room 1 South corridor
    (-1.4, -1.2),   # 7: Room 1 central walkway
    (-1.8,  0.5),   # 8: Room 1 North (near desk/chair)
]

# Graph connectivity ensuring pedestrians navigate open corridors and never walk through walls
WANDER_EDGES = {
    0: [1, 5, 6],
    1: [0, 2, 4],
    2: [1, 3, 4],
    3: [2, 4],
    4: [1, 2, 3, 5],
    5: [0, 4, 6],
    6: [0, 5, 7],
    7: [6, 8],
    8: [7, 6],
}


class IndoorTrafficSimulator(Node):
    def __init__(self):
        super().__init__('indoor_traffic_simulator')

        self.declare_parameter('walking_speed', 0.25)
        self.declare_parameter('turn_speed', 0.70)
        self.base_speed = self.get_parameter('walking_speed').get_parameter_value().double_value
        self.speed = self.base_speed
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/indoor_human/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/indoor_human/pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/indoor_pedestrian_marker', 10)

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # State
        self.waypoint_idx = 0
        self.previous_node = -1
        self.pos_x = 0.2
        self.pos_y = -1.4
        self.yaw = 1.57  # Radians
        self.is_pausing = False
        self.pause_start_time = 0.0
        self.current_pause_dur = 0.0
        self.last_update = time.time()

        self.robot_x = -2.0
        self.robot_y = -2.0
        self.has_robot_pose = False

        # 20 Hz control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('=== Indoor Moving Pedestrian Simulator (Dynamic Wanderer) Active ===')
        self.get_logger().info('Human moves organically across rooms with variable speeds and pauses.')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.has_robot_pose = True

    def select_next_waypoint(self):
        """Randomly chooses next destination among reachable topological graph neighbors."""
        if self.previous_node == -1 and self.waypoint_idx == 0:
            # Predictable initial step from doorway threshold into Room 2 South entrance
            self.previous_node = 0
            self.waypoint_idx = 1
            self.speed = 0.25
            self.is_pausing = False
            self.current_pause_dur = 0.0
            return

        neighbors = WANDER_EDGES.get(self.waypoint_idx, [1])
        # Avoid immediate bounce back if alternate paths exist
        candidates = [n for n in neighbors if n != self.previous_node]
        if not candidates:
            candidates = neighbors

        self.previous_node = self.waypoint_idx
        # Pick random candidate
        self.waypoint_idx = random.choice(candidates)

        # Natural variation in speed (0.20 to 0.30 m/s)
        self.speed = random.uniform(0.20, 0.30)

        # 35% chance to take a natural inspection pause (1.5s to 3.0s)
        if random.random() < 0.35:
            self.is_pausing = True
            self.pause_start_time = time.time()
            self.current_pause_dur = random.uniform(1.5, 3.0)
        else:
            self.is_pausing = False
            self.current_pause_dur = 0.0

    def control_loop(self):
        now = time.time()
        dt = min(now - self.last_update, 0.1)
        self.last_update = now
        twist = Twist()

        # Proximity safety check: yield if robot is very close (< 0.55m)
        dist_to_robot = math.hypot(self.pos_x - self.robot_x, self.pos_y - self.robot_y)
        robot_too_close = self.has_robot_pose and (dist_to_robot < 0.55)

        target_x, target_y = WANDER_NODES[self.waypoint_idx]
        dist_to_wp = math.hypot(target_x - self.pos_x, target_y - self.pos_y)

        if robot_too_close:
            # Yield in place
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        elif self.is_pausing:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if now - self.pause_start_time >= self.current_pause_dur:
                self.is_pausing = False

        elif dist_to_wp < 0.35:
            # Arrived at waypoint - transition to next organic destination
            self.select_next_waypoint()
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        else:
            # Steer and move toward target waypoint
            target_angle = math.atan2(target_y - self.pos_y, target_x - self.pos_x)
            heading_err = math.atan2(math.sin(target_angle - self.yaw), math.cos(target_angle - self.yaw))

            if abs(heading_err) > 0.50:
                # Turn in place toward waypoint
                twist.linear.x = 0.0
                twist.angular.z = float(math.copysign(self.turn_speed, heading_err))
                self.yaw += twist.angular.z * dt
            else:
                # Drive forward while gently steering
                cos_factor = max(0.2, math.cos(heading_err))
                twist.linear.x = self.speed * cos_factor
                twist.angular.z = float(min(self.turn_speed, max(-self.turn_speed, 1.4 * heading_err)))
                self.yaw += twist.angular.z * dt
                self.pos_x += twist.linear.x * math.cos(self.yaw) * dt
                self.pos_y += twist.linear.x * math.sin(self.yaw) * dt

            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.cmd_pub.publish(twist)

        # Publish PoseStamped
        stamp = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.pos_x
        pose_msg.pose.position.y = self.pos_y
        pose_msg.pose.position.z = 0.85
        # Quaternion from yaw
        pose_msg.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_pub.publish(pose_msg)

        # Publish 3D Markers for RViz & Visualizer
        self.publish_pedestrian_marker(stamp, robot_too_close)

    def publish_pedestrian_marker(self, stamp, robot_too_close: bool):
        markers = MarkerArray()

        # 1. Torso Cylinder
        torso = Marker()
        torso.header.stamp = stamp
        torso.header.frame_id = 'map'
        torso.ns = 'indoor_pedestrian'
        torso.id = 1
        torso.type = Marker.CYLINDER
        torso.action = Marker.ADD
        torso.pose.position.x = self.pos_x
        torso.pose.position.y = self.pos_y
        torso.pose.position.z = 0.85
        torso.pose.orientation.w = 1.0
        torso.scale.x = 0.44
        torso.scale.y = 0.44
        torso.scale.z = 1.20
        torso.color = ColorRGBA(r=0.95, g=0.12, b=0.12, a=0.95) if not robot_too_close else ColorRGBA(r=1.0, g=0.6, b=0.1, a=0.90)
        markers.markers.append(torso)

        # 2. Head Sphere
        head = Marker()
        head.header.stamp = stamp
        head.header.frame_id = 'map'
        head.ns = 'indoor_pedestrian'
        head.id = 2
        head.type = Marker.SPHERE
        head.action = Marker.ADD
        head.pose.position.x = self.pos_x
        head.pose.position.y = self.pos_y
        head.pose.position.z = 1.55
        head.pose.orientation.w = 1.0
        head.scale.x = 0.30
        head.scale.y = 0.30
        head.scale.z = 0.30
        head.color = ColorRGBA(r=0.88, g=0.70, b=0.58, a=0.95)
        markers.markers.append(head)

        # 3. Floating Text Tag
        text = Marker()
        text.header.stamp = stamp
        text.header.frame_id = 'map'
        text.ns = 'indoor_pedestrian'
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.pos_x
        text.pose.position.y = self.pos_y
        text.pose.position.z = 1.85
        text.scale.z = 0.22
        text.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)
        status_txt = "Moving Obstacle (Yielding)" if robot_too_close else "Moving Obstacle"
        text.text = f"{status_txt} ({self.pos_x:.1f}, {self.pos_y:.1f})"
        markers.markers.append(text)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = IndoorTrafficSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
