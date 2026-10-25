#!/usr/bin/env python3
"""
Interactive Terminal CLI Navigator for Blind Assistant Robot.

Provides a clean, intuitive command-line interface for the user to:
1. Monitor live exploration progress and cataloged landmarks in real time.
2. View spoken audio feedback and status updates from the robot.
3. Command the robot to guide them to any discovered room landmark
   (e.g., 'take me to restroom', 'desk', 'cabinet', 'entrance').
"""

import sys
import threading
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class RoomCliNode(Node):
    def __init__(self):
        super().__init__('room_cli')

        self.catalog = {}
        self.last_status = "INITIALIZING"
        self.last_speech = ""

        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscriptions
        self.catalog_sub = self.create_subscription(
            String, '/room_landmarks', self.catalog_callback, latched_qos)
        self.status_sub = self.create_subscription(
            String, '/exploration_status', self.status_callback, 10)
        self.speech_sub = self.create_subscription(
            String, '/voice_speech', self.speech_callback, 10)

        # Publisher
        self.guide_pub = self.create_publisher(String, '/guide_to', 10)

        self.get_logger().info('Room CLI Navigator Node initialized.')

    def catalog_callback(self, msg: String):
        try:
            self.catalog = json.loads(msg.data)
        except Exception:
            pass

    def status_callback(self, msg: String):
        self.last_status = msg.data

    def speech_callback(self, msg: String):
        self.last_speech = msg.data
        print(f"\n📢 [ASSISTANT SPEAKS]: \"{msg.data}\"")
        sys.stdout.write("\n> ")
        sys.stdout.flush()

    def send_destination(self, dest_text: str):
        msg = String()
        msg.data = dest_text.strip()
        self.guide_pub.publish(msg)


def parse_intent(user_input: str) -> str:
    """Extracts landmark keywords from natural language phrasing."""
    text = user_input.strip().lower()
    # Normalize phrases
    text = text.replace("take me to", "")
    text = text.replace("guide me to", "")
    text = text.replace("navigate to", "")
    text = text.replace("go to", "")
    text = text.replace("show me", "")
    text = text.replace("bring me to", "")
    text = text.replace("the", "")
    text = text.strip()

    if any(k in text for k in ['restroom', 'washroom', 'toilet', 'bathroom']):
        return 'restroom'
    elif any(k in text for k in ['desk', 'table']):
        return 'desk'
    elif any(k in text for k in ['chair', 'seat', 'sitting']):
        return 'chair'
    elif any(k in text for k in ['cabinet', 'cupboard', 'storage']):
        return 'cabinet'
    elif any(k in text for k in ['doorway', 'door', 'room 2']):
        return 'doorway'
    elif any(k in text for k in ['pillar', 'column']):
        return 'pillar'
    elif any(k in text for k in ['exit', 'way out', 'doorway out']):
        return 'exit'
    elif any(k in text for k in ['start', 'entrance', 'home', 'user', 'origin']):
        return 'entrance'
    return text


def print_banner():
    banner = """
╔═══════════════════════════════════════════════════════════════════════════════╗
║                  BLIND ASSISTANT ROBOT — ROOM NAVIGATOR CLI                   ║
║   Interactive Assistive Guide for Discovered Indoor Landmarks & Standoffs    ║
╚═══════════════════════════════════════════════════════════════════════════════╝
"""
    print(banner)


def print_landmarks(catalog: dict):
    print("\n📍 Discovered Room Landmarks:")
    print("-------------------------------------------------------------------------")
    if not catalog:
        print("  (Exploration in progress — no landmarks cataloged yet)")
    else:
        for k, item in sorted(catalog.items(), key=lambda x: int(x[0]) if x[0].isdigit() else 99):
            name = item.get('voice_name', item.get('name', k))
            x = item.get('x', 0.0)
            y = item.get('y', 0.0)
            print(f"  [{k}] {name:<22} (x={x:6.2f}, y={y:6.2f})")
    print("  [0] Entrance / User Home     (x= -2.00, y= -2.00)")
    print("-------------------------------------------------------------------------")


def cli_input_thread(node: RoomCliNode):
    print_banner()
    print_landmarks(node.catalog)
    print("\nCommands:")
    print("  • Type 'take me to <landmark>' (e.g., 'take me to restroom', 'desk', 'cabinet')")
    print("  • Type 'list' or 'landmarks' to refresh the catalog")
    print("  • Type 'status' to check current robot mission status")
    print("  • Type 'quit' or 'q' to exit CLI\n")

    while rclpy.ok():
        try:
            sys.stdout.write("> ")
            sys.stdout.flush()
            user_input = sys.stdin.readline()
            if not user_input:
                break
            cmd = user_input.strip()
            if not cmd:
                continue

            if cmd.lower() in ['quit', 'exit', 'q']:
                print("Exiting CLI Navigator.")
                break
            elif cmd.lower() in ['list', 'landmarks', 'ls']:
                print_landmarks(node.catalog)
                continue
            elif cmd.lower() in ['status', 'st']:
                print(f"\n📊 Current Status: {node.last_status}")
                if node.last_speech:
                    print(f"📢 Last Speech: \"{node.last_speech}\"")
                continue

            target = parse_intent(cmd)
            print(f"\n🧭 Dispatching guide request: '{target}' (from '{cmd}')...")
            node.send_destination(target)

        except (EOFError, KeyboardInterrupt):
            break


def main(args=None):
    rclpy.init(args=args)
    node = RoomCliNode()

    # Spin ROS 2 subscriptions in a background daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        cli_input_thread(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
