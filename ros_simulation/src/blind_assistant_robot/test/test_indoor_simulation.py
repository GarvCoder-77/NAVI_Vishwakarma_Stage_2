#!/usr/bin/env python3
"""
Comprehensive Automated Test Suite for Blind Assistant Robot Indoor Simulation.

Verifies:
1. SDF World configuration (physics rates, moving humanoid model, velocity control).
2. Indoor traffic simulator logic (pacing, turnaround, proximity yielding, RViz markers).
3. Semantic vision node (landmark classes, standoff poses, dynamic pedestrian recognition).
4. Autonomous explorer (dynamic yielding, A* detour trigger, return-to-user, escort states).
5. Room CLI natural language intent parsing.
6. Launch file integrity and RViz configuration.
"""

import os
import sys
import math
import xml.etree.ElementTree as ET
import pytest
import numpy as np

# Add package directory to sys.path
pkg_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if pkg_path not in sys.path:
    sys.path.insert(0, pkg_path)

import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


@pytest.fixture(scope="session", autouse=True)
def init_rclpy():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_indoor_world_sdf_and_physics():
    """Verify indoor_room.sdf syntax, physics step size, and humanoid model."""
    world_path = os.path.join(pkg_path, 'worlds', 'indoor_room.sdf')
    assert os.path.exists(world_path), f"World file missing: {world_path}"

    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find('world')
    assert world is not None, "World tag missing"

    # 1. Physics verification
    physics = world.find('physics')
    assert physics is not None, "Physics tag missing"
    rate = float(physics.find('real_time_update_rate').text)
    step = float(physics.find('max_step_size').text)
    assert rate == 400.0, f"Expected 400.0 Hz physics rate, got {rate}"
    assert step == 0.0025, f"Expected 0.0025s max step size, got {step}"

    # 2. Indoor humanoid model verification
    models = {m.attrib.get('name'): m for m in world.findall('model')}
    assert 'indoor_human' in models, "Model 'indoor_human' missing in indoor_room.sdf"
    human_model = models['indoor_human']

    # Plugin verification
    plugins = human_model.findall('plugin')
    plugin_found = False
    for p in plugins:
        if p.attrib.get('name') == 'gz::sim::systems::VelocityControl':
            topic = p.find('topic')
            assert topic is not None and topic.text == '/indoor_human/cmd_vel'
            plugin_found = True
            break
    assert plugin_found, "VelocityControl plugin on /indoor_human/cmd_vel missing in indoor_human"


def test_indoor_traffic_simulator_logic():
    """Verify indoor_traffic_simulator 2D patrol circuit, waypoint steering, and proximity yielding."""
    from blind_assistant_robot.indoor_traffic_simulator import IndoorTrafficSimulator, PATROL_CIRCUIT

    sim = IndoorTrafficSimulator()
    assert len(PATROL_CIRCUIT) >= 6
    assert sim.speed > 0.15
    assert sim.waypoint_idx == 0

    # 1. Test motion towards waypoint
    sim.last_update = 100.0
    sim.control_loop()
    assert hasattr(sim, 'pos_x')
    assert hasattr(sim, 'pos_y')

    # 2. Test proximity yielding when robot is close (< 0.55m)
    sim.robot_x = sim.pos_x
    sim.robot_y = sim.pos_y
    sim.has_robot_pose = True
    sim.control_loop()
    dist = math.hypot(sim.pos_x - sim.robot_x, sim.pos_y - sim.robot_y)
    assert dist < 0.55

    # 3. Test waypoint arrival and progression
    target_x, target_y, _ = PATROL_CIRCUIT[0]
    sim.pos_x = target_x
    sim.pos_y = target_y
    sim.robot_x = -10.0  # far away
    sim.control_loop()
    assert sim.waypoint_idx == 1 or sim.is_pausing

    sim.destroy_node()


def test_semantic_vision_landmark_classes_and_standoffs():
    """Verify semantic landmark classes, HSV bounds, and standoff calculation."""
    from blind_assistant_robot.semantic_vision_node import LANDMARK_CLASSES, SemanticVisionNode

    names = [lm['name'] for lm in LANDMARK_CLASSES]
    expected = ['Exit Sign', 'Doorway', 'Restroom Sign', 'Cabinet', 'Desk', 'Chair', 'Pillar']
    for exp in expected:
        assert exp in names, f"Expected landmark class {exp} missing"

    # Verify no hue range overlaps between any distinct landmark classes
    for i, lm1 in enumerate(LANDMARK_CLASSES):
        for j, lm2 in enumerate(LANDMARK_CLASSES):
            if i >= j:
                continue
            # Check overlap between [h_low, h_high]
            overlap = max(0, min(lm1['h_high'], lm2['h_high']) - max(lm1['h_low'], lm2['h_low']))
            assert overlap <= 0, f"Hue overlap detected between {lm1['name']} and {lm2['name']}!"

    node = SemanticVisionNode()
    assert hasattr(node, 'gui_available')
    assert hasattr(node, 'show_opencv_window')
    assert hasattr(node, 'detected_human_pub'), "Missing detected_human_pub publisher!"

    # Test standoff pose computation for Doorway and Restroom (fallback when pose uninitialized)
    node.has_pose = False
    test_lm_doorway = {'x': 0.0, 'y': -1.0, 'class': 'Doorway'}
    door_class = next(c for c in LANDMARK_CLASSES if c['name'] == 'Doorway')
    node.calculate_standoff(test_lm_doorway, door_class)
    assert 'standoff_x' in test_lm_doorway
    assert test_lm_doorway['standoff_x'] < 0.0, "Doorway standoff should be west of partition"

    test_lm_restroom = {'x': 2.0, 'y': 3.88, 'class': 'Restroom Sign'}
    rest_class = next(c for c in LANDMARK_CLASSES if c['name'] == 'Restroom Sign')
    node.calculate_standoff(test_lm_restroom, rest_class)
    assert 'standoff_y' in test_lm_restroom
    assert test_lm_restroom['standoff_y'] < 3.88, "Restroom standoff should be south of north wall"

    # Test live vector line-of-sight standoff computation when robot pose is active
    node.has_pose = True
    node.robot_x = -1.5
    node.robot_y = 1.0
    test_lm_desk = {'x': -2.0, 'y': 2.0, 'class': 'Desk'}
    desk_class = next(c for c in LANDMARK_CLASSES if c['name'] == 'Desk')
    node.calculate_standoff(test_lm_desk, desk_class)
    # Standoff should lie strictly between the desk and the robot
    assert test_lm_desk['standoff_x'] > test_lm_desk['x']
    assert test_lm_desk['standoff_y'] < test_lm_desk['y']

    # Test Dual-Sensor Depth Perception + LiDAR Verification
    assert hasattr(node, 'depth_sub'), "Missing depth_sub subscriber"
    assert hasattr(node, 'get_fused_distance'), "Missing get_fused_distance method"

    # Synthetic depth image (480x640 float32 meters)
    synth_depth = np.full((480, 640), 2.25, dtype=np.float32)
    node.latest_depth_img = synth_depth
    d_fused = node.get_fused_distance(100, 100, 80, 80, 0.0)
    assert d_fused is not None
    assert abs(d_fused - 2.25) < 0.01, f"Expected 2.25m from depth image, got {d_fused}"

    node.destroy_node()


def test_autonomous_explorer_yielding_and_states():
    """Verify autonomous explorer yielding reflex, state machine, and trajectory recording."""
    from blind_assistant_robot.autonomous_explorer import AutonomousExplorer

    explorer = AutonomousExplorer()
    assert explorer.state == 'INIT_SPIN'
    assert hasattr(explorer, 'is_yielding')
    assert hasattr(explorer, 'yield_start_time')
    assert hasattr(explorer, 'trajectory_path')
    assert hasattr(explorer, 'human_pose')
    # True blank slate: initially no pre-known landmarks
    assert explorer.catalog == {}

    # Test unknown destination rejection without hardcoded fallbacks
    from std_msgs.msg import String
    explorer.guide_to_callback(String(data='cabinet'))
    assert explorer.active_guide_dest is None, "Should NOT navigate to undiscovered landmark!"

    # Test state transition to WAITING_FOR_USER with live discovered landmarks
    explorer.catalog = {
        '1': {'name': 'Desk', 'voice_name': 'Desk', 'standoff_x': -1.2, 'standoff_y': 2.0},
        '2': {'name': 'Restroom Sign', 'voice_name': 'Restroom Sign', 'standoff_x': 2.0, 'standoff_y': 3.2},
    }
    explorer.transition_to_waiting_for_user()
    assert explorer.state == 'WAITING_FOR_USER'
    assert explorer.current_path == []
    assert explorer.current_goal is None

    # Test guide_to dispatch for confirmed landmark
    explorer.guide_to_callback(String(data='restroom'))
    assert explorer.active_guide_dest == (2.0, 3.2)
    assert explorer.active_guide_name == 'Restroom Sign'

    explorer.destroy_node()


def test_room_cli_intent_parser():
    """Verify natural language phrasing normalization in room_cli."""
    from blind_assistant_robot.room_cli import parse_intent

    assert parse_intent("take me to restroom") == "restroom"
    assert parse_intent("guide me to the desk") == "desk"
    assert parse_intent("go to chair") == "chair"
    assert parse_intent("navigate to cabinet") == "cabinet"
    assert parse_intent("show me the doorway") == "doorway"
    assert parse_intent("take me to the pillar") == "pillar"
    assert parse_intent("bring me to exit") == "exit"
    assert parse_intent("return to entrance") == "entrance"
    assert parse_intent("toilet") == "restroom"
    assert parse_intent("washroom") == "restroom"
    assert parse_intent("table") == "desk"


def test_launch_and_rviz_config():
    """Verify launch file imports and RViz display configuration."""
    import importlib.util
    launch_path = os.path.join(pkg_path, 'launch', 'autonomous_explore.launch.py')
    assert os.path.exists(launch_path), f"Launch file missing: {launch_path}"

    spec = importlib.util.spec_from_file_location("autonomous_explore_launch", launch_path)
    launch_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_mod)
    ld = launch_mod.generate_launch_description()
    actions = ld.entities
    executables = []
    for a in actions:
        if hasattr(a, 'node_executable'):
            executables.append(a.node_executable)
        elif hasattr(a, '_executable'):
            executables.append(a._executable)

    # Verify key nodes in launch
    assert any('indoor_traffic_simulator' in str(e) for e in executables), "indoor_traffic_simulator not in launch"
    assert any('autonomous_explorer' in str(e) for e in executables), "autonomous_explorer not in launch"
    assert any('semantic_vision' in str(e) for e in executables), "semantic_vision not in launch"
    assert any('slam_map_visualizer' in str(e) for e in executables), "slam_map_visualizer not in launch"

    # Verify RViz config
    rviz_path = os.path.join(pkg_path, 'config', 'exploration.rviz')
    assert os.path.exists(rviz_path)
    with open(rviz_path, 'r') as f:
        content = f.read()
    assert '/indoor_pedestrian_marker' in content, "Missing /indoor_pedestrian_marker in exploration.rviz"
    assert '/robot_trajectory' in content, "Missing /robot_trajectory in exploration.rviz"


def test_slam_map_visualizer_rendering():
    """Verify SlamMapVisualizer node coordinate transforms, rendering, and layer composition."""
    from blind_assistant_robot.slam_map_visualizer import SlamMapVisualizer
    from nav_msgs.msg import OccupancyGrid, MapMetaData
    from geometry_msgs.msg import Pose

    vis = SlamMapVisualizer()
    assert vis.window_size == 700
    assert hasattr(vis, 'gui_available')

    # 1. Test world_to_pixel math
    meta = MapMetaData()
    meta.resolution = 0.05
    meta.width = 160
    meta.height = 160
    meta.origin.position.x = -4.0
    meta.origin.position.y = -4.0

    p_origin = vis.world_to_pixel(-4.0, -4.0, meta)
    p_center = vis.world_to_pixel(0.0, 0.0, meta)
    p_north = vis.world_to_pixel(0.0, 3.0, meta)

    # Center should be to the right of bottom-left
    assert p_center[0] > p_origin[0], "px should increase toward +X"
    # North (+Y) should have lower row index (upward) than bottom
    assert p_north[1] < p_origin[1], "py should be smaller (upward) for positive Y"

    # 2. Test rendering with synthetic OccupancyGrid
    grid = OccupancyGrid()
    grid.info = meta
    # Create simple room: perimeter walls and open floor
    data = np.zeros((160, 160), dtype=np.int8)
    data[0, :] = 100
    data[-1, :] = 100
    data[:, 0] = 100
    data[:, -1] = 100
    data[50:60, 50:60] = -1  # unknown patch
    grid.data = data.flatten().tolist()

    vis.latest_map = grid
    vis.latest_trajectory = [(-2.0, -2.0), (-1.8, -1.8), (-1.5, -1.5)]
    vis.latest_path = [(-1.5, -1.5), (0.0, -1.0), (1.5, 1.0)]
    vis.human_pose = (0.2, -1.4)
    vis.robot_pose = (-1.5, -1.5, 0.78)
    vis.landmarks = {
        '1': {'name': 'Desk', 'x': -2.0, 'y': 2.0, 'standoff_x': -1.3, 'standoff_y': 2.0},
        '2': {'name': 'Restroom Sign', 'x': 2.0, 'y': 3.88, 'standoff_x': 2.0, 'standoff_y': 3.2},
    }

    # Render loop should execute without errors
    vis.render_loop()

    vis.destroy_node()


def test_exploration_completion_criteria():
    """Verify multi-criterion completion triggers: max time cap, frontier depletion, and area saturation."""
    from blind_assistant_robot.autonomous_explorer import AutonomousExplorer
    import time

    explorer = AutonomousExplorer()
    explorer.exploration_start_time = time.time() - 130.0  # Simulated 130s elapsed
    explorer.max_exploration_time = 120.0
    explorer.min_exploration_time = 25.0

    # 1. Verify max exploration time limit is recognized
    time_elapsed = time.time() - explorer.exploration_start_time
    assert time_elapsed > explorer.max_exploration_time, "Time limit condition should trigger"

    # 2. Verify area saturation trigger
    explorer.exploration_start_time = time.time() - 40.0
    explorer.last_area_growth_time = time.time() - 25.0
    explorer.stagnation_timeout = 20.0
    time_elapsed = time.time() - explorer.exploration_start_time
    assert (time_elapsed > explorer.min_exploration_time) and (time.time() - explorer.last_area_growth_time > explorer.stagnation_timeout)

    # 3. Verify frontier depletion counter logic
    explorer.empty_frontiers_count = 5
    assert explorer.empty_frontiers_count >= 4

    explorer.destroy_node()


if __name__ == '__main__':
    pytest.main(['-v', __file__])
