#!/usr/bin/env python3
"""
Test-Driven Development (TDD) Suite for Monocular Webcam + 2D LiDAR Spatial Fusion.

Verifies:
1. Optical bearing angle calculation across image resolutions and field of views.
2. 2D LiDAR ray-cast extraction with foreground clustering and background wall rejection.
3. 3D spatial map coordinate projection using robot odometry and mounting offsets.
4. ArUco marker dictionary mapping and HSV color class segmentation.
5. Safe standoff pose generation facing the physical landmark.
"""

import math
import numpy as np
import pytest

from blind_assistant_robot.webcam_lidar_fusion_node import (
    calculate_bearing,
    extract_lidar_distance_at_bearing,
    extract_roi_depth_distance,
    project_detection_to_map,
    calculate_standoff_pose,
    LANDMARK_ARUCO_MAP,
    LANDMARK_HSV_CLASSES,
)


def test_bearing_calculation():
    """Verify camera horizontal bearing angle math for center, left, and right pixels."""
    hfov = 1.15  # ~65.89 degrees
    img_w = 640

    # Center pixel -> exactly 0.0 rad bearing
    bearing_center = calculate_bearing(u=320.0, img_width=img_w, hfov=hfov)
    assert abs(bearing_center) < 1e-5

    # Leftmost pixel -> positive bearing (+HFOV/2)
    bearing_left = calculate_bearing(u=0.0, img_width=img_w, hfov=hfov)
    assert abs(bearing_left - (hfov / 2.0)) < 1e-4

    # Rightmost pixel -> negative bearing (-HFOV/2)
    bearing_right = calculate_bearing(u=640.0, img_width=img_w, hfov=hfov)
    assert abs(bearing_right - (-hfov / 2.0)) < 1e-4

    # Halfway between center and right (u = 480) -> -HFOV/4
    bearing_mid_right = calculate_bearing(u=480.0, img_width=img_w, hfov=hfov)
    assert abs(bearing_mid_right - (-hfov / 4.0)) < 1e-4


def test_lidar_distance_extraction_with_background_rejection():
    """Verify LiDAR distance extraction isolates foreground object from background wall."""
    # 360 rays: default range is 4.5m (background room wall)
    ranges = [4.50] * 360
    angle_min = -math.pi
    angle_increment = (2.0 * math.pi) / 360.0

    # Obstacle is at bearing 0.0 rad (forward), distance 1.35m, spanning +/- 4 degrees (+/- 4 beams)
    # Beams around 0.0 rad are index 180 (since min is -pi, 0 rad is at index 180)
    center_idx = 180
    for idx in range(center_idx - 4, center_idx + 5):
        ranges[idx] = 1.35

    # Also add some random background wall noise at 4.2m
    ranges[center_idx + 6] = 4.20

    # Extract distance at bearing 0.0 rad with a window of 0.12 rad (~7 deg)
    dist = extract_lidar_distance_at_bearing(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        bearing_rad=0.0,
        window_rad=0.12,
        range_min=0.12,
        range_max=10.0,
    )

    assert dist is not None
    assert abs(dist - 1.35) < 0.04, f"Expected 1.35m, got {dist}"


def test_3d_spatial_map_projection():
    """Verify 2D-to-3D projection from robot-relative coords to global map coordinates."""
    # Robot is at (x=2.0, y=1.0) facing North (yaw = pi/2)
    robot_x = 2.0
    robot_y = 1.0
    robot_yaw = math.pi / 2.0

    cam_offset_x = 0.062
    cam_offset_y = 0.0

    # Object is 1.50m directly ahead along optical axis (bearing = 0.0)
    measured_dist = 1.50
    bearing = 0.0

    map_x, map_y = project_detection_to_map(
        robot_x=robot_x,
        robot_y=robot_y,
        robot_yaw=robot_yaw,
        measured_dist=measured_dist,
        bearing=bearing,
        cam_offset_x=cam_offset_x,
        cam_offset_y=cam_offset_y,
    )

    # Relative to robot base:
    # rel_x = 0.062 + 1.50 * cos(0) = 1.562
    # rel_y = 0.0
    # In map frame (facing North, +X rotates to +Y):
    # map_x = 2.0 + 1.562 * cos(pi/2) - 0.0 * sin(pi/2) = 2.0
    # map_y = 1.0 + 1.562 * sin(pi/2) + 0.0 * cos(pi/2) = 1.0 + 1.562 = 2.562
    assert abs(map_x - 2.0) < 1e-3, f"Expected map_x=2.0, got {map_x}"
    assert abs(map_y - 2.562) < 1e-3, f"Expected map_y=2.562, got {map_y}"


def test_multicue_aruco_and_color_mappings():
    """Verify ArUco ID dictionary mappings and HSV color boundary definitions."""
    # ArUco ID checks
    assert 0 in LANDMARK_ARUCO_MAP, "ArUco ID 0 (Desk) missing"
    assert 1 in LANDMARK_ARUCO_MAP, "ArUco ID 1 (Chair) missing"
    assert 2 in LANDMARK_ARUCO_MAP, "ArUco ID 2 (Doorway) missing"
    assert 3 in LANDMARK_ARUCO_MAP, "ArUco ID 3 (Restroom Sign) missing"
    assert 4 in LANDMARK_ARUCO_MAP, "ArUco ID 4 (Exit Sign) missing"

    # Color class checks
    names = [c['name'] for c in LANDMARK_HSV_CLASSES]
    for required in ['Desk', 'Chair', 'Doorway', 'Restroom Sign', 'Exit Sign', 'Cabinet', 'Pillar']:
        assert required in names, f"Required class {required} missing from LANDMARK_HSV_CLASSES"


def test_standoff_pose_calculation():
    """Verify safe standoff pose generation facing the physical landmark in open space."""
    landmark = {'x': 4.0, 'y': 4.0, 'standoff_offset': 0.70}
    # Robot is at (0, 0)
    robot_pose = (0.0, 0.0)

    standoff_x, standoff_y = calculate_standoff_pose(landmark, robot_pose)

    # Standoff must be between the landmark and the robot
    assert standoff_x < landmark['x'], "Standoff X must be closer to robot than landmark"
    assert standoff_y < landmark['y'], "Standoff Y must be closer to robot than landmark"

    # Distance between standoff and landmark should match standoff_offset
    d_to_lm = math.hypot(standoff_x - landmark['x'], standoff_y - landmark['y'])
    assert abs(d_to_lm - 0.70) < 1e-3, f"Expected 0.70m standoff, got {d_to_lm}"


def test_hp60c_depth_roi_extraction():
    """Verify HP60C RGB-D depth ROI extraction for both 32-bit float and 16-bit uint millimeter depth images."""
    h, w = 480, 640

    # 1. Test 32FC1 (float meters, e.g. simulation or float depth drivers)
    depth_32f = np.full((h, w), 2.45, dtype=np.float32)
    # Bounding box around center: (280, 200, 80, 80)
    d_float = extract_roi_depth_distance(depth_32f, x=280, y=200, bw=80, bh=80)
    assert d_float is not None, "Float depth ROI should return valid distance"
    assert abs(d_float - 2.45) < 1e-3, f"Expected 2.45m, got {d_float}"

    # 2. Test 16UC1 (uint16 millimeters, native format of YDLIDAR HP60C / RealSense)
    # 1850 mm = 1.85 m
    depth_16u = np.full((h, w), 1850, dtype=np.uint16)
    d_uint16 = extract_roi_depth_distance(depth_16u, x=280, y=200, bw=80, bh=80)
    assert d_uint16 is not None, "16-bit depth ROI should return valid distance"
    assert abs(d_uint16 - 1.85) < 1e-3, f"Expected 1.85m, got {d_uint16}"

    # 3. Test empty/None depth image fallback
    assert extract_roi_depth_distance(None, 0, 0, 10, 10) is None
