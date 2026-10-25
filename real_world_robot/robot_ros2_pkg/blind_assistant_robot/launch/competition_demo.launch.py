#!/usr/bin/env python3
"""
Competition Demonstration Launch File for Host Laptop.
Executes autonomous room exploration, SLAM mapping, multi-cue vision-LiDAR landmark fusion,
real-time visualizer window, and interactive voice guidance.
Designed for physical hardware deployment over wireless ROS 2 (Wi-Fi).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('blind_assistant_robot')
    slam_dir = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'exploration.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    max_exploration_time = LaunchConfiguration('max_exploration_time', default='120.0')

    # 1. SLAM Toolbox: Real-time mapping from physical /scan and /odom
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items()
    )

    # 2. Multi-Cue Webcam + LiDAR Ray-Casting Fusion Node
    fusion_node = Node(
        package='blind_assistant_robot',
        executable='webcam_lidar_fusion',
        name='webcam_lidar_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'camera_hfov_deg': 68.0,
            'confidence_threshold': 3,
            'show_debug_window': True,
        }]
    )

    # 3. Autonomous Frontier Explorer (Zero prior room knowledge -> returns to start)
    explorer_node = Node(
        package='blind_assistant_robot',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'max_linear_speed': 0.28,
            'max_angular_speed': 0.70,
            'obstacle_influence_dist': 0.60,
            'critical_stop_dist': 0.32,
            'initial_spin_time': 2.0,
            'max_exploration_time': max_exploration_time,
            'min_exploration_time': 25.0,
            'stagnation_timeout': 20.0,
        }]
    )

    # 4. Voice Guidance Node (Audio alerts & feedback)
    voice_node = Node(
        package='blind_assistant_robot',
        executable='voice_guidance',
        name='voice_guidance',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 5. Real-Time SLAM Map Pop-Up Visualizer Window (OpenCV based, zero web deps)
    map_visualizer_node = Node(
        package='blind_assistant_robot',
        executable='slam_map_visualizer',
        name='slam_map_visualizer',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 6. RViz2 GUI for 3D visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true (false for physical hardware)'
        ),
        DeclareLaunchArgument(
            'max_exploration_time',
            default_value='120.0',
            description='Maximum exploration duration in seconds before auto-returning to user'
        ),
        slam_cmd,
        fusion_node,
        explorer_node,
        voice_node,
        map_visualizer_node,
        rviz_node,
    ])
