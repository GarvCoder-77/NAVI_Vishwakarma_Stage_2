import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('blind_assistant_robot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    world_file = os.path.join(pkg_dir, 'worlds', 'indoor_room.sdf')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # Ensure Gazebo Harmonic can locate mesh resources
    pkg_share_parent = os.path.dirname(pkg_dir)
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + pkg_share_parent
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = pkg_share_parent

    # =====================================================================
    # 1. Gazebo Simulator with Indoor Room World
    # =====================================================================
    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v1 ', world_file]}.items()
    )

    # =====================================================================
    # 2. Robot State Publisher (URDF + TF tree)
    # =====================================================================
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # =====================================================================
    # 3. Spawn Robot into Gazebo Room
    # =====================================================================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'indoor_room',
            '-name', 'blind_assistant_robot',
            '-topic', 'robot_description',
            '-x', '-2.0',
            '-y', '-2.0',
            '-z', '0.05'
        ]
    )

    # =====================================================================
    # 4. ROS-Gazebo Bridge (sensor, camera & command topics)
    # =====================================================================
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/indoor_human/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/camera/image', '/camera/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ],
        parameters=[{'use_sim_time': True}]
    )

    # =====================================================================
    # 5. Autonomous Frontier Explorer — the brain that drives exploration
    #    (Starts immediately to broadcast odom -> base_footprint TF)
    # =====================================================================
    explorer_node = Node(
        package='blind_assistant_robot',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_linear_speed': 0.30,
            'max_angular_speed': 0.80,
            'obstacle_influence_dist': 0.65,
            'critical_stop_dist': 0.35,
            'initial_spin_time': 0.0,
        }]
    )

    # =====================================================================
    # 6. SLAM Toolbox — real-time mapping (Delayed 3.5s so TF tree is warm)
    # =====================================================================
    slam_dir = get_package_share_directory('slam_toolbox')
    slam_cmd = TimerAction(
        period=3.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_dir, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file': slam_params,
                }.items()
            )
        ]
    )

    # =====================================================================
    # 7. Semantic Vision Node — camera object recognition & 3D landmarks
    # =====================================================================
    vision_node = Node(
        package='blind_assistant_robot',
        executable='semantic_vision',
        name='semantic_vision_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # =====================================================================
    # 8. Voice Guidance — audio announcements
    # =====================================================================
    voice_node = Node(
        package='blind_assistant_robot',
        executable='voice_guidance',
        name='voice_guidance',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # =====================================================================
    # 9. Indoor Traffic Simulator — moving pedestrian pacing across doorway
    # =====================================================================
    traffic_node = Node(
        package='blind_assistant_robot',
        executable='indoor_traffic_simulator',
        name='indoor_traffic_simulator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # =====================================================================
    # 10. Real-Time SLAM Map Pop-Up Visualizer Window
    # =====================================================================
    map_visualizer_node = Node(
        package='blind_assistant_robot',
        executable='slam_map_visualizer',
        name='slam_map_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # =====================================================================
    # 11. RViz2 Visualization
    # =====================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'exploration.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_sim_cmd,
        rsp_cmd,
        spawn_robot,
        bridge_node,
        slam_cmd,
        explorer_node,
        vision_node,
        voice_node,
        traffic_node,
        map_visualizer_node,
        rviz_node,
    ])
