import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('blind_assistant_robot')

    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyACM0')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')

    # 1. Robot State Publisher (URDF + TF)
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 2. Arduino Serial Bridge (Motors & Encoders)
    arduino_node = Node(
        package='blind_assistant_robot',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'port': arduino_port,
            'baudrate': 115200,
            'wheel_radius': 0.033,
            'wheel_separation': 0.170,
            'ticks_per_rev': 360,
            'max_speed': 0.45,
            'publish_tf': True
        }]
    )

    # 3. YDLidar X4 Driver
    lidar_node = Node(
        package='blind_assistant_robot',
        executable='ydlidar_driver',
        name='ydlidar_driver',
        output='screen',
        parameters=[{
            'port': lidar_port,
            'baudrate': 128000,
            'frame_id': 'laser_frame',
            'range_min': 0.12,
            'range_max': 10.0
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino Uno'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for YDLidar X4'
        ),
        rsp_cmd,
        arduino_node,
        lidar_node
    ])
