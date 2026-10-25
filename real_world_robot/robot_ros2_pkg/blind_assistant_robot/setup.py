import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'blind_assistant_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jatin',
    maintainer_email='jatin@local.host',
    description='Blind Assistant Robot ROS 2 Stack',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arduino_bridge = blind_assistant_robot.arduino_serial_bridge:main',
            'voice_guidance = blind_assistant_robot.voice_guidance:main',
            'ydlidar_driver = blind_assistant_robot.ydlidar_x4_driver:main',
            'autonomous_explorer = blind_assistant_robot.autonomous_explorer:main',
            'semantic_vision = blind_assistant_robot.semantic_vision_node:main',
            'indoor_traffic_simulator = blind_assistant_robot.indoor_traffic_simulator:main',
            'room_cli = blind_assistant_robot.room_cli:main',
            'slam_map_visualizer = blind_assistant_robot.slam_map_visualizer:main',
            'webcam_lidar_fusion = blind_assistant_robot.webcam_lidar_fusion_node:main',
            'webcam_publisher = blind_assistant_robot.webcam_publisher_node:main',
        ],
    },
)
