# imu_test/launch/imu_3d_visualization_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_test',
            executable='height_simulator',
            name='height_simulator'
        ),
        Node(
            package='imu_test',
            executable='ground_speed_simulator',
            name='ground_speed_simulator'
        ),
        Node(
            package='imu_test',
            executable='imu_3d_visualization',
            name='imu_3d_visualization'
        ),
    ])
