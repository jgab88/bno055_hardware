from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'install/imu_test/share/imu_test/rviz/robotmodel.rviz']
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': 'install/imu_test/share/imu_test/urdf/robotmodel.urdf'}]
        ),
        Node(
            package='imu_test',
            executable='height_simulator',
            name='height_simulator',
            output='screen'
        ),
        Node(
            package='imu_test',
            executable='ground_speed_simulator',
            name='ground_speed_simulator',
            output='screen'
        ),
        Node(
            package='imu_test',
            executable='imu_3d_visualization',
            name='imu_3d_visualization',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link_dynamic']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_imu_dynamic_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'imu_link_dynamic', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
    ])
