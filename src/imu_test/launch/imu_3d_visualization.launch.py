from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('imu_test')
    urdf_file = os.path.join(package_dir, 'urdf', 'robotmodel.urdf')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'robotmodel.rviz')

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    imu_3d_visualization_node = Node(
        package='imu_test',
        executable='imu_3d_visualization',
        name='imu_3d_visualization',
        output='screen'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        imu_3d_visualization_node,
        rviz2_node
    ])