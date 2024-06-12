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
        name='robot_state_publisher_imu',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_imu',
        output='screen'
    )

    imu_3d_visualization_node = Node(
        package='imu_test',
        executable='imu_3d_visualization',
        name='imu_3d_visualization_imu',
        output='screen'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_imu',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_imu',
        parameters=[
            {'world_frame': 'enu'},
            {'use_mag': True},
            {'publish_tf': True},
            {'fixed_frame': 'base_link'}
        ],
    )

    set_frame_id_node = Node(
        package='set_frame_id',
        executable='set_frame_id_node',
        name='set_frame_id_node',
        output='screen',
        parameters=[{'frame_id': 'imu_link'}],
        remappings=[
            ('/imu/data', '/imu/data_with_frame_id'),
            ('/imu/mag', '/imu/mag_with_frame_id'),
        ]
    )

    static_transform_publisher_base_to_imu_dynamic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_imu_dynamic',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link_dynamic']
    )

    static_transform_publisher_imu_dynamic_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_imu_dynamic_to_imu',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link_dynamic', 'imu_link']
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        imu_3d_visualization_node,
        rviz2_node,
        imu_filter_node,
        set_frame_id_node,
        static_transform_publisher_base_to_imu_dynamic,
        static_transform_publisher_imu_dynamic_to_imu
    ])
