#!/usr/bin/env python3
import rclpy
import sys
import tf2_geometry_msgs
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Quaternion, Point, PointStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

class IMU3DVisualization(Node):
    def __init__(self):
        super().__init__('imu_3d_visualization')
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            qos_profile)
        self.mag_subscription = self.create_subscription(
            MagneticField,
            'imu/mag',
            self.mag_callback,
            qos_profile)
        self.br = TransformBroadcaster(self)
        self.mag_pub = self.create_publisher(
            PoseStamped,
            "imu/mag_pose",
            10)
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'imu/pose',
            10)
        self.transformed_pos_pub = self.create_publisher(PointStamped, 'imu/transformed_position', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize variables for position estimation
        self.prev_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.filtered_pos_x = 0.0
        self.filtered_pos_y = 0.0
        self.filtered_pos_z = 0.0

    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link_dynamic'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Extract the orientation from the IMU message
        q = msg.orientation

        # Apply the orientation to the transform
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        self.br.sendTransform(t)

        # Extract linear acceleration from the IMU message
        linear_accel = msg.linear_acceleration
        ax = linear_accel.x
        ay = linear_accel.y
        az = linear_accel.z

        # Get the current timestamp
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        # Compute the time difference between the current and previous timestamp
        dt = current_time - self.prev_time

        # Integrate linear acceleration to obtain velocity
        self.vel_x += ax * dt
        self.vel_y += ay * dt
        self.vel_z += az * dt

        # Integrate velocity to obtain position
        self.pos_x += self.vel_x * dt
        self.pos_y += self.vel_y * dt
        self.pos_z += self.vel_z * dt

        # Update the previous timestamp
        self.prev_time = current_time

        # Apply a high-pass filter to the position estimate
        alpha = 0.1  # Filter coefficient (adjust as needed)
        self.filtered_pos_x = alpha * self.pos_x + (1 - alpha) * self.filtered_pos_x
        self.filtered_pos_y = alpha * self.pos_y + (1 - alpha) * self.filtered_pos_y
        self.filtered_pos_z = alpha * self.pos_z + (1 - alpha) * self.filtered_pos_z

        # Create a PoseStamped message and publish the estimated position and orientation
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = self.filtered_pos_x
        pose_msg.pose.position.y = self.filtered_pos_y
        pose_msg.pose.position.z = self.filtered_pos_z
        pose_msg.pose.orientation = msg.orientation

        self.pose_pub.publish(pose_msg)

        # Get the transform from base_link to imu_link
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'imu_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Failed to get transform from base_link to imu_link')
            return

        # Transform the estimated position from imu_link frame to base_link frame
        base_position_x = (transform.transform.rotation.w * transform.transform.rotation.w + transform.transform.rotation.x * transform.transform.rotation.x - transform.transform.rotation.y * transform.transform.rotation.y - transform.transform.rotation.z * transform.transform.rotation.z) * self.filtered_pos_x + 2 * (transform.transform.rotation.x * transform.transform.rotation.y + transform.transform.rotation.w * transform.transform.rotation.z) * self.filtered_pos_y + 2 * (transform.transform.rotation.x * transform.transform.rotation.z - transform.transform.rotation.w * transform.transform.rotation.y) * self.filtered_pos_z + transform.transform.translation.x
        base_position_y = 2 * (transform.transform.rotation.x * transform.transform.rotation.y - transform.transform.rotation.w * transform.transform.rotation.z) * self.filtered_pos_x + (transform.transform.rotation.w * transform.transform.rotation.w - transform.transform.rotation.x * transform.transform.rotation.x + transform.transform.rotation.y * transform.transform.rotation.y - transform.transform.rotation.z * transform.transform.rotation.z) * self.filtered_pos_y + 2 * (transform.transform.rotation.y * transform.transform.rotation.z + transform.transform.rotation.w * transform.transform.rotation.x) * self.filtered_pos_z + transform.transform.translation.y
        base_position_z = 2 * (transform.transform.rotation.x * transform.transform.rotation.z + transform.transform.rotation.w * transform.transform.rotation.y) * self.filtered_pos_x + 2 * (transform.transform.rotation.y * transform.transform.rotation.z - transform.transform.rotation.w * transform.transform.rotation.x) * self.filtered_pos_y + (transform.transform.rotation.w * transform.transform.rotation.w - transform.transform.rotation.x * transform.transform.rotation.x - transform.transform.rotation.y * transform.transform.rotation.y + transform.transform.rotation.z * transform.transform.rotation.z) * self.filtered_pos_z + transform.transform.translation.z

        # Create a Point object and assign it to the PoseStamped message
        base_position = Point(x=base_position_x, y=base_position_y, z=base_position_z)

        # Create a PoseStamped message and publish the estimated position and orientation
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position = base_position
        pose_msg.pose.orientation = msg.orientation

        self.pose_pub.publish(pose_msg)

        # Create a PointStamped message and publish the transformed position
        transformed_pos_msg = PointStamped()
        transformed_pos_msg.header.stamp = self.get_clock().now().to_msg()
        transformed_pos_msg.header.frame_id = 'base_link'
        transformed_pos_msg.point.x = base_position_x
        transformed_pos_msg.point.y = base_position_y
        transformed_pos_msg.point.z = base_position_z

        self.transformed_pos_pub.publish(transformed_pos_msg)

        print(f"Base Position: x={base_position.x}, y={base_position.y}, z={base_position.z}")
        print(f"Transformed Position: x={transformed_pos_msg.point.x}, y={transformed_pos_msg.point.y}, z={transformed_pos_msg.point.z}")

    def mag_callback(self, msg):
        # Process the magnetometer data
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y
        mag_z = msg.magnetic_field.z

        # Create a pose marker to represent the magnetic field direction
        pose_marker = PoseStamped()
        pose_marker.header.frame_id = "imu_link"
        pose_marker.header.stamp = self.get_clock().now().to_msg()
        pose_marker.pose.position.x = 0.0
        pose_marker.pose.position.y = 0.0
        pose_marker.pose.position.z = 0.0
        pose_marker.pose.orientation.x = 0.0
        pose_marker.pose.orientation.y = 0.0
        pose_marker.pose.orientation.z = mag_z
        pose_marker.pose.orientation.w = 1.0

        # Publish the pose marker
        self.mag_pub.publish(pose_marker)

def main(args=None):
    rclpy.init(args=args)
    imu_3d_visualization = IMU3DVisualization()
    rclpy.spin(imu_3d_visualization)
    imu_3d_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()