#!/usr/bin/env python3
import math
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
from sensor_msgs.msg import JointState
from transforms3d.euler import quat2euler

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
        self.br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'imu/pose',
            10)
        self.point_pub = self.create_publisher(PointStamped, 'imu/position', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Initialize variables for position estimation
        self.prev_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.vel_z = 0.0
        self.pos_z = 0.0

    def imu_callback(self, msg):
        # Create the rotational transform from base_link to imu_link_dynamic
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link_dynamic'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation

        # Broadcast the rotational transform
        self.br.sendTransform(t)

        # Extract linear acceleration from the IMU message
        az = msg.linear_acceleration.z

        # Get the current timestamp
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        dt = current_time - self.prev_time

        # Integrate linear acceleration to obtain velocity
        self.vel_z += az * dt

        # Integrate velocity to obtain position
        self.pos_z += self.vel_z * dt

        # Update the previous timestamp
        self.prev_time = current_time

        try:
            transform_imu_dynamic_to_imu = self.tf_buffer.lookup_transform('imu_link_dynamic', 'imu_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Failed to get transform from imu_link_dynamic to imu_link')
            return

        # Create a PointStamped object with the estimated position in the imu_link_dynamic frame
        imu_dynamic_position = PointStamped()
        imu_dynamic_position.header.frame_id = 'imu_link_dynamic'
        imu_dynamic_position.header.stamp = self.get_clock().now().to_msg()
        imu_dynamic_position.point.x = 0.0
        imu_dynamic_position.point.y = 0.0
        imu_dynamic_position.point.z = self.pos_z

        # Transform the position from imu_link_dynamic to imu_link
        imu_position = tf2_geometry_msgs.do_transform_point(imu_dynamic_position, transform_imu_dynamic_to_imu)

        # Create a JointState message and publish the position value
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['imu_joint']
        joint_state_msg.position = [imu_position.point.z]
        self.joint_state_pub.publish(joint_state_msg)

        # Print the estimated position
        self.get_logger().info(f"Estimated position: x={imu_position.point.x}, y={imu_position.point.y}, z={imu_position.point.z}")

        # Create a PoseStamped message and publish the estimated orientation and position in the imu_link frame
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'imu_link'
        pose_msg.pose.position.x = imu_position.point.x
        pose_msg.pose.position.y = imu_position.point.y
        pose_msg.pose.position.z = imu_position.point.z
        pose_msg.pose.orientation = msg.orientation
        self.pose_pub.publish(pose_msg)

        # Create a PointStamped message and publish the estimated position in the imu_link frame
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'imu_link'
        point_msg.point.x = imu_position.point.x
        point_msg.point.y = imu_position.point.y
        point_msg.point.z = imu_position.point.z
        self.point_pub.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_3d_visualization = IMU3DVisualization()
    rclpy.spin(imu_3d_visualization)
    imu_3d_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()