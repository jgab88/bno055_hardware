#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class IMU3DVisualization(Node):
    def __init__(self):
        super().__init__('imu_3d_visualization_imu')
        self.br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, 'imu/pose', 10)

        # Subscribe to the filtered odometry topic from the Madgwick filter
        self.odom_sub = self.create_subscription(
            Odometry,
            '/imu/data',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        # Print received odometry message
        self.get_logger().info(f'Received Odometry: {msg}')
        
        # Extract the orientation and position from the odometry message
        orientation = msg.pose.pose.orientation
        position = msg.pose.pose.position

        # Create a TransformStamped message for the IMU pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z
        t.transform.rotation = orientation

        # Print TransformStamped message
        self.get_logger().info(f'Transform: {t}')
        
        # Broadcast the IMU transform
        self.br.sendTransform(t)

        # Create a PoseStamped message for the IMU pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position = position
        pose.pose.orientation = orientation

        # Publish the IMU pose
        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    imu_3d_visualization = IMU3DVisualization()
    rclpy.spin(imu_3d_visualization)
    imu_3d_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
