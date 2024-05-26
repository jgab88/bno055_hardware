import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

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

    def mag_callback(self, msg):
        # TODO: Implement magnetometer data handling if needed
        pass

def main(args=None):
    rclpy.init(args=args)
    imu_3d_visualization = IMU3DVisualization()
    rclpy.spin(imu_3d_visualization)
    imu_3d_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()