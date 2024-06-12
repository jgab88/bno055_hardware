import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

class SetFrameIdNode(Node):
    def __init__(self):
        super().__init__('set_frame_id_node')
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').get_parameter_value().string_value

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)

        self.mag_subscription = self.create_subscription(
            MagneticField,
            'imu/mag',
            self.mag_callback,
            10)

        self.imu_publisher = self.create_publisher(Imu, 'imu/data_with_frame_id', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag_with_frame_id', 10)

    def imu_callback(self, msg):
        msg.header.frame_id = self.frame_id
        self.imu_publisher.publish(msg)

    def mag_callback(self, msg):
        msg.header.frame_id = self.frame_id
        self.mag_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SetFrameIdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
