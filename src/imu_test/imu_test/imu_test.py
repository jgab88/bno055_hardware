import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class IMUTEST(Node):
    def __init__(self):
        super().__init__('imu_test')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        self.br = TransformBroadcaster(self)

    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    imu_test = IMUTEST()
    rclpy.spin(imu_test)
    imu_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()