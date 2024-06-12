# imu_test/height_simulator.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class HeightSimulator(Node):
    def __init__(self):
        super().__init__('height_simulator')
        self.publisher_ = self.create_publisher(Float32, 'height', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time = 0.0

    def timer_callback(self):
        height = 1.0 + 0.5 * math.sin(self.time)
        self.time += 0.1
        msg = Float32()
        msg.data = height
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeightSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
