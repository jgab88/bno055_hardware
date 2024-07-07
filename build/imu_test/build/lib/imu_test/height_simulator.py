#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
import math

class HeightSimulator(Node):
    def __init__(self):
        super().__init__('height_simulator')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.height_pub = self.create_publisher(Float32, 'height', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_height)
        self.time = 0.0

    def publish_height(self):
        height = Float32()
        height.data = 0.5 * math.sin(self.time)
        self.height_pub.publish(height)
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    height_simulator = HeightSimulator()
    rclpy.spin(height_simulator)
    height_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
