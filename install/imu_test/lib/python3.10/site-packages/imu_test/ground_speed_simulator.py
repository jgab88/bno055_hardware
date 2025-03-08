#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

class GroundSpeedSimulator(Node):
    def __init__(self):
        super().__init__('ground_speed_simulator')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.speed_pub = self.create_publisher(Float32, 'ground_speed', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_speed)
        self.speed = 0.1

    def publish_speed(self):
        speed = Float32()
        speed.data = self.speed
        self.speed_pub.publish(speed)
        # Simulate speed change
        self.speed = 0.5 if self.speed == 0.1 else 0.1

def main(args=None):
    rclpy.init(args=args)
    ground_speed_simulator = GroundSpeedSimulator()
    rclpy.spin(ground_speed_simulator)
    ground_speed_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
