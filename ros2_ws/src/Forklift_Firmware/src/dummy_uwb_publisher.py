#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class DummyUWBPublisher(Node):
    def __init__(self):
        super().__init__('dummy_uwb_publisher')

        # Create publisher for UWB data
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/UWB_xy_Data',
            10
        )

        # Create timer for publishing at 10Hz
        self.timer = self.create_timer(0.1, self.publish_uwb_data)
        self.get_logger().info('Dummy UWB Publisher Started')

    def publish_uwb_data(self):
        msg = Float32MultiArray()
        msg.data = [1.0, 1.0]  # x=0, y=0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    dummy_uwb_publisher = DummyUWBPublisher()

    try:
        rclpy.spin(dummy_uwb_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        dummy_uwb_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
