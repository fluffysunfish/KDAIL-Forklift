#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import readchar
import threading
import time

class DummyUWBPublisher(Node):
    def __init__(self):
        super().__init__('dummy_uwb_publisher')

        # Initialize position values
        self.x_pos = 1.0
        self.y_pos = 1.0
        self.step = 0.01  # Step size for each key press
        self.running = True

        # Create publisher for UWB data
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/UWB_xy_Data',
            10
        )

        # Create timer for publishing at 10Hz
        self.timer = self.create_timer(0.1, self.publish_uwb_data)

        # Start keyboard input thread
        self.input_thread = threading.Thread(target=self.keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info('Dummy UWB Publisher Started')
        self.get_logger().info('Use W/S for X axis and E/D for Y axis')
        self.get_logger().info('Press Q to quit')

    def keyboard_input(self):
        while self.running:
            key = readchar.readchar()
            if key == 'w':  # Increase X
                self.x_pos = min(1.0, self.x_pos + self.step)
                self.get_logger().info(f'X: {self.x_pos:.1f}, Y: {self.y_pos:.1f}')
            elif key == 's':  # Decrease X
                self.x_pos = max(0.0, self.x_pos - self.step)
                self.get_logger().info(f'X: {self.x_pos:.1f}, Y: {self.y_pos:.1f}')
            elif key == 'e':  # Increase Y
                self.y_pos = min(1.0, self.y_pos + self.step)
                self.get_logger().info(f'X: {self.x_pos:.1f}, Y: {self.y_pos:.1f}')
            elif key == 'd':  # Decrease Y
                self.y_pos = max(0.0, self.y_pos - self.step)
                self.get_logger().info(f'X: {self.x_pos:.1f}, Y: {self.y_pos:.1f}')
            elif key == 'q':  # Quit
                self.running = False
                rclpy.shutdown()

    def publish_uwb_data(self):
        msg = Float32MultiArray()
        msg.data = [self.x_pos, self.y_pos]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    dummy_uwb_publisher = DummyUWBPublisher()

    try:
        rclpy.spin(dummy_uwb_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        dummy_uwb_publisher.running = False
        dummy_uwb_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
