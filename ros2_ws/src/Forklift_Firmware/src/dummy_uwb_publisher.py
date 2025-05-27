#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import readchar
import threading

class DummyUWBPublisher(Node):
    def __init__(self):
        super().__init__('dummy_uwb_publisher')

        # Initial positions and yaw
        self.x_pos = 1.0
        self.y_pos = 1.0
        self.yaw = 0.0  # degrees

        self.step = 0.01    # Step size for position
        self.yaw_step = 5.0 # Step size for yaw
        self.running = True

        # Publishers
        self.xy_publisher = self.create_publisher(Float32MultiArray, '/final_xy', 10)
        self.yaw_publisher = self.create_publisher(Float32, '/imu_1_yaw', 10)

        # Timers for publishing at 10 Hz
        self.create_timer(0.1, self.publish_data)

        # Start keyboard input thread
        self.input_thread = threading.Thread(target=self.keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info('Dummy UWB Publisher Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W/S: X+/- | E/D: Y+/-')
        self.get_logger().info('  A/F: Yaw+/- | Q: Quit')

    def keyboard_input(self):
        while self.running:
            key = readchar.readchar()
            if key == 'w':
                self.x_pos = min(10.0, self.x_pos + self.step)
            elif key == 's':
                self.x_pos = max(-10.0, self.x_pos - self.step)
            elif key == 'e':
                self.y_pos = min(10.0, self.y_pos + self.step)
            elif key == 'd':
                self.y_pos = max(-10.0, self.y_pos - self.step)
            elif key == 'a':
                self.yaw = (self.yaw + self.yaw_step) % 360
            elif key == 'f':
                self.yaw = (self.yaw - self.yaw_step) % 360
            elif key == 'q':
                self.running = False
                rclpy.shutdown()
                return

            # Log position and yaw
            self.get_logger().info(f'X: {self.x_pos:.2f}, Y: {self.y_pos:.2f}, Yaw: {self.yaw:.1f}°')

    def publish_data(self):
        # Publish position
        xy_msg = Float32MultiArray()
        xy_msg.data = [self.x_pos, self.y_pos]
        self.xy_publisher.publish(xy_msg)

        # Publish yaw
        yaw_msg = Float32()
        yaw_msg.data = self.yaw
        self.yaw_publisher.publish(yaw_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyUWBPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
