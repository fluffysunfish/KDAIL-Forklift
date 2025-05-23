#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import threading
import sys
import tty
import termios
import select

class KeyboardIMUPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_imu_publisher')
        
        # Create publisher for IMU yaw angle
        self.imu_publisher = self.create_publisher(
            Float32,
            'imu_yaw',
            10
        )
        
        # Configure publishing rate (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_imu_data)
        
        # Initialize yaw angle (in degrees)
        self.yaw_deg = 0.0
        
        # Step size for angle changes (degrees)
        self.angle_step = 5.0
        
        # Start keyboard listener thread
        self.key_thread = threading.Thread(target=self.get_key)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        self.get_logger().info('Keyboard IMU Publisher initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  R: Increase yaw angle')
        self.get_logger().info('  F: Decrease yaw angle')
        self.get_logger().info('  Q: Quit')
        
    def get_key(self):
        """Thread function to get keyboard input"""
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            
            while True:
                # Wait for key press
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    # Process key
                    if key == 'r':  # Increase yaw angle
                        self.yaw_deg += self.angle_step
                        # Normalize to 0-360
                        self.yaw_deg = self.yaw_deg % 360.0
                        self.get_logger().info(f'Yaw increased to: {self.yaw_deg:.1f}°')
                    elif key == 'f':  # Decrease yaw angle
                        self.yaw_deg -= self.angle_step
                        # Normalize to 0-360
                        self.yaw_deg = self.yaw_deg % 360.0
                        self.get_logger().info(f'Yaw decreased to: {self.yaw_deg:.1f}°')
                    elif key == 'q':  # Quit
                        self.get_logger().info('Exiting...')
                        rclpy.shutdown()
                        break
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
    def publish_imu_data(self):
        """Publish IMU yaw angle data"""
        # Create Float32 message
        imu_msg = Float32()
        imu_msg.data = self.yaw_deg
        
        # Publish the message
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardIMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()