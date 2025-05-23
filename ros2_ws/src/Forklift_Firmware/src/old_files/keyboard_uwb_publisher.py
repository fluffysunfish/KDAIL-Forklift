#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import threading
import sys
import tty
import termios
import select

class KeyboardUWBPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_uwb_publisher')
        
        # Create publisher for UWB coordinates
        self.uwb_publisher = self.create_publisher(
            Point,
            'uwb_xy_coords',
            10
        )
        
        # Configure publishing rate (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_uwb_data)
        
        # Initialize UWB position values
        self.x = 0.0
        self.y = 0.0
        
        # Step size for position changes
        self.step_size = 0.1
        
        # Start keyboard listener thread
        self.key_thread = threading.Thread(target=self.get_key)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        self.get_logger().info('Keyboard UWB Publisher initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W/S: Increase/decrease X coordinate')
        self.get_logger().info('  E/D: Increase/decrease Y coordinate')
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
                    if key == 'w':  # Increase X
                        self.x += self.step_size
                        self.get_logger().info(f'X increased to: {self.x:.2f}')
                    elif key == 's':  # Decrease X
                        self.x -= self.step_size
                        self.get_logger().info(f'X decreased to: {self.x:.2f}')
                    elif key == 'e':  # Increase Y
                        self.y += self.step_size
                        self.get_logger().info(f'Y increased to: {self.y:.2f}')
                    elif key == 'd':  # Decrease Y
                        self.y -= self.step_size
                        self.get_logger().info(f'Y decreased to: {self.y:.2f}')
                    elif key == 'q':  # Quit
                        self.get_logger().info('Exiting...')
                        rclpy.shutdown()
                        break
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
    def publish_uwb_data(self):
        """Publish UWB position data"""
        # Create Point message
        uwb_msg = Point()
        uwb_msg.x = self.x
        uwb_msg.y = self.y
        uwb_msg.z = 0.0
        
        # Publish the message
        self.uwb_publisher.publish(uwb_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardUWBPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()