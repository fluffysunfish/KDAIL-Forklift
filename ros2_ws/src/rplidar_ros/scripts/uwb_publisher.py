#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class UWBPublisher(Node):
    def __init__(self):
        super().__init__('uwb_publisher')
        
        # Create publisher for UWB coordinates
        self.uwb_publisher = self.create_publisher(
            Point,
            'uwb_xy_coords',
            10
        )
        
        # Configure publishing rate (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_uwb_data)
        
        # Constant UWB position values
        self.x = 1.0
        self.y = 1.0
        
        self.get_logger().info(f'UWB Publisher initialized with constant position: ({self.x}, {self.y})')
        
    def publish_uwb_data(self):
        """Publish constant UWB position data"""
        # Create Point message
        uwb_msg = Point()
        uwb_msg.x = self.x
        uwb_msg.y = self.y
        uwb_msg.z = 0.0
        
        # Publish the message
        self.uwb_publisher.publish(uwb_msg)
        
        # Log periodically
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
            if self.log_counter >= 50:  # Log every ~5 seconds at 10Hz
                self.get_logger().info(f'Publishing UWB position: ({self.x}, {self.y})')
                self.log_counter = 0
        else:
            self.log_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = UWBPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()