#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Create publisher for IMU yaw angle
        self.imu_publisher = self.create_publisher(
            Float32,
            'imu_yaw',
            10
        )
        
        # Configure publishing rate (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_imu_data)
        
        # Constant yaw angle value (degrees)
        self.yaw_deg = 45.0
        
        self.get_logger().info(f'IMU Publisher initialized with constant yaw angle: {self.yaw_deg}°')
        
    def publish_imu_data(self):
        """Publish constant IMU yaw angle data"""
        # Create Float32 message
        imu_msg = Float32()
        imu_msg.data = self.yaw_deg
        
        # Publish the message
        self.imu_publisher.publish(imu_msg)
        
        # Log periodically
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
            if self.log_counter >= 50:  # Log every ~5 seconds at 10Hz
                self.get_logger().info(f'Publishing IMU yaw angle: {self.yaw_deg}°')
                self.log_counter = 0
        else:
            self.log_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()