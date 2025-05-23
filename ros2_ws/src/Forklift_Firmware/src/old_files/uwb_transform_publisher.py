#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import Float32
import math

class UWBTransformPublisher(Node):
    def __init__(self):
        super().__init__('uwb_transform_publisher')
        
        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.yaw_deg = 0.0
        
        # Create subscribers for UWB and IMU data
        self.uwb_subscription = self.create_subscription(
            Point,
            'uwb_xy_coords',
            self.uwb_callback,
            10
        )
        
        self.imu_subscription = self.create_subscription(
            Float32,
            'imu_yaw',
            self.imu_callback,
            10
        )
        
        # Create timer for publishing the transform
        self.timer = self.create_timer(0.05, self.publish_transform)  # 20Hz
        
        self.get_logger().info('UWB Transform Publisher initialized')
        
    def uwb_callback(self, msg):
        """Callback for UWB position data"""
        self.x = msg.x
        self.y = msg.y
        self.get_logger().debug(f'Received UWB position: ({self.x}, {self.y})')
        
    def imu_callback(self, msg):
        """Callback for IMU yaw data"""
        self.yaw_deg = msg.data  # Assuming yaw is in degrees
        self.get_logger().debug(f'Received IMU yaw: {self.yaw_deg}°')
        
    def publish_transform(self):
        """Publish the transform from map to laser_frame based on UWB and IMU data"""
        # Create transform message
        t = TransformStamped()
        
        # Fill in header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'laser_frame'
        
        # Set position
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0  # Assuming 2D operation
        
        # Convert yaw angle to quaternion
        yaw_rad = math.radians(self.yaw_deg)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw_rad / 2.0)
        t.transform.rotation.w = math.cos(yaw_rad / 2.0)
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)
        
        # Log occasionally
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
            if self.log_counter >= 100:  # Log every ~5 seconds at 20Hz
                self.get_logger().info(f'Publishing transform: position=({self.x}, {self.y}), yaw={self.yaw_deg}°')
                self.log_counter = 0
        else:
            self.log_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = UWBTransformPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()