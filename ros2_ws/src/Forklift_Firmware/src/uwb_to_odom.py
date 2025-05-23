#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import Float64
import math

class UWBToOdom(Node):
    def __init__(self):
        super().__init__('uwb_to_odom')
        
        # Initialize position and yaw variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw_deg = 0.0
        
        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create subscriptions for UWB position and IMU yaw
        self.uwb_subscription = self.create_subscription(
            Point,
            'uwb_fused_xy',
            self.uwb_callback,
            10
        )
        
        self.yaw_subscription = self.create_subscription(
            Float64,
            'imu_1_yaw',
            self.yaw_callback,
            10
        )
        
        # Create a timer for transform broadcast (50Hz)
        self.timer = self.create_timer(0.02, self.broadcast_transform)
        
        self.get_logger().info('UWB to Odom transformation node initialized')
        
    def uwb_callback(self, msg):
        """Update position from UWB data"""
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.get_logger().debug(f'Updated position to ({self.x}, {self.y}, {self.z})')
    
    def yaw_callback(self, msg):
        """Update yaw angle from IMU data"""
        self.yaw_deg = msg.data
        self.get_logger().debug(f'Updated yaw to {self.yaw_deg} degrees')
    
    def broadcast_transform(self):
        """Broadcast the transform from odom to base_link"""
        t = TransformStamped()
        
        # Fill in the header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Set translation (position)
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        
        # Convert yaw from degrees to radians and calculate quaternion
        yaw_rad = math.radians(self.yaw_deg)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw_rad / 2.0)
        t.transform.rotation.w = math.cos(yaw_rad / 2.0)
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        
        # Log position periodically (every 2 seconds)
        if not hasattr(self, 'log_counter'):
            self.log_counter = 0
        self.log_counter += 1
        
        if self.log_counter >= 100:  # 100 * 0.02s = 2s
            self.get_logger().info(
                f'Current position: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}), '
                f'Yaw: {self.yaw_deg:.1f}°'
            )
            self.log_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = UWBToOdom()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()