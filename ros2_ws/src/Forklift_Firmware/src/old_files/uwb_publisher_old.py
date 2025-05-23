#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import Float32
import math

class UWBPublisher(Node):
    def __init__(self):
        super().__init__('uwb_publisher')
        
        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Parameters with default values
        self.declare_parameter('publish_frequency', 20.0)  # Hz
        self.declare_parameter('use_fake_data', True)  # Set to False to use real data
        
        # Fake data parameters (will be used if use_fake_data is True)
        self.declare_parameter('fake_x', 1.0)
        self.declare_parameter('fake_y', 1.0)
        self.declare_parameter('fake_yaw_degrees', 30.0)
        
        # Get parameter values
        self.use_fake_data = self.get_parameter('use_fake_data').get_parameter_value().bool_value
        self.fake_x = self.get_parameter('fake_x').get_parameter_value().double_value
        self.fake_y = self.get_parameter('fake_y').get_parameter_value().double_value
        self.fake_yaw_deg = self.get_parameter('fake_yaw_degrees').get_parameter_value().double_value
        
        # Current position and orientation (initialized with fake data)
        self.x = self.fake_x
        self.y = self.fake_y
        self.yaw_deg = self.fake_yaw_deg
        
        # Create subscribers for real data (will be used if use_fake_data is False)
        if not self.use_fake_data:
            # Subscribe to UWB position topic
            self.position_subscription = self.create_subscription(
                Point,  # Using Point message type for x,y coordinates
                'uwb_xy_coords',
                self.position_callback,
                10
            )
            
            # Subscribe to IMU yaw topic
            self.yaw_subscription = self.create_subscription(
                Float32,  # Using Float32 for yaw angle in degrees
                'imu_yaw',
                self.yaw_callback,
                10
            )
            
            self.get_logger().info('UWB Publisher initialized with real data subscriptions')
        else:
            self.get_logger().info(f'UWB Publisher initialized with fake data: position=({self.x}, {self.y}), yaw={self.yaw_deg}°')
        
        # Create timer for publishing the transform
        freq = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0/freq, self.publish_transform)
        
    def position_callback(self, msg):
        """Callback for UWB position data"""
        self.x = msg.x
        self.y = msg.y
        self.get_logger().debug(f'Received UWB position: ({self.x}, {self.y})')
        
    def yaw_callback(self, msg):
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
        
        # Periodically log the current position and orientation
        if self.use_fake_data:
            self.get_logger().debug(f'Publishing fake data: position=({self.x}, {self.y}), yaw={self.yaw_deg}°')
        else:
            self.get_logger().debug(f'Publishing real data: position=({self.x}, {self.y}), yaw={self.yaw_deg}°')

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