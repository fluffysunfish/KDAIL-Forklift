#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from rclpy.parameter import Parameter
import math

class UWBPositioning(Node):
    def __init__(self):
        super().__init__('uwb_positioning')
        
        # Declare parameters
        self.declare_parameter('x_position', 1.0)
        self.declare_parameter('y_position', 1.0)
        self.declare_parameter('z_position', 0.0)
        self.declare_parameter('yaw_angle', 0.0)  # in degrees
        self.declare_parameter('publish_frequency', 20.0)  # Hz
        
        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Get initial position from parameters
        self.x = self.get_parameter('x_position').get_parameter_value().double_value
        self.y = self.get_parameter('y_position').get_parameter_value().double_value
        self.z = self.get_parameter('z_position').get_parameter_value().double_value
        yaw_deg = self.get_parameter('yaw_angle').get_parameter_value().double_value
        
        # Calculate quaternion from yaw angle
        yaw_rad = math.radians(yaw_deg)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = math.sin(yaw_rad / 2.0)
        self.qw = math.cos(yaw_rad / 2.0)
        
        # Create a subscription to update position dynamically
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            'uwb_pose',
            self.pose_callback,
            10
        )
        
        # Create a timer with frequency from parameter
        freq = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0/freq, self.broadcast_transform)
        
        self.get_logger().info(f'UWB positioning node initialized at position ({self.x}, {self.y}, {self.z})')
        
    def pose_callback(self, msg):
        """Update position and orientation from incoming PoseStamped message"""
        # Update position
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        
        # Update orientation
        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
        self.qw = msg.pose.orientation.w
        
        self.get_logger().debug(f'Updated position to ({self.x}, {self.y}, {self.z})')
    
    def broadcast_transform(self):
        """Broadcast the transform from map to laser_frame"""
        t = TransformStamped()
        
        # Fill in the header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'laser_frame'
        
        # Set translation (position)
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        
        # Set rotation (orientation)
        t.transform.rotation.x = self.qx
        t.transform.rotation.y = self.qy
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = UWBPositioning()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()