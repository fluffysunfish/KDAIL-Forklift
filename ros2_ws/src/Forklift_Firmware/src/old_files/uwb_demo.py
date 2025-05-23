#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class UWBPositionDemo(Node):
    def __init__(self):
        super().__init__('uwb_position_demo')
        
        # Publisher for UWB position
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            'uwb_pose',
            10
        )
        
        # Parameters for the demo
        self.declare_parameter('demo_type', 'circle')  # Options: 'circle', 'square', 'figure8'
        self.declare_parameter('speed', 0.5)  # meters per second
        self.declare_parameter('radius', 2.0)  # meters (for circle)
        self.declare_parameter('update_frequency', 10.0)  # Hz
        
        # Get parameters
        self.demo_type = self.get_parameter('demo_type').get_parameter_value().string_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        update_freq = self.get_parameter('update_frequency').get_parameter_value().double_value
        
        # Create timer for position updates
        self.timer = self.create_timer(1.0/update_freq, self.publish_position)
        
        # Starting position
        self.start_time = time.time()
        
        self.get_logger().info(f'UWB demo initialized with {self.demo_type} pattern at {self.speed} m/s')
    
    def publish_position(self):
        """Calculate and publish the current position based on the selected pattern"""
        elapsed_time = time.time() - self.start_time
        
        # Create pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Calculate position based on demo type
        if self.demo_type == 'circle':
            # Circle pattern
            angle = (self.speed * elapsed_time) / self.radius
            pose_msg.pose.position.x = self.radius * math.cos(angle)
            pose_msg.pose.position.y = self.radius * math.sin(angle)
            pose_msg.pose.position.z = 0.0
            
            # Orient towards the center
            yaw = angle + math.pi/2
            pose_msg.pose.orientation.z = math.sin(yaw/2)
            pose_msg.pose.orientation.w = math.cos(yaw/2)
            
        elif self.demo_type == 'square':
            # Square pattern
            side_length = self.radius * 2
            perimeter = 4 * side_length
            position_on_perimeter = (self.speed * elapsed_time) % perimeter
            
            # Determine which side of the square we're on
            side = int(position_on_perimeter / side_length)
            progress = (position_on_perimeter % side_length) / side_length
            
            if side == 0:  # Bottom side (left to right)
                pose_msg.pose.position.x = -self.radius + progress * 2 * self.radius
                pose_msg.pose.position.y = -self.radius
                yaw = 0.0
            elif side == 1:  # Right side (bottom to top)
                pose_msg.pose.position.x = self.radius
                pose_msg.pose.position.y = -self.radius + progress * 2 * self.radius
                yaw = math.pi/2
            elif side == 2:  # Top side (right to left)
                pose_msg.pose.position.x = self.radius - progress * 2 * self.radius
                pose_msg.pose.position.y = self.radius
                yaw = math.pi
            else:  # Left side (top to bottom)
                pose_msg.pose.position.x = -self.radius
                pose_msg.pose.position.y = self.radius - progress * 2 * self.radius
                yaw = 3*math.pi/2
                
            pose_msg.pose.orientation.z = math.sin(yaw/2)
            pose_msg.pose.orientation.w = math.cos(yaw/2)
            
        elif self.demo_type == 'figure8':
            # Figure 8 pattern
            t = elapsed_time * self.speed
            scale = self.radius
            
            pose_msg.pose.position.x = scale * math.sin(t)
            pose_msg.pose.position.y = scale * math.sin(t) * math.cos(t)
            pose_msg.pose.position.z = 0.0
            
            # Calculate orientation tangent to the curve
            dx_dt = scale * math.cos(t)
            dy_dt = scale * (math.cos(t) * math.cos(t) - math.sin(t) * math.sin(t))
            yaw = math.atan2(dy_dt, dx_dt)
            
            pose_msg.pose.orientation.z = math.sin(yaw/2)
            pose_msg.pose.orientation.w = math.cos(yaw/2)
        
        # Publish the pose
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UWBPositionDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()