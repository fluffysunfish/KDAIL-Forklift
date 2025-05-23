#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import math
import time

class SensorDataSimulator(Node):
    def __init__(self):
        super().__init__('sensor_data_simulator')
        
        # Create publishers for simulated sensor data
        self.uwb_publisher = self.create_publisher(
            Point,
            'uwb_xy_coords',
            10
        )
        
        self.imu_publisher = self.create_publisher(
            Float32,
            'imu_yaw',
            10
        )
        
        # Parameters for simulation
        self.declare_parameter('movement_pattern', 'circle')  # Options: 'circle', 'square', 'random'
        self.declare_parameter('speed', 0.2)  # meters per second
        self.declare_parameter('radius', 3.0)  # meters (for circle pattern)
        self.declare_parameter('update_frequency', 10.0)  # Hz
        
        # Get parameters
        self.pattern = self.get_parameter('movement_pattern').get_parameter_value().string_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        update_freq = self.get_parameter('update_frequency').get_parameter_value().double_value
        
        # Create timer for publishing simulated data
        self.timer = self.create_timer(1.0/update_freq, self.publish_simulated_data)
        
        # Starting time for simulation
        self.start_time = time.time()
        
        self.get_logger().info(f'Sensor data simulator started with {self.pattern} pattern')
    
    def publish_simulated_data(self):
        """Generate and publish simulated UWB position and IMU yaw data"""
        elapsed_time = time.time() - self.start_time
        
        # Create messages
        uwb_msg = Point()
        imu_msg = Float32()
        
        # Calculate position and orientation based on movement pattern
        if self.pattern == 'circle':
            # Circle pattern with increasing radius
            angle = (self.speed * elapsed_time) / self.radius
            uwb_msg.x = self.radius * math.cos(angle)
            uwb_msg.y = self.radius * math.sin(angle)
            uwb_msg.z = 0.0
            
            # Orientation tangent to the circle
            imu_msg.data = math.degrees(angle) + 90.0
            
        elif self.pattern == 'square':
            # Square pattern
            side_length = self.radius * 2
            perimeter = 4 * side_length
            position_on_perimeter = (self.speed * elapsed_time) % perimeter
            
            # Determine which side of the square we're on
            side = int(position_on_perimeter / side_length)
            progress = (position_on_perimeter % side_length) / side_length
            
            if side == 0:  # Bottom side (left to right)
                uwb_msg.x = -self.radius + progress * 2 * self.radius
                uwb_msg.y = -self.radius
                imu_msg.data = 0.0
            elif side == 1:  # Right side (bottom to top)
                uwb_msg.x = self.radius
                uwb_msg.y = -self.radius + progress * 2 * self.radius
                imu_msg.data = 90.0
            elif side == 2:  # Top side (right to left)
                uwb_msg.x = self.radius - progress * 2 * self.radius
                uwb_msg.y = self.radius
                imu_msg.data = 180.0
            else:  # Left side (top to bottom)
                uwb_msg.x = -self.radius
                uwb_msg.y = self.radius - progress * 2 * self.radius
                imu_msg.data = 270.0
                
        elif self.pattern == 'random':
            # Simple random walk with smoothing
            import random
            
            # Get previous values or initialize
            if not hasattr(self, 'prev_x'):
                self.prev_x = 0.0
                self.prev_y = 0.0
                self.prev_yaw = 0.0
            
            # Random walk with small steps
            step_size = self.speed * 0.1
            self.prev_x += random.uniform(-step_size, step_size)
            self.prev_y += random.uniform(-step_size, step_size)
            
            # Keep within bounds
            self.prev_x = max(min(self.prev_x, self.radius), -self.radius)
            self.prev_y = max(min(self.prev_y, self.radius), -self.radius)
            
            # Random yaw changes
            yaw_step = 5.0  # degrees
            self.prev_yaw += random.uniform(-yaw_step, yaw_step)
            self.prev_yaw = self.prev_yaw % 360.0
            
            uwb_msg.x = self.prev_x
            uwb_msg.y = self.prev_y
            uwb_msg.z = 0.0
            
            imu_msg.data = self.prev_yaw
        
        # Publish the data
        self.uwb_publisher.publish(uwb_msg)
        self.imu_publisher.publish(imu_msg)
        
        # Log the published data
        self.get_logger().debug(f'Published UWB position: ({uwb_msg.x:.2f}, {uwb_msg.y:.2f}), IMU yaw: {imu_msg.data:.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()