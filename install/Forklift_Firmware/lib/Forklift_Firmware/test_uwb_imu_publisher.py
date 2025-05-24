#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math
import time

class TestUWBIMUPublisher(Node):
    def __init__(self):
        super().__init__('test_uwb_imu_publisher')
        
        # Publishers
        self.uwb_xy_publisher = self.create_publisher(Point, '/uwb_fused_xy', 10)
        self.imu_yaw_publisher = self.create_publisher(Float64, '/imu_1_yaw', 10)
        
        # Timer for publishing test data
        self.timer = self.create_timer(0.1, self.publish_test_data)  # 10 Hz
        
        # Test parameters for anchor-based coordinate system
        self.start_time = time.time()
        
        # Anchor layout (in cm):
        # A2(0,170) ---- A3(200,170)
        # |                       |
        # |        Robot          |
        # |                       |
        # A1(0,0)   ---- A4(200,0)
        
        # Test path parameters
        self.center_x = 100  # cm - center of rectangular area
        self.center_y = 85   # cm - center of rectangular area
        self.radius = 50     # cm - radius for circular motion
        self.angular_velocity = 0.3  # rad/s for circular motion
        
        self.get_logger().info('Test UWB/IMU publisher started - simulating motion in anchor coordinate system')
        self.get_logger().info('Anchor layout: A1(0,0), A2(0,170), A3(200,170), A4(200,0) in cm')
    
    def publish_test_data(self):
        current_time = time.time() - self.start_time
        
        # Simulate circular motion within the anchor area
        motion_angle = self.angular_velocity * current_time
        
        # UWB position (circular path in anchor coordinate system, in cm)
        uwb_msg = Point()
        uwb_msg.x = self.center_x + self.radius * math.cos(motion_angle)
        uwb_msg.y = self.center_y + self.radius * math.sin(motion_angle)
        uwb_msg.z = 0.0
        
        # IMU yaw (robot orientation)
        # 0° = facing north (towards anchor 2), aligned with positive Y axis
        # Robot faces tangent to the circular path
        imu_msg = Float64()
        imu_msg.data = math.degrees(motion_angle + math.pi/2)  # Convert to degrees, tangent to circle
        
        # Publish messages
        self.uwb_xy_publisher.publish(uwb_msg)
        self.imu_yaw_publisher.publish(imu_msg)
        
        # Log occasionally
        if int(current_time * 10) % 50 == 0:  # Every 5 seconds
            self.get_logger().info(f'Publishing: UWB pos=({uwb_msg.x:.1f}, {uwb_msg.y:.1f})cm, yaw={imu_msg.data:.1f}°')

def main(args=None):
    rclpy.init(args=args)
    node = TestUWBIMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()