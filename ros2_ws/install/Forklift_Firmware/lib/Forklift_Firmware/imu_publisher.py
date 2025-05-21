#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from time import sleep

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # 10Hz publishing rate
        self.get_logger().info('IMU Publisher Node Started')

    def publish_imu_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # TODO: Add your IMU data reading code here
        # Example:
        # accelerometer_data = read_accelerometer()
        # gyroscope_data = read_gyroscope()
        # magnetometer_data = read_magnetometer()

        # For now, we'll use dummy data
        # Replace these with actual IMU readings
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # Gravity

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        # Set covariance matrices (if known, otherwise leave as zeros)
        msg.linear_acceleration_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.orientation_covariance = [0.0] * 9

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()