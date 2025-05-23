#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Float64
import serial
import json
import math
import time
from tf_transformations import quaternion_from_euler

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Create publishers
        self.imu_publisher = self.create_publisher(Imu, '/imu_1', 10)
        self.yaw_publisher = self.create_publisher(Float64, '/imu_1_yaw', 10)

        # Serial connection parameters - try common ports
        # self.serial_ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1']
        self.serial_ports = ['/dev/ttyUSB0']
        self.baud_rate = 115200
        self.serial_connection = None

        # Initialize serial connection
        self.init_serial()

        # Timer for reading serial data
        self.timer = self.create_timer(0.05, self.read_and_publish)  # 20Hz

        self.get_logger().info('IMU Publisher Node Started')
        self.get_logger().info(f'Publishing IMU data to: /imu_1')
        self.get_logger().info(f'Publishing yaw angle to: /imu_1_yaw')

    def init_serial(self):
        """Initialize serial connection to Arduino"""
        for port in self.serial_ports:
            try:
                self.serial_connection = serial.Serial(
                    port,
                    self.baud_rate,
                    timeout=1
                )
                time.sleep(2)  # Wait for Arduino to initialize
                self.get_logger().info(f'Serial connection established on {port}')
                return
            except serial.SerialException:
                continue

        self.get_logger().error('Failed to connect to any serial port')
        self.get_logger().error(f'Tried ports: {self.serial_ports}')
        self.get_logger().error('Make sure Arduino is connected')
        exit(1)

    def read_and_publish(self):
        """Read data from Arduino and publish to ROS2 topic"""
        if not self.serial_connection or not self.serial_connection.is_open:
            self.get_logger().error('Serial connection not available')
            return

        try:
            # Read line from Arduino
            if self.serial_connection.in_waiting > 0:
                line = self.serial_connection.readline().decode('utf-8').strip()

                # Skip non-JSON lines (like status messages)
                if not line.startswith('{'):
                    return

                # Parse JSON data
                try:
                    data = json.loads(line)

                    # Extract yaw values
                    magnetic_yaw = data.get('magnetic_yaw', 0.0)
                    gyro_yaw = data.get('gyro_yaw', 0.0)
                    fused_yaw = data.get('fused_yaw', 0.0)
                    timestamp = data.get('timestamp', 0)

                    # Create and publish IMU message
                    self.publish_imu_data(fused_yaw, magnetic_yaw, gyro_yaw)

                    # Log data occasionally
                    if hasattr(self, 'message_count'):
                        self.message_count += 1
                    else:
                        self.message_count = 1

                    if self.message_count % 40 == 0:  # Every 2 seconds at 20Hz
                        self.get_logger().info(
                            f'Fused Yaw: {fused_yaw:.1f}° | '
                            f'Magnetic: {magnetic_yaw:.1f}° | '
                            f'Gyro: {gyro_yaw:.1f}°'
                        )

                except json.JSONDecodeError:
                    self.get_logger().warn(f'Invalid JSON received: {line}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.init_serial()  # Try to reconnect

    def publish_imu_data(self, fused_yaw, magnetic_yaw, gyro_yaw):
        """Create and publish IMU message"""
        imu_msg = Imu()

        # Header
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Convert yaw (degrees) to quaternion
        # Assuming yaw is rotation around Z-axis, with roll=0, pitch=0
        yaw_rad = math.radians(fused_yaw)
        quat = quaternion_from_euler(0, 0, yaw_rad)

        # Orientation (quaternion)
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        # Orientation covariance (diagonal matrix)
        # Higher values = less confidence
        imu_msg.orientation_covariance[0] = 0.01  # roll
        imu_msg.orientation_covariance[4] = 0.01  # pitch
        imu_msg.orientation_covariance[8] = 0.005 # yaw (more confident)

        # Angular velocity (we don't have reliable angular velocity from this setup)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        # Angular velocity covariance (set to -1 to indicate unknown/unavailable)
        for i in range(9):
            imu_msg.angular_velocity_covariance[i] = -1.0

        # Linear acceleration (not available from this setup)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        # Linear acceleration covariance (set to -1 to indicate unknown/unavailable)
        for i in range(9):
            imu_msg.linear_acceleration_covariance[i] = -1.0

        # Publish the IMU message
        self.imu_publisher.publish(imu_msg)

        # Create and publish yaw angle message
        yaw_msg = Float64()
        yaw_msg.data = fused_yaw
        self.yaw_publisher.publish(yaw_msg)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        imu_publisher = IMUPublisher()
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'imu_publisher' in locals():
            imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
