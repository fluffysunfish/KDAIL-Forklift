#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import json
import time

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Create publisher for yaw only
        self.yaw_publisher = self.create_publisher(Float64, '/imu_1_yaw', 100)

        # Get user input for port
        print("\nAvailable ports:")
        print("1. /dev/ttyUSB0")
        print("2. /dev/ttyUSB1")
        port_choice = input("Choose port number (1/2): ").strip()
        
        self.serial_ports = ['/dev/ttyUSB0' if port_choice == '1' else '/dev/ttyUSB1']
        self.baud_rate = 115200
        self.serial_connection = None

        # Get yaw offset from user
        try:
            self.yaw_offset = float(input("Enter yaw offset in degrees (e.g. -90): ").strip())
        except ValueError:
            self.get_logger().error('Invalid offset value. Using 0.')
            self.yaw_offset = 0.0

        # Initialize serial connection
        self.init_serial()

        # Timer for reading serial data
        self.timer = self.create_timer(0.01, self.read_and_publish)  # 100Hz

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

                    # Apply offset and wrap to 0-360
                    adjusted_yaw = (fused_yaw + self.yaw_offset) % 360
                    if adjusted_yaw < 0:
                        adjusted_yaw += 360
                    yaw_msg = Float64()
                    yaw_msg.data = adjusted_yaw
                    self.yaw_publisher.publish(yaw_msg)

                    # Log data occasionally
                    if hasattr(self, 'message_count'):
                        self.message_count += 1
                    else:
                        self.message_count = 1

                    if self.message_count % 200 == 0:  # Every 2 seconds at 100Hz
                        self.get_logger().info(
                            f'Raw Yaw: {fused_yaw:.1f}° | '
                            f'Offset: {self.yaw_offset:.1f}° | '
                            f'Published: {adjusted_yaw:.1f}°'
                        )

                except json.JSONDecodeError:
                    self.get_logger().warn(f'Invalid JSON received: {line}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.init_serial()  # Try to reconnect

    def publish_imu_data(self, fused_yaw, magnetic_yaw, gyro_yaw):
        """Create and publish yaw message with offset"""
        # Create and publish yaw angle message with offset and wrap to 0-360
        adjusted_yaw = (fused_yaw + self.yaw_offset) % 360
        if adjusted_yaw < 0:
            adjusted_yaw += 360
        yaw_msg = Float64()
        yaw_msg.data = adjusted_yaw
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
