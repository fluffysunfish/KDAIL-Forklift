#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import re
import time
import numpy as np
import glob
from dataclasses import dataclass
from typing import Optional, List

@dataclass
class KalmanParams:
    """1D Kalman Filter Parameters for distance filtering"""
    process_noise_std: float = 2.0
    measurement_noise_std: float = 10.0
    initial_distance_std: float = 15.0
    max_velocity: float = 40.0

class Distance1DKalman:
    """Simple 1D Kalman Filter for distance measurements"""

    def __init__(self, params: KalmanParams):
        self.params = params

        # State: [distance, velocity]
        self.x = np.array([0.0, 0.0])

        # State covariance matrix
        self.P = np.array([
            [params.initial_distance_std**2, 0],
            [0, (params.max_velocity/2)**2]
        ])

        # Measurement noise variance
        self.R = params.measurement_noise_std**2

        # Measurement matrix (we observe distance only)
        self.H = np.array([1.0, 0.0])

        self.last_time = None
        self.initialized = False

    def predict(self, dt: float):
        """Prediction step"""
        # State transition matrix
        F = np.array([
            [1.0, dt],
            [0.0, 1.0]
        ])

        # Process noise matrix
        q = self.params.process_noise_std**2
        Q = np.array([
            [q * dt**4 / 4, q * dt**3 / 2],
            [q * dt**3 / 2, q * dt**2]
        ])

        # Predict state and covariance
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        # Apply constraints
        self.x[0] = max(0, self.x[0])
        self.x[1] = np.clip(self.x[1], -self.params.max_velocity, self.params.max_velocity)

    def update(self, measurement: float):
        """Update step"""
        # Innovation (residual)
        y = measurement - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T / S

        # Update state and covariance
        self.x = self.x + K * y
        self.P = self.P - np.outer(K, K) * S

    def process_measurement(self, measurement: float, timestamp: float) -> Optional[float]:
        """Process a new measurement"""
        if not self.initialized:
            # Initialize with first measurement
            self.x[0] = measurement
            self.x[1] = 0.0
            self.initialized = True
            self.last_time = timestamp
            return measurement * 0.97  # Apply 3% correction

        # Calculate time delta
        dt = timestamp - self.last_time if self.last_time else 0.2
        dt = max(0.01, min(dt, 1.0))

        # Kalman filter steps
        self.predict(dt)
        self.update(measurement)

        self.last_time = timestamp

        return self.x[0] * 0.97  # Return filtered distance with 3% correction

class UWBNode(Node):
    def __init__(self):
        super().__init__('uwb_distance_node')

        # Create publisher for filtered distances (Kalman filtered)
        self.kalman_publisher_ = self.create_publisher(Float32MultiArray, '/uwb_raw_data', 10)

        # Create publisher for raw unfiltered distances
        self.raw_publisher_ = self.create_publisher(Float32MultiArray, '/raw_uwb_very_raw', 10)

        # Kalman filter parameters (fixed constants)
        self.kalman_params = KalmanParams(
            process_noise_std=2.0,
            measurement_noise_std=10.0,
            max_velocity=40.0
        )

        # Initialize Kalman filters for each distance
        self.kalman_filters = [
            Distance1DKalman(self.kalman_params),  # d1
            Distance1DKalman(self.kalman_params),  # d2
            Distance1DKalman(self.kalman_params),  # d3
            Distance1DKalman(self.kalman_params)   # d4
        ]

        # Serial connection
        self.ser = None

        # Current filtered distances
        self.filtered_distances = [0.0, 0.0, 0.0, 0.0]

        # Current raw distances
        self.raw_distances = [0.0, 0.0, 0.0, 0.0]

        # Regex patterns for 4 anchors with new JSON format
        self.distance_patterns = [
            re.compile(r'"Addr":"0x0001"[^}]*"D_cm"\s*:\s*(\d+)'),  # d1
            re.compile(r'"Addr":"0x0002"[^}]*"D_cm"\s*:\s*(\d+)'),  # d2
            re.compile(r'"Addr":"0x0003"[^}]*"D_cm"\s*:\s*(\d+)'),  # d3
            re.compile(r'"Addr":"0x0004"[^}]*"D_cm"\s*:\s*(\d+)')   # d4
        ]

        # Initialize serial connection
        if self.connect_to_port():
            self.get_logger().info('UWB Distance Node started successfully')
            # Create timer for reading serial data
            self.timer = self.create_timer(0.05, self.read_serial_data)  # 20Hz
        else:
            self.get_logger().error('Failed to connect to UWB device')

    def find_available_ports(self) -> List[str]:
        """Find all available ACM ports"""
        available_ports = []

        # Check ACM0 to ACM3
        for i in range(4):
            port = f'/dev/ttyACM{i}'
            try:
                test_ser = serial.Serial(port, 115200, timeout=0.1)
                test_ser.close()
                available_ports.append(port)
            except (serial.SerialException, FileNotFoundError):
                continue

        # Also check for USB ports as fallback
        usb_ports = glob.glob('/dev/ttyUSB*')
        for port in usb_ports:
            try:
                test_ser = serial.Serial(port, 115200, timeout=0.1)
                test_ser.close()
                available_ports.append(port)
            except serial.SerialException:
                continue

        return available_ports

    def connect_to_port(self) -> bool:
        """Try to connect to an available port"""
        available_ports = self.find_available_ports()

        if not available_ports:
            self.get_logger().error("No available ACM or USB ports found!")
            return False

        for port in available_ports:
            try:
                self.get_logger().info(f"Attempting to connect to {port}...")
                self.ser = serial.Serial(port, 115200, timeout=1.0, rtscts=True)
                self.get_logger().info(f"Successfully connected to {port}")
                return True
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to connect to {port}: {e}")
                continue

        self.get_logger().error("Failed to connect to any available port!")
        return False

    def parse_distances(self, data: str) -> List[Optional[float]]:
        """Extract d1, d2, d3, d4 from raw data"""
        distances = [None, None, None, None]

        for i, pattern in enumerate(self.distance_patterns):
            match = pattern.search(data)
            if match:
                try:
                    distance = float(match.group(1))
                    # Basic sanity check
                    if 5 <= distance <= 10000:  # 5cm to 100m range
                        distances[i] = distance
                except ValueError:
                    continue

        return distances

    def read_serial_data(self):
        """Read and process serial data"""
        if not self.ser or not self.ser.is_open:
            return

        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read_all().decode('utf-8', errors='ignore')

                # Process data if it contains JSON format with Block and results
                if '"Block":' in data and '"results":' in data:
                    # Find complete JSON messages
                    messages = []
                    start_pos = 0

                    while True:
                        start = data.find('{"Block":', start_pos)
                        if start == -1:
                            break

                        brace_count = 0
                        end = start
                        in_string = False
                        escape_next = False

                        for i, char in enumerate(data[start:]):
                            pos = start + i
                            if escape_next:
                                escape_next = False
                                continue

                            if char == '\\':
                                escape_next = True
                                continue

                            if char == '"' and not escape_next:
                                in_string = not in_string
                                continue

                            if not in_string:
                                if char == '{':
                                    brace_count += 1
                                elif char == '}':
                                    brace_count -= 1
                                    if brace_count == 0:
                                        end = pos
                                        break

                        if brace_count == 0:
                            messages.append(data[start:end+1])
                            start_pos = end + 1
                        else:
                            break

                    # Process the last complete message
                    if messages:
                        message = messages[-1]
                        raw_distances = self.parse_distances(message)
                        current_time = time.time()

                        # Update raw distances and publish them
                        raw_updated = False
                        for i, raw_dist in enumerate(raw_distances):
                            if raw_dist is not None:
                                self.raw_distances[i] = raw_dist
                                raw_updated = True

                        if raw_updated:
                            self.publish_raw_distances()

                        # Apply Kalman filter to each distance
                        kalman_updated = False
                        for i, raw_dist in enumerate(raw_distances):
                            if raw_dist is not None:
                                filtered_dist = self.kalman_filters[i].process_measurement(
                                    raw_dist, current_time
                                )
                                if filtered_dist is not None:
                                    self.filtered_distances[i] = filtered_dist
                                    kalman_updated = True

                        # Publish filtered distances if any were updated
                        if kalman_updated:
                            self.publish_kalman_distances()

        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")

    def publish_raw_distances(self):
        """Publish raw unfiltered distances to ROS topic"""
        msg = Float32MultiArray()
        msg.data = [float(d) for d in self.raw_distances]

        self.raw_publisher_.publish(msg)

        # Log the published raw data
        self.get_logger().info(
            f"Published RAW distances: d1={self.raw_distances[0]:.1f}, "
            f"d2={self.raw_distances[1]:.1f}, "
            f"d3={self.raw_distances[2]:.1f}, "
            f"d4={self.raw_distances[3]:.1f}"
        )

    def publish_kalman_distances(self):
        """Publish Kalman filtered distances to ROS topic"""
        msg = Float32MultiArray()
        msg.data = [float(d) for d in self.filtered_distances]

        self.kalman_publisher_.publish(msg)

        # Log the published filtered data
        self.get_logger().info(
            f"Published KALMAN distances: d1={self.filtered_distances[0]:.1f}, "
            f"d2={self.filtered_distances[1]:.1f}, "
            f"d3={self.filtered_distances[2]:.1f}, "
            f"d4={self.filtered_distances[3]:.1f}"
        )

    def destroy_node(self):
        """Clean up resources"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        uwb_node = UWBNode()
        rclpy.spin(uwb_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'uwb_node' in locals():
            uwb_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
