#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import math
import numpy as np

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        # Publishers and subscribers
        self.position_pub = self.create_publisher(Point, '/final_xy', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.yaw_sub = self.create_subscription(
            Float64, '/imu_1_yaw', self.yaw_callback, 10)
        # Room dimensions
        self.room_width = 1.8   # meters (X-axis)
        self.room_height = 2.3  # meters (Y-axis)
        # State variables
        self.current_yaw_deg = None
        self.current_scan = None
        self.get_logger().info('Pose Estimator node initialized')

    def yaw_callback(self, msg):
        """Store the latest yaw angle in degrees."""
        self.current_yaw_deg = msg.data
        self.estimate_and_publish_pose()

    def scan_callback(self, msg):
        """Store the latest lidar scan."""
        self.current_scan = msg
        self.estimate_and_publish_pose()

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def angle_diff(self, a, b):
        """Compute the smallest angular difference between two angles."""
        diff = self.normalize_angle(a - b)
        return diff

    def estimate_and_publish_pose(self):
        """Estimate the robot's (x, y) position and publish to /final_xy."""
        if self.current_scan is None or self.current_yaw_deg is None:
            self.get_logger().debug('Waiting for scan or yaw data')
            return

        # Convert yaw to radians
        theta_rad = math.radians(self.current_yaw_deg)

        # Define lidar angles for global directions relative to robot's yaw
        angles_to_check = [
            self.normalize_angle(-theta_rad),          # Global +X (right wall)
            self.normalize_angle(math.pi/2 - theta_rad),  # Global +Y (top wall)
            self.normalize_angle(math.pi - theta_rad),    # Global -X (left wall)
            self.normalize_angle(3*math.pi/2 - theta_rad)  # Global -Y (bottom wall)
        ]

        # Generate array of scan angles
        scan_angles = np.arange(
            self.current_scan.angle_min,
            self.current_scan.angle_max + self.current_scan.angle_increment / 2,
            self.current_scan.angle_increment
        )
        if len(scan_angles) > len(self.current_scan.ranges):
            scan_angles = scan_angles[:len(self.current_scan.ranges)]

        # Find indices of closest angles
        indices = []
        for target_angle in angles_to_check:
            angle_diffs = np.array([self.angle_diff(a, target_angle) for a in scan_angles])
            index = np.argmin(np.abs(angle_diffs))
            indices.append(index)

        # Extract distances
        ranges = self.current_scan.ranges
        D_plus_x = ranges[indices[0]]   # To right wall (+X)
        D_plus_y = ranges[indices[1]]   # To top wall (+Y)
        D_minus_x = ranges[indices[2]]  # To left wall (-X)
        D_minus_y = ranges[indices[3]]  # To bottom wall (-Y)

        # Compute x position
        x_candidates = []
        if 0 < D_minus_x < self.current_scan.range_max and not math.isinf(D_minus_x):
            x_candidates.append(D_minus_x)  # From left wall
        if 0 < D_plus_x < self.current_scan.range_max and not math.isinf(D_plus_x):
            x_candidates.append(self.room_width - D_plus_x)  # From right wall
        x = np.mean(x_candidates) if x_candidates else 0.0
        if not x_candidates:
            self.get_logger().warn('No valid x measurements')

        # Compute y position
        y_candidates = []
        if 0 < D_minus_y < self.current_scan.range_max and not math.isinf(D_minus_y):
            y_candidates.append(D_minus_y)  # From bottom wall
        if 0 < D_plus_y < self.current_scan.range_max and not math.isinf(D_plus_y):
            y_candidates.append(self.room_height - D_plus_y)  # From top wall
        y = np.mean(y_candidates) if y_candidates else 0.0
        if not y_candidates:
            self.get_logger().warn('No valid y measurements')

        # Ensure position is within room bounds
        x = max(0.0, min(x, self.room_width))
        y = max(0.0, min(y, self.room_height))

        # Publish position
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = 0.0
        self.position_pub.publish(point_msg)
        self.get_logger().info(f'Published position: x={x:.2f}m, y={y:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Pose Estimator')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
