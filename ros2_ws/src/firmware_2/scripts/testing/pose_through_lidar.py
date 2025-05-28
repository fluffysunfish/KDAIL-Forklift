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
        # Subscribe to /scan and /imu_1_yaw topics
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.yaw_sub = self.create_subscription(
            Float64, '/imu_1_yaw', self.yaw_callback, 10)
        # Publish to /final_xy
        self.position_pub = self.create_publisher(Point, '/final_xy', 10)
        # Initialize variables
        self.current_yaw_deg = None
        self.current_scan = None
        self.room_width = 1.8  # meters
        self.room_height = 2.3  # meters

    def yaw_callback(self, msg):
        """Store the latest yaw angle in degrees."""
        self.current_yaw_deg = msg.data
        self.estimate_pose()

    def scan_callback(self, msg):
        """Store the latest scan and trigger position estimation."""
        self.current_scan = msg
        self.estimate_pose()

    def estimate_pose(self):
        """Estimate and publish the robot's (x, y) position."""
        if self.current_yaw_deg is None or self.current_scan is None:
            return

        # Convert yaw from degrees to radians
        theta_rad = math.radians(self.current_yaw_deg)

        # Calculate lidar angles in robot frame for global directions
        alpha_plus_x = -theta_rad                # Global +X (right wall)
        alpha_plus_y = math.pi / 2 - theta_rad   # Global +Y (top wall)
        alpha_minus_x = math.pi - theta_rad      # Global -X (left wall)
        alpha_minus_y = 3 * math.pi / 2 - theta_rad  # Global -Y (bottom wall)

        # Normalize angles to match LaserScan range (typically -pi to pi)
        def normalize_angle(angle):
            return (angle + math.pi) % (2 * math.pi) - math.pi

        angles_to_check = [
            normalize_angle(alpha_plus_x),
            normalize_angle(alpha_plus_y),
            normalize_angle(alpha_minus_x),
            normalize_angle(alpha_minus_y)
        ]

        # Generate array of scan angles
        scan_angles = np.arange(
            self.current_scan.angle_min,
            self.current_scan.angle_max + self.current_scan.angle_increment,
            self.current_scan.angle_increment
        )
        if len(scan_angles) > len(self.current_scan.ranges):
            scan_angles = scan_angles[:len(self.current_scan.ranges)]

        # Find indices of closest angles
        indices = [np.argmin(np.abs(scan_angles - a)) for a in angles_to_check]

        # Get corresponding ranges
        D_plus_x = self.current_scan.ranges[indices[0]]    # To right wall
        D_plus_y = self.current_scan.ranges[indices[1]]    # To top wall
        D_minus_x = self.current_scan.ranges[indices[2]]   # To left wall
        D_minus_y = self.current_scan.ranges[indices[3]]   # To bottom wall

        # Calculate x position
        x_from_left = D_minus_x if D_minus_x < self.current_scan.range_max else None
        x_from_right = (self.room_width - D_plus_x) if D_plus_x < self.current_scan.range_max else None

        if x_from_left is not None and x_from_right is not None:
            x = (x_from_left + x_from_right) / 2
        elif x_from_left is not None:
            x = x_from_left
        elif x_from_right is not None:
            x = x_from_right
        else:
            x = 0.0  # Fallback if no valid readings

        # Calculate y position
        y_from_bottom = D_minus_y if D_minus_y < self.current_scan.range_max else None
        y_from_top = (self.room_height - D_plus_y) if D_plus_y < self.current_scan.range_max else None

        if y_from_bottom is not None and y_from_top is not None:
            y = (y_from_bottom + y_from_top) / 2
        elif y_from_bottom is not None:
            y = y_from_bottom
        elif y_from_top is not None:
            y = y_from_top
        else:
            y = 0.0  # Fallback if no valid readings

        # Publish the estimated position
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = 0.0
        self.position_pub.publish(point_msg)
        self.get_logger().info(f'Estimated position: x={x:.2f}m, y={y:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
