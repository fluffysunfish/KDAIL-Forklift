#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LidarFeatureDetector(Node):
    def __init__(self):
        super().__init__('lidar_feature_detector')

        # Subscribe to the scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.get_logger().info("LiDAR Feature Detector Node Started - Waiting for scan data...")

        # Variables to store data
        self.distances_array = []
        self.angle_increment = 0.0
        self.first_scan_processed = False

    def scan_callback(self, msg):
        """Callback function that processes incoming LaserScan messages"""

        # Extract distance data from the LaserScan message
        ranges = msg.ranges
        self.angle_increment = msg.angle_increment

        # Clear previous data and store new distances
        self.distances_array = []

        # Process each range measurement
        for i, distance in enumerate(ranges):
            # Handle infinite or invalid readings by replacing with max range
            if math.isinf(distance) or math.isnan(distance) or distance == 0.0:
                self.distances_array.append(msg.range_max)
            else:
                self.distances_array.append(distance)

        # Process the data for feature detection
        self.process_lidar_data()

    def process_lidar_data(self):
        """Process the LiDAR data to detect features"""

        if not self.distances_array:
            return

        # 1. Report array information
        num_elements = len(self.distances_array)
        angle_increment_deg = math.degrees(self.angle_increment)

        self.get_logger().info(f"=== LiDAR Data Analysis ===")
        self.get_logger().info(f"Number of distance data points: {num_elements}")
        self.get_logger().info(f"Angle increment between readings: {angle_increment_deg:.2f} degrees")

        # 2. Calculate differences between every 10th data point
        differences_array = []
        difference_indices = []  # Store original indices for angle calculation

        # Calculate differences between points that are 10 positions apart
        for i in range(len(self.distances_array) - 10):
            current_distance = self.distances_array[i]
            next_distance = self.distances_array[i + 10]

            # Calculate absolute difference
            difference = abs(current_distance - next_distance)
            differences_array.append(difference)
            difference_indices.append(i)  # Store the index of the first point

        if not differences_array:
            self.get_logger().warn("Not enough data points to calculate differences")
            return

        # 3. Find the point with maximum difference
        max_difference_idx = np.argmax(differences_array)
        max_difference = differences_array[max_difference_idx]
        original_point_idx = difference_indices[max_difference_idx]

        # Calculate the angle of the original point
        angle_of_max_diff = original_point_idx * angle_increment_deg

        # Get the two distances that produced this maximum difference
        distance1 = self.distances_array[original_point_idx]
        distance2 = self.distances_array[original_point_idx + 10]
        shorter_distance = min(distance1, distance2)

        # 4. Output results
        self.get_logger().info(f"=== Feature Detection Results ===")
        self.get_logger().info(f"Maximum difference found: {max_difference:.3f} meters")
        self.get_logger().info(f"Angle of original point with max difference: {angle_of_max_diff:.1f} degrees")
        self.get_logger().info(f"Distance at original point: {distance1:.3f} meters")
        self.get_logger().info(f"Distance at +10th point: {distance2:.3f} meters")
        self.get_logger().info(f"Shorter distance of the two: {shorter_distance:.3f} meters")

        # 5. Additional analysis for feature detection
        self.analyze_features(differences_array, difference_indices, angle_increment_deg)

    def analyze_features(self, differences_array, difference_indices, angle_increment_deg):
        """Additional analysis to help identify features like boxes"""

        # Find significant differences (potential edges/corners)
        mean_diff = np.mean(differences_array)
        std_diff = np.std(differences_array)
        threshold = mean_diff + 2 * std_diff  # Points that are 2 standard deviations above mean

        significant_features = []
        for i, diff in enumerate(differences_array):
            if diff > threshold:
                angle = difference_indices[i] * angle_increment_deg
                distance1 = self.distances_array[difference_indices[i]]
                distance2 = self.distances_array[difference_indices[i] + 10]
                shorter_dist = min(distance1, distance2)

                significant_features.append({
                    'angle': angle,
                    'difference': diff,
                    'shorter_distance': shorter_dist,
                    'distance1': distance1,
                    'distance2': distance2
                })

        if significant_features:
            self.get_logger().info(f"=== Potential Features Detected ===")
            self.get_logger().info(f"Found {len(significant_features)} significant features (threshold: {threshold:.3f}m)")

            # Sort by difference magnitude
            significant_features.sort(key=lambda x: x['difference'], reverse=True)

            for i, feature in enumerate(significant_features[:5]):  # Show top 5 features
                self.get_logger().info(
                    f"Feature {i+1}: Angle={feature['angle']:.1f}Â°, "
                    f"Diff={feature['difference']:.3f}m, "
                    f"Shorter_dist={feature['shorter_distance']:.3f}m"
                )
        else:
            self.get_logger().info("No significant features detected above threshold")

        # Statistics
        self.get_logger().info(f"=== Statistics ===")
        self.get_logger().info(f"Mean difference: {mean_diff:.3f} meters")
        self.get_logger().info(f"Standard deviation: {std_diff:.3f} meters")
        self.get_logger().info(f"Detection threshold: {threshold:.3f} meters")

def main(args=None):
    rclpy.init(args=args)

    node = LidarFeatureDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LiDAR Feature Detector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
