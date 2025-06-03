#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64
from geometry_msgs.msg import Point
import numpy as np
import math
from typing import Tuple, Optional, List

class UWBLocalizationNode(Node):
    def __init__(self):
        super().__init__('uwb_localization_node')

        # UWB Anchor positions (x, y) in cm
        self.anchors = {
            'd1': np.array([0.0, 0.0]),      # (0, 0)
            'd2': np.array([95.0, 0.0]),    # (180, 0)
            'd3': np.array([0.0, 65.0]),    # (0, 220)
            'd4': np.array([95.0, 65.0])   # (180, 220)
        }

        # Subscribers
        self.uwb_subscriber = self.create_subscription(
            Float32MultiArray,
            '/uwb_raw_data',
            self.uwb_callback,
            10
        )

        self.imu_subscriber = self.create_subscription(
            Float64,
            '/imu_1_yaw',
            self.imu_callback,
            10
        )

        # Publisher for x,y coordinates
        self.position_publisher = self.create_publisher(
            Point,
            '/final_xy',
            10
        )

        # Current data
        self.current_distances = [0.0, 0.0, 0.0, 0.0]  # [d1, d2, d3, d4]
        self.current_yaw = 0.0  # degrees
        self.uwb_data_received = False
        self.imu_data_received = False

        self.get_logger().info('UWB Localization Node started')
        self.get_logger().info('Anchor positions:')
        self.get_logger().info(f'  d1: {self.anchors["d1"]}')
        self.get_logger().info(f'  d2: {self.anchors["d2"]}')
        self.get_logger().info(f'  d3: {self.anchors["d3"]}')
        self.get_logger().info(f'  d4: {self.anchors["d4"]}')

    def uwb_callback(self, msg):
        """Callback for UWB distance data"""
        if len(msg.data) >= 4:
            self.current_distances = list(msg.data[:4])
            self.uwb_data_received = True
            self.process_localization()

    def imu_callback(self, msg):
        """Callback for IMU yaw data"""
        self.current_yaw = msg.data
        self.imu_data_received = True
        self.process_localization()

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to 0-360 range"""
        while angle < 0:
            angle += 360
        while angle >= 360:
            angle -= 360
        return angle

    def get_directional_weights(self, yaw: float) -> dict:
        """Get anchor weights based on yaw direction"""
        yaw = self.normalize_angle(yaw)

        # Define base weights (all anchors have some weight)
        base_weight = 0.3
        priority_weight = 1.0

        weights = {
            'd1': base_weight,
            'd2': base_weight,
            'd3': base_weight,
            'd4': base_weight
        }

        # Apply directional priority based on yaw ranges
        if (yaw >= 315) or (yaw <= 45):
            # Facing North/Up - prioritize d4 and d2 (top anchors)
            weights['d4'] = priority_weight
            weights['d2'] = priority_weight
            direction = "North (315°-45°)"

        elif (yaw > 45) and (yaw <= 135):
            # Facing East/Right - prioritize d3 and d4 (right side)
            weights['d3'] = priority_weight
            weights['d4'] = priority_weight
            direction = "East (45°-135°)"

        elif (yaw > 135) and (yaw <= 225):
            # Facing South/Down - prioritize d3 and d1 (bottom anchors)
            weights['d3'] = priority_weight
            weights['d1'] = priority_weight
            direction = "South (135°-225°)"

        elif (yaw > 225) and (yaw < 315):
            # Facing West/Left - prioritize d1 and d2 (left side)
            weights['d1'] = priority_weight
            weights['d2'] = priority_weight
            direction = "West (225°-315°)"

        self.get_logger().debug(f"Yaw: {yaw:.1f}° - Direction: {direction}")

        return weights

    def trilaterate_two_circles(self, anchor1: np.ndarray, dist1: float,
                               anchor2: np.ndarray, dist2: float) -> List[np.ndarray]:
        """Find intersection points of two circles"""
        # Vector from anchor1 to anchor2
        d_vec = anchor2 - anchor1
        d = np.linalg.norm(d_vec)

        # Check if circles can intersect
        if d > (dist1 + dist2) or d < abs(dist1 - dist2) or d == 0:
            return []

        # Calculate intersection points
        a = (dist1**2 - dist2**2 + d**2) / (2 * d)
        h = math.sqrt(dist1**2 - a**2)

        # Point on line between centers
        p = anchor1 + a * (d_vec / d)

        # Perpendicular vector
        perp = np.array([-d_vec[1], d_vec[0]]) / d

        # Two intersection points
        point1 = p + h * perp
        point2 = p - h * perp

        return [point1, point2]

    def calculate_weighted_position(self, positions: List[np.ndarray],
                                  weights: List[float]) -> np.ndarray:
        """Calculate weighted average of positions"""
        if not positions or not weights:
            return np.array([0.0, 0.0])

        weighted_sum = np.zeros(2)
        total_weight = 0.0

        for pos, weight in zip(positions, weights):
            weighted_sum += pos * weight
            total_weight += weight

        if total_weight > 0:
            return weighted_sum / total_weight
        else:
            return np.array([0.0, 0.0])

    def solve_position_from_pairs(self) -> Optional[np.ndarray]:
        """Calculate position using anchor pairs with directional weighting"""
        distances = self.current_distances
        anchor_positions = [
            self.anchors['d1'],  # [0, 0]
            self.anchors['d2'],  # [180, 0]
            self.anchors['d3'],  # [0, 220]
            self.anchors['d4']   # [180, 220]
        ]

        # Get directional weights
        weights = self.get_directional_weights(self.current_yaw)

        # Define anchor pairs and their combinations
        pairs = [
            ('d1', 'd2', 0, 1),  # d1-d2 pair (horizontal bottom)
            ('d1', 'd3', 0, 2),  # d1-d3 pair (vertical left)
            ('d2', 'd4', 1, 3),  # d2-d4 pair (vertical right)
            ('d3', 'd4', 2, 3)   # d3-d4 pair (horizontal top)
        ]

        valid_positions = []
        position_weights = []

        for anchor1_name, anchor2_name, idx1, idx2 in pairs:
            anchor1 = anchor_positions[idx1]
            anchor2 = anchor_positions[idx2]
            dist1 = distances[idx1]
            dist2 = distances[idx2]

            # Skip if distances are invalid
            if dist1 <= 0 or dist2 <= 0:
                continue

            # Find intersection points
            intersections = self.trilaterate_two_circles(anchor1, dist1, anchor2, dist2)

            if intersections:
                # Choose the intersection point that's within the anchor rectangle
                # and makes more sense based on other anchors
                for point in intersections:
                    x, y = point
                    # Basic bounds check (within extended anchor area)
                    if -50 <= x <= 230 and -50 <= y <= 270:
                        # Calculate pair weight based on anchor priorities
                        pair_weight = (weights[anchor1_name] + weights[anchor2_name]) / 2.0
                        valid_positions.append(point)
                        position_weights.append(pair_weight)

                        self.get_logger().debug(
                            f"Pair {anchor1_name}-{anchor2_name}: ({x:.1f}, {y:.1f}) "
                            f"weight: {pair_weight:.2f}"
                        )

        if valid_positions:
            # Calculate weighted average position
            final_position = self.calculate_weighted_position(valid_positions, position_weights)
            return final_position

        return None

    def process_localization(self):
        """Process localization when both UWB and IMU data are available"""
        if not (self.uwb_data_received and self.imu_data_received):
            return

        # Check if all distance values are valid
        if any(d < 0 for d in self.current_distances):
            self.get_logger().warn("Invalid distance values received")
            return

        try:
            # Calculate position
            position = self.solve_position_from_pairs()

            if position is not None:
                # Create and publish position message
                pos_msg = Point()
                pos_msg.x = float(position[0])
                pos_msg.y = float(position[1])
                pos_msg.z = 0.0  # 2D localization

                self.position_publisher.publish(pos_msg)

                # Log the result
                weights = self.get_directional_weights(self.current_yaw)
                priority_anchors = [k for k, v in weights.items() if v > 0.5]

                self.get_logger().info(
                    f"Position: ({position[0]:.1f}, {position[1]:.1f}) cm | "
                    f"Yaw: {self.current_yaw:.1f}° | "
                    f"Priority: {priority_anchors} | "
                    f"Distances: d1={self.current_distances[0]:.1f}, "
                    f"d2={self.current_distances[1]:.1f}, "
                    f"d3={self.current_distances[2]:.1f}, "
                    f"d4={self.current_distances[3]:.1f}"
                )
            else:
                self.get_logger().warn("Could not calculate valid position")

        except Exception as e:
            self.get_logger().error(f"Error in localization: {e}")

def main(args=None):
    rclpy.init(args=args)

    try:
        localization_node = UWBLocalizationNode()
        rclpy.spin(localization_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'localization_node' in locals():
            localization_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
