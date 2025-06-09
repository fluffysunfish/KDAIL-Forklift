#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point
import numpy as np
import math
from typing import Tuple, Optional, List
import json
import time

class UWBPositionCalculator(Node):
    def __init__(self):
        super().__init__('uwb_position_calculator')

        # Anchor positions (in cm)
        self.anchor_positions = {
            1: (0.0, 0.0),      # Anchor 1 at origin
            2: (700.0, 0.0),    # Anchor 2 at (Xmax, 0)
            3: (700.0, 700.0),   # Anchor 3 at (Xmax, Ymax)
            4: (0.0, 700.0)      # Anchor 4 at (0, Ymax)
        }

        # Subscribe to both UWB topics
        self.raw_subscriber = self.create_subscription(
            Float32MultiArray,
            '/raw_uwb_very_raw',
            self.raw_callback,
            10
        )

        self.kalman_subscriber = self.create_subscription(
            Float32MultiArray,
            '/uwb_raw_data',
            self.kalman_callback,
            10
        )

        # Publisher for final averaged position
        self.final_xy_publisher = self.create_publisher(Point, '/final_xy', 10)
        # Publisher for detailed UWB data
        self.uwb_data_publisher = self.create_publisher(String, '/uwb_detailed_data', 10)

        # Store latest distances
        self.raw_distances = [0.0, 0.0, 0.0, 0.0]  # d1, d2, d3, d4
        self.kalman_distances = [0.0, 0.0, 0.0, 0.0]  # d1, d2, d3, d4

        # Timestamps for distance updates
        self.raw_distances_time = [0.0, 0.0, 0.0, 0.0]
        self.kalman_distances_time = [0.0, 0.0, 0.0, 0.0]

        # Data received flags
        self.raw_received = False
        self.kalman_received = False

        self.get_logger().info('UWB Position Calculator Node started')
        self.get_logger().info(f'Anchor positions: {self.anchor_positions}')

    def raw_callback(self, msg):
        """Callback for raw UWB distances"""
        if len(msg.data) >= 4:
            self.raw_distances = list(msg.data[:4])
            self.raw_distances_time = [time.time()] * 4
            self.raw_received = True
            self.process_positions()

    def kalman_callback(self, msg):
        """Callback for Kalman filtered UWB distances"""
        if len(msg.data) >= 4:
            self.kalman_distances = list(msg.data[:4])
            self.kalman_distances_time = [time.time()] * 4
            self.kalman_received = True
            self.process_positions()

    def trilaterate_two_circles(self, anchor1_pos: Tuple[float, float], d1: float,
                               anchor2_pos: Tuple[float, float], d2: float) -> Optional[Tuple[float, float]]:
        """
        Calculate position using two anchor distances (intersection of two circles)
        Returns the intersection point closer to the rectangle center
        """
        x1, y1 = anchor1_pos
        x2, y2 = anchor2_pos

        anchor_dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if anchor_dist > (d1 + d2) or anchor_dist < abs(d1 - d2) or anchor_dist == 0:
            return None

        a = (d1**2 - d2**2 + anchor_dist**2) / (2 * anchor_dist)
        h = math.sqrt(d1**2 - a**2)

        px = x1 + a * (x2 - x1) / anchor_dist
        py = y1 + a * (y2 - y1) / anchor_dist

        point1_x = px + h * (y2 - y1) / anchor_dist
        point1_y = py - h * (x2 - x1) / anchor_dist
        point2_x = px - h * (y2 - y1) / anchor_dist
        point2_y = py + h * (x2 - x1) / anchor_dist

        center_x, center_y = 77.5, 30.0
        dist1 = math.sqrt((point1_x - center_x)**2 + (point1_y - center_y)**2)
        dist2 = math.sqrt((point2_x - center_x)**2 + (point2_y - center_y)**2)

        return (point1_x, point1_y) if dist1 < dist2 else (point2_x, point2_y)

    def trilaterate_least_squares(self, distances: List[float]) -> Optional[Tuple[float, float]]:
        """
        Calculate position using least squares trilateration with all 4 anchors
        """
        anchors = np.array(list(self.anchor_positions.values()))
        dists = np.array(distances)
        x0, y0 = 77.5, 30.0
        x, y = x0, y0

        for iteration in range(50):
            calculated_dists = np.sqrt((anchors[:, 0] - x)**2 + (anchors[:, 1] - y)**2)
            residuals = calculated_dists - dists
            J = np.zeros((4, 2))
            for i in range(4):
                if calculated_dists[i] > 0:
                    J[i, 0] = (x - anchors[i, 0]) / calculated_dists[i]
                    J[i, 1] = (y - anchors[i, 1]) / calculated_dists[i]
            try:
                delta = np.linalg.lstsq(J, -residuals, rcond=None)[0]
                x += delta[0]
                y += delta[1]
            except np.linalg.LinAlgError:
                return None
            if np.linalg.norm(delta) < 0.01:
                break
        return (x, y)

    def process_positions(self):
        """Process and calculate positions when both data sources are available"""
        if not (self.raw_received and self.kalman_received):
            return

        self.get_logger().info("=" * 60)
        self.get_logger().info("UWB POSITION CALCULATION")
        self.get_logger().info("=" * 60)

        self.get_logger().info(f"Raw distances:    d1={self.raw_distances[0]:.1f}, d2={self.raw_distances[1]:.1f}, d3={self.raw_distances[2]:.1f}, d4={self.raw_distances[3]:.1f}")
        self.get_logger().info(f"Kalman distances: d1={self.kalman_distances[0]:.1f}, d2={self.kalman_distances[1]:.1f}, d3={self.kalman_distances[2]:.1f}, d4={self.kalman_distances[3]:.1f}")

        self.get_logger().info("\n--- ADJACENT ANCHOR PAIR CALCULATIONS ---")
        adjacent_pairs = [
            (1, 2, "Anchor 1-2"),
            (2, 3, "Anchor 2-3"),
            (3, 4, "Anchor 3-4"),
            (4, 1, "Anchor 4-1")
        ]

        raw_positions = []
        kalman_positions = []

        for anchor1_id, anchor2_id, pair_name in adjacent_pairs:
            anchor1_pos = self.anchor_positions[anchor1_id]
            anchor2_pos = self.anchor_positions[anchor2_id]

            raw_pos = self.trilaterate_two_circles(
                anchor1_pos, self.raw_distances[anchor1_id-1],
                anchor2_pos, self.raw_distances[anchor2_id-1]
            )
            kalman_pos = self.trilaterate_two_circles(
                anchor1_pos, self.kalman_distances[anchor1_id-1],
                anchor2_pos, self.kalman_distances[anchor2_id-1]
            )

            if raw_pos:
                raw_positions.append(raw_pos)
                self.get_logger().info(f"{pair_name} (Raw):    X={raw_pos[0]:.2f}, Y={raw_pos[1]:.2f}")
            else:
                self.get_logger().info(f"{pair_name} (Raw):    No valid intersection")

            if kalman_pos:
                kalman_positions.append(kalman_pos)
                self.get_logger().info(f"{pair_name} (Kalman): X={kalman_pos[0]:.2f}, Y={kalman_pos[1]:.2f}")
            else:
                self.get_logger().info(f"{pair_name} (Kalman): No valid intersection")

        self.get_logger().info("\n--- LEAST SQUARES TRILATERATION (ALL 4 ANCHORS) ---")
        raw_ls_pos = self.trilaterate_least_squares(self.raw_distances)
        kalman_ls_pos = self.trilaterate_least_squares(self.kalman_distances)

        if raw_ls_pos:
            self.get_logger().info(f"Raw Least Squares:    X={raw_ls_pos[0]:.2f}, Y={raw_ls_pos[1]:.2f}")
        else:
            self.get_logger().info("Raw Least Squares:    Failed to converge")

        if kalman_ls_pos:
            self.get_logger().info(f"Kalman Least Squares: X={kalman_ls_pos[0]:.2f}, Y={kalman_ls_pos[1]:.2f}")
        else:
            self.get_logger().info("Kalman Least Squares: Failed to converge")

        # Calculate separate raw and Kalman averages
        raw_avg_pos = None
        kalman_avg_pos = None
        all_positions = []
        raw_positions_list = raw_positions + ([raw_ls_pos] if raw_ls_pos else [])
        kalman_positions_list = kalman_positions + ([kalman_ls_pos] if kalman_ls_pos else [])

        if raw_positions_list:
            raw_avg_x = sum(pos[0] for pos in raw_positions_list) / len(raw_positions_list)
            raw_avg_y = sum(pos[1] for pos in raw_positions_list) / len(raw_positions_list)
            raw_avg_pos = [raw_avg_x, raw_avg_y]
            self.get_logger().info(f"Raw Average Position: X={raw_avg_x:.2f}, Y={raw_avg_y:.2f}")

        if kalman_positions_list:
            kalman_avg_x = sum(pos[0] for pos in kalman_positions_list) / len(kalman_positions_list)
            kalman_avg_y = sum(pos[1] for pos in kalman_positions_list) / len(kalman_positions_list)
            kalman_avg_pos = [kalman_avg_x, kalman_avg_y]
            self.get_logger().info(f"Kalman Average Position: X={kalman_avg_x:.2f}, Y={kalman_avg_y:.2f}")

        # Combine for final average
        all_positions.extend(raw_positions_list)
        all_positions.extend(kalman_positions_list)

        # Prepare detailed UWB data
        uwb_data = {
            'raw_distances': self.raw_distances,
            'kalman_distances': self.kalman_distances,
            'raw_distances_time': self.raw_distances_time,
            'kalman_distances_time': self.kalman_distances_time,
            'raw_positions': [list(pos) if pos else None for pos in raw_positions],
            'kalman_positions': [list(pos) if pos else None for pos in kalman_positions],
            'raw_ls_pos': list(raw_ls_pos) if raw_ls_pos else None,
            'kalman_ls_pos': list(kalman_ls_pos) if kalman_ls_pos else None,
            'raw_avg_pos': raw_avg_pos,
            'kalman_avg_pos': kalman_avg_pos,
            'avg_pos': None,
            'distance_from_origin': None,
            'num_valid_calcs': len(all_positions)
        }

        if all_positions:
            avg_x = sum(pos[0] for pos in all_positions) / len(all_positions)
            avg_y = sum(pos[1] for pos in all_positions) / len(all_positions)
            distance_from_origin = math.sqrt(avg_x**2 + avg_y**2)

            self.get_logger().info(f"Final Average Position: X={avg_x:.2f}, Y={avg_y:.2f}")
            self.get_logger().info(f"Distance from Origin (Anchor 1): {distance_from_origin:.2f} cm")
            self.get_logger().info(f"Number of valid calculations: {len(all_positions)}")

            point_msg = Point()
            point_msg.x = avg_x
            point_msg.y = avg_y
            point_msg.z = distance_from_origin
            self.final_xy_publisher.publish(point_msg)

            uwb_data['avg_pos'] = [avg_x, avg_y]
            uwb_data['distance_from_origin'] = distance_from_origin

        else:
            self.get_logger().warn("No valid position calculations available!")

        uwb_data_msg = String()
        uwb_data_msg.data = json.dumps(uwb_data)
        self.uwb_data_publisher.publish(uwb_data_msg)

        self.get_logger().info("=" * 60)

def main(args=None):
    rclpy.init(args=args)
    try:
        position_calculator = UWBPositionCalculator()
        rclpy.spin(position_calculator)
    except KeyboardInterrupt:
        pass
    finally:
        if 'position_calculator' in locals():
            position_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
