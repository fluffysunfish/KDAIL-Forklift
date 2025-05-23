#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Point
import math
import numpy as np
from collections import deque
import statistics

class UWBIMUFusion(Node):
    def __init__(self):
        super().__init__('uwb_imu_fusion')
        
        # Publishers
        self.position_publisher = self.create_publisher(Point, '/uwb_fused_xy', 10)
        
        # Subscribers
        self.uwb_subscription = self.create_subscription(
            Float32MultiArray,
            '/uwb_raw_distances',
            self.uwb_callback,
            10)
        self.imu_subscription = self.create_subscription(
            Float32,
            '/imu_1_yaw',
            self.imu_callback,
            10)
        
        # Anchor positions (x, y) in cm
        self.anchor1 = np.array([0, 0])      # d1
        self.anchor2 = np.array([0, 170])    # d2  
        self.anchor3 = np.array([200, 170])  # d3
        self.anchor4 = np.array([200, 0])    # virtual anchor for reference
        
        # Current sensor data
        self.current_distances = [None, None, None]  # d1, d2, d3
        self.current_yaw = 0.0  # degrees
        self.last_valid_position = None
        
        # Enhanced filtering parameters
        self.position_history = deque(maxlen=10)
        self.distance_history = deque(maxlen=5)
        self.yaw_history = deque(maxlen=3)
        
        # Fusion parameters
        self.min_distance_weight = 0.1
        self.max_distance_weight = 1.0
        self.orientation_boost_factor = 2.0
        self.max_position_jump = 50.0  # cm
        
        # Quality thresholds
        self.min_valid_distances = 2
        self.max_reasonable_distance = 400  # cm
        self.min_reasonable_distance = 10   # cm
        
        self.get_logger().info('UWB-IMU Fusion node initialized')
        
        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.1, self.fusion_callback)
    
    def uwb_callback(self, msg):
        """Callback for UWB distance data"""
        if len(msg.data) >= 3:
            self.current_distances = [
                msg.data[0] if msg.data[0] > 0 else None,
                msg.data[1] if msg.data[1] > 0 else None,
                msg.data[2] if msg.data[2] > 0 else None
            ]
            
            # Add to distance history for temporal filtering
            self.distance_history.append(self.current_distances.copy())
    
    def imu_callback(self, msg):
        """Callback for IMU yaw data"""
        self.current_yaw = msg.data
        self.yaw_history.append(self.current_yaw)
    
    def get_filtered_yaw(self):
        """Get temporally filtered yaw angle"""
        if len(self.yaw_history) < 2:
            return self.current_yaw
        return statistics.median(self.yaw_history)
    
    def get_filtered_distances(self):
        """Get temporally filtered distances"""
        if len(self.distance_history) < 2:
            return self.current_distances
        
        filtered = [None, None, None]
        for i in range(3):
            values = [hist[i] for hist in self.distance_history if hist[i] is not None]
            if len(values) >= 2:
                filtered[i] = statistics.median(values)
        
        return filtered
    
    def calculate_anchor_weights_by_orientation(self, yaw_deg):
        """
        Calculate weights for each anchor based on robot orientation
        Higher weight for anchors the robot is facing
        """
        # Normalize yaw to 0-360
        yaw_deg = yaw_deg % 360
        
        # Calculate center points of anchor pairs for orientation reference
        center_12 = (self.anchor1 + self.anchor2) / 2  # [0, 85] - 0°
        center_23 = (self.anchor2 + self.anchor3) / 2  # [100, 170] - 90°
        center_34 = (self.anchor3 + self.anchor4) / 2  # [200, 85] - 180°
        center_41 = (self.anchor4 + self.anchor1) / 2  # [100, 0] - 270°
        
        # Calculate angular distances from current heading to each anchor
        # Anchor 1 (0,0) - typically behind when facing 0°
        angle_to_a1 = 225  # Anchor 1 is at bottom-left
        weight_a1 = self.calculate_directional_weight(yaw_deg, angle_to_a1)
        
        # Anchor 2 (0,170) - typically left when facing 0°  
        angle_to_a2 = 135  # Anchor 2 is at top-left
        weight_a2 = self.calculate_directional_weight(yaw_deg, angle_to_a2)
        
        # Anchor 3 (200,170) - typically right when facing 0°
        angle_to_a3 = 45   # Anchor 3 is at top-right
        weight_a3 = self.calculate_directional_weight(yaw_deg, angle_to_a3)
        
        return [weight_a1, weight_a2, weight_a3]
    
    def calculate_directional_weight(self, current_yaw, anchor_direction):
        """
        Calculate weight based on how much the robot is facing the anchor
        Returns higher weight when facing the anchor
        """
        # Calculate angular difference
        angle_diff = abs(current_yaw - anchor_direction)
        angle_diff = min(angle_diff, 360 - angle_diff)  # Take shortest path
        
        # Convert to weight (facing = high weight, back = low weight)
        if angle_diff <= 45:  # Facing towards anchor
            weight = 1.0
        elif angle_diff <= 90:  # Side view
            weight = 0.7
        elif angle_diff <= 135:  # Diagonal back
            weight = 0.4
        else:  # Mostly behind
            weight = 0.2
        
        return weight
    
    def calculate_position_with_weights(self, distances, weights):
        """
        Calculate position using weighted trilateration
        """
        valid_pairs = []
        
        # Check all anchor pairs
        anchors = [self.anchor1, self.anchor2, self.anchor3]
        
        for i in range(len(anchors)):
            for j in range(i + 1, len(anchors)):
                if distances[i] is not None and distances[j] is not None:
                    # Validate distance readings
                    if (self.min_reasonable_distance <= distances[i] <= self.max_reasonable_distance and
                        self.min_reasonable_distance <= distances[j] <= self.max_reasonable_distance):
                        
                        positions = self.calculate_intersection_points(
                            anchors[i], distances[i], anchors[j], distances[j])
                        
                        if positions:
                            # Calculate combined weight for this pair
                            pair_weight = (weights[i] + weights[j]) / 2
                            for pos in positions:
                                valid_pairs.append((pos, pair_weight))
        
        if not valid_pairs:
            return None
        
        # Select best position using weighted consensus
        return self.select_best_weighted_position(valid_pairs)
    
    def calculate_intersection_points(self, p1, d1, p2, d2):
        """Calculate intersection points of two circles"""
        x1, y1 = p1
        x2, y2 = p2
        
        # Distance between anchors
        D = np.linalg.norm(p2 - p1)
        
        # Check if circles intersect
        if d1 + d2 < D or abs(d1 - d2) > D or D == 0:
            return []
        
        try:
            # Calculate intersection points
            a = (d1**2 - d2**2 + D**2) / (2 * D)
            h_squared = d1**2 - a**2
            
            if h_squared < 0:
                return []
            
            h = math.sqrt(h_squared)
            
            # Point on line between anchors
            px = x1 + a * (x2 - x1) / D
            py = y1 + a * (y2 - y1) / D
            
            # Two possible intersection points
            positions = []
            
            if abs(h) > 0.1:  # Two distinct solutions
                x_pos1 = px + h * (y2 - y1) / D
                y_pos1 = py - h * (x2 - x1) / D
                
                x_pos2 = px - h * (y2 - y1) / D
                y_pos2 = py + h * (x2 - x1) / D
                
                positions.append(np.array([x_pos1, y_pos1]))
                positions.append(np.array([x_pos2, y_pos2]))
            else:  # Tangent case
                positions.append(np.array([px, py]))
            
            return positions
            
        except (ValueError, ZeroDivisionError):
            return []
    
    def select_best_weighted_position(self, weighted_positions):
        """Select best position from weighted candidates"""
        if not weighted_positions:
            return None
        
        # Filter positions within reasonable bounds
        valid_positions = []
        for pos, weight in weighted_positions:
            x, y = pos
            if -50 <= x <= 250 and -50 <= y <= 220:  # Extended bounds
                valid_positions.append((pos, weight))
        
        if not valid_positions:
            valid_positions = weighted_positions  # Use all if none in bounds
        
        # If we have previous position, prefer nearby positions
        if self.last_valid_position is not None:
            scored_positions = []
            for pos, weight in valid_positions:
                distance_penalty = np.linalg.norm(pos - self.last_valid_position) / 100.0
                continuity_weight = max(0.1, 1.0 - distance_penalty)
                final_score = weight * continuity_weight
                scored_positions.append((pos, final_score))
            
            # Select position with highest score
            best_pos, best_score = max(scored_positions, key=lambda x: x[1])
            return best_pos
        
        # No previous position, use weighted average
        total_weight = sum(weight for _, weight in valid_positions)
        if total_weight > 0:
            weighted_sum = sum(pos * weight for pos, weight in valid_positions)
            return weighted_sum / total_weight
        
        return valid_positions[0][0]
    
    def apply_kalman_like_smoothing(self, new_position):
        """Apply Kalman-like smoothing to position estimates"""
        if self.last_valid_position is None:
            return new_position
        
        # Calculate movement distance
        movement = np.linalg.norm(new_position - self.last_valid_position)
        
        # If movement is too large, apply more conservative update
        if movement > self.max_position_jump:
            # Blend with previous position
            alpha = 0.3  # Conservative update
            smoothed = alpha * new_position + (1 - alpha) * self.last_valid_position
            return smoothed
        else:
            # Normal update with slight smoothing
            alpha = 0.8
            smoothed = alpha * new_position + (1 - alpha) * self.last_valid_position
            return smoothed
    
    def fusion_callback(self):
        """Main fusion processing callback"""
        try:
            # Get filtered sensor data
            filtered_distances = self.get_filtered_distances()
            filtered_yaw = self.get_filtered_yaw()
            
            # Check if we have enough valid distance data
            valid_distances = sum(1 for d in filtered_distances if d is not None)
            if valid_distances < self.min_valid_distances:
                self.get_logger().debug(f'Insufficient distance data: {valid_distances}/{self.min_valid_distances}')
                return
            
            # Calculate orientation-based weights
            orientation_weights = self.calculate_anchor_weights_by_orientation(filtered_yaw)
            
            # Calculate position using weighted trilateration
            fused_position = self.calculate_position_with_weights(filtered_distances, orientation_weights)
            
            if fused_position is not None:
                # Apply smoothing
                smoothed_position = self.apply_kalman_like_smoothing(fused_position)
                
                # Update position history
                self.position_history.append(smoothed_position)
                self.last_valid_position = smoothed_position
                
                # Publish result
                pos_msg = Point()
                pos_msg.x = float(smoothed_position[0])
                pos_msg.y = float(smoothed_position[1])
                pos_msg.z = 0.0
                
                self.position_publisher.publish(pos_msg)
                
                self.get_logger().info(f'Fused Position: ({smoothed_position[0]:.1f}, {smoothed_position[1]:.1f}) | '
                            f'Yaw: {filtered_yaw:.1f}° | Distances: {filtered_distances} | '
                            f'Weights: {[f"{w:.2f}" for w in orientation_weights]}')
            
        except Exception as e:
            self.get_logger().error(f'Error in fusion callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        fusion_node = UWBIMUFusion()
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'UWB-IMU Fusion error: {e}')
    finally:
        if 'fusion_node' in locals():
            fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()