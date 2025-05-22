#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time

class StaticSLAM(Node):
    def __init__(self):
        super().__init__('static_slam')

        # Map parameters
        self.map_resolution = 0.05  # meters per pixel
        self.map_width = 1000      # pixels
        self.map_height = 1000     # pixels
        self.map_origin_x = -self.map_width * self.map_resolution / 2.0
        self.map_origin_y = -self.map_height * self.map_resolution / 2.0

        # Create empty map
        self.occupancy_map = np.zeros((self.map_height, self.map_width), dtype=np.int8) - 1  # -1 is unknown

        # Subscriber for laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Publisher for occupancy grid
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            'map',
            10)

        # No longer broadcasting TF since uwb_xy.py handles it
        # self.tf_broadcaster = TransformBroadcaster(self)
        
        # We need to receive the transform that uwb_xy.py publishes
        from tf2_ros import TransformListener, Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for publishing map
        self.map_timer = self.create_timer(0.5, self.publish_map)

        # Logging
        self.get_logger().info('Real-time SLAM node initialized')

        # Map update parameters
        self.occupied_threshold = 0.65  # Probability threshold for a cell to be considered occupied
        self.free_threshold = 0.35      # Probability threshold for a cell to be considered free
        self.log_odds_occupied = 0.65   # Log odds for occupied cells
        self.log_odds_free = 0.35       # Log odds for free cells
        self.log_odds_prior = 0.5       # Prior probability
        self.log_odds_map = np.ones((self.map_height, self.map_width)) * self.log_odds_prior

        # Real-time update parameters
        self.decay_factor = 0.2        # How quickly to forget old observations
        self.update_count = 0
        self.recent_scans_weight = 2.0  # Weight recent scans more heavily
        self.scan_buffer_size = 3       # Number of recent scans to keep for comparison
        self.recent_scans = []          # Buffer to store recent scans

    def scan_callback(self, msg):
        """Process incoming laser scan data and update the map in real-time"""
        self.get_logger().info(f'Processing scan {self.update_count + 1}')

        # Get transform from the UWB node
        try:
            # Look up transform from map to laser_frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                'laser_frame',
                rclpy.time.Time())
            
            # Extract position from transform
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Log the robot position
            self.get_logger().info(f'Robot position: ({robot_x:.2f}, {robot_y:.2f})')
        except Exception as e:
            self.get_logger().error(f'Failed to lookup transform: {e}')
            return

        # Center of the map in map coordinates
        center_x = self.map_width // 2
        center_y = self.map_height // 2
        
        # Calculate robot position in map coordinates
        robot_map_x = int(center_x + robot_x / self.map_resolution)
        robot_map_y = int(center_y + robot_y / self.map_resolution)

        # Store current scan for comparison
        current_scan = []

        # Apply decay to the entire map to gradually forget old observations
        # This helps with adapting to moving objects
        self.apply_decay()

        # Process each laser reading
        for i, distance in enumerate(msg.ranges):
            # Skip invalid readings
            if math.isinf(distance) or math.isnan(distance) or distance < msg.range_min or distance > msg.range_max:
                continue

            # Calculate angle
            angle = msg.angle_min + i * msg.angle_increment

            # Convert polar to cartesian coordinates
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            # Store scan point
            current_scan.append((angle, distance))

            # Convert to map coordinates, accounting for robot position
            map_x = int(robot_map_x + x / self.map_resolution)
            map_y = int(robot_map_y + y / self.map_resolution)

            # Check if point is within map bounds
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                # Mark endpoint as occupied
                # Weight more heavily if this is a consistent observation
                weight = self.recent_scans_weight if self.is_consistent_observation(angle, distance) else 1.0
                self.log_odds_map[map_y, map_x] += self.log_odds_occupied * weight

                # Bresenham's line algorithm to mark free space
                self.mark_free_space(robot_map_x, robot_map_y, map_x, map_y)

        # Update the scan buffer
        self.recent_scans.append(current_scan)
        if len(self.recent_scans) > self.scan_buffer_size:
            self.recent_scans.pop(0)

        # Update occupancy grid based on log odds
        self.update_occupancy_grid()

        # Increment update counter
        self.update_count += 1

        # Publish map after each update
        self.publish_map()

    def apply_decay(self):
        """Apply decay to the log odds map to gradually forget old observations"""
        # Move all values slightly toward the prior
        diff = self.log_odds_map - self.log_odds_prior
        self.log_odds_map -= diff * self.decay_factor

    def is_consistent_observation(self, angle, distance):
        """Check if this observation is consistent with recent scans"""
        if not self.recent_scans:
            return False

        # Check if this point is similar to points in recent scans
        for scan in self.recent_scans:
            for scan_angle, scan_distance in scan:
                # If angles are close, compare distances
                if abs(angle - scan_angle) < 0.05:  # ~3 degrees
                    # If distances are similar, it's a consistent observation
                    if abs(distance - scan_distance) < 0.1:  # 10cm
                        return True
        return False

    def mark_free_space(self, x0, y0, x1, y1):
        """Use Bresenham's line algorithm to mark cells as free along the line"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while (x != x1 or y != y1):
            # Skip the endpoint (which is occupied)
            if x == x1 and y == y1:
                break

            # Mark as free
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.log_odds_map[y, x] -= self.log_odds_free

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def update_occupancy_grid(self):
        """Update occupancy grid based on log odds values"""
        # Convert log odds to probabilities
        for y in range(self.map_height):
            for x in range(self.map_width):
                log_odds = self.log_odds_map[y, x]

                if log_odds > self.occupied_threshold:
                    self.occupancy_map[y, x] = 100  # Occupied (100)
                elif log_odds < self.free_threshold:
                    self.occupancy_map[y, x] = 0    # Free (0)
                else:
                    self.occupancy_map[y, x] = -1   # Unknown (-1)

    def broadcast_transform(self):
        """
        Transform broadcasting is now handled by uwb_xy.py
        This method is kept for compatibility but does nothing
        """
        pass

    def publish_map(self):
        """Publish the occupancy grid map"""
        map_msg = OccupancyGrid()

        # Header
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

        # Map metadata
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatten the numpy array to a list
        map_msg.data = self.occupancy_map.flatten().tolist()

        # Publish the map
        self.map_publisher.publish(map_msg)

        # Log map status periodically (every 10 updates)
        if self.update_count % 10 == 0:
            occupied = np.sum(self.occupancy_map == 100)
            free = np.sum(self.occupancy_map == 0)
            unknown = np.sum(self.occupancy_map == -1)
            self.get_logger().info(f'Map status: {occupied} occupied, {free} free, {unknown} unknown cells')

def main(args=None):
    rclpy.init(args=args)
    node = StaticSLAM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
