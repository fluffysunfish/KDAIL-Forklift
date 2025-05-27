#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import tf2_ros
import math

class SLAMMapper(Node):
    def __init__(self):
        super().__init__('slam_mapper')

        # Map parameters
        self.map_resolution = 0.05  # 5cm per pixel (higher resolution)
        self.map_width = 1000       # 20m width (1000 * 0.02)
        self.map_height = 1000      # 20m height
        self.map_origin_x = -10.0   # Map origin offset
        self.map_origin_y = -10.0

        # Initialize occupancy grid and log-odds map
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # Unknown = -1
        self.log_odds_map = np.ones((self.map_height, self.map_width)) * 0.5  # Prior log-odds = 0.5

        # Map update parameters
        self.occupied_threshold = 0.65  # Probability threshold for occupied
        self.free_threshold = 0.35      # Probability threshold for free
        self.log_odds_occupied = 0.65   # Log-odds increment for occupied cells
        self.log_odds_free = 0.35       # Log-odds decrement for free cells
        self.decay_factor = 0.8         # Decay factor for forgetting old observations
        self.recent_scans_weight = 2.0  # Weight for consistent observations
        self.scan_buffer_size = 3       # Number of recent scans to keep
        self.recent_scans = []          # Buffer to store recent scans
        self.update_count = 0           # Counter for map updates

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Timer to publish map
        self.map_timer = self.create_timer(0.1, self.publish_map)  # 10 Hz for faster updates

        self.get_logger().info("SLAM Mapper initialized")
        self.get_logger().info(f"Map size: {self.map_width}x{self.map_height}, resolution: {self.map_resolution}m")

    def world_to_map(self, x, y):
        """Convert world coordinates to map grid coordinates"""
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y

    def is_valid_map_coord(self, map_x, map_y):
        """Check if map coordinates are within bounds"""
        return 0 <= map_x < self.map_width and 0 <= map_y < self.map_height

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return points

    def apply_decay(self):
        """Apply decay to the log-odds map to gradually forget old observations"""
        diff = self.log_odds_map - 0.5  # Move toward prior (0.5)
        self.log_odds_map -= diff * self.decay_factor

    def is_consistent_observation(self, angle, distance):
        """Check if this observation is consistent with recent scans"""
        if not self.recent_scans:
            return False
        for scan in self.recent_scans:
            for scan_angle, scan_distance in scan:
                if abs(angle - scan_angle) < 0.05:  # ~3 degrees
                    if abs(distance - scan_distance) < 0.1:  # 10cm
                        return True
        return False

    def scan_callback(self, msg):
        """Process laser scan data and update map"""
        try:
            # Get robot pose in anchor1_frame
            transform = self.tf_buffer.lookup_transform(
                'anchor1_frame',
                'laser_frame',  # Use laser_frame directly since transform is provided
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            quat = transform.transform.rotation
            _, _, robot_yaw = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")
            return

        # Convert robot position to map coordinates
        robot_map_x, robot_map_y = self.world_to_map(robot_x, robot_y)

        if not self.is_valid_map_coord(robot_map_x, robot_map_y):
            return

        # Apply decay to the map
        self.apply_decay()

        # Store current scan for consistency checks
        current_scan = []

        # Process each laser ray
        angle = msg.angle_min
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val) or range_val < msg.range_min or range_val > msg.range_max:
                angle += msg.angle_increment
                continue

            # Calculate end point of laser ray
            laser_angle = robot_yaw + angle
            end_x = robot_x + range_val * math.cos(laser_angle)
            end_y = robot_y + range_val * math.sin(laser_angle)

            # Store scan point for consistency check
            current_scan.append((laser_angle, range_val))

            end_map_x, end_map_y = self.world_to_map(end_x, end_y)

            # Ray tracing from robot to obstacle
            ray_points = self.bresenham_line(robot_map_x, robot_map_y, end_map_x, end_map_y)

            # Mark free space along the ray (except the last point)
            for px, py in ray_points[:-1]:
                if self.is_valid_map_coord(px, py):
                    self.log_odds_map[py, px] -= self.log_odds_free

            # Mark obstacle at the end point
            if self.is_valid_map_coord(end_map_x, end_map_y):
                weight = self.recent_scans_weight if self.is_consistent_observation(laser_angle, range_val) else 1.0
                self.log_odds_map[end_map_y, end_map_x] += self.log_odds_occupied * weight

            angle += msg.angle_increment

        # Update recent scans buffer
        self.recent_scans.append(current_scan)
        if len(self.recent_scans) > self.scan_buffer_size:
            self.recent_scans.pop(0)

        # Update occupancy grid
        self.update_occupancy_grid()

        # Increment update counter
        self.update_count += 1

    def update_occupancy_grid(self):
        """Update occupancy grid based on log-odds values"""
        for y in range(self.map_height):
            for x in range(self.map_width):
                log_odds = self.log_odds_map[y, x]
                if log_odds > self.occupied_threshold:
                    self.occupancy_grid[y, x] = 100  # Occupied
                elif log_odds < self.free_threshold:
                    self.occupancy_grid[y, x] = 0    # Free
                else:
                    self.occupancy_grid[y, x] = -1   # Unknown

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles"""
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return 0, 0, yaw  # Only return yaw

    def publish_map(self):
        """Publish the occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'anchor1_frame'

        # Map metadata
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Map data
        map_msg.data = self.occupancy_grid.flatten().tolist()
        self.map_pub.publish(map_msg)

        # Log map status periodically
        if self.update_count % 10 == 0:
            occupied = np.sum(self.occupancy_grid == 100)
            free = np.sum(self.occupancy_grid == 0)
            unknown = np.sum(self.occupancy_grid == -1)
            self.get_logger().info(f'Map status: {occupied} occupied, {free} free, {unknown} unknown cells')

def main(args=None):
    rclpy.init(args=args)
    node = SLAMMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
