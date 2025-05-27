#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class UWBStaticSLAM(Node):
    def __init__(self):
        super().__init__('uwb_static_slam')

        # Declare parameters
        self.declare_parameter('map_resolution', 0.05)  # meters per pixel
        self.declare_parameter('map_width', 2000)       # pixels (100m at 0.05m/pixel)
        self.declare_parameter('map_height', 2000)      # pixels (100m at 0.05m/pixel)
        self.declare_parameter('update_frequency', 10.0)  # Hz
        self.declare_parameter('occupied_threshold', 0.65)
        self.declare_parameter('free_threshold', 0.35)
        self.declare_parameter('log_odds_occupied', 0.4)
        self.declare_parameter('log_odds_free', 0.2)
        self.declare_parameter('max_range', 12.0)  # Maximum range to consider from laser

        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.occupied_threshold = self.get_parameter('occupied_threshold').get_parameter_value().double_value
        self.free_threshold = self.get_parameter('free_threshold').get_parameter_value().double_value
        self.log_odds_occupied = self.get_parameter('log_odds_occupied').get_parameter_value().double_value
        self.log_odds_free = self.get_parameter('log_odds_free').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value

        # Map origin (center of the map)
        self.map_origin_x = -self.map_width * self.map_resolution / 2.0
        self.map_origin_y = -self.map_height * self.map_resolution / 2.0

        # Initialize maps
        self.log_odds_map = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        self.occupancy_map = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # -1 = unknown

        # Transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # Timer for publishing map
        update_freq = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.map_timer = self.create_timer(1.0/update_freq, self.publish_map)

        # Statistics
        self.scan_count = 0
        self.last_robot_position = None

        self.get_logger().info('UWB Static SLAM node initialized')
        self.get_logger().info(f'Map size: {self.map_width}x{self.map_height} pixels ({self.map_width*self.map_resolution:.1f}x{self.map_height*self.map_resolution:.1f} meters)')
        self.get_logger().info(f'Map resolution: {self.map_resolution} meters/pixel')
        self.get_logger().info(f'Max laser range: {self.max_range} meters')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        try:
            # Get transform from map to laser_frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                'laser_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Extract robot position and orientation
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            robot_yaw = self.quaternion_to_yaw(transform.transform.rotation)

            # Convert robot position to map coordinates
            robot_map_x = int((robot_x - self.map_origin_x) / self.map_resolution)
            robot_map_y = int((robot_y - self.map_origin_y) / self.map_resolution)

            # Check if robot is within map bounds
            if not (0 <= robot_map_x < self.map_width and 0 <= robot_map_y < self.map_height):
                self.get_logger().warn(f'Robot position ({robot_x:.2f}, {robot_y:.2f}) is outside map bounds')
                return

            # Process laser scan
            self.process_laser_scan(msg, robot_map_x, robot_map_y, robot_yaw)
            
            # Update statistics
            self.scan_count += 1
            self.last_robot_position = (robot_x, robot_y, robot_yaw)

            # Log progress
            if self.scan_count % 50 == 0:
                self.get_logger().info(f'Processed {self.scan_count} laser scans. Robot at ({robot_x:.2f}, {robot_y:.2f})')

        except Exception as e:
            self.get_logger().debug(f'Transform lookup failed: {e}')

    def process_laser_scan(self, scan_msg, robot_map_x, robot_map_y, robot_yaw):
        """Process laser scan and update occupancy map"""
        for i, distance in enumerate(scan_msg.ranges):
            # Skip invalid readings
            if (math.isinf(distance) or math.isnan(distance) or 
                distance < scan_msg.range_min or distance > scan_msg.range_max or
                distance > self.max_range):
                continue

            # Calculate laser beam angle
            beam_angle = scan_msg.angle_min + i * scan_msg.angle_increment
            global_angle = robot_yaw + beam_angle

            # Calculate endpoint in map coordinates
            endpoint_x = robot_map_x + int((distance * math.cos(global_angle)) / self.map_resolution)
            endpoint_y = robot_map_y + int((distance * math.sin(global_angle)) / self.map_resolution)

            # Check if endpoint is within map bounds
            if 0 <= endpoint_x < self.map_width and 0 <= endpoint_y < self.map_height:
                # Mark endpoint as occupied
                self.log_odds_map[endpoint_y, endpoint_x] += self.log_odds_occupied

                # Mark free space along the ray using Bresenham's algorithm
                self.mark_ray_free(robot_map_x, robot_map_y, endpoint_x, endpoint_y)

        # Update occupancy grid from log odds
        self.update_occupancy_grid()

    def mark_ray_free(self, x0, y0, x1, y1):
        """Mark cells as free along a ray using Bresenham's line algorithm"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while x != x1 or y != y1:
            # Mark current cell as free (but not the endpoint)
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.log_odds_map[y, x] -= self.log_odds_free

            # Move to next cell
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def update_occupancy_grid(self):
        """Convert log odds to occupancy probabilities"""
        for y in range(self.map_height):
            for x in range(self.map_width):
                log_odds = self.log_odds_map[y, x]
                
                # Convert log odds to probability
                if log_odds > 0:
                    prob = 1.0 / (1.0 + math.exp(-log_odds))
                else:
                    prob = math.exp(log_odds) / (1.0 + math.exp(log_odds))

                # Convert probability to occupancy grid values
                if prob > self.occupied_threshold:
                    self.occupancy_map[y, x] = 100  # Occupied
                elif prob < self.free_threshold:
                    self.occupancy_map[y, x] = 0    # Free
                else:
                    self.occupancy_map[y, x] = -1   # Unknown

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

        # Map data
        map_msg.data = self.occupancy_map.flatten().tolist()

        # Publish
        self.map_publisher.publish(map_msg)

        # Log statistics periodically
        if self.scan_count > 0 and self.scan_count % 100 == 0:
            occupied = np.sum(self.occupancy_map == 100)
            free = np.sum(self.occupancy_map == 0)
            unknown = np.sum(self.occupancy_map == -1)
            total = self.map_width * self.map_height
            
            self.get_logger().info(f'Map stats - Occupied: {occupied} ({100*occupied/total:.1f}%), '
                                 f'Free: {free} ({100*free/total:.1f}%), '
                                 f'Unknown: {unknown} ({100*unknown/total:.1f}%)')

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        
        # Calculate yaw from quaternion
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = UWBStaticSLAM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()