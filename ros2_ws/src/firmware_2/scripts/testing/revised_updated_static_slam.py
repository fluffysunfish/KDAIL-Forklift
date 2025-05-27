#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class StaticSLAM(Node):
    def __init__(self):
        super().__init__('static_slam')

        # Map parameters
        self.map_resolution = 0.05
        self.map_width = 1000
        self.map_height = 1000
        self.map_origin_x = -self.map_width * self.map_resolution / 2.0
        self.map_origin_y = -self.map_height * self.map_resolution / 2.0

        self.occupancy_map = np.zeros((self.map_height, self.map_width), dtype=np.int8) - 1
        self.log_odds_map = np.ones((self.map_height, self.map_width)) * 0.5

        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.map_timer = self.create_timer(0.5, self.publish_map)

        self.occupied_threshold = 0.65
        self.free_threshold = 0.35
        self.log_odds_occupied = 0.65
        self.log_odds_free = 0.35
        self.log_odds_prior = 0.5

        self.decay_factor = 0.8
        self.update_count = 0
        self.recent_scans_weight = 2.0
        self.scan_buffer_size = 3
        self.recent_scans = []

        # Dynamic transform variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # Radians
        self.last_position_time = self.get_clock().now()
        self.last_yaw_time = self.get_clock().now()

        # Subscribers for dynamic pose
        self.create_subscription(Point, '/final_xy', self.position_callback, 10)
        self.create_subscription(Float64, '/imu_1_yaw', self.yaw_callback, 10)

        self.get_logger().info('Dynamic SLAM node initialized')

    def position_callback(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.last_position_time = self.get_clock().now()

    def yaw_callback(self, msg):
        self.robot_yaw = math.radians(msg.data)
        self.last_yaw_time = self.get_clock().now()

    def scan_callback(self, msg):
        self.get_logger().info(f'Processing scan {self.update_count + 1}')
        self.broadcast_transform()

        # Robot's position in grid coordinates
        robot_map_x = int((self.robot_x - self.map_origin_x) / self.map_resolution)
        robot_map_y = int((self.robot_y - self.map_origin_y) / self.map_resolution)
        current_scan = []
        self.apply_decay()

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            # Point in laser_frame
            x_laser = distance * math.cos(angle)
            y_laser = distance * math.sin(angle)
            # Transform to map frame
            cos_yaw = math.cos(self.robot_yaw)
            sin_yaw = math.sin(self.robot_yaw)
            x_map = self.robot_x + x_laser * cos_yaw - y_laser * sin_yaw
            y_map = self.robot_y + x_laser * sin_yaw + y_laser * cos_yaw
            # Convert to grid indices
            map_x = int((x_map - self.map_origin_x) / self.map_resolution)
            map_y = int((y_map - self.map_origin_y) / self.map_resolution)

            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                current_scan.append((angle, distance))
                weight = self.recent_scans_weight if self.is_consistent_observation(angle, distance) else 1.0
                self.log_odds_map[map_y, map_x] += self.log_odds_occupied * weight
                self.mark_free_space(robot_map_x, robot_map_y, map_x, map_y)

        self.recent_scans.append(current_scan)
        if len(self.recent_scans) > self.scan_buffer_size:
            self.recent_scans.pop(0)

        self.update_occupancy_grid()
        self.update_count += 1
        self.publish_map()


    def apply_decay(self):
        diff = self.log_odds_map - self.log_odds_prior
        self.log_odds_map -= diff * self.decay_factor

    def is_consistent_observation(self, angle, distance):
        if not self.recent_scans:
            return False
        for scan in self.recent_scans:
            for scan_angle, scan_distance in scan:
                if abs(angle - scan_angle) < 0.05 and abs(distance - scan_distance) < 0.1:
                    return True
        return False

    def mark_free_space(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while (x != x1 or y != y1):
            if x == x1 and y == y1:
                break
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
        for y in range(self.map_height):
            for x in range(self.map_width):
                log_odds = self.log_odds_map[y, x]
                if log_odds > self.occupied_threshold:
                    self.occupancy_map[y, x] = 100
                elif log_odds < self.free_threshold:
                    self.occupancy_map[y, x] = 0
                else:
                    self.occupancy_map[y, x] = -1

    def broadcast_transform(self):
        now = self.get_clock().now()
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'laser_frame'

        # Use last known position and yaw
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0

        # Yaw to quaternion
        qz = math.sin(self.robot_yaw / 2.0)
        qw = math.cos(self.robot_yaw / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # Warn if data is stale
        if (now - self.last_position_time).nanoseconds > 1e9:
            self.get_logger().warn("Position data is stale")
        if (now - self.last_yaw_time).nanoseconds > 1e9:
            self.get_logger().warn("Yaw data is stale")

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = self.occupancy_map.flatten().tolist()
        self.map_publisher.publish(map_msg)

        if self.update_count % 10 == 0:
            occupied = np.sum(self.occupancy_map == 100)
            free = np.sum(self.occupancy_map == 0)
            unknown = np.sum(self.occupancy_map == -1)
            self.get_logger().info(f'Map status: {occupied} occupied, {free} free, {unknown} unknown')

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
