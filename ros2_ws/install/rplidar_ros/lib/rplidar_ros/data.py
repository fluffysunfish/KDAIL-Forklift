#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rplidar import RPLidar
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Define LiDAR parameters
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200  # Default baud rate for RPLIDAR A2 M8
TARGET_ANGLES = {0, 90, 180, 270}  # Angles for special visualization
ANGLE_TOLERANCE = 2  # Allow small angle variations

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.lidar = RPLidar(PORT, baudrate=BAUDRATE)
        
        # Frame ID for all published messages
        self.frame_id = "laser_frame"

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.line_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.text_pub = self.create_publisher(Marker, 'text_marker', 10)  # New publisher for text labels

        # Timer to fetch LiDAR data
        self.timer = self.create_timer(0.1, self.publish_lidar_data)

    def publish_lidar_data(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * math.pi
        scan_msg.angle_increment = math.radians(1)
        scan_msg.range_min = 0.15  # Minimum valid range (15 cm)
        scan_msg.range_max = 12.0  # Maximum range of RPLIDAR

        ranges = [float('inf')] * 360  # Default values for LaserScan
        line_marker = Marker()
        text_marker = Marker()

        # Configure line marker (for 0°, 90°, 180°, 270°)
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "lines"
        line_marker.id = 0
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.a = 1.0  # Fully visible
        line_marker.color.r = 1.0  # Red color

        # Configure text marker (for labels)
        text_marker.header.frame_id = self.frame_id
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "labels"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.scale.z = 0.2  # Text size
        text_marker.color.a = 1.0  # Fully visible
        text_marker.color.b = 1.0  # Blue text

        for scan in self.lidar.iter_scans():
            line_marker.points.clear()
            text_marker.points.clear()
            text_marker.text = ""

            for (_, angle, distance) in scan:
                if distance == 0:  # Ignore invalid readings
                    continue

                rad_angle = math.radians(angle)
                x = (distance / 1000.0) * math.cos(rad_angle)  # Convert mm to meters
                y = (distance / 1000.0) * math.sin(rad_angle)

                # Store distance in LaserScan message
                index = int(angle) % 360
                ranges[index] = distance / 1000.0

                # Check if angle matches a target angle
                for target in TARGET_ANGLES:
                    if abs(int(angle) - target) <= ANGLE_TOLERANCE:
                        # Create a line from (0,0) to the target point
                        line_marker.points.append(Point(x=0.0, y=0.0, z=0.0))
                        line_marker.points.append(Point(x=x, y=y, z=0.0))

                        # Add text label at the endpoint
                        label = Marker()
                        label.header.frame_id = self.frame_id
                        label.header.stamp = self.get_clock().now().to_msg()
                        label.ns = "labels"
                        label.id = target
                        label.type = Marker.TEXT_VIEW_FACING
                        label.action = Marker.ADD
                        label.pose.position.x = x
                        label.pose.position.y = y
                        label.pose.position.z = 0.1  # Slightly above the point
                        label.scale.z = 0.2  # Text size
                        label.color.a = 1.0  # Fully visible
                        label.color.b = 1.0  # Blue text
                        label.color.g = 1.0
                        label.color.r = 1.0
                        label.text = f"{target}°: {distance/1000.0:.2f} m"
                        self.text_pub.publish(label)

            # Publish messages
            scan_msg.ranges = ranges
            self.scan_pub.publish(scan_msg)
            self.line_pub.publish(line_marker)

    def shutdown(self):
        self.get_logger().info("Stopping LiDAR...")
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
