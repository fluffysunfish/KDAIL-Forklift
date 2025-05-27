#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PointStamped
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class TransformPublisher(Node):
    def __init__(self):
        super().__init__('transform_publisher')

        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize variables with default values
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_xy_time = None
        self.last_yaw_time = None
        self.data_timeout = 0.2  # Timeout for data freshness (seconds)

        # Subscribers with queue size optimized for 10 Hz
        self.xy_sub = self.create_subscription(
            PointStamped,
            '/final_xy',
            self.xy_callback,
            10  # Queue size matches expected message rate
        )

        self.yaw_sub = self.create_subscription(
            Float64,
            '/imu_1_yaw',
            self.yaw_callback,
            10  # Queue size matches expected message rate
        )

        # Timer to publish transform at 10 Hz (matching input data rate)
        self.timer = self.create_timer(0.1, self.publish_transform)

        self.get_logger().info("Transform Publisher initialized")

    def xy_callback(self, msg):
        """Callback Branfor UWB position data"""
        self.current_x = msg.point.x
        self.current_y = msg.point.y
        self.last_xy_time = self.get_clock().now()
        self.get_logger().info(f"Received position: x={self.current_x:.2f}, y={self.current_y:.2f}")

    def yaw_callback(self, msg):
        """Callback for IMU yaw data (in degrees)"""
        self.current_yaw = math.radians(msg.data)  # Convert degrees to radians
        self.last_yaw_time = self.get_clock().now()
        self.get_logger().info(f"Received yaw: {msg.data:.2f} degrees ({self.current_yaw:.2f} radians)")

    def publish_transform(self):
        """Publish transform from anchor1_frame to base_link and base_link to laser_frame"""
        current_time = self.get_clock().now()

        # Check for data freshness, revert to defaults if data is missing or stale
        use_defaults = False
        if self.last_xy_time is None or self.last_yaw_time is None:
            use_defaults = True
            self.get_logger().warn("No position or yaw data received, using defaults (0, 0, 0)")
        else:
            xy_age = (current_time - self.last_xy_time).nanoseconds / 1e9
            yaw_age = (current_time - self.last_yaw_time).nanoseconds / 1e9
            if xy_age > self.data_timeout or yaw_age > self.data_timeout:
                use_defaults = True
                self.get_logger().warn(f"Data too old: xy_age={xy_age:.2f}s, yaw_age={yaw_age:.2f}s, using defaults (0, 0, 0)")

        # Use default values if necessary
        x = 0.0 if use_defaults else self.current_x
        y = 0.0 if use_defaults else self.current_y
        yaw = 0.0 if use_defaults else self.current_yaw

        # Publish transform: anchor1_frame -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'anchor1_frame'
        t.child_frame_id = 'base_link'

        # Translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Rotation (yaw in radians)
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

        # Publish transform: base_link -> laser_frame (static transform)
        laser_t = TransformStamped()
        laser_t.header.stamp = current_time.to_msg()
        laser_t.header.frame_id = 'base_link'
        laser_t.child_frame_id = 'laser_frame'

        # Static transform (assuming LiDAR is at robot center)
        laser_t.transform.translation.x = 0.0
        laser_t.transform.translation.y = 0.0
        laser_t.transform.translation.z = 0.0
        laser_t.transform.rotation.x = 0.0
        laser_t.transform.rotation.y = 0.0
        laser_t.transform.rotation.z = 0.0
        laser_t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(laser_t)
        self.get_logger().debug(f"Published transforms: anchor1_frame -> base_link -> laser_frame (x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f} degrees)")

def main(args=None):
    rclpy.init(args=args)
    node = TransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
