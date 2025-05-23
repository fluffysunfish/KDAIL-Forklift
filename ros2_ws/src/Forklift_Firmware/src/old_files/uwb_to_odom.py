#!/usr/bin/env python3

import rclpy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
from tf_transformations import quaternion_from_euler

class UWBtoOdom(Node):
    def __init__(self):
        super().__init__('uwb_to_odom_node')

        # Create subscriber for UWB data
        self.uwb_sub = self.create_subscription(
            Float32MultiArray,
            '/UWB_xy_Data',
            self.uwb_callback,
            10
        )

        # Create publisher for odometry data
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize previous position for velocity calculation
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_time = self.get_clock().now()

        self.get_logger().info('UWB to Odometry converter node started')

    def uwb_callback(self, msg):
        current_time = self.get_clock().now()

        # Extract x, y coordinates from UWB data
        x = msg.data[0]  # Assuming first value is x
        y = msg.data[1]  # Assuming second value is y

        # Calculate time difference
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds

        # Calculate velocities (if dt is not zero)
        if dt > 0:
            vx = (x - self.prev_x) / dt
            vy = (y - self.prev_y) / dt
        else:
            vx = 0.0
            vy = 0.0

        # Create and fill odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Set orientation (assuming robot is moving in the direction of motion)
        yaw = math.atan2(vy, vx) if (vx != 0 or vy != 0) else 0.0
        q = quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set velocities
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

        # Publish odometry message
        self.odom_pub.publish(odom)

        # Update previous values for next iteration
        self.prev_x = x
        self.prev_y = y
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    uwb_to_odom = UWBtoOdom()

    try:
        rclpy.spin(uwb_to_odom)
    except KeyboardInterrupt:
        pass
    finally:
        uwb_to_odom.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
