#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import Float64
import math

class UWBIMUTransformPublisher(Node):
    def __init__(self):
        super().__init__('uwb_imu_transform_publisher')

        # Declare parameters
        self.declare_parameter('z_position', 0.0)
        self.declare_parameter('publish_frequency', 50.0)  # Hz
        self.declare_parameter('anchor_1_x', 0.0)  # Anchor 1 position (reference point)
        self.declare_parameter('anchor_1_y', 0.0)
        self.declare_parameter('yaw_offset', 0.0)  # IMU yaw offset in degrees

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize position and orientation
        self.robot_x = 0.0  # Robot position in map frame (meters)
        self.robot_y = 0.0
        self.robot_z = self.get_parameter('z_position').get_parameter_value().double_value
        self.robot_yaw = 0.0  # Robot yaw in radians

        # Data received flags
        self.xy_received = False
        self.yaw_received = False

        # Create subscriptions to your specific topics
        self.uwb_subscription = self.create_subscription(
            Point,
            '/fused_xy_uwb',
            self.uwb_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Float64,
            '/imu_1_yaw',
            self.imu_callback,
            10
        )

        # Create a timer for publishing transforms
        freq = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0/freq, self.broadcast_transform)

        self.get_logger().info('UWB-IMU Transform Publisher initialized')
        self.get_logger().info('Subscribing to /fused_xy_uwb for position and /imu_1_yaw for orientation')
        self.get_logger().info('Publishing map -> base_link -> laser_frame transforms')

    def uwb_callback(self, msg):
        """Process UWB position data from /fused_xy_uwb topic"""
        try:
            # Convert UWB coordinates to map coordinates
            # Assuming UWB data is in meters and anchor 1 is at origin
            anchor_1_x = self.get_parameter('anchor_1_x').get_parameter_value().double_value
            anchor_1_y = self.get_parameter('anchor_1_y').get_parameter_value().double_value
            
            # Robot position relative to anchor 1 (which is our map origin)
            self.robot_x = msg.x - anchor_1_x
            self.robot_y = msg.y - anchor_1_y
            self.robot_z = msg.z if hasattr(msg, 'z') and msg.z != 0.0 else self.robot_z

            if not self.xy_received:
                self.get_logger().info(f'First UWB position received: ({self.robot_x:.3f}, {self.robot_y:.3f}, {self.robot_z:.3f})')
                self.xy_received = True

            self.get_logger().debug(f'UWB Position: ({self.robot_x:.3f}, {self.robot_y:.3f})')

        except Exception as e:
            self.get_logger().error(f'Error processing UWB data: {e}')

    def imu_callback(self, msg):
        """Process IMU yaw data from /imu_1_yaw topic"""
        try:
            # Get yaw offset parameter
            yaw_offset_deg = self.get_parameter('yaw_offset').get_parameter_value().double_value
            yaw_offset_rad = math.radians(yaw_offset_deg)

            # Process yaw data
            raw_yaw = msg.data
            
            # Check if data is in degrees or radians
            if abs(raw_yaw) > 2 * math.pi:
                # Likely in degrees
                yaw_rad = math.radians(raw_yaw)
                if not self.yaw_received:
                    self.get_logger().info('IMU yaw data appears to be in degrees')
            else:
                # Likely in radians
                yaw_rad = raw_yaw
                if not self.yaw_received:
                    self.get_logger().info('IMU yaw data appears to be in radians')

            # Apply offset and store
            self.robot_yaw = yaw_rad + yaw_offset_rad

            if not self.yaw_received:
                self.get_logger().info(f'First IMU yaw received: {math.degrees(self.robot_yaw):.2f}°')
                self.yaw_received = True

            self.get_logger().debug(f'IMU Yaw: {math.degrees(self.robot_yaw):.2f}°')

        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')

    def broadcast_transform(self):
        """Broadcast transforms: map -> base_link -> laser_frame"""
        # Only broadcast if we have received data from both sources
        if not (self.xy_received and self.yaw_received):
            return

        try:
            current_time = self.get_clock().now().to_msg()

            # Transform 1: map -> base_link
            t1 = TransformStamped()
            t1.header.stamp = current_time
            t1.header.frame_id = 'map'
            t1.child_frame_id = 'base_link'
            
            # Position
            t1.transform.translation.x = self.robot_x
            t1.transform.translation.y = self.robot_y
            t1.transform.translation.z = self.robot_z
            
            # Orientation (quaternion from yaw)
            t1.transform.rotation.x = 0.0
            t1.transform.rotation.y = 0.0
            t1.transform.rotation.z = math.sin(self.robot_yaw / 2.0)
            t1.transform.rotation.w = math.cos(self.robot_yaw / 2.0)

            # Transform 2: base_link -> laser_frame
            t2 = TransformStamped()
            t2.header.stamp = current_time
            t2.header.frame_id = 'base_link'
            t2.child_frame_id = 'laser_frame'
            
            # Static transform (adjust these values based on your robot's physical layout)
            t2.transform.translation.x = 0.0  # Laser offset from robot center
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 0.0
            
            # No additional rotation (laser aligned with robot)
            t2.transform.rotation.x = 0.0
            t2.transform.rotation.y = 0.0
            t2.transform.rotation.z = 0.0
            t2.transform.rotation.w = 1.0

            # Send both transforms
            self.tf_broadcaster.sendTransform([t1, t2])

        except Exception as e:
            self.get_logger().error(f'Error broadcasting transforms: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UWBIMUTransformPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()