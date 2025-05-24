#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point, PointStamped
from std_msgs.msg import Float64
import math

class DynamicUWBPositioning(Node):
    def __init__(self):
        super().__init__('dynamic_uwb_positioning')

        # Declare parameters
        self.declare_parameter('z_position', 0.0)
        self.declare_parameter('publish_frequency', 20.0)  # Hz

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Anchor coordinate system (in cm, from uwb_fusion)
        # Anchor 1 is at origin (0,0) - this will be our map origin
        # Anchor 2 is at (0,170) - positive Y direction
        # Anchor 3 is at (200,170) - positive X and Y
        # Anchor 4 is at (200,0) - positive X direction
        
        # Initialize position and orientation in map frame
        self.map_x = 0.0  # meters, relative to anchor 1
        self.map_y = 0.0  # meters, relative to anchor 1
        self.map_z = self.get_parameter('z_position').get_parameter_value().double_value
        self.map_yaw = 0.0  # radians, 0° = facing north (towards anchor 2)
        
        # Laser frame rotation offset (85 degrees counter-clockwise)
        self.laser_rotation_offset = math.radians(85)  # Convert 85 degrees to radians

        # Calculate initial quaternion (no rotation)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        # Data received flags
        self.xy_received = False
        self.yaw_received = False

        # Create subscriptions
        self.xy_subscription = self.create_subscription(
            Point,
            '/uwb_fused_xy',
            self.xy_callback,
            10
        )

        self.yaw_subscription = self.create_subscription(
            Float64,
            '/imu_1_yaw',
            self.yaw_callback,
            10
        )

        # Create a timer with frequency from parameter
        freq = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0/freq, self.broadcast_transform)

        self.get_logger().info('Dynamic UWB positioning node initialized')
        self.get_logger().info('Map frame: Anchor 1 at origin, Y-axis towards Anchor 2')
        self.get_logger().info('Coordinate system: 0° yaw = facing north (Anchor1->Anchor2), laser frame rotated +85°')
        self.get_logger().info('Waiting for UWB position data on /uwb_fused_xy and IMU yaw data on /imu_1_yaw')

    def xy_callback(self, msg):
        """Update position from incoming Point message (x, y from UWB)"""
        try:
            # Extract raw UWB coordinates (in cm from fusion system)
            if hasattr(msg, 'point'):
                # PointStamped message
                uwb_x_cm = msg.point.x
                uwb_y_cm = msg.point.y
                if hasattr(msg.point, 'z') and msg.point.z is not None:
                    uwb_z_cm = msg.point.z
                else:
                    uwb_z_cm = 0.0
            else:
                # Point message
                uwb_x_cm = msg.x
                uwb_y_cm = msg.y
                uwb_z_cm = msg.z if hasattr(msg, 'z') and msg.z is not None else 0.0

            # Convert UWB coordinates to map coordinates
            # UWB system: Anchor 1 at (0,0), Anchor 2 at (0,170), etc.
            # Map system: Anchor 1 at origin, Y-axis points towards Anchor 2
            
            # Direct conversion: UWB coordinates are already in the correct orientation
            # with Anchor 1 as origin and positive Y towards Anchor 2
            self.map_x = uwb_x_cm / 100.0  # Convert cm to meters
            self.map_y = uwb_y_cm / 100.0  # Convert cm to meters
            self.map_z = uwb_z_cm / 100.0 if uwb_z_cm != 0.0 else self.get_parameter('z_position').get_parameter_value().double_value

            if not self.xy_received:
                self.get_logger().info(f'First UWB position received: map({self.map_x:.3f}, {self.map_y:.3f}, {self.map_z:.3f})')
                self.xy_received = True

            self.get_logger().debug(f'UWB({uwb_x_cm:.1f}, {uwb_y_cm:.1f}) -> Map({self.map_x:.3f}, {self.map_y:.3f})')

        except Exception as e:
            self.get_logger().error(f'Error processing UWB position data: {e}')

    def yaw_callback(self, msg):
        """Update yaw angle from incoming Float64 message (yaw from IMU)"""
        try:
            # Get raw IMU yaw
            raw_yaw = msg.data
            
            # Check if the yaw data is in degrees or radians
            if abs(raw_yaw) > 2 * math.pi:
                # Likely in degrees, convert to radians
                imu_yaw_rad = math.radians(raw_yaw)
                if not self.yaw_received:
                    self.get_logger().info('IMU yaw data appears to be in degrees, converting to radians')
            else:
                # Likely already in radians
                imu_yaw_rad = raw_yaw
                if not self.yaw_received:
                    self.get_logger().info('IMU yaw data appears to be in radians')

            # Convert IMU yaw to map frame yaw
            # In the UWB coordinate system:
            # - 0° should mean facing from Anchor 1 towards Anchor 2 (positive Y direction)
            # - 90° should mean facing from Anchor 1 towards Anchor 4 (positive X direction)
            # - The IMU may have a different reference, so we align it with our map
            
            # Assuming IMU 0° aligns with our desired 0° (facing anchor 1->2)
            # If not, add an offset here: self.map_yaw = imu_yaw_rad + offset
            self.map_yaw = imu_yaw_rad

            # Apply laser frame rotation offset (85 degrees counter-clockwise)
            laser_frame_yaw = self.map_yaw + self.laser_rotation_offset

            # Calculate quaternion from laser frame yaw angle
            self.qx = 0.0
            self.qy = 0.0
            self.qz = math.sin(laser_frame_yaw / 2.0)
            self.qw = math.cos(laser_frame_yaw / 2.0)

            if not self.yaw_received:
                self.get_logger().info(f'First IMU yaw received: {math.degrees(self.map_yaw):.2f}° (map frame)')
                self.get_logger().info(f'Laser frame rotation: {math.degrees(self.laser_rotation_offset):.1f}° counter-clockwise')
                self.get_logger().info(f'Final laser frame yaw: {math.degrees(laser_frame_yaw):.2f}°')
                self.yaw_received = True

            self.get_logger().debug(f'IMU: {math.degrees(imu_yaw_rad):.1f}° -> Map: {math.degrees(self.map_yaw):.1f}° -> Laser: {math.degrees(laser_frame_yaw):.1f}°')

        except Exception as e:
            self.get_logger().error(f'Error processing IMU yaw data: {e}')

    def broadcast_transform(self):
        """Broadcast the transform from map to laser_frame"""
        # Only broadcast if we have received data from both sources
        if not (self.xy_received and self.yaw_received):
            if not self.xy_received:
                self.get_logger().debug('Waiting for UWB position data...')
            if not self.yaw_received:
                self.get_logger().debug('Waiting for IMU yaw data...')
            return

        try:
            t = TransformStamped()

            # Fill in the header
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'  # Fixed map frame with Anchor 1 as origin
            t.child_frame_id = 'laser_frame'  # Robot/laser frame that moves

            # Set translation (robot position in map frame)
            t.transform.translation.x = self.map_x
            t.transform.translation.y = self.map_y
            t.transform.translation.z = self.map_z

            # Set rotation (robot orientation in map frame)
            t.transform.rotation.x = self.qx
            t.transform.rotation.y = self.qy
            t.transform.rotation.z = self.qz
            t.transform.rotation.w = self.qw

            # Send the transform
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Error broadcasting transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicUWBPositioning()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
