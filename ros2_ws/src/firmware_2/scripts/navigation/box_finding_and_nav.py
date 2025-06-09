#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
import numpy as np
import math
import time
import threading

class RobotNavigationController(Node):
    def __init__(self):
        super().__init__('robot_navigation_controller')

        # Robot state
        self.robot_state = "IDLE"  # IDLE, ROTATING, MOVING_FORWARD, ARRIVED, WAITING_USER_INPUT
        self.target_angle = None
        self.target_distance = None
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        self.moved_distance = 0.0
        self.last_scan_time = time.time()
        self.rotation_start_time = None
        self.movement_start_time = None

        # Navigation parameters
        self.angle_tolerance = 3.0  # degrees
        self.distance_tolerance = 0.3  # meters (30cm stopping distance)
        self.max_detection_distance = 3.0  # Maximum distance to consider for navigation
        self.target_lidar_angle = 90.0  # Target alignment angle (90° = front of robot)

        # Speed parameters (you can modify these)
        self.angular_speed = 85.0  # degrees per second
        self.linear_speed = 0.42   # meters per second

        # LiDAR processing variables
        self.distances_array = []
        self.angle_increment = 0.0
        self.lidar_angle_min = 0.0
        self.lidar_angle_max = 0.0

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu_1_yaw', self.imu_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.motor_cmd_pub = self.create_publisher(String, '/motor_commands', 10)

        # Timer for main control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # Threading for user input
        self.user_input_thread = None
        self.awaiting_user_input = False

        self.get_logger().info("=" * 60)
        self.get_logger().info("🤖 ROBOT NAVIGATION CONTROLLER STARTED!")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📐 Angular Speed: {self.angular_speed}°/s")
        self.get_logger().info(f"🏃 Linear Speed: {self.linear_speed} m/s")
        self.get_logger().info(f"🎯 Target LiDAR Angle: {self.target_lidar_angle}°")
        self.get_logger().info(f"📏 Distance Tolerance: {self.distance_tolerance}m")
        self.get_logger().info("⏳ Waiting for LiDAR and IMU data...")
        self.get_logger().info("=" * 60)

    def scan_callback(self, msg):
        """Process LiDAR scan data to detect features"""
        ranges = msg.ranges
        self.angle_increment = msg.angle_increment
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_max = msg.angle_max

        # Store distances array
        self.distances_array = []
        for distance in ranges:
            if math.isinf(distance) or math.isnan(distance) or distance == 0.0:
                self.distances_array.append(msg.range_max)
            else:
                self.distances_array.append(distance)

        # Only process if robot is idle (not currently navigating)
        if self.robot_state == "IDLE":
            self.detect_target_feature()

    def detect_target_feature(self):
        """Detect the target feature (box) using distance differences"""
        if len(self.distances_array) < 11:  # Need at least 11 points for 10-point difference
            return

        # Calculate differences between every 10th data point
        differences_array = []
        difference_indices = []

        for i in range(len(self.distances_array) - 10):
            current_distance = self.distances_array[i]
            next_distance = self.distances_array[i + 10]

            # Only consider reasonable distances
            if (current_distance < self.max_detection_distance and
                next_distance < self.max_detection_distance):

                difference = abs(current_distance - next_distance)
                differences_array.append(difference)
                difference_indices.append(i)

        if not differences_array:
            return

        # Find the point with maximum difference
        max_difference_idx = np.argmax(differences_array)
        max_difference = differences_array[max_difference_idx]
        original_point_idx = difference_indices[max_difference_idx]

        # Calculate angle and distances
        angle_increment_deg = math.degrees(self.angle_increment)
        detected_angle = original_point_idx * angle_increment_deg

        # Convert to actual LiDAR angle (from angle_min)
        actual_lidar_angle = math.degrees(self.lidar_angle_min) + detected_angle

        distance1 = self.distances_array[original_point_idx]
        distance2 = self.distances_array[original_point_idx + 10]
        target_distance = min(distance1, distance2)  # Use shorter distance (likely the box)

        # Check if this is a significant feature worth navigating to
        mean_diff = np.mean(differences_array)
        std_diff = np.std(differences_array)
        threshold = mean_diff + 1.5 * std_diff

        if max_difference > threshold and target_distance > self.distance_tolerance:
            self.target_angle = actual_lidar_angle
            self.target_distance = target_distance - self.distance_tolerance  # Stop before hitting

            self.get_logger().info("🎯 " + "=" * 50)
            self.get_logger().info("📦 BOX DETECTED!")
            self.get_logger().info("🎯 " + "=" * 50)
            self.get_logger().info(f"📐 Detected at LiDAR angle: {actual_lidar_angle:.1f}°")
            self.get_logger().info(f"📏 Distance: {target_distance:.2f}m")
            self.get_logger().info(f"🔄 Need to rotate to align with {self.target_lidar_angle}°")
            self.get_logger().info(f"⚡ Max difference value: {max_difference:.3f}")

            # Ask user for confirmation
            self.ask_user_confirmation()

    def ask_user_confirmation(self):
        """Ask user if they want to navigate to the detected box"""
        self.robot_state = "WAITING_USER_INPUT"
        self.awaiting_user_input = True

        self.get_logger().info("🤔 " + "=" * 50)
        self.get_logger().info("❓ USER INPUT REQUIRED")
        self.get_logger().info("🤔 " + "=" * 50)

        # Start user input thread
        if self.user_input_thread is None or not self.user_input_thread.is_alive():
            self.user_input_thread = threading.Thread(target=self.get_user_input)
            self.user_input_thread.daemon = True
            self.user_input_thread.start()

    def get_user_input(self):
        """Get user input in separate thread"""
        try:
            while self.awaiting_user_input:
                response = input("🤖 Box found! Should I go to pick it up? (y/n): ").strip().lower()
                if response in ['y', 'yes']:
                    self.get_logger().info("✅ User confirmed: Starting navigation!")
                    self.start_navigation()
                    break
                elif response in ['n', 'no']:
                    self.get_logger().info("❌ User declined: Continuing to scan...")
                    self.reset_detection()
                    break
                else:
                    print("Please enter 'y' for yes or 'n' for no.")
        except Exception as e:
            self.get_logger().error(f"Error getting user input: {e}")
            self.reset_detection()

    def reset_detection(self):
        """Reset detection state"""
        self.robot_state = "IDLE"
        self.target_angle = None
        self.target_distance = None
        self.awaiting_user_input = False
        self.get_logger().info("🔄 Reset complete. Scanning for new targets...")

    def imu_callback(self, msg):
        """Process IMU data to get current orientation"""
        # Extract yaw from quaternion
        orientation = msg.orientation
        self.current_yaw = self.quaternion_to_yaw(orientation.x, orientation.y,
                                                 orientation.z, orientation.w)

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle in degrees"""
        # Convert quaternion to Euler angles
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def start_navigation(self):
        """Start navigation to target"""
        if self.target_angle is None or self.target_distance is None:
            return

        self.robot_state = "ROTATING"
        self.start_yaw = self.current_yaw
        self.moved_distance = 0.0
        self.rotation_start_time = time.time()
        self.awaiting_user_input = False

        # Calculate required rotation
        angle_to_target = self.target_angle - self.target_lidar_angle

        self.get_logger().info("🚀 " + "=" * 50)
        self.get_logger().info("🚀 STARTING NAVIGATION!")
        self.get_logger().info("🚀 " + "=" * 50)
        self.get_logger().info(f"📐 Current robot yaw: {self.current_yaw:.1f}°")
        self.get_logger().info(f"🎯 Target LiDAR angle: {self.target_lidar_angle}°")
        self.get_logger().info(f"📦 Box at LiDAR angle: {self.target_angle:.1f}°")
        self.get_logger().info(f"🔄 Required rotation: {angle_to_target:.1f}°")
        self.get_logger().info(f"📏 Target distance: {self.target_distance:.2f}m")

        status = f"Starting navigation: rotate {angle_to_target:.1f}° then move {self.target_distance:.2f}m"
        self.publish_status(status)

    def control_loop(self):
        """Main control loop"""
        if self.robot_state == "ROTATING":
            self.handle_rotation()
        elif self.robot_state == "MOVING_FORWARD":
            self.handle_forward_movement()
        elif self.robot_state == "ARRIVED":
            self.handle_arrival()

    def handle_rotation(self):
        """Handle robot rotation to align box with 90° LiDAR angle"""
        if self.target_angle is None:
            return

        # Calculate how much we need to rotate to align box with target_lidar_angle (90°)
        angle_diff = self.target_angle - self.target_lidar_angle

        # Normalize angle to [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        # Calculate expected rotation time
        expected_rotation_time = abs(angle_diff) / self.angular_speed
        elapsed_time = time.time() - self.rotation_start_time if self.rotation_start_time else 0

        self.get_logger().info(f"🔄 Rotating... Angle diff: {angle_diff:.1f}°, "
                              f"Time: {elapsed_time:.1f}s/{expected_rotation_time:.1f}s")

        if abs(angle_diff) <= self.angle_tolerance or elapsed_time >= expected_rotation_time:
            # Rotation complete
            self.send_motor_command("stop")
            self.robot_state = "MOVING_FORWARD"
            self.movement_start_time = time.time()

            self.get_logger().info("✅ " + "=" * 50)
            self.get_logger().info("✅ ROTATION COMPLETE!")
            self.get_logger().info("✅ " + "=" * 50)
            self.get_logger().info(f"📐 Final angle difference: {angle_diff:.1f}°")
            self.get_logger().info(f"⏱️  Rotation took: {elapsed_time:.1f}s")
            self.get_logger().info("🏃 Now moving forward to target...")

            self.publish_status("Rotation complete, moving forward")
        else:
            # Continue rotating
            if angle_diff > 0:
                self.send_motor_command("rotate_cw")  # Rotate clockwise
                self.get_logger().info("🔄 Rotating clockwise...")
            else:
                self.send_motor_command("rotate_acw")  # Rotate anti-clockwise
                self.get_logger().info("🔄 Rotating anti-clockwise...")

    def handle_forward_movement(self):
        """Handle forward movement to target"""
        if self.movement_start_time is None:
            self.movement_start_time = time.time()

        # Calculate expected movement time
        expected_movement_time = self.target_distance / self.linear_speed
        elapsed_time = time.time() - self.movement_start_time
        estimated_distance = elapsed_time * self.linear_speed

        self.get_logger().info(f"🏃 Moving forward... Distance: {estimated_distance:.2f}m/"
                              f"{self.target_distance:.2f}m, Time: {elapsed_time:.1f}s")

        if estimated_distance >= self.target_distance or elapsed_time >= expected_movement_time:
            # Arrived at target
            self.send_motor_command("stop")
            self.robot_state = "ARRIVED"

            self.get_logger().info("🎉 " + "=" * 50)
            self.get_logger().info("🎉 ARRIVED AT TARGET!")
            self.get_logger().info("🎉 " + "=" * 50)
            self.get_logger().info(f"📏 Distance traveled: {estimated_distance:.2f}m")
            self.get_logger().info(f"⏱️  Movement took: {elapsed_time:.1f}s")
            self.get_logger().info("🛑 Robot stopped at target location")

            self.publish_status("Arrived at target!")
        else:
            self.send_motor_command("move_fwd")

    def handle_arrival(self):
        """Handle arrival at target"""
        self.get_logger().info("✨ " + "=" * 50)
        self.get_logger().info("✨ MISSION ACCOMPLISHED!")
        self.get_logger().info("✨ " + "=" * 50)
        self.get_logger().info("📦 Robot is now positioned near the box")
        self.get_logger().info("🔄 Resetting for next target detection in 3 seconds...")

        # Wait 3 seconds then reset
        time.sleep(3)
        self.robot_state = "IDLE"
        self.target_angle = None
        self.target_distance = None
        self.moved_distance = 0.0
        self.rotation_start_time = None
        self.movement_start_time = None

        self.get_logger().info("🔍 Ready for next target detection...")
        self.get_logger().info("=" * 60)
        self.publish_status("Ready for next target")

    def send_motor_command(self, command):
        """Send command to /motor_commands topic"""
        msg = String()
        msg.data = command
        self.motor_cmd_pub.publish(msg)

        # Log the command
        command_symbols = {
            "rotate_cw": "↻",
            "rotate_acw": "↺",
            "move_fwd": "↑",
            "move_bkw": "↓",
            "stop": "⏹️"
        }
        symbol = command_symbols.get(command, "❓")
        self.get_logger().info(f"🎮 Motor Command: {command} {symbol}")

    def publish_status(self, status):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.send_motor_command("stop")
        self.awaiting_user_input = False
        if self.user_input_thread and self.user_input_thread.is_alive():
            self.user_input_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = RobotNavigationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Shutting down Robot Navigation Controller...")
        node.get_logger().info("👋 Goodbye!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
