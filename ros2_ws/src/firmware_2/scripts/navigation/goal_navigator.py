#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64
import math
import time

class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')

        # Robot state
        self.robot_state = "IDLE"  # IDLE, ROTATING, MOVING, CHECKING, ARRIVED
        self.current_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.target_pose = {"x": None, "y": None}
        self.retry_count = 0
        self.max_retries = 3
        self.start_time = None

        # Navigation parameters
        self.position_tolerance = 5.0  # cm
        self.angle_tolerance = 5.0    # degrees
        self.linear_speed = 42.0      # cm per second
        self.angular_speed = 155.0    # degrees per second
        self.command_debounce_time = 0.5  # seconds

        # Publishers and Subscribers
        self.final_xy_sub = self.create_subscription(Point, '/final_xy', self.position_callback, 10)
        self.imu_yaw_sub = self.create_subscription(Float64, '/imu_1_yaw', self.yaw_callback, 10)
        self.goto_xy_sub = self.create_subscription(Point, '/goto_pose/xy', self.goto_xy_callback, 10)
        self.motor_cmd_sub = self.create_subscription(String, '/motor_commands', self.motor_cmd_callback, 10)
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.motor_cmd_pub = self.create_publisher(String, '/motor_commands', 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Command tracking
        self.last_motor_command = None
        self.command_timestamp = 0
        self.is_self_stop = False
        self.required_time = 0.0  # Time to move or rotate

        self.get_logger().info("=== Goal Navigator Started ===")
        self.get_logger().info(f"Position Tolerance: ±{self.position_tolerance} cm")
        self.get_logger().info(f"Angle Tolerance: ±{self.angle_tolerance}°")
        self.get_logger().info(f"Linear Speed: {self.linear_speed} cm/s")
        self.get_logger().info(f"Angular Speed: {self.angular_speed} deg/s")
        self.get_logger().info("Waiting for navigation goals...")

    def position_callback(self, msg):
        self.current_pose["x"] = msg.x
        self.current_pose["y"] = msg.y

    def yaw_callback(self, msg):
        self.current_pose["yaw"] = msg.data

    def goto_xy_callback(self, msg):
        if self.robot_state == "IDLE":
            self.target_pose["x"] = msg.x
            self.target_pose["y"] = msg.y
            self.get_logger().info(f"Received goal: x={msg.x:.2f}, y={msg.y:.2f}")
            self.start_navigation()

    def motor_cmd_callback(self, msg):
        if (msg.data == "stop" and self.robot_state != "IDLE" and not self.is_self_stop and
                time.time() - self.command_timestamp > self.command_debounce_time):
            self.get_logger().info("External stop command received")
            self.stop_navigation("Navigation stopped by external command")
        if msg.data == "stop":
            self.is_self_stop = False

    def start_navigation(self):
        if self.target_pose["x"] is None or self.target_pose["y"] is None:
            self.get_logger().warning("No target position set")
            return

        dx = self.target_pose["x"] - self.current_pose["x"]
        dy = self.target_pose["y"] - self.current_pose["y"]
        distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.degrees(math.atan2(dy, dx))
        angle_diff = self.normalize_angle(target_heading - self.current_pose["yaw"])

        # Calculate rotation time
        self.required_time = abs(angle_diff) / self.angular_speed

        self.get_logger().info(f"Starting navigation to x={self.target_pose['x']:.2f}, y={self.target_pose['y']:.2f}")
        self.get_logger().info(f"Distance: {distance:.2f} cm, Required rotation: {angle_diff:.2f}°")
        self.get_logger().info(f"Rotation time: {self.required_time:.2f} s")

        self.robot_state = "ROTATING"
        self.retry_count = 0
        self.start_time = time.time()
        self.publish_status("Starting navigation")

    def control_loop(self):
        if self.robot_state == "ROTATING":
            self.handle_rotation()
        elif self.robot_state == "MOVING":
            self.handle_movement()
        elif self.robot_state == "CHECKING":
            self.handle_checking()
        elif self.robot_state == "ARRIVED":
            self.handle_arrival()

    def handle_rotation(self):
        dx = self.target_pose["x"] - self.current_pose["x"]
        dy = self.target_pose["y"] - self.current_pose["y"]
        target_heading = math.degrees(math.atan2(dy, dx))
        angle_diff = self.normalize_angle(target_heading - self.current_pose["yaw"])

        elapsed_time = time.time() - self.start_time

        if elapsed_time >= self.required_time or abs(angle_diff) <= self.angle_tolerance:
            self.send_motor_command("stop")
            self.robot_state = "MOVING"
            # Calculate movement time based on distance
            distance = math.sqrt(dx**2 + dy**2)
            self.required_time = distance / self.linear_speed
            self.start_time = time.time()
            self.get_logger().info(f"Rotation complete, starting movement for {self.required_time:.2f} s")
            self.publish_status("Rotation complete")
        else:
            # Anticlockwise rotation increases yaw (based on your convention)
            command = "rotate_cw" if angle_diff > 0 else "rotate_acw"
            self.send_motor_command(command)
            self.get_logger().info(f"Rotating {'anticlockwise' if angle_diff > 0 else 'clockwise'} "
                                 f"(angle diff: {angle_diff:.2f}°, {elapsed_time:.2f}/{self.required_time:.2f} s)")

    def handle_movement(self):
        elapsed_time = time.time() - self.start_time

        if elapsed_time >= self.required_time:
            self.send_motor_command("stop")
            self.robot_state = "CHECKING"
            self.start_time = time.time()
            self.get_logger().info("Movement complete, checking position")
            self.publish_status("Checking position")
        else:
            self.send_motor_command("move_fwd")
            dx = self.target_pose["x"] - self.current_pose["x"]
            dy = self.target_pose["y"] - self.current_pose["y"]
            distance = math.sqrt(dx**2 + dy**2)
            self.get_logger().info(f"Moving forward (distance: {distance:.2f} cm, "
                                 f"{elapsed_time:.2f}/{self.required_time:.2f} s)")

    def handle_checking(self):
        dx = self.target_pose["x"] - self.current_pose["x"]
        dy = self.target_pose["y"] - self.current_pose["y"]
        distance = math.sqrt(dx**2 + dy**2)

        if distance <= self.position_tolerance:
            self.robot_state = "ARRIVED"
            self.get_logger().info(f"Reached target (distance error: {distance:.2f} cm)")
        else:
            self.retry_count += 1
            if self.retry_count < self.max_retries:
                self.get_logger().info(f"Retry {self.retry_count}/{self.max_retries}, distance error: {distance:.2f} cm")
                self.robot_state = "ROTATING"
                target_heading = math.degrees(math.atan2(dy, dx))
                angle_diff = self.normalize_angle(target_heading - self.current_pose["yaw"])
                self.required_time = abs(angle_diff) / self.angular_speed
                self.start_time = time.time()
                self.publish_status(f"Retrying navigation (attempt {self.retry_count})")
            else:
                self.robot_state = "ARRIVED"
                self.get_logger().warning(f"Max retries reached, stopping (distance error: {distance:.2f} cm)")
                self.publish_status("Max retries reached")

    def handle_arrival(self):
        self.get_logger().info("=== Arrived at Target ===")
        self.get_logger().info(f"Final position: x={self.current_pose['x']:.2f}, y={self.current_pose['y']:.2f}")
        self.robot_state = "IDLE"
        self.target_pose = {"x": None, "y": None}
        self.retry_count = 0
        self.publish_status("Arrived at target")

    def stop_navigation(self, reason):
        self.send_motor_command("stop")
        self.robot_state = "IDLE"
        self.target_pose = {"x": None, "y": None}
        self.retry_count = 0
        self.get_logger().info(f"Navigation stopped: {reason}")
        self.publish_status(reason)

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def send_motor_command(self, command):
        if (command == self.last_motor_command and
                time.time() - self.command_timestamp < self.command_debounce_time):
            return

        msg = String()
        msg.data = command
        self.last_motor_command = command
        self.command_timestamp = time.time()
        self.is_self_stop = (command == "stop")
        self.motor_cmd_pub.publish(msg)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.send_motor_command("stop")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
