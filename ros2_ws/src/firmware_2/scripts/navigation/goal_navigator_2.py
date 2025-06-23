#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64
import math
import time
import numpy as np
from collections import deque
from enum import Enum

class NavigationState(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    ROTATING = "ROTATING"
    MOVING = "MOVING"
    FINE_TUNING = "FINE_TUNING"
    ARRIVED = "ARRIVED"
    STUCK = "STUCK"

class SmartNavigator(Node):
    def __init__(self):
        super().__init__('smart_navigator')

        # Robot specifications (calibrate these for your robot)
        self.linear_speed = 42.0      # cm/s - your robot's fixed forward speed
        self.angular_speed = 155.0    # deg/s - your robot's fixed rotation speed

        # Advanced navigation parameters
        self.position_tolerance = 3.0   # cm - tighter tolerance
        self.angle_tolerance = 2.0      # degrees - tighter tolerance
        self.approach_tolerance = 8.0   # cm - when to start fine-tuning
        self.overshoot_factor = 0.95    # Compensate for momentum/delays

        # Timing and control parameters
        self.min_move_time = 0.1        # seconds - minimum movement time
        self.max_move_time = 5.0        # seconds - maximum single movement
        self.command_debounce = 0.05    # seconds - command debouncing
        self.position_update_rate = 10  # Hz - how often we check position

        # Advanced algorithms parameters
        self.prediction_time = 0.3      # seconds - predict future position
        self.stuck_threshold = 2.0      # cm - movement to consider not stuck
        self.stuck_time_limit = 3.0     # seconds - time before considering stuck
        self.oscillation_threshold = 5  # number of direction changes to detect oscillation

        # Robot state
        self.state = NavigationState.IDLE
        self.current_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.predicted_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.target_pose = {"x": None, "y": None}

        # Movement tracking
        self.movement_start_time = None
        self.movement_start_pose = None
        self.last_command = None
        self.last_command_time = 0
        self.current_movement_time = 0

        # Navigation intelligence
        self.position_history = deque(maxlen=50)  # 5 seconds at 10Hz
        self.command_history = deque(maxlen=20)   # Track recent commands
        self.stuck_recovery_attempts = 0
        self.max_recovery_attempts = 3
        self.oscillation_detector = deque(maxlen=10)

        # Performance tracking
        self.navigation_attempts = 0
        self.successful_navigations = 0
        self.total_navigation_time = 0
        self.total_distance_traveled = 0

        # Adaptive learning
        self.movement_calibration = {"forward": 1.0, "rotation": 1.0}
        self.recent_errors = deque(maxlen=10)

        # Setup communication
        self.setup_communication()

        # Control timer - high frequency for precise control
        self.control_timer = self.create_timer(1.0/self.position_update_rate, self.control_loop)

        self.get_logger().info("=== Smart Navigator Started ===")
        self.log_parameters()

    def setup_communication(self):
        """Setup all ROS2 publishers and subscribers"""
        # Subscribers
        self.final_xy_sub = self.create_subscription(
            Point, '/final_xy', self.position_callback, 10)
        self.imu_yaw_sub = self.create_subscription(
            Float64, '/imu_1_yaw', self.yaw_callback, 10)
        self.goto_xy_sub = self.create_subscription(
            Point, '/goto_pose/xy', self.goto_xy_callback, 10)
        self.motor_cmd_sub = self.create_subscription(
            String, '/motor_commands', self.motor_cmd_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.motor_cmd_pub = self.create_publisher(String, '/motor_commands', 10)
        self.diagnostics_pub = self.create_publisher(String, '/nav_diagnostics', 10)

    def log_parameters(self):
        """Log current navigation parameters"""
        self.get_logger().info(f"Robot Specs: {self.linear_speed} cm/s, {self.angular_speed} deg/s")
        self.get_logger().info(f"Tolerances: ±{self.position_tolerance} cm, ±{self.angle_tolerance}°")
        self.get_logger().info(f"Overshoot Compensation: {self.overshoot_factor}")

    def position_callback(self, msg):
        """Update current position with enhanced tracking"""
        prev_pose = self.current_pose.copy()
        self.current_pose["x"] = msg.x
        self.current_pose["y"] = msg.y

        # Track position history
        current_time = time.time()
        self.position_history.append({
            'x': msg.x, 'y': msg.y, 'time': current_time,
            'yaw': self.current_pose["yaw"]
        })

        # Calculate distance moved
        if self.position_history:
            distance_moved = math.sqrt(
                (msg.x - prev_pose["x"])**2 + (msg.y - prev_pose["y"])**2
            )
            self.total_distance_traveled += distance_moved

    def yaw_callback(self, msg):
        """Update current orientation"""
        self.current_pose["yaw"] = msg.data

    def goto_xy_callback(self, msg):
        """Handle new navigation goal"""
        if self.state in [NavigationState.IDLE, NavigationState.ARRIVED]:
            self.target_pose["x"] = msg.x
            self.target_pose["y"] = msg.y
            self.get_logger().info(f"New goal: ({msg.x:.2f}, {msg.y:.2f})")
            self.start_navigation()
        else:
            self.get_logger().warning("Navigation in progress, ignoring new goal")

    def motor_cmd_callback(self, msg):
        """Handle external motor commands"""
        if msg.data == "stop" and self.state != NavigationState.IDLE:
            self.get_logger().info("External stop command received")
            self.stop_navigation("Stopped by external command")

    def start_navigation(self):
        """Initialize smart navigation"""
        if self.target_pose["x"] is None or self.target_pose["y"] is None:
            return

        self.state = NavigationState.PLANNING
        self.navigation_attempts += 1
        self.navigation_start_time = time.time()
        self.stuck_recovery_attempts = 0
        self.oscillation_detector.clear()

        # Calculate initial navigation plan
        dx = self.target_pose["x"] - self.current_pose["x"]
        dy = self.target_pose["y"] - self.current_pose["y"]
        distance = math.sqrt(dx**2 + dy**2)
        required_heading = math.degrees(math.atan2(dy, dx))

        self.get_logger().info(f"Planning navigation:")
        self.get_logger().info(f"  Distance: {distance:.2f} cm")
        self.get_logger().info(f"  Required heading: {required_heading:.1f}°")
        self.get_logger().info(f"  Current heading: {self.current_pose['yaw']:.1f}°")

        # Start with rotation if needed
        self.transition_to_rotation()
        self.publish_status("Navigation started")

    def control_loop(self):
        """Main control loop with state machine"""
        if self.state == NavigationState.ROTATING:
            self.handle_smart_rotation()
        elif self.state == NavigationState.MOVING:
            self.handle_smart_movement()
        elif self.state == NavigationState.FINE_TUNING:
            self.handle_fine_tuning()

        # Update predicted position
        self.update_predicted_position()

        # Check for stuck condition
        self.check_stuck_condition()

        # Detect oscillation
        self.detect_oscillation()

        # Publish diagnostics periodically
        if hasattr(self, 'last_diag_time'):
            if time.time() - self.last_diag_time > 2.0:
                self.publish_diagnostics()
        else:
            self.last_diag_time = time.time()

    def handle_smart_rotation(self):
        """Smart rotation with predictive stopping"""
        dx = self.target_pose["x"] - self.current_pose["x"]
        dy = self.target_pose["y"] - self.current_pose["y"]
        target_heading = math.degrees(math.atan2(dy, dx))

        current_error = self.normalize_angle(target_heading - self.current_pose["yaw"])
        predicted_error = self.normalize_angle(target_heading - self.predicted_pose["yaw"])

        # Check if we should stop rotating
        if (abs(current_error) <= self.angle_tolerance or
            abs(predicted_error) <= self.angle_tolerance or
            self.should_stop_rotation(current_error, predicted_error)):

            self.send_motor_command("stop")
            self.get_logger().info(f"Rotation complete. Error: {current_error:.2f}°")
            self.transition_to_movement()
            return

        # Determine rotation direction
        if abs(current_error) > self.angle_tolerance:
            command = "rotate_acw" if current_error > 0 else "rotate_cw"
            self.send_motor_command(command)

            # Log rotation progress
            if time.time() - getattr(self, 'last_rotation_log', 0) > 0.5:
                self.get_logger().info(f"Rotating: current_err={current_error:.1f}°, pred_err={predicted_error:.1f}°")
                self.last_rotation_log = time.time()

    def handle_smart_movement(self):
        """Smart movement with predictive stopping and course correction"""
        dx = self.target_pose["x"] - self.current_pose["x"]
        dy = self.target_pose["y"] - self.current_pose["y"]
        current_distance = math.sqrt(dx**2 + dy**2)

        # Calculate predicted distance
        pred_dx = self.target_pose["x"] - self.predicted_pose["x"]
        pred_dy = self.target_pose["y"] - self.predicted_pose["y"]
        predicted_distance = math.sqrt(pred_dx**2 + pred_dy**2)

        # Check if we should stop moving
        if (current_distance <= self.position_tolerance or
            predicted_distance <= self.position_tolerance or
            self.should_stop_movement(current_distance, predicted_distance)):

            self.send_motor_command("stop")
            self.get_logger().info(f"Movement complete. Distance: {current_distance:.2f} cm")

            if current_distance <= self.approach_tolerance:
                self.transition_to_fine_tuning()
            else:
                self.handle_arrival()
            return

        # Check if we need course correction
        target_heading = math.degrees(math.atan2(dy, dx))
        heading_error = self.normalize_angle(target_heading - self.current_pose["yaw"])

        if abs(heading_error) > self.angle_tolerance * 3:  # Significant heading error
            self.get_logger().info(f"Course correction needed. Heading error: {heading_error:.1f}°")
            self.transition_to_rotation()
            return

        # Continue moving forward
        self.send_motor_command("move_fwd")

        # Log movement progress
        if time.time() - getattr(self, 'last_movement_log', 0) > 0.5:
            self.get_logger().info(f"Moving: dist={current_distance:.1f}cm, pred_dist={predicted_distance:.1f}cm")
            self.last_movement_log = time.time()

    def handle_fine_tuning(self):
        """Fine-tuning phase for precise positioning"""
        dx = self.target_pose["x"] - self.current_pose["x"]
        dy = self.target_pose["y"] - self.current_pose["y"]
        distance = math.sqrt(dx**2 + dy**2)

        if distance <= self.position_tolerance:
            self.handle_arrival()
            return

        # Calculate required heading for fine adjustment
        target_heading = math.degrees(math.atan2(dy, dx))
        heading_error = self.normalize_angle(target_heading - self.current_pose["yaw"])

        # Prioritize heading correction in fine-tuning
        if abs(heading_error) > self.angle_tolerance:
            command = "rotate_acw" if heading_error > 0 else "rotate_cw"
            self.send_motor_command(command)
            self.get_logger().info(f"Fine-tuning rotation: {heading_error:.1f}°")
        else:
            # Make small forward movements
            self.send_motor_command("move_fwd")
            self.get_logger().info(f"Fine-tuning movement: {distance:.2f} cm")

        # Prevent infinite fine-tuning
        if not hasattr(self, 'fine_tuning_start'):
            self.fine_tuning_start = time.time()
        elif time.time() - self.fine_tuning_start > 10.0:  # 10 seconds max
            self.get_logger().warning("Fine-tuning timeout, accepting current position")
            self.handle_arrival()

    def should_stop_rotation(self, current_error, predicted_error):
        """Determine if rotation should stop based on prediction"""
        # Stop if we're about to overshoot
        if abs(predicted_error) > abs(current_error):
            return True

        # Stop if error is decreasing and we're close
        if (abs(current_error) < self.angle_tolerance * 2 and
            abs(predicted_error) < abs(current_error)):
            return True

        return False

    def should_stop_movement(self, current_distance, predicted_distance):
        """Determine if movement should stop based on prediction"""
        # Stop if we're about to overshoot
        if predicted_distance > current_distance:
            return True

        # Stop if we're very close
        if current_distance < self.position_tolerance * 2:
            return True

        # Stop if we're slowing down appropriately
        stopping_distance = self.calculate_stopping_distance()
        if current_distance <= stopping_distance * self.overshoot_factor:
            return True

        return False

    def calculate_stopping_distance(self):
        """Calculate estimated stopping distance based on current speed"""
        # Simple model: distance = speed * reaction_time + braking_distance
        reaction_time = 0.1  # seconds
        braking_distance = 2.0  # cm (estimated)
        return self.linear_speed * reaction_time + braking_distance

    def update_predicted_position(self):
        """Update predicted position based on current movement"""
        if self.last_command in ["move_fwd", "move_bwd"]:
            # Predict linear movement
            direction = 1 if self.last_command == "move_fwd" else -1
            distance = self.linear_speed * self.prediction_time * direction

            yaw_rad = math.radians(self.current_pose["yaw"])
            self.predicted_pose["x"] = self.current_pose["x"] + distance * math.cos(yaw_rad)
            self.predicted_pose["y"] = self.current_pose["y"] + distance * math.sin(yaw_rad)
            self.predicted_pose["yaw"] = self.current_pose["yaw"]

        elif self.last_command in ["rotate_cw", "rotate_acw"]:
            # Predict rotational movement
            direction = 1 if self.last_command == "rotate_acw" else -1
            angle_change = self.angular_speed * self.prediction_time * direction

            self.predicted_pose["x"] = self.current_pose["x"]
            self.predicted_pose["y"] = self.current_pose["y"]
            self.predicted_pose["yaw"] = self.current_pose["yaw"] + angle_change

        else:
            # No movement, predicted position equals current
            self.predicted_pose = self.current_pose.copy()

    def transition_to_rotation(self):
        """Transition to rotation state"""
        self.state = NavigationState.ROTATING
        self.movement_start_time = time.time()
        self.movement_start_pose = self.current_pose.copy()

    def transition_to_movement(self):
        """Transition to movement state"""
        self.state = NavigationState.MOVING
        self.movement_start_time = time.time()
        self.movement_start_pose = self.current_pose.copy()

    def transition_to_fine_tuning(self):
        """Transition to fine-tuning state"""
        self.state = NavigationState.FINE_TUNING
        self.fine_tuning_start = time.time()

    def check_stuck_condition(self):
        """Check if robot is stuck"""
        if (self.state not in [NavigationState.ROTATING, NavigationState.MOVING, NavigationState.FINE_TUNING] or
            len(self.position_history) < 20):
            return

        # Check movement in recent history
        recent_positions = list(self.position_history)[-20:]  # Last 2 seconds
        if len(recent_positions) < 2:
            return

        # Calculate total movement
        total_movement = 0
        for i in range(1, len(recent_positions)):
            dx = recent_positions[i]['x'] - recent_positions[i-1]['x']
            dy = recent_positions[i]['y'] - recent_positions[i-1]['y']
            total_movement += math.sqrt(dx**2 + dy**2)

        # Check if stuck
        if total_movement < self.stuck_threshold:
            current_time = time.time()
            if not hasattr(self, 'stuck_start_time'):
                self.stuck_start_time = current_time
            elif current_time - self.stuck_start_time > self.stuck_time_limit:
                self.handle_stuck_condition()
        else:
            # Reset stuck detection
            if hasattr(self, 'stuck_start_time'):
                delattr(self, 'stuck_start_time')

    def handle_stuck_condition(self):
        """Handle stuck condition with intelligent recovery"""
        self.get_logger().warning(f"Robot stuck, attempting recovery {self.stuck_recovery_attempts + 1}")

        if self.stuck_recovery_attempts >= self.max_recovery_attempts:
            self.get_logger().error("Max recovery attempts reached, stopping navigation")
            self.stop_navigation("Stuck - max recovery attempts reached")
            return

        self.stuck_recovery_attempts += 1

        # Recovery strategy
        if self.stuck_recovery_attempts == 1:
            # Try backing up
            self.get_logger().info("Recovery: Backing up")
            self.send_motor_command("move_bwd")
            self.create_timer(1.0, lambda: self.send_motor_command("stop"))
        elif self.stuck_recovery_attempts == 2:
            # Try rotating to different angle
            self.get_logger().info("Recovery: Rotating to find clear path")
            self.send_motor_command("rotate_cw")
            self.create_timer(1.5, lambda: self.send_motor_command("stop"))
        else:
            # Try different approach angle
            self.get_logger().info("Recovery: Trying different approach")
            self.transition_to_rotation()

        # Reset stuck detection
        if hasattr(self, 'stuck_start_time'):
            delattr(self, 'stuck_start_time')

    def detect_oscillation(self):
        """Detect if robot is oscillating"""
        if len(self.command_history) < 8:
            return

        recent_commands = list(self.command_history)[-8:]

        # Count direction changes
        direction_changes = 0
        for i in range(1, len(recent_commands)):
            if ((recent_commands[i-1] in ["rotate_cw", "rotate_acw"] and
                 recent_commands[i] in ["rotate_cw", "rotate_acw"] and
                 recent_commands[i-1] != recent_commands[i]) or
                (recent_commands[i-1] in ["move_fwd", "move_bwd"] and
                 recent_commands[i] in ["move_fwd", "move_bwd"] and
                 recent_commands[i-1] != recent_commands[i])):
                direction_changes += 1

        if direction_changes >= self.oscillation_threshold:
            self.get_logger().warning("Oscillation detected, switching to fine-tuning mode")
            self.transition_to_fine_tuning()

    def handle_arrival(self):
        """Handle successful arrival"""
        self.send_motor_command("stop")
        self.state = NavigationState.ARRIVED

        # Calculate final statistics
        navigation_time = time.time() - self.navigation_start_time
        final_error = math.sqrt(
            (self.target_pose["x"] - self.current_pose["x"])**2 +
            (self.target_pose["y"] - self.current_pose["y"])**2
        )

        # Update performance statistics
        self.successful_navigations += 1
        self.total_navigation_time += navigation_time
        self.recent_errors.append(final_error)

        self.get_logger().info("=== NAVIGATION COMPLETED ===")
        self.get_logger().info(f"Final error: {final_error:.2f} cm")
        self.get_logger().info(f"Time taken: {navigation_time:.2f} seconds")
        self.get_logger().info(f"Success rate: {self.successful_navigations}/{self.navigation_attempts}")

        self.publish_status(f"Arrived (error: {final_error:.1f}cm)")

        # Learn from this navigation
        self.update_calibration(final_error)

        # Reset for next navigation
        self.reset_navigation_state()

    def update_calibration(self, final_error):
        """Update movement calibration based on performance"""
        if len(self.recent_errors) >= 3:
            avg_error = sum(self.recent_errors) / len(self.recent_errors)

            if avg_error > self.position_tolerance * 1.5:
                # Consistently overshooting, reduce overshoot factor
                self.overshoot_factor = max(0.8, self.overshoot_factor * 0.98)
            elif avg_error < self.position_tolerance * 0.5:
                # Consistently undershooting, increase overshoot factor
                self.overshoot_factor = min(1.1, self.overshoot_factor * 1.02)

            self.get_logger().info(f"Updated overshoot factor: {self.overshoot_factor:.3f}")

    def stop_navigation(self, reason):
        """Stop current navigation"""
        self.send_motor_command("stop")
        self.state = NavigationState.IDLE
        self.get_logger().info(f"Navigation stopped: {reason}")
        self.publish_status(f"Stopped: {reason}")
        self.reset_navigation_state()

    def reset_navigation_state(self):
        """Reset navigation state"""
        self.target_pose = {"x": None, "y": None}
        self.position_history.clear()
        self.command_history.clear()
        self.stuck_recovery_attempts = 0
        if hasattr(self, 'fine_tuning_start'):
            delattr(self, 'fine_tuning_start')
        if hasattr(self, 'stuck_start_time'):
            delattr(self, 'stuck_start_time')
        self.state = NavigationState.IDLE

    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180] degrees"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def send_motor_command(self, command):
        """Send motor command with intelligent debouncing"""
        current_time = time.time()

        # Avoid sending duplicate commands too quickly
        if (command == self.last_command and
            current_time - self.last_command_time < self.command_debounce):
            return

        # Send command
        msg = String()
        msg.data = command
        self.motor_cmd_pub.publish(msg)

        # Update tracking
        self.last_command = command
        self.last_command_time = current_time
        self.command_history.append(command)

        # Log command changes
        if len(self.command_history) > 1 and self.command_history[-1] != self.command_history[-2]:
            self.get_logger().debug(f"Command: {command}")

    def publish_status(self, status):
        """Publish navigation status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_diagnostics(self):
        """Publish diagnostic information"""
        if self.state == NavigationState.IDLE:
            return

        success_rate = (self.successful_navigations / max(1, self.navigation_attempts)) * 100
        avg_error = sum(self.recent_errors) / max(1, len(self.recent_errors))

        diagnostics = {
            'state': self.state.value,
            'success_rate': f"{success_rate:.1f}%",
            'avg_error': f"{avg_error:.2f}cm",
            'overshoot_factor': f"{self.overshoot_factor:.3f}",
            'stuck_recoveries': self.stuck_recovery_attempts
        }

        if self.target_pose["x"]:
            distance_to_target = math.sqrt(
                (self.target_pose["x"] - self.current_pose["x"])**2 +
                (self.target_pose["y"] - self.current_pose["y"])**2
            )
            diagnostics['distance_remaining'] = f"{distance_to_target:.1f}cm"

        msg = String()
        msg.data = str(diagnostics)
        self.diagnostics_pub.publish(msg)
        self.last_diag_time = time.time()

    def destroy_node(self):
        """Cleanup when shutting down"""
        self.send_motor_command("stop")
        self.get_logger().info("Smart Navigator shutdown complete")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SmartNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Smart Navigator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
