#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# GPIO control for motors (Raspberry Pi)
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("WARNING: RPi.GPIO not available. Motor control will be simulated.")

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        # GPIO pin definitions (matching your original script)
        self.IN1 = 18  # Left motor forward
        self.IN2 = 17  # Left motor backward
        self.IN3 = 22  # Right motor forward
        self.IN4 = 23  # Right motor backward

        # Store GPIO availability as instance variable
        self.gpio_available = GPIO_AVAILABLE

        # Motor state tracking
        self.current_command = "stop"
        self.last_command_time = time.time()
        self.command_timeout = 1.0  # Stop motors if no command received for 1 second

        # Setup GPIO
        self.setup_gpio()

        # Subscribe to motor commands topic
        self.motor_cmd_sub = self.create_subscription(
            String,
            '/motor_commands',
            self.motor_command_callback,
            10
        )

        # Publisher for motor status feedback
        self.motor_status_pub = self.create_publisher(String, '/motor_status', 10)

        # Timer for safety timeout check
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # Check every 100ms

        # Log startup
        self.get_logger().info("🤖 " + "=" * 50)
        self.get_logger().info("🤖 MOTOR CONTROLLER NODE STARTED")
        self.get_logger().info("🤖 " + "=" * 50)
        self.get_logger().info(f"📌 GPIO Pins - Left Motor: IN1={self.IN1}, IN2={self.IN2}")
        self.get_logger().info(f"📌 GPIO Pins - Right Motor: IN3={self.IN3}, IN4={self.IN4}")
        self.get_logger().info(f"⏱️  Safety timeout: {self.command_timeout}s")
        self.get_logger().info("📡 Subscribed to: /motor_commands")
        self.get_logger().info("📢 Publishing to: /motor_status")
        self.get_logger().info("✅ Ready to receive motor commands!")
        self.get_logger().info("🤖 " + "=" * 50)

    def setup_gpio(self):
        """Setup GPIO pins for motor control"""
        if not self.gpio_available:
            self.get_logger().warn("⚠️  GPIO not available - running in SIMULATION mode")
            return

        try:
            # Setup GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            # Setup motor pins as outputs
            GPIO.setup(self.IN1, GPIO.OUT)
            GPIO.setup(self.IN2, GPIO.OUT)
            GPIO.setup(self.IN3, GPIO.OUT)
            GPIO.setup(self.IN4, GPIO.OUT)

            # Initialize all pins to LOW (motors stopped)
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.LOW)
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.LOW)

            self.get_logger().info("✅ GPIO setup completed successfully")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to setup GPIO: {e}")
            self.get_logger().error("🔄 Switching to simulation mode")
            # Set class attribute instead of global
            self.gpio_available = False

    def motor_command_callback(self, msg):
        """Handle incoming motor commands"""
        command = msg.data.lower().strip()
        self.last_command_time = time.time()

        # Log received command
        self.get_logger().info(f"📨 Received command: '{command}'")

        # Execute the appropriate motor function
        if command == "move_fwd" or command == "forward":
            self.move_forward()
        elif command == "move_bkw" or command == "backward":
            self.move_backward()
        elif command == "rotate_cw" or command == "turn_right":
            self.turn_right()
        elif command == "rotate_acw" or command == "turn_left":
            self.turn_left()
        elif command == "stop":
            self.stop_motors()
        else:
            self.get_logger().warn(f"⚠️  Unknown command: '{command}' - stopping motors for safety")
            self.stop_motors()

        # Update current command
        self.current_command = command

        # Publish motor status
        self.publish_motor_status(command)

    def move_forward(self):
        """Move robot forward"""
        if self.gpio_available:
            GPIO.output(self.IN1, GPIO.HIGH)  # Left motor forward
            GPIO.output(self.IN2, GPIO.LOW)
            GPIO.output(self.IN3, GPIO.HIGH)  # Right motor forward
            GPIO.output(self.IN4, GPIO.LOW)
            self.get_logger().info("🏃 Moving FORWARD ↑")
        else:
            self.get_logger().info("🏃 SIMULATED: Moving FORWARD ↑")

    def move_backward(self):
        """Move robot backward"""
        if self.gpio_available:
            GPIO.output(self.IN1, GPIO.LOW)   # Left motor backward
            GPIO.output(self.IN2, GPIO.HIGH)
            GPIO.output(self.IN3, GPIO.LOW)   # Right motor backward
            GPIO.output(self.IN4, GPIO.HIGH)
            self.get_logger().info("🔙 Moving BACKWARD ↓")
        else:
            self.get_logger().info("🔙 SIMULATED: Moving BACKWARD ↓")

    def turn_left(self):
        """Turn robot left (anti-clockwise)"""
        if self.gpio_available:
            GPIO.output(self.IN1, GPIO.LOW)   # Left motor stop/slow
            GPIO.output(self.IN2, GPIO.LOW)
            GPIO.output(self.IN3, GPIO.HIGH)  # Right motor forward
            GPIO.output(self.IN4, GPIO.LOW)
            self.get_logger().info("↺ Turning LEFT (Anti-clockwise)")
        else:
            self.get_logger().info("↺ SIMULATED: Turning LEFT (Anti-clockwise)")

    def turn_right(self):
        """Turn robot right (clockwise)"""
        if self.gpio_available:
            GPIO.output(self.IN1, GPIO.HIGH)  # Left motor forward
            GPIO.output(self.IN2, GPIO.LOW)
            GPIO.output(self.IN3, GPIO.LOW)   # Right motor stop/slow
            GPIO.output(self.IN4, GPIO.LOW)
            self.get_logger().info("↻ Turning RIGHT (Clockwise)")
        else:
            self.get_logger().info("↻ SIMULATED: Turning RIGHT (Clockwise)")

    def stop_motors(self):
        """Stop all motors"""
        if self.gpio_available:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.LOW)
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.LOW)
            self.get_logger().info("🛑 Motors STOPPED ⏹️")
        else:
            self.get_logger().info("🛑 SIMULATED: Motors STOPPED ⏹️")

    def safety_check(self):
        """Safety timeout check - stop motors if no recent commands"""
        current_time = time.time()
        time_since_last_command = current_time - self.last_command_time

        # If no command received for timeout period and motors are not already stopped
        if (time_since_last_command > self.command_timeout and
            self.current_command != "stop"):

            self.get_logger().warn(f"⚠️  Safety timeout! No command for {time_since_last_command:.1f}s")
            self.get_logger().warn("🛑 Stopping motors for safety")
            self.stop_motors()
            self.current_command = "stop"
            self.publish_motor_status("safety_stop")

    def publish_motor_status(self, command):
        """Publish current motor status"""
        status_msg = String()
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        status_msg.data = f"[{timestamp}] Motor executing: {command}"
        self.motor_status_pub.publish(status_msg)

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info("🔄 Shutting down motor controller...")

        # Stop motors before cleanup
        self.stop_motors()

        # Cleanup GPIO
        if self.gpio_available:
            try:
                GPIO.cleanup()
                self.get_logger().info("✅ GPIO pins cleaned up")
            except Exception as e:
                self.get_logger().error(f"❌ Error cleaning up GPIO: {e}")

        self.get_logger().info("👋 Motor Controller Node shutdown complete")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        # Create and run the motor controller node
        motor_controller = MotorControllerNode()

        # Keep the node running
        rclpy.spin(motor_controller)

    except KeyboardInterrupt:
        print("\n🛑 Motor Controller interrupted by user")
    except Exception as e:
        print(f"\n❌ An error occurred: {e}")
    finally:
        # Cleanup
        try:
            motor_controller.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("🏁 Motor Controller shutdown complete")

if __name__ == '__main__':
    main()
