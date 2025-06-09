#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        self.publisher_ = self.create_publisher(String, '/motor_commands', 10)
        self.get_logger().info("🚀 Motor Command Publisher Initialized")

    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"📤 Sent command: '{command}'")

def interactive_panel():
    rclpy.init()
    node = MotorCommandPublisher()

    print("\n" + "="*60)
    print("🤖 INTERACTIVE MOTOR CONTROL PANEL")
    print("="*60)
    print("Available commands:")
    print("➡️  forward")
    print("⬅️  backward")
    print("↪️  turn_right")
    print("↩️  turn_left")
    print("⏹️  stop")
    print("❌ exit")
    print("-" * 60)

    try:
        while rclpy.ok():
            command = input("\n📝 Enter motor command: ").strip().lower()

            if command == 'exit':
                print("👋 Exiting interactive control panel.")
                break

            if command not in ['forward', 'backward', 'turn_right', 'turn_left', 'stop']:
                print("⚠️  Invalid command. Try again.")
                continue

            duration = 0
            if command != "stop":
                try:
                    duration = float(input("⏱️  Enter duration (in seconds): "))
                    if duration <= 0:
                        print("⚠️  Duration must be positive.")
                        continue
                except ValueError:
                    print("⚠️  Invalid duration. Try again.")
                    continue

            # Send motor command
            node.send_command(command)

            # If stop, no need to wait
            if command == "stop":
                continue

            # Wait for the specified time
            print(f"⌛ Executing '{command}' for {duration} seconds...")
            time.sleep(duration)

            # Send stop command
            node.send_command("stop")
            print("🛑 Command completed, motors stopped.")

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Node shutdown complete")

if __name__ == "__main__":
    interactive_panel()
