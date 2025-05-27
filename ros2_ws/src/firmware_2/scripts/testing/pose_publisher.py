#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.position_pub = self.create_publisher(Point, '/final_xy', 10)
        self.yaw_pub = self.create_publisher(Float64, '/imu_1_yaw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.pos = 0.0  # Position parameter from 0 to 1
        self.direction = 1  # 1 for (0,0) to (1,1), -1 for (1,1) to (0,0)
        self.step = 0.01  # Move 0.01 per 0.1s, so 1.0 in 10s

    def timer_callback(self):
        # Update position
        self.pos += self.direction * self.step
        if self.pos >= 1.0:
            self.pos = 1.0
            self.direction = -1
        elif self.pos <= 0.0:
            self.pos = 0.0
            self.direction = 1

        # Set x, y coordinates (along y = x)
        x = self.pos
        y = self.pos

        # Set yaw: 45° toward (1,1), 225° toward (0,0)
        yaw = 45.0 if self.direction == 1 else 225.0

        # Publish position
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = 0.0
        self.position_pub.publish(point_msg)

        # Publish yaw (in degrees, as expected by SLAM script)
        yaw_msg = Float64()
        yaw_msg.data = yaw
        self.yaw_pub.publish(yaw_msg)

        self.get_logger().info(f'Published position: ({x}, {y}), yaw: {yaw}°')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
