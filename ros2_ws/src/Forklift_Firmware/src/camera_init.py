#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # For PiCam using OpenCV

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            if len(frame.shape) == 2 or frame.shape[2] == 1:
                # Grayscale image
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
            else:
                # Color image
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
        else:
            self.get_logger().warn("Failed to capture image")


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
