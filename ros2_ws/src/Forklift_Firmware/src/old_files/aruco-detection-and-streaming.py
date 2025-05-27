#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ArucoDetectorStreamer(Node):
    def __init__(self):
        super().__init__('aruco_detector_streamer')
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera calibration parameters (you should calibrate your specific camera)
        # These are approximate values for Raspberry Pi Camera - CALIBRATE YOUR CAMERA!
        self.camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.array([0.1, -0.2, 0.0, 0.0, 0.0], dtype=np.float32)
        
        # ArUco marker size in meters (measure your printed marker)
        self.marker_size = 0.05  # 5cm marker - ADJUST TO YOUR ACTUAL MARKER SIZE!
        
        # Detection thresholds
        self.distance_threshold = 0.05  # 5cm in meters
        self.angle_threshold = 10.0     # degrees tolerance for alignment
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, 'camera/image_compressed', 10)
        self.status_pub = self.create_publisher(String, 'aruco_status', 10)
        
        # Timer for main loop
        self.timer = self.create_timer(1/30.0, self.process_frame)  # 30 FPS
        
        self.get_logger().info("ArUco Detector and Streamer Node Started!")
        self.get_logger().warn("IMPORTANT: Calibrate your camera for accurate distance measurements!")
        
    def process_frame(self):
        """Main processing loop"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return
            
        # Detect ArUco markers
        corners, ids, _ = self.detector.detectMarkers(frame)
        
        status_msg = String()
        alignment_achieved = False
        
        if ids is not None and len(ids) > 0:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                
                # Draw coordinate axes
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, 
                                rvec, tvec, self.marker_size * 0.5)
                
                # Calculate distance (Z component of translation vector)
                distance = abs(tvec[2])
                
                # Calculate yaw angle from rotation vector
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                yaw_angle = self.calculate_yaw_angle(rotation_matrix)
                
                # Check conditions
                distance_ok = distance <= self.distance_threshold
                angle_ok = abs(yaw_angle) <= self.angle_threshold
                
                # Display information on frame
                info_text = [
                    f"ID: {marker_id}",
                    f"Dist: {distance*100:.1f}cm",
                    f"Yaw: {yaw_angle:.1f}deg",
                    f"D_OK: {distance_ok}",
                    f"A_OK: {angle_ok}"
                ]
                
                # Draw info text
                y_offset = 30
                for j, text in enumerate(info_text):
                    color = (0, 255, 0) if (distance_ok and angle_ok) else (0, 255, 255)
                    cv2.putText(frame, text, (10, y_offset + j*25), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # Check if both conditions are satisfied
                if distance_ok and angle_ok:
                    alignment_achieved = True
                    # Display SUCCESS message
                    cv2.putText(frame, "ALIGNED!", (250, 50), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                    
                    # Output character when conditions are met
                    print("A")  # Character output as requested
                    
                    status_msg.data = f"ALIGNED - ID:{marker_id}, Dist:{distance*100:.1f}cm, Yaw:{yaw_angle:.1f}deg"
                else:
                    status_msg.data = f"SEARCHING - ID:{marker_id}, Dist:{distance*100:.1f}cm, Yaw:{yaw_angle:.1f}deg"
                    
        else:
            # No markers detected
            cv2.putText(frame, "No ArUco markers detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            status_msg.data = "NO_MARKERS_DETECTED"
        
        # Add frame information
        cv2.putText(frame, f"Threshold: {self.distance_threshold*100:.0f}cm, {self.angle_threshold:.0f}deg", 
                   (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Publish status
        self.status_pub.publish(status_msg)
        
        # Publish raw image
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_frame"
            self.image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish raw image: {e}")
        
        # Publish compressed image for streaming
        try:
            # Encode as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            _, buffer = cv2.imencode('.jpg', frame, encode_param)
            
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = "camera_frame"
            compressed_msg.format = "jpeg"
            compressed_msg.data = buffer.tobytes()
            
            self.compressed_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish compressed image: {e}")
    
    def calculate_yaw_angle(self, rotation_matrix):
        """Calculate yaw angle from rotation matrix"""
        # Extract yaw from rotation matrix (rotation around Z-axis)
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        return math.degrees(yaw)
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = ArucoDetectorStreamer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArUco Detector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()