#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Point
import time

class UWBFusionTester(Node):
    def __init__(self):
        super().__init__('uwb_fusion_tester')
        
        # Data storage
        self.raw_distances = [None, None, None]
        self.imu_yaw = None
        self.fused_position = None
        self.last_update_time = time.time()
        
        # Subscribers
        self.uwb_subscription = self.create_subscription(
            Float32MultiArray,
            '/uwb_raw_distances',
            self.uwb_callback,
            10)
        self.imu_subscription = self.create_subscription(
            Float32,
            '/imu_1_yaw',
            self.imu_callback,
            10)
        self.pos_subscription = self.create_subscription(
            Point,
            '/uwb_fused_xy',
            self.position_callback,
            10)
        
        # Statistics
        self.position_count = 0
        self.start_time = time.time()
        
        self.get_logger().info('UWB Fusion Tester initialized')
        self.get_logger().info('Monitoring topics: /uwb_raw_distances, /imu_1_yaw, /uwb_fused_xy')
        
        # Timer for status updates
        self.status_timer = self.create_timer(2.0, self.print_status)
    
    def uwb_callback(self, msg):
        if len(msg.data) >= 3:
            self.raw_distances = [
                msg.data[0] if msg.data[0] > 0 else None,
                msg.data[1] if msg.data[1] > 0 else None,
                msg.data[2] if msg.data[2] > 0 else None
            ]
    
    def imu_callback(self, msg):
        self.imu_yaw = msg.data
    
    def position_callback(self, msg):
        self.fused_position = (msg.x, msg.y)
        self.position_count += 1
        self.last_update_time = time.time()
    
    def print_status(self):
        current_time = time.time()
        runtime = current_time - self.start_time
        
        print("\n" + "="*80)
        print(f"UWB FUSION SYSTEM STATUS - Runtime: {runtime:.1f}s")
        print("="*80)
        
        # Raw distance data
        valid_distances = sum(1 for d in self.raw_distances if d is not None)
        print(f"Raw Distances: d1={self.raw_distances[0]}, d2={self.raw_distances[1]}, d3={self.raw_distances[2]}")
        print(f"Valid distances: {valid_distances}/3")
        
        # IMU data
        if self.imu_yaw is not None:
            print(f"IMU Yaw: {self.imu_yaw:.1f}°")
        else:
            print("IMU Yaw: No data")
        
        # Fused position
        if self.fused_position is not None:
            time_since_update = current_time - self.last_update_time
            print(f"Fused Position: ({self.fused_position[0]:.1f}, {self.fused_position[1]:.1f})")
            print(f"Time since last position update: {time_since_update:.1f}s")
            
            # Calculate update rate
            if runtime > 0:
                update_rate = self.position_count / runtime
                print(f"Position update rate: {update_rate:.2f} Hz")
        else:
            print("Fused Position: No data")
        
        # System health check
        print("\nSystem Health:")
        if valid_distances >= 2:
            print("✓ Sufficient distance data")
        else:
            print("✗ Insufficient distance data")
        
        if self.imu_yaw is not None:
            print("✓ IMU data available")
        else:
            print("✗ No IMU data")
        
        if self.fused_position is not None and (current_time - self.last_update_time) < 5.0:
            print("✓ Position fusion active")
        else:
            print("✗ Position fusion inactive")
        
        # Expected anchor layout reminder
        print("\nAnchor Layout:")
        print("A2(0,170) ---- A3(200,170)")
        print("|                       |")
        print("|        Robot          |")
        print("|                       |")
        print("A1(0,0)   ---- A4(200,0)")
        print("Yaw: 0°=facing A1-A2, 90°=facing A2-A3, 180°=facing A3-A4, 270°=facing A4-A1")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = UWBFusionTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'UWB Fusion Tester error: {e}')
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()