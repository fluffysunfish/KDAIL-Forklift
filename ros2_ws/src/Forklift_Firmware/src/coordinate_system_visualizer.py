#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tf2_ros import TransformListener, Buffer
import math
import time

class CoordinateSystemVisualizer(Node):
    def __init__(self):
        super().__init__('coordinate_system_visualizer')
        
        # TF2 setup for reading transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Data storage
        self.uwb_position = None
        self.imu_yaw = None
        self.laser_transform = None
        self.last_positions = []
        
        # Subscribers
        self.uwb_subscription = self.create_subscription(
            Point,
            '/uwb_fused_xy',
            self.uwb_callback,
            10)
        self.imu_subscription = self.create_subscription(
            Float64,
            '/imu_1_yaw',
            self.imu_callback,
            10)
        
        # Timer for visualization updates
        self.viz_timer = self.create_timer(1.0, self.visualize_system)
        
        # Anchor positions for reference (in cm, as used in fusion)
        self.anchors = {
            'A1': (0, 0),
            'A2': (0, 170),
            'A3': (200, 170),
            'A4': (200, 0)
        }
        
        self.get_logger().info('Coordinate System Visualizer started')
        self.get_logger().info('Monitoring UWB position, IMU yaw, and laser frame transform')
        
    def uwb_callback(self, msg):
        self.uwb_position = (msg.x, msg.y)  # in cm
        
    def imu_callback(self, msg):
        self.imu_yaw = msg.data  # in degrees or radians
        
    def get_laser_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'laser_frame',
                rclpy.time.Time())
            
            # Extract position (in meters)
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Extract quaternion and convert to yaw
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Convert quaternion to yaw angle
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            self.laser_transform = {
                'position': (x, y, z),
                'yaw_rad': yaw,
                'yaw_deg': math.degrees(yaw)
            }
            
        except Exception as e:
            self.laser_transform = None
            
    def visualize_system(self):
        self.get_laser_transform()
        
        print("\n" + "="*90)
        print("COORDINATE SYSTEM VISUALIZATION")
        print("="*90)
        
        # Anchor layout
        print("Anchor Layout (cm):")
        print("  A2(0,170) ──────── A3(200,170)")
        print("  │                           │")
        print("  │          ROBOT            │")
        print("  │                           │")
        print("  A1(0,0)   ──────── A4(200,0)")
        print()
        
        # UWB Position
        if self.uwb_position:
            x_cm, y_cm = self.uwb_position
            x_m, y_m = x_cm / 100.0, y_cm / 100.0
            print(f"UWB Position:     ({x_cm:6.1f}, {y_cm:6.1f}) cm  =  ({x_m:5.3f}, {y_m:5.3f}) m")
            
            # Store position history
            self.last_positions.append((x_cm, y_cm))
            if len(self.last_positions) > 5:
                self.last_positions.pop(0)
        else:
            print("UWB Position:     No data")
            
        # IMU Yaw
        if self.imu_yaw is not None:
            if abs(self.imu_yaw) > 2 * math.pi:
                yaw_deg = self.imu_yaw
                yaw_rad = math.radians(self.imu_yaw)
                print(f"IMU Yaw:          {yaw_deg:6.1f}°  =  {yaw_rad:5.3f} rad (input in degrees)")
            else:
                yaw_rad = self.imu_yaw
                yaw_deg = math.degrees(self.imu_yaw)
                print(f"IMU Yaw:          {yaw_deg:6.1f}°  =  {yaw_rad:5.3f} rad (input in radians)")
        else:
            print("IMU Yaw:          No data")
            
        # Laser Frame Transform
        if self.laser_transform:
            pos = self.laser_transform['position']
            yaw_deg = self.laser_transform['yaw_deg']
            yaw_rad = self.laser_transform['yaw_rad']
            print(f"Laser Transform:  ({pos[0]:5.3f}, {pos[1]:5.3f}, {pos[2]:5.3f}) m")
            print(f"Laser Yaw:        {yaw_deg:6.1f}°  =  {yaw_rad:5.3f} rad (with 85° offset)")
        else:
            print("Laser Transform:  No transform available")
            
        # Calculate expected vs actual
        if self.uwb_position and self.imu_yaw is not None and self.laser_transform:
            print("\n" + "-"*90)
            print("VERIFICATION:")
            
            # Position comparison
            uwb_x_m, uwb_y_m = self.uwb_position[0]/100.0, self.uwb_position[1]/100.0
            tf_x_m, tf_y_m = self.laser_transform['position'][0], self.laser_transform['position'][1]
            
            pos_error_x = abs(uwb_x_m - tf_x_m)
            pos_error_y = abs(uwb_y_m - tf_y_m)
            
            print(f"Position Error:   X: {pos_error_x*1000:4.1f}mm,  Y: {pos_error_y*1000:4.1f}mm")
            
            # Yaw comparison
            if abs(self.imu_yaw) > 2 * math.pi:
                imu_yaw_rad = math.radians(self.imu_yaw)
            else:
                imu_yaw_rad = self.imu_yaw
                
            expected_laser_yaw = imu_yaw_rad + math.radians(85)  # Add 85° offset
            actual_laser_yaw = self.laser_transform['yaw_rad']
            
            # Normalize angles to [-pi, pi]
            expected_laser_yaw = math.atan2(math.sin(expected_laser_yaw), math.cos(expected_laser_yaw))
            actual_laser_yaw = math.atan2(math.sin(actual_laser_yaw), math.cos(actual_laser_yaw))
            
            yaw_error = abs(expected_laser_yaw - actual_laser_yaw)
            if yaw_error > math.pi:
                yaw_error = 2 * math.pi - yaw_error
                
            print(f"Expected Laser:   {math.degrees(expected_laser_yaw):6.1f}°")
            print(f"Actual Laser:     {math.degrees(actual_laser_yaw):6.1f}°")
            print(f"Yaw Error:        {math.degrees(yaw_error):6.1f}°")
            
            # Status indicators
            pos_ok = pos_error_x < 0.01 and pos_error_y < 0.01  # 1cm tolerance
            yaw_ok = yaw_error < math.radians(2)  # 2 degree tolerance
            
            print(f"\nStatus:           Position: {'✓' if pos_ok else '✗'}  Orientation: {'✓' if yaw_ok else '✗'}")
            
        # Movement tracking
        if len(self.last_positions) > 1:
            print("\n" + "-"*90)
            print("MOVEMENT TRACKING:")
            
            current = self.last_positions[-1]
            previous = self.last_positions[-2]
            
            dx = current[0] - previous[0]
            dy = current[1] - previous[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0.1:  # Only show if moved more than 1mm
                direction = math.degrees(math.atan2(dy, dx))
                print(f"Movement:         {distance:5.1f}cm at {direction:6.1f}° (in anchor coordinates)")
                
        # Quadrant information
        if self.uwb_position:
            x, y = self.uwb_position
            quadrant = ""
            if 0 <= x <= 100 and 0 <= y <= 85:
                quadrant = "Near A1 (bottom-left)"
            elif 0 <= x <= 100 and 85 < y <= 170:
                quadrant = "Near A2 (top-left)"
            elif 100 < x <= 200 and 85 < y <= 170:
                quadrant = "Near A3 (top-right)"
            elif 100 < x <= 200 and 0 <= y <= 85:
                quadrant = "Near A4 (bottom-right)"
            else:
                quadrant = "Outside anchor area"
                
            print(f"\nLocation:         {quadrant}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        visualizer = CoordinateSystemVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Coordinate System Visualizer error: {e}')
    finally:
        if 'visualizer' in locals():
            visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()