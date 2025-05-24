#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import re
import time
from collections import deque
import statistics

class UWBRawPublisher(Node):
    def __init__(self):
        super().__init__('uwb_raw_publisher')
        
        # ROS2 Publisher
        self.publisher_ = self.create_publisher(Float32MultiArray, '/uwb_raw_distances', 10)
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0, rtscts=True)
            self.get_logger().info(f'Connected to UWB sensor on {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to UWB sensor: {e}')
            return
        
        # Basic filtering parameters
        self.filter_window = 3
        self.max_change_threshold = 100  # cm
        
        # Filter buffers
        self.d1_buffer = deque(maxlen=self.filter_window)
        self.d2_buffer = deque(maxlen=self.filter_window)
        self.d3_buffer = deque(maxlen=self.filter_window)
        
        # Last valid readings
        self.last_d1 = None
        self.last_d2 = None
        self.last_d3 = None
        
        # Regex patterns
        self.d1_pattern = re.compile(r'mac_address=0x0001.*?distance\[cm\]=(\d+)')
        self.d2_pattern = re.compile(r'mac_address=0x0002.*?distance\[cm\]=(\d+)')
        self.d3_pattern = re.compile(r'mac_address=0x0003.*?distance\[cm\]=(\d+)')
        
        # Timer for reading data
        self.timer = self.create_timer(0.1, self.read_and_publish)  # 10 Hz
        
        self.get_logger().info('UWB Raw Publisher initialized')
    
    def is_valid_reading(self, new_val, last_val):
        """Check if reading is within acceptable change threshold"""
        if last_val is None or new_val is None:
            return True
        return abs(new_val - last_val) <= self.max_change_threshold
    
    def apply_basic_filter(self, buffer):
        """Apply median filter to buffer"""
        if len(buffer) < 2:
            return None
        return statistics.median(buffer)
    
    def parse_distances(self, data):
        """Extract d1, d2, d3 from raw data"""
        d1 = d2 = d3 = None
        
        d1_match = self.d1_pattern.search(data)
        d2_match = self.d2_pattern.search(data)
        d3_match = self.d3_pattern.search(data)
        
        if d1_match:
            d1_raw = int(d1_match.group(1))
            if self.is_valid_reading(d1_raw, self.last_d1):
                d1 = d1_raw
                self.last_d1 = d1_raw
        
        if d2_match:
            d2_raw = int(d2_match.group(1))
            if self.is_valid_reading(d2_raw, self.last_d2):
                d2 = d2_raw
                self.last_d2 = d2_raw
        
        if d3_match:
            d3_raw = int(d3_match.group(1))
            if self.is_valid_reading(d3_raw, self.last_d3):
                d3 = d3_raw
                self.last_d3 = d3_raw
        
        return d1, d2, d3
    
    def read_and_publish(self):
        """Timer callback to read and publish UWB distances"""
        try:
            if hasattr(self, 'ser') and self.ser.in_waiting > 0:
                data = self.ser.read_all().decode('utf-8', errors='ignore')
                
                if 'SESSION_INFO_NTF:' in data:
                    messages = data.split('SESSION_INFO_NTF:')
                    for message in messages[1:]:  # Skip first empty part
                        end = message.find('}')
                        if end != -1:
                            message = 'SESSION_INFO_NTF:' + message[:end+1]
                            
                            # Parse distances
                            d1, d2, d3 = self.parse_distances(message)
                            
                            # Add to filter buffers
                            if d1 is not None:
                                self.d1_buffer.append(d1)
                            if d2 is not None:
                                self.d2_buffer.append(d2)
                            if d3 is not None:
                                self.d3_buffer.append(d3)
                            
                            # Get filtered distances
                            d1_f = self.apply_basic_filter(self.d1_buffer)
                            d2_f = self.apply_basic_filter(self.d2_buffer)
                            d3_f = self.apply_basic_filter(self.d3_buffer)
                            
                            # Publish distances
                            msg = Float32MultiArray()
                            msg.data = [
                                float(d1_f) if d1_f is not None else -1.0,
                                float(d2_f) if d2_f is not None else -1.0,
                                float(d3_f) if d3_f is not None else -1.0
                            ]
                            self.publisher_.publish(msg)
                            
                            # Log for debugging
                            valid_count = sum(1 for d in [d1_f, d2_f, d3_f] if d is not None)
                            self.get_logger().debug(f'Published distances: d1={d1_f}, d2={d2_f}, d3={d3_f} ({valid_count}/3 valid)')
                            break
                
        except Exception as e:
            self.get_logger().error(f'Error in read loop: {e}')
    
    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        publisher = UWBRawPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'UWB Raw Publisher error: {e}')
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()