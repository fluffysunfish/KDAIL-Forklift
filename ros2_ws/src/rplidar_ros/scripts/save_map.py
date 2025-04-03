#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os
from datetime import datetime

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        # Create a subscription to the map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        
        self.get_logger().info('Map saver node initialized. Waiting for map data...')
        self.map_received = False
        
    def map_callback(self, msg):
        """Process incoming map data and save it to a file"""
        if self.map_received:
            return  # Only save the map once
            
        self.get_logger().info('Received map data. Saving to file...')
        
        # Convert the map data to a numpy array
        map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        
        # Create a timestamp for the filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create maps directory if it doesn't exist
        maps_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'maps')
        os.makedirs(maps_dir, exist_ok=True)
        
        # Save the map as a numpy file
        map_file = os.path.join(maps_dir, f'map_{timestamp}.npy')
        np.save(map_file, map_data)
        
        # Save map metadata
        metadata_file = os.path.join(maps_dir, f'map_{timestamp}_metadata.txt')
        with open(metadata_file, 'w') as f:
            f.write(f"Resolution: {msg.info.resolution}\n")
            f.write(f"Width: {msg.info.width}\n")
            f.write(f"Height: {msg.info.height}\n")
            f.write(f"Origin X: {msg.info.origin.position.x}\n")
            f.write(f"Origin Y: {msg.info.origin.position.y}\n")
            f.write(f"Origin Z: {msg.info.origin.position.z}\n")
        
        # Also save as a PGM file (Portable Gray Map) for compatibility with ROS map_server
        pgm_file = os.path.join(maps_dir, f'map_{timestamp}.pgm')
        yaml_file = os.path.join(maps_dir, f'map_{timestamp}.yaml')
        
        # Convert occupancy grid to PGM format
        # -1 (unknown) -> 205 (gray), 0 (free) -> 254 (white), 100 (occupied) -> 0 (black)
        pgm_data = np.zeros_like(map_data, dtype=np.uint8)
        pgm_data[map_data == -1] = 205  # Unknown -> Gray
        pgm_data[map_data == 0] = 254   # Free -> White
        pgm_data[map_data == 100] = 0   # Occupied -> Black
        
        # Save PGM file
        with open(pgm_file, 'wb') as f:
            f.write(f"P5\n{msg.info.width} {msg.info.height}\n255\n".encode())
            pgm_data.tofile(f)
        
        # Save YAML metadata for map_server
        with open(yaml_file, 'w') as f:
            f.write(f"image: {os.path.basename(pgm_file)}\n")
            f.write(f"resolution: {msg.info.resolution}\n")
            f.write(f"origin: [{msg.info.origin.position.x}, {msg.info.origin.position.y}, 0.0]\n")
            f.write(f"negate: 0\n")
            f.write(f"occupied_thresh: 0.65\n")
            f.write(f"free_thresh: 0.196\n")
        
        self.get_logger().info(f'Map saved to {map_file}, {pgm_file}, and {yaml_file}')
        self.map_received = True
        
        # Shutdown the node after saving the map
        self.get_logger().info('Map saved successfully. Shutting down...')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if not node.map_received:
            rclpy.shutdown()

if __name__ == '__main__':
    main() 