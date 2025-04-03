#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import glob

def load_map(map_file):
    """Load a map from a NumPy file"""
    if not os.path.exists(map_file):
        print(f"Error: Map file {map_file} not found.")
        return None
        
    try:
        map_data = np.load(map_file)
        return map_data
    except Exception as e:
        print(f"Error loading map: {e}")
        return None

def display_map(map_data, title="Map Visualization"):
    """Display the map using matplotlib"""
    # Create a colormap for the occupancy grid
    # -1 (unknown) -> gray, 0 (free) -> white, 100 (occupied) -> black
    cmap = plt.cm.colors.ListedColormap(['gray', 'white', 'black'])
    bounds = [-1.5, -0.5, 0.5, 100.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
    
    # Create the figure and plot the map
    plt.figure(figsize=(10, 10))
    plt.imshow(map_data, cmap=cmap, norm=norm, origin='lower')
    plt.colorbar(ticks=[-1, 0, 100], label='Cell State')
    plt.title(title)
    plt.xlabel('X (cells)')
    plt.ylabel('Y (cells)')
    plt.grid(False)
    plt.show()

def list_available_maps():
    """List all available maps in the maps directory"""
    maps_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'maps')
    if not os.path.exists(maps_dir):
        print("No maps directory found. Run the SLAM and save a map first.")
        return []
        
    map_files = glob.glob(os.path.join(maps_dir, 'map_*.npy'))
    if not map_files:
        print("No maps found in the maps directory.")
        return []
        
    print("Available maps:")
    for i, map_file in enumerate(map_files):
        timestamp = os.path.basename(map_file).replace('map_', '').replace('.npy', '')
        print(f"{i+1}. {timestamp}")
        
    return map_files

def main():
    parser = argparse.ArgumentParser(description='Visualize a saved map')
    parser.add_argument('--map', type=str, help='Path to the map file (.npy)')
    parser.add_argument('--list', action='store_true', help='List available maps')
    parser.add_argument('--index', type=int, help='Index of the map to display (from --list)')
    
    args = parser.parse_args()
    
    if args.list:
        map_files = list_available_maps()
        if not map_files:
            return
            
        if args.index is not None:
            if 1 <= args.index <= len(map_files):
                map_file = map_files[args.index - 1]
                map_data = load_map(map_file)
                if map_data is not None:
                    timestamp = os.path.basename(map_file).replace('map_', '').replace('.npy', '')
                    display_map(map_data, f"Map from {timestamp}")
            else:
                print(f"Invalid index. Please choose a number between 1 and {len(map_files)}.")
        return
        
    if args.map:
        map_data = load_map(args.map)
        if map_data is not None:
            display_map(map_data)
    else:
        # If no arguments provided, list maps and prompt user to select one
        map_files = list_available_maps()
        if not map_files:
            return
            
        try:
            selection = int(input("Enter the number of the map to display: "))
            if 1 <= selection <= len(map_files):
                map_file = map_files[selection - 1]
                map_data = load_map(map_file)
                if map_data is not None:
                    timestamp = os.path.basename(map_file).replace('map_', '').replace('.npy', '')
                    display_map(map_data, f"Map from {timestamp}")
            else:
                print(f"Invalid selection. Please choose a number between 1 and {len(map_files)}.")
        except ValueError:
            print("Invalid input. Please enter a number.")

if __name__ == '__main__':
    main() 