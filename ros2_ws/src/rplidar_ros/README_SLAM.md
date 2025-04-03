# Real-time SLAM for RPLidar

This package provides a real-time SLAM (Simultaneous Localization and Mapping) implementation for a stationary RPLidar sensor. It creates a 2D occupancy grid map of the environment based on the laser scan data and continuously updates it to track moving objects.

## Overview

The real-time SLAM implementation is designed for scenarios where the lidar sensor is fixed in place but objects in the environment may be moving. It builds and continuously updates a map by:

1. Processing laser scan data from the RPLidar
2. Converting range and angle measurements to Cartesian coordinates
3. Marking occupied cells at the endpoints of laser beams
4. Marking free space along the beam path using Bresenham's line algorithm
5. Applying a decay factor to gradually forget old observations
6. Updating an occupancy grid map using a probabilistic approach
7. Publishing the map for visualization in RViz

## Key Features

- **Real-time mapping**: The map continuously updates to reflect changes in the environment
- **Moving object tracking**: Objects that move will be updated in the map
- **Decay mechanism**: Old observations gradually fade away, allowing the map to adapt to changes
- **Consistency weighting**: Consistent observations across multiple scans are weighted more heavily
- **Stationary lidar**: Optimized for a fixed lidar setup

## Prerequisites

- ROS 2 (tested with Foxy/Galactic/Humble)
- RPLidar ROS 2 package (this repository)
- Python 3 with NumPy
- Matplotlib (for map visualization)

## Usage

### 1. Build the package

```bash
cd ~/ros2_ws/
colcon build --packages-select rplidar_ros
source install/setup.bash
```

### 2. Quick Start (All-in-One)

For convenience, you can use the provided shell script to run the entire SLAM process and save the map in one command:

```bash
./scripts/run_slam_and_save.sh
```

This script will:
1. Start the RPLidar driver and SLAM node
2. Run the mapping process until you press Ctrl+C
3. Automatically save the map when you stop the process

### 3. Manual Process

If you prefer to run each step manually, follow these instructions:

#### 3.1 Launch the SLAM node

```bash
ros2 launch rplidar_ros static_slam.launch.py
```

This will:
- Start the RPLidar driver
- Launch the SLAM node
- Open RViz with a configuration for visualizing the map

#### 3.2 Visualizing the Map

The map is published on the `/map` topic as a `nav_msgs/OccupancyGrid` message. In RViz:
- The map is displayed with the standard color scheme:
  - Black: Occupied cells (100)
  - White: Free cells (0)
  - Gray: Unknown cells (-1)
- The laser scan data is shown in red
- The coordinate frames (map and laser_frame) are visualized
- **The map will continuously update as objects move in the environment**

#### 3.3 Saving the Map

Once you're satisfied with the generated map, you can save it to disk using the provided map saver utility:

```bash
ros2 run rplidar_ros save_map.py
```

This will:
- Subscribe to the `/map` topic
- Save the map in multiple formats:
  - `.npy`: NumPy binary format for further processing
  - `.pgm`: Portable Gray Map format compatible with ROS map_server
  - `.yaml`: YAML metadata file for use with ROS navigation stack
- Create a timestamped filename to avoid overwriting previous maps
- Store maps in the `~/ros2_ws/maps/` directory

The saved map can be used later for navigation or other applications.

### 4. Viewing Saved Maps

You can view saved maps using the provided visualization tool:

```bash
# List all available maps
python3 scripts/view_map.py --list

# View a specific map by index
python3 scripts/view_map.py --list --index 1

# View a specific map file
python3 scripts/view_map.py --map ~/ros2_ws/maps/map_20230101_120000.npy

# Interactive mode (lists maps and prompts for selection)
python3 scripts/view_map.py
```

This tool uses Matplotlib to display the map with the proper color scheme for occupied, free, and unknown cells.

## Parameters

You can adjust the following parameters in the `static_slam.py` file to tune the real-time mapping behavior:

- `map_resolution`: Resolution of the map in meters per pixel (default: 0.05)
- `map_width` and `map_height`: Size of the map in pixels (default: 1000x1000)
- `decay_factor`: How quickly to forget old observations (default: 0.05)
- `recent_scans_weight`: Weight for consistent observations across scans (default: 2.0)
- `scan_buffer_size`: Number of recent scans to keep for comparison (default: 3)
- Probability thresholds for occupied/free space classification

## How the Real-time Mapping Works

1. **Decay Mechanism**: A small decay factor is applied to the entire map on each update, gradually moving all cells toward the prior probability. This allows the map to "forget" old observations.

2. **Consistency Tracking**: The system keeps a buffer of recent scans and compares new observations with previous ones. Consistent observations (points that appear in the same place across multiple scans) are weighted more heavily.

3. **Continuous Updates**: Unlike traditional SLAM that might stop updating after a fixed number of scans, this implementation continues to update indefinitely, allowing it to track moving objects.

## Limitations

- This implementation is designed for a stationary lidar and does not handle sensor movement
- The map size is fixed and predefined
- Very fast-moving objects might appear blurred or leave "trails" in the map
- The implementation uses a simple log-odds approach for occupancy grid mapping

## Troubleshooting

- If the map appears empty, check that the lidar is publishing data on the `/scan` topic
- If the map is not updating, ensure the transform between `map` and `laser_frame` is being published correctly
- Adjust the map size and resolution if the environment is too large or too detailed for the current settings
- If moving objects leave too much of a "trail", increase the `decay_factor` parameter
- If the map changes too quickly, decrease the `decay_factor` parameter

## License

This package is licensed under the same terms as the RPLidar ROS package. 