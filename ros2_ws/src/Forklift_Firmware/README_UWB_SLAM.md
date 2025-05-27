# UWB-Based Static SLAM System

This package provides a static SLAM (Simultaneous Localization and Mapping) system that uses UWB (Ultra-Wideband) sensors for robot positioning and IMU for orientation, combined with LiDAR data for mapping.

## System Overview

The system consists of:
- **UWB positioning**: 4 anchors at rectangle corners + 1 tag on robot
- **IMU orientation**: Provides robot yaw angle
- **LiDAR mapping**: Creates occupancy grid map
- **Static SLAM**: Builds map using known robot position

## Hardware Setup

### UWB Anchor Configuration
```
Anchor 2 (0, Y_max)  -------- Anchor 3 (X_max, Y_max)
|                                                    |
|                                                    |
|                Robot with UWB tag                  |
|                     +                              |
|                                                    |
|                                                    |
Anchor 1 (0, 0)      -------- Anchor 4 (X_max, 0)
```

- **Anchor 1**: Fixed at origin (0,0) - reference point
- **Anchor 2**: Positive Y direction  
- **Anchor 3**: Positive X and Y direction
- **Anchor 4**: Positive X direction
- **Robot tag**: Provides position relative to anchors

## Topics

### Input Topics
- `/fused_xy_uwb` (geometry_msgs/Point): Robot X,Y position from UWB fusion
- `/imu_1_yaw` (std_msgs/Float64): Robot yaw angle from IMU
- `/scan` (sensor_msgs/LaserScan): LiDAR scan data

### Output Topics
- `/map` (nav_msgs/OccupancyGrid): Generated occupancy grid map
- TF transforms: `map -> base_link -> laser_frame`

## Usage

### 1. Basic Launch
```bash
ros2 launch Forklift_Firmware uwb_slam_system.launch.py
```

### 2. Launch with Custom Parameters
```bash
ros2 launch Forklift_Firmware uwb_slam_system.launch.py \
    map_resolution:=0.1 \
    map_size:=1000 \
    max_range:=10.0 \
    yaw_offset:=90.0
```

### 3. Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_resolution` | 0.05 | Map resolution in meters per pixel |
| `map_size` | 2000 | Map size in pixels (width × height) |
| `max_range` | 12.0 | Maximum laser range to consider (meters) |
| `yaw_offset` | 0.0 | IMU yaw calibration offset (degrees) |

## Configuration

### UWB Transform Publisher Parameters
```yaml
uwb_imu_transform_publisher:
  z_position: 0.0          # Robot height above ground
  publish_frequency: 50.0   # Transform publishing rate (Hz)
  anchor_1_x: 0.0          # Anchor 1 X coordinate (reference)
  anchor_1_y: 0.0          # Anchor 1 Y coordinate (reference)
  yaw_offset: 0.0          # IMU yaw offset in degrees
```

### SLAM Parameters
```yaml
uwb_static_slam:
  map_resolution: 0.05      # Meters per pixel
  map_width: 2000          # Map width in pixels
  map_height: 2000         # Map height in pixels
  update_frequency: 10.0    # Map update rate (Hz)
  occupied_threshold: 0.65  # Occupied cell probability threshold
  free_threshold: 0.35     # Free cell probability threshold
  log_odds_occupied: 0.4   # Log odds increment for occupied
  log_odds_free: 0.2       # Log odds decrement for free
  max_range: 12.0          # Maximum laser range (meters)
```

## Coordinate Systems

### Map Frame
- **Origin**: Anchor 1 position (0, 0)
- **X-axis**: Towards Anchor 4 (positive X)
- **Y-axis**: Towards Anchor 2 (positive Y)
- **Z-axis**: Up (right-hand rule)

### Robot Frame
- **base_link**: Robot center, aligned with map frame
- **laser_frame**: LiDAR sensor frame, child of base_link

## Calibration

### IMU Yaw Calibration
1. Place robot facing from Anchor 1 towards Anchor 2
2. Check IMU yaw reading
3. Set `yaw_offset` parameter to make yaw = 0°

### UWB Position Calibration
1. Ensure Anchor 1 coordinates are set to (0, 0)
2. Verify other anchors form a proper rectangle
3. Check robot position readings match physical location

## Troubleshooting

### No Map Generated
- Check if `/scan` topic is publishing
- Verify transforms are being published: `ros2 run tf2_tools view_frames.py`
- Ensure UWB and IMU data are being received

### Incorrect Robot Position
- Verify UWB anchor positions
- Check `yaw_offset` parameter
- Confirm `/fused_xy_uwb` topic data format

### Poor Map Quality
- Adjust `map_resolution` for finer detail
- Tune `occupied_threshold` and `free_threshold`
- Increase `max_range` if needed
- Check laser scan quality

### Transform Errors
- Ensure all topics are publishing at reasonable rates
- Check topic names match your system
- Verify message types are correct

## Visualization

The system launches RViz2 with preconfigured visualization showing:
- Occupancy grid map
- Laser scan data
- Robot position and orientation
- TF frame tree

## File Structure

```
src/Forklift_Firmware/
├── src/
│   ├── uwb_imu_transform_publisher.py  # UWB+IMU → TF transforms
│   └── uwb_static_slam.py              # SLAM mapping node
├── launch/
│   └── uwb_slam_system.launch.py       # Main launch file
├── rviz/
│   └── uwb_slam.rviz                   # RViz configuration
└── README_UWB_SLAM.md                  # This file
```

## Dependencies

- ROS 2 Humble
- Python 3
- NumPy
- tf2_ros
- sensor_msgs
- nav_msgs
- geometry_msgs
- std_msgs