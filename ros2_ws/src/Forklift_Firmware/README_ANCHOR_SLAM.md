# Anchor-Based SLAM System

## Overview

This system implements SLAM using a fixed anchor-based coordinate system where Anchor 1 serves as the origin of the map frame. The robot moves within this fixed coordinate system, providing consistent localization relative to the physical anchor layout.

## Coordinate System

### Anchor Layout
```
A2(0,170) ---- A3(200,170)
|                       |
|        Robot          |
|                       |
A1(0,0)   ---- A4(200,0)
```

All coordinates are in centimeters in the UWB system, converted to meters for ROS.

### Reference Frame
- **Map Origin**: Anchor 1 at (0,0)
- **X-axis**: Points from Anchor 1 towards Anchor 4 (East)
- **Y-axis**: Points from Anchor 1 towards Anchor 2 (North)
- **Z-axis**: Points upward (standard ROS convention)

### Orientation Convention
- **0°**: Robot facing North (from Anchor 1 towards Anchor 2)
- **90°**: Robot facing East (from Anchor 1 towards Anchor 4)
- **180°**: Robot facing South (from Anchor 2 towards Anchor 1)
- **270°**: Robot facing West (from Anchor 4 towards Anchor 1)

## Key Files

### `dynamic_uwb_positioning.py`
- Subscribes to `/uwb_fused_xy` for position data
- Subscribes to `/imu_1_yaw` for orientation data
- Publishes `map -> laser_frame` transform
- Maintains fixed map frame with Anchor 1 as origin
- Applies 85° counter-clockwise rotation to align laser frame with yaw angle

### `uwb_imu_fusion.py`
- Fuses UWB distance measurements with IMU data
- Publishes fused position to `/uwb_fused_xy`
- Uses orientation-weighted trilateration for improved accuracy

### `test_uwb_imu_publisher.py`
- Simulates UWB and IMU data for testing
- Generates circular motion within anchor area
- Useful for testing without physical hardware

## Launch Files

### Production Use
```bash
ros2 launch rplidar_ros anchor_based_slam.launch.py
```
- Uses real UWB/IMU fusion
- Requires physical UWB anchors and IMU
- Includes system health monitoring

### Testing
```bash
ros2 launch rplidar_ros test_anchor_slam.launch.py
```
- Uses simulated UWB/IMU data
- Good for development and testing
- No physical hardware required

## Topics

### Input Topics
- `/uwb_raw_distances` (Float32MultiArray): Raw UWB distances to anchors
- `/imu_1_yaw` (Float64): IMU yaw angle in degrees or radians
- `/scan` (LaserScan): LiDAR scan data

### Output Topics
- `/uwb_fused_xy` (Point): Fused position in anchor coordinates (cm)
- `/map` (OccupancyGrid): SLAM-generated map
- `/tf` (TF2): Transform tree including map->laser_frame

## System Requirements

1. **UWB Anchors**: Must be positioned exactly as specified in anchor layout
2. **IMU**: Must provide yaw angle aligned with coordinate system
3. **LiDAR**: Compatible with rplidar_ros package

## Calibration

### IMU Alignment
If IMU yaw doesn't align with the coordinate system:
1. Modify `yaw_callback()` in `dynamic_uwb_positioning.py`
2. Add offset: `self.map_yaw = imu_yaw_rad + your_offset`

### Laser Frame Alignment
The laser frame is automatically rotated 85° counter-clockwise from the IMU yaw to align with the physical laser orientation. If this needs adjustment:
1. Modify `self.laser_rotation_offset` in `dynamic_uwb_positioning.py`
2. Value is in radians (current: 85° = 1.484 radians)

### Anchor Position Verification
Verify anchor positions match the layout in `uwb_imu_fusion.py`:
```python
self.anchor1 = np.array([0, 0])      # A1
self.anchor2 = np.array([0, 170])    # A2  
self.anchor3 = np.array([200, 170])  # A3
self.anchor4 = np.array([200, 0])    # A4 (virtual reference)
```

## Troubleshooting

### No Position Updates
- Check `/uwb_fused_xy` topic: `ros2 topic echo /uwb_fused_xy`
- Check `/imu_1_yaw` topic: `ros2 topic echo /imu_1_yaw`
- Verify anchor positions and UWB signal quality

### Map Not Fixed
- Ensure `dynamic_uwb_positioning.py` is running
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify map->laser_frame transform is being published

### Laser Scan Misalignment
- Verify the 85° laser rotation offset is correct for your hardware
- Check that laser data aligns with robot movement in RViz
- Adjust `laser_rotation_offset` if needed

### Poor Localization
- Check UWB distance measurements for consistency
- Verify IMU calibration and orientation alignment
- Ensure anchors are positioned as specified

## Monitoring

Use the fusion tester to monitor system health:
```bash
ros2 run Forklift_Firmware uwb_fusion_test.py
```

This provides real-time status of:
- UWB distance measurements
- IMU data
- Position fusion quality
- Update rates