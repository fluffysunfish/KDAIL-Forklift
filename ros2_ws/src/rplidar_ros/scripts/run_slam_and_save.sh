#!/bin/bash

# Run SLAM and save map script for RPLidar
# This script runs the static SLAM node, waits for the map to be generated,
# and then saves it automatically.

echo "Starting Static SLAM for RPLidar..."
echo "Press Ctrl+C when you want to stop mapping and save the map."

# Launch the static SLAM node in the background
ros2 launch rplidar_ros static_slam.launch.py &
SLAM_PID=$!

# Wait for user to press Ctrl+C
trap 'kill $SLAM_PID; echo "SLAM process stopped."; echo "Saving map..."; ros2 run rplidar_ros save_map.py; exit 0' INT

# Keep the script running until Ctrl+C is pressed
echo "SLAM is running. Press Ctrl+C to stop and save the map."
while true; do
    sleep 1
done 