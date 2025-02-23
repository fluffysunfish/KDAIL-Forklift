from rplidar import RPLidar

# Define the serial port (adjust based on your system)
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200  # Default baud rate for RPLIDAR A2 M8

# Initialize LiDAR
lidar = RPLidar(PORT, baudrate=BAUDRATE)

try:
    print("Starting LiDAR data collection...")
    
    # Collect and print LiDAR readings
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            print(f"Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
        
        print("-" * 40)  # Separator for each scan

except KeyboardInterrupt:
    print("Stopping LiDAR...")

finally:
    # Properly stop and disconnect the LiDAR
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("LiDAR disconnected.")

