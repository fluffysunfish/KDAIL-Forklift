import math
import time
from src.ISM330DHCX_accelerometer.ism330dhcx import ISM330DHCX, FS_XL_2G, FS_G_250DPS, ODR_XL_104Hz, ODR_G_104Hz
from src.MMCA5983MA_magnetometer.mmc5983ma import MMC5983MA

class SEN19895:
    def __init__(self, bus=1, declination=0.0):
        self.imu = ISM330DHCX(bus=bus)
        self.mag = MMC5983MA(bus=bus)
        self.declination = declination  # Magnetic declination in degrees
        self.mag_offsets = [0, 0, 0]  # Hard-iron offsets (x, y, z)
        self.initial_yaw = None  # For relative yaw

    def begin(self):
        """Initialize both sensors and configure settings."""
        if not self.imu.begin() or not self.mag.begin():
            return False
        # Configure IMU
        self.imu.set_accel_full_scale(FS_XL_2G)  # ±2g
        self.imu.set_gyro_full_scale(FS_G_250DPS)  # ±250dps
        self.imu.set_accel_data_rate(ODR_XL_104Hz)  # 104Hz
        self.imu.set_gyro_data_rate(ODR_G_104Hz)  # 104Hz
        # Configure magnetometer
        self.mag.set_filter_bandwidth(100)  # 100Hz
        self.mag.enable_x_channel()
        self.mag.enable_yz_channels()
        return True

    def calibrate_magnetometer(self, samples=100):
        """Perform simple hard-iron calibration by averaging readings."""
        print("Calibrating magnetometer... Rotate sensor in all directions.")
        x_sum, y_sum, z_sum = 0, 0, 0
        for _ in range(samples):
            x, y, z = self.mag.get_measurement_xyz()
            x_sum += x
            y_sum += y
            z_sum += z
            time.sleep(0.01)
        self.mag_offsets = [
            x_sum / samples,
            y_sum / samples,
            z_sum / samples
        ]
        print(f"Magnetometer offsets: x={self.mag_offsets[0]:.2f}, y={self.mag_offsets[1]:.2f}, z={self.mag_offsets[2]:.2f}")

    def get_pitch_roll(self):
        """Calculate pitch and roll from accelerometer data in degrees."""
        accel = self.imu.get_accel()  # In mg
        # Convert to g (1g = 1000mg)
        ax = accel.x / 1000.0
        ay = accel.y / 1000.0
        az = accel.z / 1000.0
        # Pitch (rotation around y-axis)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        # Roll (rotation around x-axis)
        roll = math.atan2(ay, az)
        return math.degrees(pitch), math.degrees(roll)

    def get_yaw(self):
        """Calculate yaw from magnetometer data, compensated for pitch and roll."""
        pitch, roll = self.get_pitch_roll()
        x, y, z = self.mag.get_measurement_xyz()
        # Apply hard-iron calibration
        x -= self.mag_offsets[0]
        y -= self.mag_offsets[1]
        z -= self.mag_offsets[2]
        # Convert pitch and roll to radians for compensation
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        # Compensate magnetometer readings for tilt
        x_h = x * math.cos(pitch_rad) + y * math.sin(roll_rad) * math.sin(pitch_rad) + z * math.cos(roll_rad) * math.sin(pitch_rad)
        y_h = y * math.cos(roll_rad) - z * math.sin(roll_rad)
        # Calculate yaw (magnetic azimuth)
        yaw = math.atan2(-y_h, x_h)
        yaw_deg = math.degrees(yaw) + self.declination
        # Normalize to 0-360°
        yaw_deg = (yaw_deg + 360) % 360
        return yaw_deg

    def store_initial_yaw(self):
        """Store the current yaw as the initial reference for relative yaw."""
        self.initial_yaw = self.get_yaw()
        print(f"Initial yaw stored: {self.initial_yaw:.2f}°")

    def get_relative_yaw(self):
        """Calculate yaw relative to the initial yaw."""
        if self.initial_yaw is None:
            print("Initial yaw not set. Call store_initial_yaw() first.")
            return None
        current_yaw = self.get_yaw()
        relative_yaw = current_yaw - self.initial_yaw
        # Normalize to -180 to 180°
        relative_yaw = ((relative_yaw + 180) % 360) - 180
        return relative_yaw

# Example usage
if __name__ == "__main__":
    sensor = SEN19895(declination=0.0)  # Adjust declination for your location
    if sensor.begin():
        print("SEN-19895 connected successfully!")
        # Calibrate magnetometer
        sensor.calibrate_magnetometer()
        # Store initial yaw for relative measurements
        sensor.store_initial_yaw()
        try:
            while True:
                yaw = sensor.get_yaw()
                relative_yaw = sensor.get_relative_yaw()
                pitch, roll = sensor.get_pitch_roll()
                print(f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, Yaw: {yaw:.2f}°, Relative Yaw: {relative_yaw:.2f}°")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopped by user")
    else:
        print("Failed to connect to SEN-19895")