import smbus
import math
from time import sleep

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# Initialize I2C bus and device address
bus = smbus.SMBus(1)  # For Raspberry Pi newer versions
Device_Address = 0x68  # MPU6050 I2C address

def MPU_Init():
    """Initialize the MPU6050 sensor."""
    try:
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 0x01)  # Wake up
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)     # Sample rate
        bus.write_byte_data(Device_Address, CONFIG, 0)         # Set DLPF
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 0x18) # ±2000°/s range
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)     # Enable interrupts
    except Exception as e:
        print(f"Error initializing MPU6050: {e}")

def read_raw_data(addr):
    """Read 16-bit value from given register address."""
    try:
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr + 1)
        value = (high << 8) | low  # Combine high and low bytes

        # Convert to signed 16-bit value
        if value > 32767:
            value -= 65536
        return value
    except Exception as e:
        print(f"Error reading data from MPU6050: {e}")
        return None

# Initialize MPU6050
MPU_Init()
print("Reading Gyroscope and Accelerometer Data...")

while True:
    # Read accelerometer raw values
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    # Read gyroscope raw values
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Check for None values due to I2C errors
    if None in (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        print("Error: Failed to read from MPU6050. Skipping iteration...")
        continue

    # Convert raw values to proper units
    Ax = acc_x / 16384.0  # ±2g range
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0

    Gx = gyro_x / 16.4    # ±2000°/s range
    Gy = gyro_y / 16.4
    Gz = gyro_z / 16.4

    # Calculate Roll & Pitch using accelerometer data
    denominator = math.sqrt(Ay**2 + Az**2)
    pitch = math.atan2(-Ax, denominator if denominator != 0 else 1e-6) * 180 / math.pi
    roll = math.atan2(Ay, Az) * 180 / math.pi

    # Print the sensor data
    print(f"Gx={Gx:.2f}°/s\tGy={Gy:.2f}°/s\tGz={Gz:.2f}°/s", end="\t")
    print(f"Ax={Ax:.2f}g\tAy={Ay:.2f}g\tAz={Az:.2f}g", end="\t")
    print(f"Roll={roll:.2f}°\tPitch={pitch:.2f}°")

    sleep(0.2)  # Adjusted to reduce CPU load

