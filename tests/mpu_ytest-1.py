import smbus
import math
import time

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_ZOUT_H  = 0x47

# Initialize I2C bus and device address
bus = smbus.SMBus(1)
Device_Address = 0x68

def MPU_Init():
    """Initialize the MPU6050 sensor."""
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 0x01)  # Wake up sensor

def read_raw_data(addr):
    """ Read 16-bit raw data from given register """
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

# Initialize MPU6050
MPU_Init()

yaw = 0.0
alpha = 0.98  # Complementary filter coefficient (adjustable)
prev_time = time.time()

while True:
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    # Read raw values
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Convert to proper units
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    Gz = gyro_z / 131.0

    # Compute Roll & Pitch
    roll = math.atan2(Ay, Az) * 180 / math.pi
    pitch = math.atan2(-Ax, math.sqrt(Ay**2 + Az**2)) * 180 / math.pi

    # Compute yaw using complementary filter
    yaw_gyro = yaw + Gz * dt  # Gyro integration
    yaw_acc = math.atan2(Ay, Ax) * 180 / math.pi  # Approximate yaw from accel
    yaw = alpha * yaw_gyro + (1 - alpha) * yaw_acc  # Complementary filter

    print(f"Roll={roll:.2f}°\tPitch={pitch:.2f}°\tYaw={yaw:.2f}°")

    time.sleep(0.1)

