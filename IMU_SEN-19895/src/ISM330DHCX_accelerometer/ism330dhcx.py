import smbus2
from collections import namedtuple

# Constants
I2C_ADDR_LOW = 0x6A
I2C_ADDR_HIGH = 0x6B
WHO_AM_I = 0x0F
WHO_AM_I_VALUE = 0x6B
CTRL1_XL = 0x10
CTRL2_G = 0x11
CTRL3_C = 0x12
STATUS_REG = 0x1E
OUT_TEMP_L = 0x20
OUT_TEMP_H = 0x21
OUTX_L_G = 0x22
OUTX_H_G = 0x23
OUTY_L_G = 0x24
OUTY_H_G = 0x25
OUTZ_L_G = 0x26
OUTZ_H_G = 0x27
OUTX_L_A = 0x28
OUTX_H_A = 0x29
OUTY_L_A = 0x2A
OUTY_H_A = 0x2B
OUTZ_L_A = 0x2C
OUTZ_H_A = 0x2D

# Accelerometer full scale
FS_XL_2G = 0
FS_XL_16G = 1
FS_XL_4G = 2
FS_XL_8G = 3

# Gyroscope full scale
FS_G_125DPS = 2
FS_G_250DPS = 0
FS_G_500DPS = 4
FS_G_1000DPS = 8
FS_G_2000DPS = 12
FS_G_4000DPS = 1

# Data rates
ODR_XL_OFF = 0
ODR_XL_12Hz5 = 1
ODR_XL_26Hz = 2
ODR_XL_52Hz = 3
ODR_XL_104Hz = 4
ODR_XL_208Hz = 5
ODR_XL_416Hz = 6
ODR_XL_833Hz = 7
ODR_XL_1666Hz = 8
ODR_XL_3332Hz = 9
ODR_XL_6667Hz = 10
ODR_XL_1Hz6 = 11

ODR_G_OFF = 0
ODR_G_12Hz5 = 1
ODR_G_26Hz = 2
ODR_G_52Hz = 3
ODR_G_104Hz = 4
ODR_G_208Hz = 5
ODR_G_416Hz = 6
ODR_G_833Hz = 7
ODR_G_1666Hz = 8
ODR_G_3332Hz = 9
ODR_G_6667Hz = 10

# Data structures
RawData = namedtuple('RawData', ['x', 'y', 'z'])
ConvertedData = namedtuple('ConvertedData', ['x', 'y', 'z'])

class ISM330DHCX_IO:
    """Handles I2C communication with the ISM330DHCX sensor."""
    def __init__(self, bus=1, address=I2C_ADDR_HIGH):
        self.bus = smbus2.SMBus(bus)
        self.address = address

    def read_single_byte(self, reg):
        """Read a single byte from a register."""
        return self.bus.read_byte_data(self.address, reg)

    def write_single_byte(self, reg, value):
        """Write a single byte to a register."""
        self.bus.write_byte_data(self.address, reg, value)

    def read_multiple_bytes(self, reg, length):
        """Read multiple bytes from a register."""
        return self.bus.read_i2c_block_data(self.address, reg, length)

    def write_multiple_bytes(self, reg, data):
        """Write multiple bytes to a register."""
        self.bus.write_i2c_block_data(self.address, reg, data)

class ISM330DHCX:
    """Main class to interact with the ISM330DHCX sensor."""
    def __init__(self, bus=1, address=I2C_ADDR_HIGH):
        self.io = ISM330DHCX_IO(bus, address)
        self.full_scale_accel = FS_XL_2G
        self.full_scale_gyro = FS_G_250DPS

    def begin(self):
        """Initialize the sensor and verify connection."""
        who_am_i = self.io.read_single_byte(WHO_AM_I)
        if who_am_i != WHO_AM_I_VALUE:
            return False
        # Set block data update (BDU) to ensure consistent reads
        self.io.write_single_byte(CTRL3_C, 0x40)
        return True

    def set_accel_full_scale(self, val):
        """Set accelerometer full scale."""
        if val not in [FS_XL_2G, FS_XL_4G, FS_XL_8G, FS_XL_16G]:
            return False
        ctrl1_xl = self.io.read_single_byte(CTRL1_XL)
        ctrl1_xl = (ctrl1_xl & 0xF3) | (val << 2)
        self.io.write_single_byte(CTRL1_XL, ctrl1_xl)
        self.full_scale_accel = val
        return True

    def set_gyro_full_scale(self, val):
        """Set gyroscope full scale."""
        if val not in [FS_G_125DPS, FS_G_250DPS, FS_G_500DPS, FS_G_1000DPS, FS_G_2000DPS, FS_G_4000DPS]:
            return False
        ctrl2_g = self.io.read_single_byte(CTRL2_G)
        ctrl2_g = (ctrl2_g & 0xF0) | val
        self.io.write_single_byte(CTRL2_G, ctrl2_g)
        self.full_scale_gyro = val
        return True

    def set_accel_data_rate(self, rate):
        """Set accelerometer data rate."""
        if rate not in [ODR_XL_OFF, ODR_XL_12Hz5, ODR_XL_26Hz, ODR_XL_52Hz, ODR_XL_104Hz,
                        ODR_XL_208Hz, ODR_XL_416Hz, ODR_XL_833Hz, ODR_XL_1666Hz,
                        ODR_XL_3332Hz, ODR_XL_6667Hz, ODR_XL_1Hz6]:
            return False
        ctrl1_xl = self.io.read_single_byte(CTRL1_XL)
        ctrl1_xl = (ctrl1_xl & 0x0F) | (rate << 4)
        self.io.write_single_byte(CTRL1_XL, ctrl1_xl)
        return True

    def set_gyro_data_rate(self, rate):
        """Set gyroscope data rate."""
        if rate not in [ODR_G_OFF, ODR_G_12Hz5, ODR_G_26Hz, ODR_G_52Hz, ODR_G_104Hz,
                        ODR_G_208Hz, ODR_G_416Hz, ODR_G_833Hz, ODR_G_1666Hz,
                        ODR_G_3332Hz, ODR_G_6667Hz]:
            return False
        ctrl2_g = self.io.read_single_byte(CTRL2_G)
        ctrl2_g = (ctrl2_g & 0x0F) | (rate << 4)
        self.io.write_single_byte(CTRL2_G, ctrl2_g)
        return True

    def get_raw_accel(self):
        """Read raw accelerometer data."""
        data = self.io.read_multiple_bytes(OUTX_L_A, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        # Convert to signed 16-bit integers
        if x & 0x8000: x -= 65536
        if y & 0x8000: y -= 65536
        if z & 0x8000: z -= 65536
        return RawData(x, y, z)

    def get_raw_gyro(self):
        """Read raw gyroscope data."""
        data = self.io.read_multiple_bytes(OUTX_L_G, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        # Convert to signed 16-bit integers
        if x & 0x8000: x -= 65536
        if y & 0x8000: y -= 65536
        if z & 0x8000: z -= 65536
        return RawData(x, y, z)

    def get_accel(self):
        """Read converted accelerometer data in mg."""
        raw = self.get_raw_accel()
        scale = self._get_accel_scale()
        return ConvertedData(raw.x * scale, raw.y * scale, raw.z * scale)

    def get_gyro(self):
        """Read converted gyroscope data in mdps."""
        raw = self.get_raw_gyro()
        scale = self._get_gyro_scale()
        return ConvertedData(raw.x * scale, raw.y * scale, raw.z * scale)

    def _get_accel_scale(self):
        """Get accelerometer scale factor in mg/LSB."""
        scales = {
            FS_XL_2G: 0.061,
            FS_XL_4G: 0.122,
            FS_XL_8G: 0.244,
            FS_XL_16G: 0.488
        }
        return scales.get(self.full_scale_accel, 0.061)

    def _get_gyro_scale(self):
        """Get gyroscope scale factor in mdps/LSB."""
        scales = {
            FS_G_125DPS: 4.375,
            FS_G_250DPS: 8.75,
            FS_G_500DPS: 17.50,
            FS_G_1000DPS: 35.0,
            FS_G_2000DPS: 70.0,
            FS_G_4000DPS: 140.0
        }
        return scales.get(self.full_scale_gyro, 8.75)

    def is_accel_data_ready(self):
        """Check if accelerometer data is ready."""
        status = self.io.read_single_byte(STATUS_REG)
        return status & 0x01 != 0

    def is_gyro_data_ready(self):
        """Check if gyroscope data is ready."""
        status = self.io.read_single_byte(STATUS_REG)
        return status & 0x02 != 0

# Example usage
if __name__ == "__main__":
    sensor = ISM330DHCX()
    if sensor.begin():
        print("ISM330DHCX connected successfully!")
        sensor.set_accel_full_scale(FS_XL_2G)
        sensor.set_gyro_full_scale(FS_G_250DPS)
        sensor.set_accel_data_rate(ODR_XL_104Hz)
        sensor.set_gyro_data_rate(ODR_G_104Hz)
        
        # Wait for data to be ready (optional)
        while not (sensor.is_accel_data_ready() and sensor.is_gyro_data_ready()):
            pass
        
        accel = sensor.get_accel()
        gyro = sensor.get_gyro()
        print(f"Accelerometer: x={accel.x:.2f} mg, y={accel.y:.2f} mg, z={accel.z:.2f} mg")
        print(f"Gyroscope: x={gyro.x:.2f} mdps, y={gyro.y:.2f} mdps, z={gyro.z:.2f} mdps")
    else:
        print("Failed to connect to ISM330DHCX")