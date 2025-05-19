import smbus2
from constants_mmc5983ma import I2C_ADDR

class MMC5983MA_IO:
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)
        self.address = I2C_ADDR

    def read_single_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def write_single_byte(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read_multiple_bytes(self, reg, length):
        return self.bus.read_i2c_block_data(self.address, reg, length)

    def write_multiple_bytes(self, reg, data):
        self.bus.write_i2c_block_data(self.address, reg, data)

    def set_register_bit(self, reg, bit_mask):
        value = self.read_single_byte(reg)
        value |= bit_mask
        self.write_single_byte(reg, value)

    def clear_register_bit(self, reg, bit_mask):
        value = self.read_single_byte(reg)
        value &= ~bit_mask
        self.write_single_byte(reg, value)

    def is_bit_set(self, reg, bit_mask):
        value = self.read_single_byte(reg)
        return value & bit_mask != 0