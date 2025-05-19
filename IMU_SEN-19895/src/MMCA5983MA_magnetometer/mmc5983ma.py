import time
from constants_mmc5983ma import *
from io import MMC5983MA_IO

class MMC5983MA:
    def __init__(self, bus=1):
        self.io = MMC5983MA_IO(bus)
        self.shadow = {
            INT_CTRL_0_REG: 0x00,
            INT_CTRL_1_REG: 0x00,
            INT_CTRL_2_REG: 0x00,
            INT_CTRL_3_REG: 0x00
        }

    def begin(self):
        return self.is_connected()

    def is_connected(self):
        try:
            prod_id = self.io.read_single_byte(PROD_ID_REG)
            return prod_id == PROD_ID
        except:
            return False

    def set_shadow_bit(self, reg, bit_mask):
        self.shadow[reg] |= bit_mask
        self.io.write_single_byte(reg, self.shadow[reg])

    def clear_shadow_bit(self, reg, bit_mask):
        self.shadow[reg] &= ~bit_mask
        self.io.write_single_byte(reg, self.shadow[reg])

    def is_shadow_bit_set(self, reg, bit_mask):
        return self.shadow[reg] & bit_mask != 0

    def get_temperature(self):
        self.set_shadow_bit(INT_CTRL_0_REG, TM_T)
        timeout = 5
        while not self.io.is_bit_set(STATUS_REG, MEAS_T_DONE) and timeout > 0:
            time.sleep(0.001)
            timeout -= 1
        self.clear_shadow_bit(INT_CTRL_0_REG, TM_T)
        if timeout == 0:
            return -99
        result = self.io.read_single_byte(T_OUT_REG)
        temperature = -75.0 + (result * (200.0 / 255.0))
        return int(temperature)

    def soft_reset(self):
        self.set_shadow_bit(INT_CTRL_1_REG, SW_RST)
        time.sleep(0.015)
        self.clear_shadow_bit(INT_CTRL_1_REG, SW_RST)

    def enable_interrupt(self):
        self.set_shadow_bit(INT_CTRL_0_REG, INT_MEAS_DONE_EN)

    def disable_interrupt(self):
        self.clear_shadow_bit(INT_CTRL_0_REG, INT_MEAS_DONE_EN)

    def enable_x_channel(self):
        self.clear_shadow_bit(INT_CTRL_1_REG, X_INHIBIT)

    def disable_x_channel(self):
        self.set_shadow_bit(INT_CTRL_1_REG, X_INHIBIT)

    def enable_yz_channels(self):
        self.clear_shadow_bit(INT_CTRL_1_REG, YZ_INHIBIT)

    def disable_yz_channels(self):
        self.set_shadow_bit(INT_CTRL_1_REG, YZ_INHIBIT)

    def set_filter_bandwidth(self, bandwidth):
        if bandwidth not in [100, 200, 400, 800]:
            return False
        bw_bits = {100: 0b00, 200: 0b01, 400: 0b10, 800: 0b11}[bandwidth]
        self.shadow[INT_CTRL_1_REG] &= 0xFC  # Clear BW0 and BW1
        self.shadow[INT_CTRL_1_REG] |= bw_bits
        self.io.write_single_byte(INT_CTRL_1_REG, self.shadow[INT_CTRL_1_REG])
        return True

    def get_filter_bandwidth(self):
        bw_bits = self.shadow[INT_CTRL_1_REG] & 0x03
        return {0b00: 100, 0b01: 200, 0b10: 400, 0b11: 800}[bw_bits]

    def enable_continuous_mode(self):
        self.set_shadow_bit(INT_CTRL_2_REG, CMM_EN)

    def disable_continuous_mode(self):
        self.clear_shadow_bit(INT_CTRL_2_REG, CMM_EN)

    def set_continuous_mode_frequency(self, frequency):
        freq_bits = {
            0: 0b000, 1: 0b001, 10: 0b010, 20: 0b011,
            50: 0b100, 100: 0b101, 200: 0b110, 1000: 0b111
        }.get(frequency, None)
        if freq_bits is None:
            return False
        self.shadow[INT_CTRL_2_REG] &= 0xF8  # Clear CM_FREQ[2:0]
        self.shadow[INT_CTRL_2_REG] |= freq_bits
        self.io.write_single_byte(INT_CTRL_2_REG, self.shadow[INT_CTRL_2_REG])
        return True

    def get_measurement_xyz(self):
        self.set_shadow_bit(INT_CTRL_0_REG, TM_M)
        timeout = self.get_timeout()
        while not self.io.is_bit_set(STATUS_REG, MEAS_M_DONE) and timeout > 0:
            time.sleep(0.001)
            timeout -= 1
        self.clear_shadow_bit(INT_CTRL_0_REG, TM_M)
        if timeout == 0:
            return 0, 0, 0
        buffer = self.io.read_multiple_bytes(X_OUT_0_REG, 7)
        x = (buffer[0] << 10) | (buffer[1] << 2) | (buffer[6] >> 6)
        y = (buffer[2] << 10) | (buffer[3] << 2) | ((buffer[6] >> 4) & 0x03)
        z = (buffer[4] << 10) | (buffer[5] << 2) | ((buffer[6] >> 2) & 0x03)
        return x, y, z

    def get_timeout(self):
        bw = self.get_filter_bandwidth()
        time_out = 800 / bw
        time_out *= 4
        time_out += 1
        return int(time_out)