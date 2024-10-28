# lsm6ds33.py

import board
import busio

class LSM6DS33:
    WHO_AM_I = 0x0F
    LSM6DS33_WHO_AM_I_RESPONSE = 0x6C
    CTRL1_XL = 0x10
    CTRL2_G = 0x11
    OUTX_L_XL = 0x28  # Accelerometer X-axis low byte
    OUTX_H_XL = 0x29  # Accelerometer X-axis high byte
    OUTY_L_XL = 0x2A  # Accelerometer Y-axis low byte
    OUTY_H_XL = 0x2B  # Accelerometer Y-axis high byte
    OUTZ_L_XL = 0x2C  # Accelerometer Z-axis low byte
    OUTZ_H_XL = 0x2D  # Accelerometer Z-axis high byte
    OUTX_L_G = 0x22   # Gyroscope X-axis low byte
    OUTX_H_G = 0x23   # Gyroscope X-axis high byte
    OUTY_L_G = 0x24   # Gyroscope Y-axis low byte
    OUTY_H_G = 0x25   # Gyroscope Y-axis high byte
    OUTZ_L_G = 0x26   # Gyroscope Z-axis low byte
    OUTZ_H_G = 0x27   # Gyroscope Z-axis high byte

    # Configuration values
    ACCEL_CONFIG = 0b01000000  # 104 Hz, 2g
    GYRO_CONFIG = 0b01000000   # 104 Hz, 250 dps

    def __init__(self, address=0x6B):
        self.address = address
        # Initialize I2C bus
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self._initialize_sensor()

    def _initialize_sensor(self):
        # Check if the sensor is responding correctly
        who_am_i = self.read_register(self.WHO_AM_I)
        if who_am_i != self.LSM6DS33_WHO_AM_I_RESPONSE:
            raise RuntimeError("Failed to detect LSM6DS33 sensor.")
        
        # Configure accelerometer and gyroscope
        self.write_register(self.CTRL1_XL, self.ACCEL_CONFIG)
        self.write_register(self.CTRL2_G, self.GYRO_CONFIG)

    def read_register(self, reg):
        # Read a single byte from the specified register
        self.i2c.writeto(self.address, bytes([reg]))
        result = bytearray(1)
        self.i2c.readfrom_into(self.address, result)
        return result[0]

    def write_register(self, reg, value):
        # Write a single byte to the specified register
        self.i2c.writeto(self.address, bytes([reg, value]))

    def read_sensor_data(self, low_reg, high_reg):
        # Read 16-bit signed data from the sensor (low and high byte)
        low = self.read_register(low_reg)
        high = self.read_register(high_reg)
        value = (high << 8) | low
        if value >= 32768:
            value -= 65536
        return value

    def get_acceleration(self):
        # Get accelerometer data (X, Y, Z)
        acc_x = self.read_sensor_data(self.OUTX_L_XL, self.OUTX_H_XL)
        acc_y = self.read_sensor_data(self.OUTY_L_XL, self.OUTY_H_XL)
        acc_z = self.read_sensor_data(self.OUTZ_L_XL, self.OUTZ_H_XL)
        return acc_x, acc_y, acc_z

    def get_gyroscope(self):
        # Get gyroscope data (X, Y, Z)
        gyro_x = self.read_sensor_data(self.OUTX_L_G, self.OUTX_H_G)
        gyro_y = self.read_sensor_data(self.OUTY_L_G, self.OUTY_H_G)
        gyro_z = self.read_sensor_data(self.OUTZ_L_G, self.OUTZ_H_G)
        return gyro_x, gyro_y, gyro_z
