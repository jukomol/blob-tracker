import board
import busio
import time
from math import sqrt

class LSM6DS0:
    WHO_AM_I = 0x0F
    CTRL1_XL = 0x20
    CTRL2_G = 0x10
    OUTX_L_XL = 0x28
    OUTX_H_XL = 0x29
    OUTY_L_XL = 0x2A
    OUTY_H_XL = 0x2B
    OUTZ_L_XL = 0x2C
    OUTZ_H_XL = 0x2D
    OUTX_L_G = 0x18
    OUTX_H_G = 0x19
    OUTY_L_G = 0x1A
    OUTY_H_G = 0x1B
    OUTZ_L_G = 0x1C
    OUTZ_H_G = 0x1D

    # Sensitivity (from datasheet)
    ACCEL_SENSITIVITY = {
        2: 0.061,  # mg/LSB
        4: 0.122,
        8: 0.244,
        16: 0.732
    }
    GYRO_SENSITIVITY = {
        125: 4.375,  # mdps/LSB
        250: 8.75,
        500: 17.5,
        1000: 35.0,
        2000: 70.0
    }

    def __init__(self, address=0x6A):
        self.address = address
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.accel_scale = 2  # ±2g by default
        self.gyro_scale = 245  # ±245 dps by default
        self.gyro_offsets = [0, 0, 0]
        self._initialize_sensor()

    def _initialize_sensor(self):
        # Configure accelerometer and gyroscope
        self.write_register(self.CTRL1_XL, 0x60)  # 104 Hz, ±2g
        self.write_register(self.CTRL2_G, 0x60)   # 104 Hz, ±245 dps

    def write_register(self, reg, value):
        self.i2c.writeto(self.address, bytes([reg, value]))

    def read_register(self, reg, length=1):
        self.i2c.writeto(self.address, bytes([reg]))
        result = bytearray(length)
        self.i2c.readfrom_into(self.address, result)
        return result

    def read_sensor_data(self, low_reg, high_reg):
        low = self.read_register(low_reg)[0]
        high = self.read_register(high_reg)[0]
        value = (high << 8) | low
        if value >= 32768:
            value -= 65536
        return value

    def readRawAccel(self):
        x = self.read_sensor_data(self.OUTX_L_XL, self.OUTX_H_XL)
        y = self.read_sensor_data(self.OUTY_L_XL, self.OUTY_H_XL)
        z = self.read_sensor_data(self.OUTZ_L_XL, self.OUTZ_H_XL)
        return x, y, z

    def readNormalizeAccel(self):
        x, y, z = self.readRawAccel()
        sensitivity = self.ACCEL_SENSITIVITY[self.accel_scale]  # Sensitivity in mg/LSB
        return [val * sensitivity * 9.80665 / 1000 for val in [x, y, z]]  # Convert to m/s²

    def readScaledAccel(self):
        x, y, z = self.readRawAccel()
        sensitivity = self.ACCEL_SENSITIVITY[self.accel_scale]  # Sensitivity in mg/LSB
        return [val * sensitivity for val in [x, y, z]]  # Convert to mg

    def readRawGyro(self):
        x = self.read_sensor_data(self.OUTX_L_G, self.OUTX_H_G)
        y = self.read_sensor_data(self.OUTY_L_G, self.OUTY_H_G)
        z = self.read_sensor_data(self.OUTZ_L_G, self.OUTZ_H_G)
        return x, y, z

    def readNormalizeGyro(self):
        x, y, z = self.readRawGyro()
        sensitivity = self.GYRO_SENSITIVITY[self.gyro_scale]  # Sensitivity in mdps/LSB
        offsets = self.gyro_offsets
        return [(raw - offset) * sensitivity / 1000 for raw, offset in zip([x, y, z], offsets)]  # Convert to dps

    def calibrateGyro(self, samples=100):
        print("Calibrating gyroscope. Please keep the sensor stationary.")
        sum_x, sum_y, sum_z = 0, 0, 0
        for _ in range(samples):
            raw_x, raw_y, raw_z = self.readRawGyro()
            sum_x += raw_x
            sum_y += raw_y
            sum_z += raw_z
            time.sleep(0.01)

        self.gyro_offsets = [sum_x / samples, sum_y / samples, sum_z / samples]
        print(f"Gyroscope offsets: {self.gyro_offsets}")
