"""LSM6DS3 IMU driver for MicroPython (I2C) on Arduino Nano 33 IoT."""

from micropython import const
import struct

# I2C address
_LSM6DS3_ADDR = const(0x6A)

# Registers
_WHO_AM_I = const(0x0F)
_CTRL1_XL = const(0x10)  # Accelerometer control
_CTRL2_G = const(0x11)   # Gyroscope control
_STATUS_REG = const(0x1E)
_OUTX_L_G = const(0x22)  # Gyro data start
_OUTX_L_XL = const(0x28) # Accel data start

# Status bits
_XLDA = const(0x01)  # Accelerometer data available
_GDA = const(0x02)   # Gyroscope data available

# Sensitivity scales (matching Arduino_LSM6DS3 library defaults)
# Accel: ±4g at 104 Hz  -> sensitivity 0.122 mg/LSB
# Gyro:  ±245 dps at 104 Hz -> sensitivity 8.75 mdps/LSB
_ACCEL_SENSITIVITY = 0.000122  # g per LSB
_GYRO_SENSITIVITY = 0.00875    # dps per LSB


class LSM6DS3:
    def __init__(self, i2c):
        self._i2c = i2c
        self._addr = _LSM6DS3_ADDR
        self._buf6 = bytearray(6)

    def begin(self):
        who = self._read_byte(_WHO_AM_I)
        if who != 0x69:
            raise RuntimeError("LSM6DS3 not found (WHO_AM_I=0x{:02X})".format(who))

        # Accel: 104 Hz, ±4g
        self._write_byte(_CTRL1_XL, 0x4A)
        # Gyro: 104 Hz, ±245 dps
        self._write_byte(_CTRL2_G, 0x42)

    def acceleration_available(self):
        return bool(self._read_byte(_STATUS_REG) & _XLDA)

    def gyroscope_available(self):
        return bool(self._read_byte(_STATUS_REG) & _GDA)

    def read_acceleration(self):
        """Returns (ax, ay, az) in g."""
        self._i2c.readfrom_mem_into(self._addr, _OUTX_L_XL, self._buf6)
        raw = struct.unpack_from('<hhh', self._buf6)
        return (
            raw[0] * _ACCEL_SENSITIVITY,
            raw[1] * _ACCEL_SENSITIVITY,
            raw[2] * _ACCEL_SENSITIVITY,
        )

    def read_gyroscope(self):
        """Returns (gx, gy, gz) in degrees per second."""
        self._i2c.readfrom_mem_into(self._addr, _OUTX_L_G, self._buf6)
        raw = struct.unpack_from('<hhh', self._buf6)
        return (
            raw[0] * _GYRO_SENSITIVITY,
            raw[1] * _GYRO_SENSITIVITY,
            raw[2] * _GYRO_SENSITIVITY,
        )

    def _read_byte(self, reg):
        return self._i2c.readfrom_mem(self._addr, reg, 1)[0]

    def _write_byte(self, reg, val):
        self._i2c.writeto_mem(self._addr, reg, bytes([val]))
