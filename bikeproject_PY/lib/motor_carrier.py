"""Arduino Motor Carrier driver for MicroPython (I2C).

The Motor Carrier board has a SAMD11 co-processor at I2C address 0x66.
It accepts simple register-write commands to control motors and servos.

Protocol (reverse-engineered from ArduinoMotorCarrier library):
  - Write 3 bytes: [register, value_high, value_low]
  - Motor duty is a signed 16-bit value (-100 to +100 maps to full reverse/forward)
  - Servo angle is 0-180
"""

from micropython import const

_CARRIER_ADDR = const(0x66)

# Motor register base addresses (from ArduinoMotorCarrier source)
_M1_REG = const(0x20)
_M2_REG = const(0x21)
_M3_REG = const(0x22)
_M4_REG = const(0x23)

# Servo register base addresses
_SERVO1_REG = const(0x10)
_SERVO2_REG = const(0x11)
_SERVO3_REG = const(0x12)
_SERVO4_REG = const(0x13)

# Ping/init register
_PING_REG = const(0x01)


class Motor:
    def __init__(self, carrier, reg):
        self._carrier = carrier
        self._reg = reg

    def set_duty(self, duty):
        """Set motor duty cycle (-100 to +100)."""
        duty = max(-100, min(100, int(duty)))
        self._carrier._write_reg16(self._reg, duty)


class Servo:
    def __init__(self, carrier, reg):
        self._carrier = carrier
        self._reg = reg

    def set_angle(self, angle):
        """Set servo angle (0-180)."""
        angle = max(0, min(180, int(angle)))
        self._carrier._write_reg16(self._reg, angle)


class MotorCarrier:
    def __init__(self, i2c):
        self._i2c = i2c
        self._addr = _CARRIER_ADDR

        self.M1 = Motor(self, _M1_REG)
        self.M2 = Motor(self, _M2_REG)
        self.M3 = Motor(self, _M3_REG)
        self.M4 = Motor(self, _M4_REG)

        self.servo1 = Servo(self, _SERVO1_REG)
        self.servo2 = Servo(self, _SERVO2_REG)
        self.servo3 = Servo(self, _SERVO3_REG)
        self.servo4 = Servo(self, _SERVO4_REG)

    def begin(self):
        """Initialize the motor carrier board."""
        # Ping the carrier to verify it's present
        try:
            self._i2c.writeto(self._addr, bytes([_PING_REG]))
        except OSError:
            raise RuntimeError("Motor Carrier not found at 0x{:02X}".format(self._addr))

    def _write_reg16(self, reg, value):
        """Write a 16-bit signed value to a register."""
        # Convert signed to unsigned 16-bit for transport
        if value < 0:
            value = value + 65536
        high = (value >> 8) & 0xFF
        low = value & 0xFF
        self._i2c.writeto(self._addr, bytes([reg, high, low]))
