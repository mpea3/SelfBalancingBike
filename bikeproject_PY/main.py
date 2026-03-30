"""Self-balancing motorcycle — MicroPython on Arduino Nano 33 IoT."""

from machine import I2C, Pin
from lib.lsm6ds3 import LSM6DS3
from lib.motor_carrier import MotorCarrier
from lib.fall_down_effect import FallDownEffect

# Nano 33 IoT I2C pins: SDA=A4 (Pin 11), SCL=A5 (Pin 12)
i2c = I2C(0, scl=Pin(13), sda=Pin(11), freq=400_000)

imu = LSM6DS3(i2c)
carrier = MotorCarrier(i2c)

# PDP: Kp=65.0, Kd=5.0, Kp_w=0.015
balancer = FallDownEffect(imu, carrier, Kp=65.0, Kd=5.0, Kp_w=0.015, servo_center=90)

balancer.setup()

while True:
    balancer.start_balance()
    # balancer.console_log()
