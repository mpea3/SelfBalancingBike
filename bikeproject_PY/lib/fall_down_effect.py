"""Self-balancing motorcycle controller — MicroPython port of fall_down_effect.h"""

import math
import time


class FallDownEffect:
    def __init__(self, imu, carrier, Kp=65.0, Kd=5.0, Kp_w=0.015, servo_center=90):
        self._imu = imu
        self._carrier = carrier

        # Sensor data
        self.Ax = 0.0
        self.Ay = 0.0
        self.Az = 0.0
        self.Gx = 0.0
        self.Gy = 0.0
        self.Gz = 0.0

        # State
        self.accel_angle = 0.0
        self.current_angle = 0.0
        self.theta_dot = 0.0
        self.current_duty = 0.0
        self.current_servo = float(servo_center)
        self._last_gyro_time = 0

        # PDP controller gains
        self.Kp = Kp       # P: proportional to lean angle
        self.Kd = Kd        # D: proportional to angular velocity
        self.Kp_w = Kp_w    # P: proportional to wheel speed (prevents runaway)

        self.ramp_rate = 4.0
        self.servo_ramp_rate = 2.0
        self.servo_center = servo_center

    def setup(self):
        self._carrier.begin()
        self._imu.begin()
        self._last_gyro_time = time.ticks_us()

    # ---- TASK 1: Gyroscope — detect fall angle ----
    def _update_gyroscope(self):
        now = time.ticks_us()
        dt = time.ticks_diff(now, self._last_gyro_time) / 1_000_000.0
        self._last_gyro_time = now

        if self._imu.gyroscope_available():
            self.Gx, self.Gy, self.Gz = self._imu.read_gyroscope()

            # Complementary filter: gyro for fast response, accel for drift correction
            self.current_angle = (
                0.98 * (self.current_angle + self.Gx * 57.3 * dt)
                + 0.02 * self.accel_angle
            )

            # Angular velocity of lean
            self.theta_dot = -self.Gx * 57.3

    # ---- TASK 2: Flywheel — balance control ----
    def _control_flywheel(self):
        if self._imu.acceleration_available():
            self.Ay, self.Ax, self.Az = self._imu.read_acceleration()
            self.accel_angle = math.atan2(self.Ay, self.Az) * 57.3

        # PDP control: angle(P) + angular velocity(D) + wheel speed(P)
        target_angle = 0.0
        error = self.current_angle - target_angle
        torque = (self.Kp * error) + (self.Kd * self.theta_dot) + (self.Kp_w * self.current_duty)

        target_duty = max(-70.0, min(70.0, -torque))

        # Ramping — emergency boost when falling fast
        dynamic_ramp = self.ramp_rate
        if abs(self.theta_dot) > 50:
            dynamic_ramp = self.ramp_rate * 2

        if self.current_duty < target_duty:
            self.current_duty = min(self.current_duty + dynamic_ramp, target_duty)
        elif self.current_duty > target_duty:
            self.current_duty = max(self.current_duty - dynamic_ramp, target_duty)

        self._carrier.M3.set_duty(int(self.current_duty))

    # ---- TASK 3: Front wheel — steer on lean (disabled) ----
    # def _control_front_wheel(self):
    #     target_servo = self.servo_center
    #     if abs(self.current_angle) > 10:
    #         steer_offset = max(-30, min(30, int(self.current_angle * -0.5)))
    #         target_servo = max(0, min(180, self.servo_center + steer_offset))
    #     if self.current_servo < target_servo:
    #         self.current_servo = min(self.current_servo + self.servo_ramp_rate, target_servo)
    #     elif self.current_servo > target_servo:
    #         self.current_servo = max(self.current_servo - self.servo_ramp_rate, target_servo)
    #     self._carrier.servo3.set_angle(int(self.current_servo))

    def start_balance(self):
        """Call every loop — runs all control tasks."""
        self._update_gyroscope()
        self._control_flywheel()

    def console_log(self):
        print(
            "Angle:{:.1f} Ax:{:.2f} Ay:{:.2f} Az:{:.2f}"
            " Gx:{:.2f} Gy:{:.2f} Gz:{:.2f}"
            " Duty:{} Servo:{}".format(
                self.current_angle,
                self.Ax, self.Ay, self.Az,
                self.Gx, self.Gy, self.Gz,
                int(self.current_duty),
                int(self.current_servo),
            )
        )
