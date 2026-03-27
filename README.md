# Self-Balancing Motorcycle

A self-balancing motorcycle built on the **Arduino Nano 33 IoT** and **Arduino Nano Motor Carrier**. The bike stays upright using a flywheel (inertia wheel) for primary torque, counter-steering for large-angle recovery, and a cascaded PID control architecture that fuses accelerometer and gyroscope data in real time.

## Hardware

| Component | Role |
|---|---|
| Arduino Nano 33 IoT | Main controller (SAMD21 ARM Cortex-M0+, onboard LSM6DS3 IMU) |
| Arduino Nano Motor Carrier | Motor driver board (I2C to SAMD11 co-processor, 4 motor outputs, 4 servo outputs, 2 encoder inputs) |
| Flywheel motor | Connected to **M3** -- spins an inertia wheel to generate balancing torque |
| Flywheel encoder | Connected to **Encoder1** -- measures flywheel speed for saturation detection |
| Steering servo | Connected to **Servo3** -- turns the front wheel for counter-steering assist |
| Drive motor | Connected to **M2** -- rear wheel (available for forward/backward, not used during balancing) |

## How It Works

### The Balancing Problem

A motorcycle standing still is an inverted pendulum. If it leans left, gravity pulls it further left until it falls. To stay upright the controller must detect the lean and apply a corrective torque faster than gravity can accelerate the fall.

### Sensor Fusion (IMU)

The onboard **LSM6DS3** provides two complementary sensors:

- **Accelerometer** -- measures the direction of gravity. From `atan2(ay, az)` we get the lean angle. Accurate long-term but noisy and corrupted by vibration or sudden movement.
- **Gyroscope** -- measures angular velocity. Integrating it gives angle changes. Accurate short-term but drifts over time.

A **complementary filter** fuses both:

```
leanAngle = 0.98 * (leanAngle + gyroRate * dt) + 0.02 * accelAngle
```

98% of the estimate comes from integrating the gyroscope (fast, clean), and 2% comes from the accelerometer (corrects drift). When the accelerometer detects vibration (total g-force deviates from 1g beyond a threshold), the filter automatically shifts to 99.5% gyro trust to avoid corrupting the angle estimate.

**Calibration** runs at startup: the bike must be held still for ~2 seconds while the system averages 200 gyroscope readings to measure bias offsets and records the accelerometer's resting values. These biases are subtracted from all future readings.

### Cascaded PID Control

Instead of a single PID loop, the controller uses two loops in series:

```
                 Outer Loop                    Inner Loop
Setpoint (0 deg) ---> [ Angle PID ] ---> desired rate ---> [ Rate PID ] ---> flywheel duty
                          ^                                     ^
                          |                                     |
                     lean angle                            lean rate
                    (from IMU)                            (from IMU)
```

**Outer loop (angle)** -- Compares the current lean angle to the setpoint (0 deg = upright). Outputs a *desired angular velocity* -- how fast the bike should be rotating to correct the lean. This loop is relatively slow and corrective.

**Inner loop (rate)** -- Compares the actual angular velocity (gyroscope) to the desired rate from the outer loop. Outputs a flywheel motor duty cycle. This loop is fast and reactive -- it can respond to disturbances within a single control cycle (5ms).

The cascaded structure is more stable than a single loop because the inner loop handles fast dynamics (angular acceleration) while the outer loop handles slow dynamics (angle drift), and each can be tuned independently.

### PID Improvements

The PID controller includes several refinements over a basic implementation:

- **Derivative on measurement** -- The derivative term is computed on the measurement signal rather than the error. This avoids "derivative kick" (a large spike) when the setpoint changes suddenly.
- **Low-pass filtered derivative** -- A first-order filter on the derivative term suppresses high-frequency sensor noise that would otherwise be amplified. The filter coefficient `N` is configurable.
- **Anti-windup** -- The integral accumulator is clamped to prevent it from growing unboundedly when the output is saturated. This avoids sluggish recovery after sustained errors.

### Flywheel Desaturation

The flywheel has a maximum speed. Once it reaches that speed, it cannot provide additional torque in that direction and the bike loses control authority. The desaturation system monitors flywheel speed via the encoder and, when it approaches the limit, gently shifts the lean angle setpoint to let gravity slow the flywheel down. The offset is small (max 3 deg) and proportional to how far past the threshold the flywheel speed is.

### Counter-Steering Assist

When the lean angle exceeds 5 deg, the front wheel servo steers *into* the lean. This shifts the tire contact patch under the center of gravity, providing a restoring force that supplements the flywheel. The steering offset is proportional to lean angle, capped at 20 deg from center. Below 5 deg lean, the servo stays centered to avoid unnecessary jitter.

### Dead Zone Compensation

Small motor duty cycles don't overcome static friction, creating a "dead zone" where the motor doesn't respond. Instead of adding an abrupt offset (which causes oscillation at zero crossing), the controller uses a smooth ramp that maps any nonzero duty request into the motor's responsive range with a continuous function.

## Control Loop

The main loop runs at **200 Hz** (5ms period) and executes the following steps each cycle:

1. **Read IMU** -- update lean angle and lean rate via the complementary filter
2. **Safety check** -- if lean exceeds 45 deg, stop all motors (bike has fallen)
3. **Flywheel desaturation** -- if flywheel speed is near max, compute a setpoint offset
4. **Outer PID** -- angle error to desired angular rate
5. **Inner PID** -- rate error to flywheel duty cycle
6. **Apply flywheel** -- send duty command to the motor
7. **Counter-steering** -- apply servo offset if lean angle is large enough
8. **Debug output** -- print telemetry at 10 Hz over serial

## State Machine

```
  INITIALIZING ──> CALIBRATING ──> BALANCING
                       ^               |
                       |    (lean > 45 deg or 's' command)
                       |               v
                       └─── FALLEN ────┘
                          ('r' command)
```

| State | What happens |
|---|---|
| INITIALIZING | Motor carrier, servo, and IMU are set up in `setup()` |
| CALIBRATING | 2-second hold, then 200-sample gyro/accel bias calibration |
| BALANCING | Full cascaded control loop runs at 200 Hz |
| FALLEN | All motors stopped, waiting for reset command |

## Serial Commands

Connect at **115200 baud**. Commands are single characters optionally followed by a float value:

| Command | Effect |
|---|---|
| `p<value>` | Set outer (angle) loop Kp |
| `i<value>` | Set outer (angle) loop Ki |
| `P<value>` | Set inner (rate) loop Kp |
| `I<value>` | Set inner (rate) loop Ki |
| `D<value>` | Set inner (rate) loop Kd |
| `r` | Reset -- recalibrate and re-enter balancing |
| `s` | Stop -- enter fallen state |

**Debug output format** (printed at 10 Hz):
```
A:<angle> R:<rate> dR:<desiredRate> F:<duty> W:<flywheelSpeed> DS:<desatOffset>
```

## Project Structure

```
BikeProject/
  platformio.ini          Build configuration and dependencies
  include/
    config.h              All tunable parameters in one place
    imu.h                 IMU sensor interface
    pid_controller.h      PID controller interface
    motor_control.h       Motor and encoder interface
    steering.h            Steering servo interface
  src/
    main.cpp              State machine and control loop
    imu.cpp               Complementary filter, calibration, outlier rejection
    pid_controller.cpp    PID with derivative-on-measurement and low-pass filter
    motor_control.cpp     Flywheel/drive motor control, dead zone, saturation tracking
    steering.cpp          Servo routing and angle control
```

## Configuration

All tunable parameters live in [`include/config.h`](include/config.h). Key values to adjust for your build:

| Parameter | Default | Purpose |
|---|---|---|
| `LEAN_ANGLE_SETPOINT` | 0.0 | Offset if center of gravity isn't centered |
| `ANGLE_KP` / `ANGLE_KI` | 18.0 / 0.5 | Outer loop gains (how aggressively it corrects lean) |
| `RATE_KP` / `RATE_KI` / `RATE_KD` | 0.6 / 0.05 / 0.005 | Inner loop gains (how the flywheel tracks desired rate) |
| `FLYWHEEL_DEAD_ZONE` | 15.0 | Motor friction compensation (increase if motor doesn't start) |
| `STEER_ASSIST_GAIN` | 0.8 | How aggressively the servo assists (0 = disabled) |
| `COMPLEMENTARY_ALPHA` | 0.98 | Gyro vs accel trust ratio |
| `LOOP_FREQUENCY_HZ` | 200 | Control loop speed |

## Tuning Guide

1. **Check motor direction** -- tilt the bike and confirm the flywheel opposes the tilt. If it accelerates the fall, reverse the motor wiring or negate the duty in code.
2. **Measure the CG offset** -- hold the bike upright, read the angle from serial, set `LEAN_ANGLE_SETPOINT` to that value.
3. **Tune inner loop first** -- send `p0` and `i0` to disable the outer loop. Increase `Rate Kp` (`P<val>`) until the flywheel firmly resists pushes without buzzing. Add `Rate Kd` for damping.
4. **Tune outer loop** -- re-enable `Angle Kp` (`p<val>`). Increase until the bike self-corrects from small pushes. Add `Angle Ki` to eliminate steady-state lean.
5. **Iterate** -- small changes, one parameter at a time.

## Building and Uploading

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Upload to board
pio run --target upload

# Open serial monitor
pio device monitor
```

## Dependencies

- [ArduinoMotorCarrier](https://github.com/arduino-libraries/ArduinoMotorCarrier) v2.0.0+
- [Arduino_LSM6DS3](https://github.com/arduino-libraries/Arduino_LSM6DS3) v1.0.2+
