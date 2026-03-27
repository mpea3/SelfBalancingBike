#pragma once

// =============================================================================
// Self-Balancing Motorcycle Configuration
// Arduino Engineering Kit Rev2
// =============================================================================

// -- Control Loop Timing ------------------------------------------------------
constexpr float LOOP_FREQUENCY_HZ = 200.0f;  // 200 Hz for tighter control
constexpr unsigned long LOOP_PERIOD_US = static_cast<unsigned long>(1000000.0f / LOOP_FREQUENCY_HZ);

// -- Cascaded PID: Outer Loop (Lean Angle → desired lean rate) ----------------
// Slower loop that converts angle error into a target angular velocity
constexpr float ANGLE_KP = 18.0f;
constexpr float ANGLE_KI = 0.5f;
constexpr float ANGLE_KD = 0.0f;     // Derivative handled by inner loop
constexpr float ANGLE_OUTPUT_MIN = -300.0f;  // deg/s target range
constexpr float ANGLE_OUTPUT_MAX = 300.0f;
constexpr float ANGLE_INTEGRAL_MIN = -40.0f;
constexpr float ANGLE_INTEGRAL_MAX = 40.0f;

// -- Cascaded PID: Inner Loop (Lean Rate → flywheel duty) ---------------------
// Fast loop that tracks the desired angular velocity with the flywheel
constexpr float RATE_KP = 0.6f;
constexpr float RATE_KI = 0.05f;
constexpr float RATE_KD = 0.005f;
constexpr float RATE_OUTPUT_MIN = -100.0f;
constexpr float RATE_OUTPUT_MAX = 100.0f;
constexpr float RATE_INTEGRAL_MIN = -30.0f;
constexpr float RATE_INTEGRAL_MAX = 30.0f;

// -- Derivative Low-Pass Filter -----------------------------------------------
// First-order filter on derivative term to suppress sensor noise
// Cutoff ≈ LOOP_FREQUENCY / (2*PI*N), where N is the filter coefficient
constexpr float DERIVATIVE_FILTER_N = 20.0f;

// -- IMU Configuration --------------------------------------------------------
// Complementary filter coefficient (0..1, higher = more gyro trust)
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

// Accelerometer outlier rejection threshold (g-force magnitude)
// Valid range: if total accel magnitude is outside [1-threshold, 1+threshold]
// the accel reading is likely corrupted by vibration and should be ignored
constexpr float ACCEL_OUTLIER_THRESHOLD = 0.3f;

// Lean angle setpoint in degrees (0 = perfectly upright)
// Adjust this to compensate for center-of-gravity offset
constexpr float LEAN_ANGLE_SETPOINT = 0.0f;

// Safety cutoff: if lean exceeds this, stop motors (bike has fallen)
constexpr float LEAN_ANGLE_CUTOFF = 45.0f;

// -- Flywheel Saturation Management -------------------------------------------
// When flywheel speed (encoder ticks/sec) exceeds this, begin desaturation
constexpr float FLYWHEEL_SATURATION_SPEED = 8000.0f;
// Maximum lean angle offset applied for desaturation (degrees)
constexpr float DESAT_MAX_OFFSET = 3.0f;
// Gain for desaturation: offset = gain * (speed - threshold)
constexpr float DESAT_GAIN = 0.0005f;

// -- Steering Assist ----------------------------------------------------------
// Counter-steering kicks in above this lean angle (degrees)
constexpr float STEER_ASSIST_THRESHOLD = 5.0f;
// Steering gain: servo_offset = gain * lean_angle
constexpr float STEER_ASSIST_GAIN = 0.8f;
// Maximum steering assist offset from center (degrees)
constexpr float STEER_ASSIST_MAX = 20.0f;

// -- Motor Configuration (Nano Motor Carrier) ---------------------------------
// Flywheel motor (inertia wheel) - connects to M3 on Motor Carrier
constexpr int FLYWHEEL_MOTOR_INDEX = 3;

// Drive motor (rear wheel) - connects to M2 on Motor Carrier
constexpr int DRIVE_MOTOR_INDEX = 2;

// Flywheel encoder - connects to Encoder1 on Motor Carrier
constexpr int FLYWHEEL_ENCODER_INDEX = 1;

// Maximum flywheel speed (duty cycle percentage)
constexpr float FLYWHEEL_MAX_DUTY = 100.0f;

// Flywheel motor PWM dead zone - uses smooth compensation curve
constexpr float FLYWHEEL_DEAD_ZONE = 15.0f;

// -- Servo Configuration (Steering) -------------------------------------------
// Steering servo - connects to Servo1 on Motor Carrier
constexpr int STEERING_SERVO_INDEX = 3;

// Servo angle range
constexpr int SERVO_CENTER = 90;       // Straight ahead
constexpr int SERVO_MIN = 45;          // Max left
constexpr int SERVO_MAX = 135;         // Max right

// -- Serial Debug -------------------------------------------------------------
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr bool DEBUG_ENABLED = true;

// Print interval for debug data (every N loop cycles)
constexpr int DEBUG_PRINT_INTERVAL = 20;  // 10 Hz at 200 Hz loop
