#pragma once

// =============================================================================
// Self-Balancing Motorcycle Configuration
// Arduino Engineering Kit Rev2
// =============================================================================

// -- Control Loop Timing ------------------------------------------------------
constexpr float LOOP_FREQUENCY_HZ = 100.0f;
constexpr unsigned long LOOP_PERIOD_US = static_cast<unsigned long>(1000000.0f / LOOP_FREQUENCY_HZ);

// -- PID Gains (Balancing - flywheel control) ---------------------------------
// These need to be tuned experimentally for your specific build.
// Starting values based on community-tested parameters.
constexpr float PID_KP = 90.0f;
constexpr float PID_KI = 0.8f;
constexpr float PID_KD = 10.0f;
constexpr float PID_MASTER_GAIN = 0.1f;

// PID output limits (maps to motor duty cycle -100..+100)
constexpr float PID_OUTPUT_MIN = -100.0f;
constexpr float PID_OUTPUT_MAX = 100.0f;

// Integrator windup limits
constexpr float PID_INTEGRAL_MIN = -50.0f;
constexpr float PID_INTEGRAL_MAX = 50.0f;

// -- IMU Configuration --------------------------------------------------------
// Complementary filter coefficient (0..1, higher = more gyro trust)
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

// Lean angle setpoint in degrees (0 = perfectly upright)
// Adjust this to compensate for center-of-gravity offset
constexpr float LEAN_ANGLE_SETPOINT = 0.0f;

// Safety cutoff: if lean exceeds this, stop motors (bike has fallen)
constexpr float LEAN_ANGLE_CUTOFF = 45.0f;

// -- Motor Configuration (Nano Motor Carrier) ---------------------------------
// Flywheel motor (inertia wheel) - connects to M3 on Motor Carrier
constexpr int FLYWHEEL_MOTOR_INDEX = 3;

// Drive motor (rear wheel) - connects to M2 on Motor Carrier
constexpr int DRIVE_MOTOR_INDEX = 2;

// Flywheel encoder - connects to Encoder1 on Motor Carrier
constexpr int FLYWHEEL_ENCODER_INDEX = 1;

// Maximum flywheel speed (duty cycle percentage)
constexpr float FLYWHEEL_MAX_DUTY = 100.0f;

// Flywheel motor PWM offset to overcome static friction
constexpr float FLYWHEEL_DEAD_ZONE = 15.0f;

// -- Servo Configuration (Steering) -------------------------------------------
// Steering servo - connects to Servo1 on Motor Carrier
constexpr int STEERING_SERVO_INDEX = 1;

// Servo angle range
constexpr int SERVO_CENTER = 90;       // Straight ahead
constexpr int SERVO_MIN = 45;          // Max left
constexpr int SERVO_MAX = 135;         // Max right

// -- Serial Debug -------------------------------------------------------------
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr bool DEBUG_ENABLED = true;

// Print interval for debug data (every N loop cycles)
constexpr int DEBUG_PRINT_INTERVAL = 10;
