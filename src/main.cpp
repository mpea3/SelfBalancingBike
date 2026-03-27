#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "pid_controller.h"
#include "motor_control.h"
#include "steering.h"

// =============================================================================
// Self-Balancing Motorcycle - Main Control Loop
//
// Cascaded control architecture:
//   Outer loop: lean ANGLE error  → desired lean RATE  (slower, corrective)
//   Inner loop: lean RATE error   → flywheel DUTY      (fast, reactive)
//
// Additional stabilization:
//   - Flywheel desaturation: shifts setpoint to unwind flywheel momentum
//   - Counter-steering assist: servo nudge at larger lean angles
//   - Accelerometer outlier rejection in IMU
//   - Derivative-on-measurement with low-pass filter in PID
//
// Hardware: Arduino Nano 33 IoT + Nano Motor Carrier
// =============================================================================

// -- Module instances ---------------------------------------------------------
IMUSensor      imu;
PIDController  anglePID(ANGLE_KP, ANGLE_KI, ANGLE_KD);
PIDController  ratePID(RATE_KP, RATE_KI, RATE_KD);
MotorControl   motors;
Steering       steering;

// -- State --------------------------------------------------------------------
enum class BikeState {
    INITIALIZING,
    CALIBRATING,
    BALANCING,
    FALLEN
};

BikeState state = BikeState::INITIALIZING;
unsigned long lastLoopTime = 0;
int loopCounter = 0;

// Runtime-adjustable gains (initialized from config, modifiable via serial)
float runtimeAngleKp = ANGLE_KP;
float runtimeAngleKi = ANGLE_KI;
float runtimeRateKp  = RATE_KP;
float runtimeRateKi  = RATE_KI;
float runtimeRateKd  = RATE_KD;

// -- Serial command processing ------------------------------------------------
void processSerialCommands() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    float val = 0;
    if (Serial.available()) {
        val = Serial.parseFloat();
    }

    switch (cmd) {
        // Outer loop (angle) tuning
        case 'p': runtimeAngleKp = val;
                  anglePID.setGains(val, runtimeAngleKi, ANGLE_KD);
                  Serial.print("Angle Kp = "); Serial.println(val); break;
        case 'i': runtimeAngleKi = val;
                  anglePID.setGains(runtimeAngleKp, val, ANGLE_KD);
                  Serial.print("Angle Ki = "); Serial.println(val); break;

        // Inner loop (rate) tuning
        case 'P': runtimeRateKp = val;
                  ratePID.setGains(val, runtimeRateKi, runtimeRateKd);
                  Serial.print("Rate Kp = "); Serial.println(val); break;
        case 'I': runtimeRateKi = val;
                  ratePID.setGains(runtimeRateKp, val, runtimeRateKd);
                  Serial.print("Rate Ki = "); Serial.println(val); break;
        case 'D': runtimeRateKd = val;
                  ratePID.setGains(runtimeRateKp, runtimeRateKi, val);
                  Serial.print("Rate Kd = "); Serial.println(val); break;

        case 'r': // Reset - re-enter balancing
                  anglePID.reset();
                  ratePID.reset();
                  motors.stopAll();
                  steering.center();
                  state = BikeState::CALIBRATING;
                  Serial.println("Reset!"); break;
        case 's': // Stop
                  motors.stopAll();
                  steering.center();
                  state = BikeState::FALLEN;
                  Serial.println("Stopped."); break;
        default: break;
    }
}

// -- Debug output -------------------------------------------------------------
void printDebug(float leanAngle, float leanRate, float desiredRate,
                float flywheelDuty, float flywheelSpeed, float setpointOffset) {
    if (!DEBUG_ENABLED) return;
    if (++loopCounter < DEBUG_PRINT_INTERVAL) return;
    loopCounter = 0;

    Serial.print("A:");   Serial.print(leanAngle, 1);
    Serial.print(" R:");  Serial.print(leanRate, 1);
    Serial.print(" dR:"); Serial.print(desiredRate, 1);
    Serial.print(" F:");  Serial.print(flywheelDuty, 1);
    Serial.print(" W:");  Serial.print(flywheelSpeed, 0);
    if (fabsf(setpointOffset) > 0.01f) {
        Serial.print(" DS:"); Serial.print(setpointOffset, 2);
    }
    Serial.println();
}

// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    // Wait briefly for serial, but don't block if no USB connected
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000));

    Serial.println("=== Self-Balancing Motorcycle v2 ===");
    Serial.println("Cascaded PID + Desaturation + Steering Assist");
    Serial.println("Initializing...");

    // Initialize motor carrier
    if (!motors.begin()) {
        Serial.println("FATAL: Motor Carrier init failed!");
        while (true) { delay(1000); }
    }
    Serial.println("Motor Carrier OK");

    // Initialize steering
    steering.begin();
    steering.center();
    Serial.println("Steering centered");

    // Initialize IMU
    if (!imu.begin()) {
        Serial.println("FATAL: IMU init failed!");
        while (true) { delay(1000); }
    }
    Serial.println("IMU OK");

    // Configure PID limits
    anglePID.setOutputLimits(ANGLE_OUTPUT_MIN, ANGLE_OUTPUT_MAX);
    anglePID.setIntegralLimits(ANGLE_INTEGRAL_MIN, ANGLE_INTEGRAL_MAX);

    ratePID.setOutputLimits(RATE_OUTPUT_MIN, RATE_OUTPUT_MAX);
    ratePID.setIntegralLimits(RATE_INTEGRAL_MIN, RATE_INTEGRAL_MAX);

    // Transition to calibration
    state = BikeState::CALIBRATING;
}

// =============================================================================
void loop() {
    unsigned long now = micros();
    float dt = (now - lastLoopTime) / 1000000.0f;

    // Enforce fixed loop timing
    if (dt < (LOOP_PERIOD_US / 1000000.0f)) {
        return;
    }
    lastLoopTime = now;

    // Handle serial commands at any time
    processSerialCommands();

    // Keep motor carrier communication alive
    motors.keepAlive();

    switch (state) {

    case BikeState::INITIALIZING:
        // Handled in setup()
        break;

    case BikeState::CALIBRATING:
        Serial.println("Hold bike upright for calibration...");
        delay(2000);
        imu.calibrate(200);
        anglePID.reset();
        ratePID.reset();
        motors.resetFlywheelEncoder();
        steering.center();
        lastLoopTime = micros();
        state = BikeState::BALANCING;
        Serial.println("BALANCING - bike is live!");
        Serial.println("Tune: p/i=angle  P/I/D=rate  r=reset  s=stop");
        break;

    case BikeState::BALANCING: {
        // =====================================================================
        // STEP 1: Read sensors
        // =====================================================================
        imu.update(dt);
        float leanAngle = imu.getLeanAngle();
        float leanRate  = imu.getLeanRate();

        // =====================================================================
        // STEP 2: Safety check - has the bike fallen over?
        // =====================================================================
        if (fabsf(leanAngle) > LEAN_ANGLE_CUTOFF) {
            motors.stopAll();
            steering.center();
            anglePID.reset();
            ratePID.reset();
            state = BikeState::FALLEN;
            Serial.println("FALLEN! Lean angle exceeded cutoff.");
            Serial.println("Send 'r' to reset.");
            break;
        }

        // =====================================================================
        // STEP 3: Flywheel desaturation
        // When the flywheel is spinning too fast, it can't provide more torque.
        // We gently shift the lean setpoint to let gravity unwind the flywheel.
        // =====================================================================
        float flywheelSpeed = motors.getFlywheelSpeed(dt);
        float desatOffset = 0.0f;

        if (motors.isFlywheelSaturated()) {
            // Shift setpoint in the direction that will slow the flywheel
            // If flywheel spins positive → lean slightly positive → flywheel decelerates
            float excess = fabsf(flywheelSpeed) - FLYWHEEL_SATURATION_SPEED;
            desatOffset = DESAT_GAIN * excess;
            desatOffset = constrain(desatOffset, 0.0f, DESAT_MAX_OFFSET);

            // Apply in the direction of flywheel spin
            if (flywheelSpeed > 0) {
                desatOffset = desatOffset;  // lean positive
            } else {
                desatOffset = -desatOffset; // lean negative
            }
        }

        float effectiveSetpoint = LEAN_ANGLE_SETPOINT + desatOffset;

        // =====================================================================
        // STEP 4: Cascaded PID control
        //
        // Outer loop: angle error → desired angular rate
        //   "How fast should we be rotating to correct this lean?"
        //
        // Inner loop: rate error → flywheel duty
        //   "How much flywheel torque to achieve the desired rotation rate?"
        // =====================================================================

        // Outer loop: lean angle → desired lean rate
        float desiredRate = anglePID.compute(effectiveSetpoint, leanAngle, dt);

        // Inner loop: lean rate → flywheel duty
        float flywheelDuty = ratePID.compute(desiredRate, leanRate, dt);

        // =====================================================================
        // STEP 5: Apply flywheel command
        // =====================================================================
        motors.setFlywheelDuty(flywheelDuty);

        // =====================================================================
        // STEP 6: Counter-steering assist
        // At larger lean angles, a small steering input helps recovery.
        // Counter-steering: steer INTO the lean to shift the contact patch
        // under the center of gravity.
        // =====================================================================
        if (fabsf(leanAngle) > STEER_ASSIST_THRESHOLD) {
            float steerOffset = STEER_ASSIST_GAIN * leanAngle;
            steerOffset = constrain(steerOffset, -STEER_ASSIST_MAX, STEER_ASSIST_MAX);
            steering.steer(static_cast<int>(steerOffset));
        } else {
            steering.center();
        }

        // =====================================================================
        // STEP 7: Debug output
        // =====================================================================
        printDebug(leanAngle, leanRate, desiredRate, flywheelDuty,
                   flywheelSpeed, desatOffset);
        break;
    }

    case BikeState::FALLEN:
        // Motors are stopped. Wait for reset command.
        motors.stopAll();
        break;
    }
}
