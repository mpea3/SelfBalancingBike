#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "pid_controller.h"
#include "motor_control.h"
#include "steering.h"

// =============================================================================
// Self-Balancing Motorcycle - Main Control Loop
//
// The bike balances using an inverted pendulum approach:
//   1. IMU reads the lean angle (roll)
//   2. PID controller computes corrective torque
//   3. Flywheel motor spins to counteract the lean
//
// Hardware: Arduino Nano 33 IoT + Nano Motor Carrier
// =============================================================================

// -- Module instances ---------------------------------------------------------
IMUSensor      imu;
PIDController  balancePID(PID_KP, PID_KI, PID_KD, PID_MASTER_GAIN);
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

// -- Serial command processing ------------------------------------------------
void processSerialCommands() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    float val = 0;
    if (Serial.available()) {
        val = Serial.parseFloat();
    }

    switch (cmd) {
        case 'p': balancePID.setGains(val, PID_KI, PID_KD);
                  Serial.print("Kp = "); Serial.println(val); break;
        case 'i': balancePID.setGains(PID_KP, val, PID_KD);
                  Serial.print("Ki = "); Serial.println(val); break;
        case 'd': balancePID.setGains(PID_KP, PID_KI, val);
                  Serial.print("Kd = "); Serial.println(val); break;
        case 'g': balancePID.setMasterGain(val);
                  Serial.print("Gain = "); Serial.println(val); break;
        case 'r': // Reset - re-enter balancing
                  balancePID.reset();
                  motors.stopAll();
                  state = BikeState::CALIBRATING;
                  Serial.println("Reset!"); break;
        case 's': // Stop
                  motors.stopAll();
                  state = BikeState::FALLEN;
                  Serial.println("Stopped."); break;
        default: break;
    }
}

// -- Debug output -------------------------------------------------------------
void printDebug(float leanAngle, float pidOutput) {
    if (!DEBUG_ENABLED) return;
    if (++loopCounter < DEBUG_PRINT_INTERVAL) return;
    loopCounter = 0;

    Serial.print("Lean: ");   Serial.print(leanAngle, 2);
    Serial.print("  PID: ");  Serial.print(pidOutput, 2);
    Serial.print("  P: ");    Serial.print(balancePID.getProportional(), 2);
    Serial.print("  I: ");    Serial.print(balancePID.getIntegral(), 2);
    Serial.print("  D: ");    Serial.print(balancePID.getDerivative(), 2);
    Serial.println();
}

// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    // Wait briefly for serial, but don't block if no USB connected
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000));

    Serial.println("=== Self-Balancing Motorcycle ===");
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

    // Transition to calibration
    state = BikeState::CALIBRATING;
}

// =============================================================================
void loop() {
    unsigned long now = micros();
    float dt = (now - lastLoopTime) / 1000000.0f;

    // Enforce fixed loop timingp
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
        balancePID.reset();
        motors.resetFlywheelEncoder();
        lastLoopTime = micros();
        state = BikeState::BALANCING;
        Serial.println("BALANCING - bike is live!");
        Serial.println("Commands: p<val> i<val> d<val> g<val> r=reset s=stop");
        break;

    case BikeState::BALANCING: {
        // 1. Update IMU lean angle
        imu.update(dt);
        float leanAngle = imu.getLeanAngle();

        // 2. Safety check: has the bike fallen over?
        if (fabs(leanAngle) > LEAN_ANGLE_CUTOFF) {
            motors.stopAll();
            balancePID.reset();
            state = BikeState::FALLEN;
            Serial.println("FALLEN! Lean angle exceeded cutoff.");
            Serial.println("Send 'r' to reset.");
            break;
        }

        // 3. PID compute: target is the lean angle setpoint (typically 0)
        float pidOutput = balancePID.compute(LEAN_ANGLE_SETPOINT, leanAngle, dt);

        // 4. Drive flywheel to counteract lean
        //    Positive lean → spin flywheel one way
        //    Negative lean → spin flywheel the other way
        motors.setFlywheelDuty(pidOutput);

        // 5. Debug output
        printDebug(leanAngle, pidOutput);
        break;
    }

    case BikeState::FALLEN:
        // Motors are stopped. Wait for reset command.
        motors.stopAll();
        break;
    }
}
