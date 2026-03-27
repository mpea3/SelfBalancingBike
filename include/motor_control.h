#pragma once

#include <Arduino.h>

// =============================================================================
// Motor Control Module
// Controls the flywheel (inertia wheel) and rear drive motor
// via the Arduino Nano Motor Carrier.
// Includes flywheel saturation detection for desaturation logic.
// =============================================================================

class MotorControl {
public:
    bool begin();

    // Flywheel control (balancing)
    // duty: -100 to +100 (percentage, sign = direction)
    void setFlywheelDuty(float duty);

    // Drive motor control (forward/backward movement)
    // duty: -100 to +100
    void setDriveDuty(float duty);

    // Stop all motors immediately
    void stopAll();

    // Read flywheel encoder
    long getFlywheelEncoderCount();
    void resetFlywheelEncoder();

    // Get flywheel speed in encoder ticks per second
    float getFlywheelSpeed(float dt);

    // Returns true if flywheel is approaching saturation
    bool isFlywheelSaturated() const { return _saturated; }

    // Signed flywheel speed (positive = one direction)
    float getFlywheelSpeedSigned() const { return _flywheelSpeed; }

    // Ping the motor carrier to keep communication alive
    void keepAlive();

private:
    float applyDeadZone(float duty);
    long _prevEncoderCount = 0;
    float _flywheelSpeed = 0.0f;
    bool _saturated = false;
};
