#pragma once

#include <Arduino.h>

// =============================================================================
// Steering Module
// Controls the front wheel steering servo via the Nano Motor Carrier
// =============================================================================

class Steering {
public:
    bool begin();

    // Set steering angle in degrees (SERVO_MIN..SERVO_MAX)
    // 90 = center/straight, <90 = left, >90 = right
    void setAngle(int angle);

    // Center the steering
    void center();

    // Set a steering offset from center (-45..+45 degrees)
    void steer(int offset);

    int getCurrentAngle() const { return _currentAngle; }

private:
    int _currentAngle = 90;
};
