#pragma once

#include <Arduino.h>

// =============================================================================
// IMU Module - Lean Angle Sensing
// Uses the LSM6DS3 IMU on the Arduino Nano 33 IoT
// Computes roll (lean) angle via complementary filter with outlier rejection
// =============================================================================

class IMUSensor {
public:
    bool begin();
    void update(float dt);
    void calibrate(int samples = 200);

    float getLeanAngle() const { return _leanAngle; }
    float getLeanRate() const { return _leanRate; }
    float getRawAccelAngle() const { return _accelAngle; }
    float getRawGyroRate() const { return _gyroRateX; }
    bool isCalibrated() const { return _calibrated; }

private:
    // Filtered lean angle (degrees)
    float _leanAngle = 0.0f;

    // Angular rate of lean (degrees/sec)
    float _leanRate = 0.0f;

    // Raw sensor-derived values
    float _accelAngle = 0.0f;
    float _gyroRateX = 0.0f;

    // Gyroscope bias offsets (from calibration)
    float _gyroBiasX = 0.0f;
    float _gyroBiasY = 0.0f;
    float _gyroBiasZ = 0.0f;

    // Accelerometer bias offsets (from calibration)
    float _accelBiasY = 0.0f;
    float _accelBiasZ = 0.0f;

    // Track whether the last accel reading was valid (for adaptive alpha)
    bool _accelValid = true;

    bool _calibrated = false;
};
