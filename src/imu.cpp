#include "imu.h"
#include "config.h"
#include <Arduino_LSM6DS3.h>

bool IMUSensor::begin() {
    if (!IMU.begin()) {
        return false;
    }
    return true;
}

void IMUSensor::calibrate(int samples) {
    float sumGx = 0, sumGy = 0, sumGz = 0;
    int count = 0;

    Serial.println("Calibrating IMU - keep bike still...");

    while (count < samples) {
        float gx, gy, gz;
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(gx, gy, gz);
            sumGx += gx;
            sumGy += gy;
            sumGz += gz;
            count++;
        }
        delay(5);
    }

    _gyroBiasX = sumGx / count;
    _gyroBiasY = sumGy / count;
    _gyroBiasZ = sumGz / count;
    _calibrated = true;

    // Initialize lean angle from accelerometer
    float ax, ay, az;
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        _leanAngle = atan2(ay, az) * 180.0f / PI;
    }

    Serial.print("Gyro bias X: "); Serial.println(_gyroBiasX);
    Serial.print("Gyro bias Y: "); Serial.println(_gyroBiasY);
    Serial.println("IMU calibration complete.");
}

void IMUSensor::update(float dt) {
    float ax, ay, az;
    float gx, gy, gz;

    // Read accelerometer
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        // Roll angle from accelerometer (lean angle)
        // atan2(ay, az) gives the angle of tilt around the X axis
        _accelAngle = atan2(ay, az) * 180.0f / PI;
    }

    // Read gyroscope
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
        // Remove bias and get rate around X axis (roll rate = lean rate)
        _gyroRateX = gx - _gyroBiasX;
    }

    // Complementary filter: fuse accelerometer and gyroscope
    // Gyro is reliable short-term, accelerometer long-term
    _leanAngle = COMPLEMENTARY_ALPHA * (_leanAngle + _gyroRateX * dt)
               + (1.0f - COMPLEMENTARY_ALPHA) * _accelAngle;

    _leanRate = _gyroRateX;
}
