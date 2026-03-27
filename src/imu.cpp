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
    float sumAy = 0, sumAz = 0;
    int gyroCount = 0;
    int accelCount = 0;

    Serial.println("Calibrating IMU - keep bike still...");

    while (gyroCount < samples) {
        float gx, gy, gz;
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(gx, gy, gz);
            sumGx += gx;
            sumGy += gy;
            sumGz += gz;
            gyroCount++;
        }

        float ax, ay, az;
        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(ax, ay, az);
            sumAy += ay;
            sumAz += az;
            accelCount++;
        }
        delay(5);
    }

    _gyroBiasX = sumGx / gyroCount;
    _gyroBiasY = sumGy / gyroCount;
    _gyroBiasZ = sumGz / gyroCount;

    // Store accelerometer bias: at rest and upright, ay≈0, az≈1g
    // We store the offset from the expected values so we can correct later
    if (accelCount > 0) {
        _accelBiasY = sumAy / accelCount;        // should be ~0 at rest upright
        _accelBiasZ = (sumAz / accelCount) - 1.0f; // should be ~1g at rest upright
    }

    _calibrated = true;

    // Initialize lean angle from accelerometer
    float ax, ay, az;
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        ay -= _accelBiasY;
        az -= _accelBiasZ;
        _leanAngle = atan2(ay, az) * 180.0f / PI;
    }

    Serial.print("Gyro bias X: "); Serial.println(_gyroBiasX, 4);
    Serial.print("Gyro bias Y: "); Serial.println(_gyroBiasY, 4);
    Serial.print("Accel bias Y: "); Serial.println(_accelBiasY, 4);
    Serial.print("Accel bias Z: "); Serial.println(_accelBiasZ, 4);
    Serial.println("IMU calibration complete.");
}

void IMUSensor::update(float dt) {
    float ax, ay, az;
    float gx, gy, gz;

    // Read accelerometer
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);

        // Apply calibration offsets
        ay -= _accelBiasY;
        az -= _accelBiasZ;

        // Outlier rejection: check if total acceleration magnitude is near 1g
        // During vibration, impacts, or rapid movement, the accelerometer reads
        // forces beyond gravity and the angle computation becomes unreliable
        float accelMag = sqrtf(ax * ax + ay * ay + az * az);
        _accelValid = (fabsf(accelMag - 1.0f) < ACCEL_OUTLIER_THRESHOLD);

        if (_accelValid) {
            _accelAngle = atan2(ay, az) * 180.0f / PI;
        }
        // If invalid, we keep the previous _accelAngle (stale but less noisy)
    }

    // Read gyroscope
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
        // Remove bias and get rate around X axis (roll rate = lean rate)
        _gyroRateX = gx - _gyroBiasX;
    }

    // Complementary filter with adaptive trust
    // When accelerometer data is valid, use normal alpha
    // When invalid (vibration/impact), trust gyro even more to avoid corruption
    float alpha = _accelValid ? COMPLEMENTARY_ALPHA : 0.995f;

    _leanAngle = alpha * (_leanAngle + _gyroRateX * dt)
               + (1.0f - alpha) * _accelAngle;

    _leanRate = _gyroRateX;
}
