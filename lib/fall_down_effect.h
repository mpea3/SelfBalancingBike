#pragma once
#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoMotorCarrier.h>

/* PDP Controller: u(t) = Kp*e(t) + Kd*de/dt + Kp2*gyroRate */

class FallDownEffect
{
private:
  Print* logger = &Serial;

  // Sensor data
  float Ax = 0, Ay = 0, Az = 0;
  float Gx = 0, Gy = 0, Gz = 0;

  // State
  float accelAngle = 0;
  float currentAngle = 0;
  float currentDuty = 0;
  float currentServo = 0;
  unsigned long lastGyroTime = 0;

  // PDP state
  float prevError = 0;

  // PDP gains
  float Kp;
  float Kd;
  float Kp2;

  float rampRate = 4.0;
  float servoRampRate = 2.0;
  int servoCenter = 180;

  // Read gyro, fuse with accel for angle estimate
  void updateSensors()
  {
    if (IMU.accelerationAvailable())
    {
      IMU.readAcceleration(Ay, Ax, Az);
      accelAngle = atan2f(Ay, Az) * 57.3f;
    }

    if (IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(Gx, Gy, Gz);

      unsigned long now = micros();
      float dt = (now - lastGyroTime) / 1000000.0f;
      lastGyroTime = now;

      // Complementary filter: gyro for fast response, accel for drift correction
      currentAngle = 0.98f * (currentAngle + Gx * 57.3f * dt) + 0.02f * accelAngle;
    }
  }

  // PDP: u(t) = Kp*e(t) + Kd*de/dt + Kp2*gyroRate
  void controlFlywheel(float dt)
  {
    float yRef = 0.0f;  // target angle
    float error = currentAngle - yRef;

    // D: rate of change of error
    float derivative = (error - prevError) / dt;
    prevError = error;

    // u(t) = Kp*e + Kd*de/dt + Kp2*gyroRate
    float torque = (Kp * error) + (Kd * derivative) + (Kp2 * Gx);

    float targetDuty = constrain(-torque, -70, 70);

    // Ramp with emergency boost for fast falls
    float dynamicRamp = rampRate;
    if (fabsf(derivative) > 50.0f)
      dynamicRamp = rampRate * 2.0f;

    if (currentDuty < targetDuty)
      currentDuty = min(currentDuty + dynamicRamp, targetDuty);
    else if (currentDuty > targetDuty)
      currentDuty = max(currentDuty - dynamicRamp, targetDuty);

    M3.setDuty((int)currentDuty);
  }

public:
  void setup()
  {
    controller.begin();
    IMU.begin();
    this->lastGyroTime = micros();
  }

  FallDownEffect(float Kp, float Kd, float Kp2, int servoCenter = 90)
  {
    this->Kp = Kp;
    this->Kd = Kd;
    this->Kp2 = Kp2;
    this->servoCenter = servoCenter;
    this->currentServo = servoCenter;
  };

  void setLogger(Print& output)
  {
    logger = &output;
  }

  void startBalance()
  {
    unsigned long now = micros();
    if (now - lastGyroTime < 1000) return;  // 1ms update rate
    float dt = (now - lastGyroTime) / 1000000.0f;
    lastGyroTime = now;

    updateSensors();
    controlFlywheel(dt);
  }

  void consoleLog()
  {
    logger->print("Angle:");
    logger->print(currentAngle);
    logger->print(" Ax:");
    logger->print(Ax);
    logger->print(" Ay:");
    logger->print(Ay);
    logger->print(" Az:");
    logger->print(Az);
    logger->print(" Gx:");
    logger->print(Gx);
    logger->print(" Gy:");
    logger->print(Gy);
    logger->print(" Gz:");
    logger->print(Gz);
    logger->print(" Duty:");
    logger->print((int)currentDuty);
    logger->print(" Servo:");
    logger->println((int)currentServo);
  }
};
