#pragma once
#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoMotorCarrier.h>

/* PID Controller: u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de/dt */

class FallDownEffect
{
private:
  // Sensor data
  float Ax = 0, Ay = 0, Az = 0;
  float Gx = 0, Gy = 0, Gz = 0;

  // State
  float accelAngle = 0;
  float currentAngle = 0;
  float currentDuty = 0;
  float currentServo = 0;
  unsigned long lastGyroTime = 0;

  // PID state
  float integral = 0;
  float prevError = 0;

  // PID gains
  float Kp;
  float Ki;
  float Kd;

  float integralLimit = 50.0f;  // anti-windup clamp

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

  // PID: u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de/dt
  void controlFlywheel(float dt)
  {
    float yRef = 0.0f;  // target angle
    float error = currentAngle - yRef;

    // I: accumulate error over time
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit);

    // D: rate of change of error
    float derivative = (error - prevError) / dt;
    prevError = error;

    // u(t) = Kp*e + Ki*∫e + Kd*de/dt
    float torque = (Kp * error) + (Ki * integral) + (Kd * derivative);

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

  FallDownEffect(float Kp, float Ki, float Kd, int servoCenter = 90)
  {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->servoCenter = servoCenter;
    this->currentServo = servoCenter;
  };

  void startBalance()
  {
    unsigned long now = micros();
    float dt = (now - lastGyroTime) / 1000000.0f;
    lastGyroTime = now;

    updateSensors();
    controlFlywheel(dt);
  }

  void consoleLog()
  {
    Serial.print("Angle:");
    Serial.print(currentAngle);
    Serial.print(" Ax:");
    Serial.print(Ax);
    Serial.print(" Ay:");
    Serial.print(Ay);
    Serial.print(" Az:");
    Serial.print(Az);
    Serial.print(" Gx:");
    Serial.print(Gx);
    Serial.print(" Gy:");
    Serial.print(Gy);
    Serial.print(" Gz:");
    Serial.print(Gz);
    Serial.print(" Duty:");
    Serial.print((int)currentDuty);
    Serial.print(" Servo:");
    Serial.println((int)currentServo);
  }
};
