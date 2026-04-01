#pragma once
#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoMotorCarrier.h>

class FallDownEffect
{
private:
  // Sensor data
  float Ax = 0, Ay = 0, Az = 0;
  float Gx = 0, Gy = 0, Gz = 0;

  // State
  float accelAngle = 0;
  float currentAngle = 0;
  float thetaDot = 0;
  float wheelSpeed = 0;
  float currentDuty = 0;
  unsigned long lastGyroTime = 0;

  // PID state
  float integralError = 0;
  float lastError = 0;
  float dt = 0;

  // Controller gains
  float Kp = 5.0;
  float Ki = 0.0;
  float Kd = 0.5;
  float KpIW = 0.0;
  float Kbrake = 0.05; // Flywheel braking: opposes wheelSpeed every frame

  // Anti-windup limit for integral term
  float integralLimit = 15.0;

  float rampRate = 4.0;

  // ---- TASK 1: Gyroscope - detect fall angle ----
  void updateGyroscope()
  {
    unsigned long now = micros();
    dt = (now - lastGyroTime) / 1000000.0;
    if (dt <= 0 || dt > 0.5) dt = 0.01;
    lastGyroTime = now;

    if (IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(Gx, Gy, Gz);

      // Complementary filter: gyro for fast response, accel for drift correction
      currentAngle = 0.98 * (currentAngle + Gx * 57.3 * dt) + 0.02 * accelAngle;

      thetaDot = Gx * 57.3;
    }

    wheelSpeed = (wheelSpeed + (currentDuty * dt));
  }

  void controlFlywheel()
  {
    if (IMU.accelerationAvailable())
    {
      IMU.readAcceleration(Ay, Ax, Az);
      accelAngle = atan2(Ay, Az) * 57.3;
    }

    float targetAngle = 0.0; // Set this to the observed resting angle from Serial Monitor

    float error = currentAngle - targetAngle;

    // Integral term with anti-windup clamping
    integralError += error * dt;
    integralError = constrain(integralError, -integralLimit, integralLimit);

    // Balance torque: PID corrects lean angle
    float balanceTorque = (Kp * error)
                        + (Ki * integralError)
                        + (Kd * thetaDot)
                        + (KpIW * wheelSpeed);

    // Braking torque: actively opposes flywheel speed every frame
    // This prevents the flywheel from spinning up indefinitely
    float brakingTorque = -Kbrake * wheelSpeed;

    float torque = balanceTorque + brakingTorque;
    float targetDuty = constrain(torque, -70, 70);

    // Dynamic ramp: allow faster response when falling quickly
    float dynamicRamp = (abs(thetaDot) > 50) ? rampRate * 2 : rampRate;

    if (currentDuty < targetDuty)
      currentDuty = min(currentDuty + dynamicRamp, targetDuty);
    else if (currentDuty > targetDuty)
      currentDuty = max(currentDuty - dynamicRamp, targetDuty);

    M3.setDuty((int)currentDuty);
  }

public:
  // Constructor without Ki — start here, add Ki once PD is stable
  FallDownEffect(float Kp, float Kd, float KpIW)
  {
    this->Kp = Kp;
    this->Kd = Kd;
    this->KpIW = KpIW;
    this->Ki = 0.0;
  }

  // Constructor with Ki
  FallDownEffect(float Kp, float Ki, float Kd, float KpIW)
  {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->KpIW = KpIW;
  }

  // Constructor with Ki and Kbrake
  FallDownEffect(float Kp, float Ki, float Kd, float KpIW, float Kbrake)
  {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->KpIW = KpIW;
    this->Kbrake = Kbrake;
  }

  void setup()
  {
    controller.begin();
    IMU.begin();
    this->lastGyroTime = micros();
  }

  void startBalance()
  {
    updateGyroscope();
    controlFlywheel();
  }

  void consoleLog()
  {
    Serial.print("Angle:");
    Serial.print(currentAngle, 2);
    Serial.print("\tthetaDot:");
    Serial.print(thetaDot, 2);
    Serial.print("\tIntegral:");
    Serial.print(integralError, 2);
    Serial.print("\twheelSpeed:");
    Serial.print(wheelSpeed, 2);
    Serial.print("\tDuty:");
    Serial.println((int)currentDuty);
  }
};