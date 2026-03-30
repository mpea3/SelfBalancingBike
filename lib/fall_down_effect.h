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
  float currentServo = 0;
  unsigned long lastGyroTime = 0;

  // Controller gains
  float Kp = 5.0;
  float Kd = 0.5;
  float KpIW = 0.0001;

  float rampRate = 4.0;
  float servoRampRate = 2.0;
  int servoCenter = 180;

  // ---- TASK 1: Gyroscope - detect fall angle ----
  // Reads gyro whenever available, fuses with latest accel for angle estimate
  void updateGyroscope()
  {
    unsigned long now = micros();
    float dt = (now - lastGyroTime) / 1000000.0;
    lastGyroTime = now;

    if (IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(Gx, Gy, Gz);

      // Complementary filter: gyro for fast response, accel for drift correction
      currentAngle = 0.98 * (currentAngle + Gx * 57.3 * dt) + 0.02 * accelAngle;

      // Angular velocity of lean
      thetaDot = -Gx * 57.3;
    }

    // wheelSpeed = (wheelSpeed + (currentDuty * dt)) * 0.95;
    wheelSpeed = wheelSpeed * exp(-dt / 1.0) + currentDuty * dt;
  }

  void controlFlywheel()
  {
    if (IMU.accelerationAvailable())
    {
      IMU.readAcceleration(Ay, Ax, Az);
      // Use atan2 for a stable 360-degree reference of "down"
      accelAngle = atan2(Ay, Az) * 57.3;
    }

    // 1. The "Catch": Kd reacts to the SPEED of the fall (from Gyro)
    // 2. The "Hold": Kp reacts to the ANGLE of the fall (from Accel/Filter)
    // 3. The "Inertia": KpIW prevents the wheel from spinning faster and faster forever
    float targetAngle = 0; // Put your "Sweet Spot" angle here
    float error = currentAngle - targetAngle;
    float torque = (Kp * error) + (Kd * thetaDot) + (KpIW * wheelSpeed);

    float targetDuty = constrain(-torque, -70, 70);

    // Ramping logic (Your "Physical Awareness" code)
    // If the bike is falling FAST (high thetaDot), we should
    // perhaps allow a faster ramp to save it!
    float dynamicRamp = rampRate;
    if (abs(thetaDot) > 50)
      dynamicRamp = rampRate * 2; // Emergency boost

    if (currentDuty < targetDuty)
    {
      currentDuty = min(currentDuty + dynamicRamp, targetDuty);
    }
    else if (currentDuty > targetDuty)
    {
      currentDuty = max(currentDuty - dynamicRamp, targetDuty);
    }

    M3.setDuty((int)currentDuty);
  }

  // -- --TASK 3 : Front wheel - steer on gyro fall, center on accel center-- --Reacts to gyro - detected fall, returns to center when accel is centered 
  void controlFrontWheel()
  {
    float targetServo = servoCenter;

    // Only steer when significantly tilted — wide deadband to prevent jitter
    if (abs(currentAngle) > 10)
    {
      int steerOffset = constrain((int)(currentAngle * -0.5), -30, 30);
      targetServo = constrain(servoCenter + steerOffset, 0, 180);
    }

    // Ramp smoothly — slow rate prevents left-right oscillation
    if (currentServo < targetServo)
    {
      currentServo = min(currentServo + servoRampRate, targetServo);
    }
    else if (currentServo > targetServo)
    {
      currentServo = max(currentServo - servoRampRate, targetServo);
    }

    servo3.setAngle((int)currentServo);
  }

public:
  void setup()
  {
    controller.begin();
    IMU.begin();
    this->lastGyroTime = micros();
  }

  FallDownEffect(float Kp, float Kd, float KpIW, int servoCenter = 90)
  {
    this->Kp = Kp;
    this->Kd = Kd;
    this->KpIW = KpIW;
    this->servoCenter = servoCenter;
    this->currentServo = servoCenter;
  };

  // Call every loop — runs all three tasks independently
  void startBalance()
  {
    updateGyroscope();
    controlFlywheel();
    controlFrontWheel();
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