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

    wheelSpeed += currentDuty * dt;
  }

  // ---- TASK 2: Flywheel - balance with accelerometer + gyro ----
  // Uses both sensors to compute torque and drive M3
  void controlFlywheel()
  {
    // Read accelerometer independently (non-blocking)
    if (IMU.accelerationAvailable())
    {
      IMU.readAcceleration(Ay, Ax, Az);
      accelAngle = atan2(Ay, Az) * 57.3;
    }

    // PD + inertia wheel speed controller — ALWAYS responds
    float torque = Kp * currentAngle + Kd * thetaDot + KpIW * wheelSpeed;

    // Accel not centered — add correction scaled 0 to 100
    float axCorrection = 0;
    if (abs(Ax) > 0.05)
    {
      float power = map(abs(Ax) * 100, 5, 100, 0, 50);
      power = constrain(power, 0, 50);
      axCorrection = (Ax > 0) ? -power : power;
    }

    // Combine both — always driving, never stops
    float targetDuty = constrain(torque + axCorrection, -100, 100);

    // Ramp linearly to prevent wheel spin
    if (currentDuty < targetDuty)
    {
      currentDuty = min(currentDuty + rampRate, targetDuty);
    }
    else if (currentDuty > targetDuty)
    {
      currentDuty = max(currentDuty - rampRate, targetDuty);
    }

    M3.setDuty((int)currentDuty);
  }

  // ---- TASK 3: Front wheel - steer on gyro fall, center on accel center ----
  // Reacts to gyro-detected fall, returns to center when accel is centered
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
