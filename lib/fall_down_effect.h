#pragma once
#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoMotorCarrier.h>

class FallDownEffect
{
private:
  float Ax, Ay, Az;
  float Gx, Gy, Gz;
  float currentAngle = 0;
  unsigned long lastTime = 0;
  float sensitivity = 5.0;

  void updateBalance()
  {
    unsigned long currentTime = micros();
    // Calculate delta time (time between loops in seconds)
    float dt = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable())
    {
      IMU.readAcceleration(Ay, Ax, Az);
      IMU.readGyroscope(Gx, Gy, Gz);

      // Az for the lean
      float accelAngle = atan2(Ay, Az) * 57.3;

      // COMPLEMENTARY FILTER
      // Trust the Gyro (Gx) for quick changes, use Accel to fix drift
      currentAngle = 0.98 * (currentAngle + Gx * 57.3 * dt) + 0.02 * (accelAngle);
    }
  }

public:
  void setup()
  {
    controller.begin();
    IMU.begin();
    this->lastTime = micros();
  }

  FallDownEffect(float sensitivity)
  {
    this->sensitivity = sensitivity;
  };

  void startBalance()
  {
    this->updateBalance();

    float stabilityControl = currentAngle * this->sensitivity;
    if (stabilityControl > 25)
    {
      M3.setDuty(constrain(stabilityControl, -40, 40));
    }
    else if (stabilityControl < -25)
    {
      M3.setDuty(constrain(stabilityControl, -40, 40));
    }
    else
    {
      // add the Ax shit with that third wheel

      // works well with just single
      // M3.setDuty(0);
    }
  }

  void consoleLog()
  {
    Serial.print("Angle:");
    Serial.println(currentAngle);
    // Serial.print("stability:");
    // Serial.println(stabilityControl);
  }
};
