#include <Arduino.h>
#include <../lib/fall_down_effect.h>

// Kp=2, Kd=0.3, KpIW=0.0001, servoCenter=50
// FallDownEffect balancer(4.1, 1.83, 0.2001, 50);
// 7.3, 1.8, 0.001, 50
FallDownEffect balancer(7.3, 1.8, 2.6, 50);

void setup()
{
  Serial.begin(115200);
  balancer.setup();
}

void loop()
{
  balancer.startBalance();
  balancer.consoleLog();
  M2.setDuty(0);
}
