#include <Arduino.h>
#include <../lib/fall_down_effect.h>

// Kp=2, Kd=0.3, KpIW=0.0001, servoCenter=50
FallDownEffect balancer(2.0, 0.3, 0.0001, 50);

void setup()
{
  Serial.begin(115200);
  balancer.setup();
}

void loop()
{
  balancer.startBalance();
  balancer.consoleLog();
}
