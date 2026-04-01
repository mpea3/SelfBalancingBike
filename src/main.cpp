#include <Arduino.h>
#include <../lib/fall_down_effect.h>

FallDownEffect balancer(12.3, 0.1, 0.01, 0.02);

// speed 75
// FallDownEffect balancer(3.3, 5.0, -7.06);

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
