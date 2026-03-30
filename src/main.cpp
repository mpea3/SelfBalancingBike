#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiNINA.h>
#include <../lib/fall_down_effect.h>

// WiFi credentials — update these for your network
const char* WIFI_SSID = "MaeLaptop";
const char* WIFI_PASS = "opensesame";

// PID gains: Kp, Ki, Kd
FallDownEffect balancer(50.0, 0.015, 5.0, 90);

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected — IP: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  connectWiFi();
  ArduinoOTA.begin(WiFi.localIP(), "nano33iot", "password", InternalStorage);
  balancer.setup();
}

void loop() {
  ArduinoOTA.poll();
  balancer.startBalance();
  // balancer.consoleLog();
}
