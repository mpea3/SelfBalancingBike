#include <Arduino.h>
#include <WiFiNINA.h>
#include <ArduinoOTA.h>
#include <../lib/fall_down_effect.h>

// WiFi credentials — update these for your network
const char* WIFI_SSID = "MaeLaptop";
const char* WIFI_PASS = "opensesame";
const uint16_t OTA_SERIAL_PORT = 65281;

WiFiServer otaSerialServer(OTA_SERIAL_PORT);
WiFiClient otaSerialClient;

class TeePrint : public Print {
public:
  TeePrint(Print& primary, WiFiClient* mirrorClient)
    : primary_(primary), mirrorClient_(mirrorClient) {}

  size_t write(uint8_t c) override {
    primary_.write(c);
    if (mirrorClient_ != nullptr && *mirrorClient_ && mirrorClient_->connected()) {
      mirrorClient_->write(&c, 1);
    }
    return 1;
  }

  size_t write(const uint8_t* buffer, size_t size) override {
    primary_.write(buffer, size);
    if (mirrorClient_ != nullptr && *mirrorClient_ && mirrorClient_->connected()) {
      mirrorClient_->write(buffer, size);
    }
    return size;
  }

private:
  Print& primary_;
  WiFiClient* mirrorClient_;
};

TeePrint Log(Serial, &otaSerialClient);

// PDP gains: Kp, Kd, Kp2
FallDownEffect balancer(30.0, 5.0, 0.015, 90);

void pollOtaSerialMonitor() {
  if (otaSerialClient && !otaSerialClient.connected()) {
    otaSerialClient.stop();
    Serial.println("OTA serial monitor disconnected");
  }

  WiFiClient incoming = otaSerialServer.available();
  if (!incoming) {
    return;
  }

  if (otaSerialClient && otaSerialClient.connected()) {
    incoming.println("OTA serial monitor already connected");
    incoming.stop();
    return;
  }

  otaSerialClient = incoming;
  Serial.print("OTA serial monitor connected from ");
  Serial.println(otaSerialClient.remoteIP());
}

void connectWiFi() {
  Log.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Log.print(".");
  }
  Log.println();
  Log.print("Connected - IP: ");
  Log.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  connectWiFi();
  otaSerialServer.begin();
  ArduinoOTA.begin(WiFi.localIP(), "nano33iot", "password", InternalStorage);
  balancer.setLogger(Log);
  Log.print("OTA serial monitor ready on TCP port ");
  Log.println(OTA_SERIAL_PORT);
  balancer.setup();
}

void loop() {
  ArduinoOTA.poll();
  pollOtaSerialMonitor();
  balancer.startBalance();
  // balancer.consoleLog();
}
